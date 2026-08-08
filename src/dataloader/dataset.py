"""ROS-independent reader for converted dataloader sequences."""

import csv
import os
from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .lidar import point_dtype, read_structured_points, standard_points
from .pose_utils import quat_to_matrix, timestamp_to_ns


def _local_metadata_path(root, raw_path, label):
    raw_path = Path(raw_path)
    if raw_path.is_absolute():
        raise ValueError("{} must be relative: {}".format(label, raw_path))
    path = (root / raw_path).resolve()
    if path != root and root not in path.parents:
        raise ValueError("{} escapes the sequence directory: {}".format(label, raw_path))
    return path


def _event_path(root, raw_path, storage_mode):
    raw_path = Path(raw_path)
    if raw_path.is_absolute():
        if storage_mode != "reference":
            raise ValueError(
                "absolute event path requires reference storage: {}".format(raw_path)
            )
        return raw_path
    path = Path(os.path.abspath(str(root / raw_path)))
    if path != root and root not in path.parents:
        raise ValueError("event path escapes the sequence directory: {}".format(raw_path))
    return path


@dataclass(frozen=True)
class Event:
    timestamp_ns: int
    sensor: str
    path: Path
    kind: str
    format: str
    frame_id: str = ""

    def as_lidar(self):
        if self.kind not in ("pointcloud", "livox_custom"):
            raise TypeError("{} is not a LiDAR event".format(self.sensor))
        return LidarFrame(
            timestamp_ns=self.timestamp_ns,
            sensor=self.sensor,
            path=self.path,
            kind=self.kind,
            format=self.format,
            frame_id=self.frame_id,
        )


@dataclass(frozen=True)
class LidarFrame(Event):
    def structured(self):
        """Return all fields using the sensor format's packed NumPy dtype."""
        return read_structured_points(self.path, self.format, self.timestamp_ns)

    def numpy(self):
        """Return a common ``float32[N,4]`` array: x, y, z, intensity."""
        return standard_points(self.structured())

    @property
    def fields(self):
        return point_dtype(self.format, self.timestamp_ns).names


@dataclass(frozen=True)
class Pose:
    timestamp_ns: int
    translation: np.ndarray
    quaternion: np.ndarray

    def matrix(self):
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = quat_to_matrix(*self.quaternion)
        transform[:3, 3] = self.translation
        return transform


class Dataset:
    """Read one converted sequence through a dataset-independent API."""

    def __init__(self, sequence_dir):
        self.root = Path(sequence_dir).expanduser().resolve()
        manifest_path = self.root / "manifest.yaml"
        if not manifest_path.is_file():
            raise FileNotFoundError("manifest.yaml not found: {}".format(manifest_path))
        with manifest_path.open("r") as handle:
            self.manifest = yaml.safe_load(handle) or {}
        if not isinstance(self.manifest, dict):
            raise ValueError("manifest.yaml must contain a mapping")
        self.dataset = self.manifest.get("dataset", "")
        self.sequence = self.manifest.get("sequence", self.root.name)
        self.storage_mode = self.manifest.get("storage_mode", "")
        self.primary_lidar = self.manifest.get("primary_lidar")
        self.sensors = self.manifest.get("sensors", {})
        self.events = tuple(self._read_events())
        self._events_by_sensor = {}
        self._stamps_by_sensor = {}
        for event in self.events:
            self._events_by_sensor.setdefault(event.sensor, []).append(event)
        for sensor, events in self._events_by_sensor.items():
            self._stamps_by_sensor[sensor] = [event.timestamp_ns for event in events]
        self._pose_cache = {}

    def _read_events(self):
        timeline_name = self.manifest.get("timeline", "timeline.csv")
        timeline_path = _local_metadata_path(self.root, timeline_name, "timeline")
        if not timeline_path.is_file():
            raise FileNotFoundError("timeline not found: {}".format(timeline_path))
        events = []
        with timeline_path.open("r", newline="") as handle:
            for row in csv.DictReader(handle):
                sensor = row["sensor"]
                spec = self.sensors.get(sensor, {})
                path = _event_path(
                    self.root,
                    row["relative_path"],
                    self.storage_mode,
                )
                events.append(
                    Event(
                        timestamp_ns=int(row["timestamp_ns"]),
                        sensor=sensor,
                        path=path,
                        kind=spec.get("kind", ""),
                        format=spec.get("format", ""),
                        frame_id=spec.get("frame_id", ""),
                    )
                )
        events.sort(key=lambda event: (event.timestamp_ns, event.sensor))
        return events

    def __iter__(self):
        return iter(self.events)

    def __len__(self):
        return len(self.events)

    @property
    def duration_ns(self):
        if len(self.events) < 2:
            return 0
        return self.events[-1].timestamp_ns - self.events[0].timestamp_ns

    def sensor_events(self, sensor):
        return tuple(self._events_by_sensor.get(sensor, ()))

    def between(self, sensor, start_ns, end_ns):
        """Return sensor events in the inclusive timestamp interval."""
        events = self._events_by_sensor.get(sensor, ())
        stamps = self._stamps_by_sensor.get(sensor, ())
        left = bisect_left(stamps, int(start_ns))
        right = bisect_right(stamps, int(end_ns))
        return tuple(events[left:right])

    def nearest(self, sensor, timestamp_ns):
        """Return the nearest sensor event, preferring the earlier on ties."""
        events = self._events_by_sensor.get(sensor, ())
        stamps = self._stamps_by_sensor.get(sensor, ())
        if not events:
            return None
        stamp = int(timestamp_ns)
        index = bisect_left(stamps, stamp)
        if index == 0:
            return events[0]
        if index == len(events):
            return events[-1]
        before = events[index - 1]
        after = events[index]
        if stamp - before.timestamp_ns <= after.timestamp_ns - stamp:
            return before
        return after

    def lidar(self, sensor=None, frame=0):
        """Return a LiDAR frame by zero-based sensor frame index."""
        sensor = sensor or self.primary_lidar
        if not sensor:
            raise ValueError("no primary_lidar is declared; pass sensor")
        events = self._events_by_sensor.get(sensor, ())
        try:
            event = events[frame]
        except IndexError as exc:
            raise IndexError(
                "LiDAR frame {} is out of range for '{}' ({} frames)".format(
                    frame, sensor, len(events)
                )
            ) from exc
        return event.as_lidar()

    def poses(self, source="gt", sensor=None):
        """Load a manifest TUM pose stream."""
        sensor = sensor or self.primary_lidar or "default"
        key = (source, sensor)
        if key in self._pose_cache:
            return self._pose_cache[key]
        group = self.manifest.get("poses", {}).get(source, {})
        relative = group.get(sensor) or group.get("default")
        if not relative:
            return ()
        path = _local_metadata_path(self.root, relative, "pose path")
        rows = []
        with path.open("r") as handle:
            for line in handle:
                line = line.split("#", 1)[0].strip()
                if not line:
                    continue
                parts = line.replace(",", " ").split()
                if len(parts) < 8:
                    continue
                rows.append(
                    Pose(
                        timestamp_ns=timestamp_to_ns(parts[0]),
                        translation=np.array(parts[1:4], dtype=np.float64),
                        quaternion=np.array(parts[4:8], dtype=np.float64),
                    )
                )
        rows.sort(key=lambda pose: pose.timestamp_ns)
        result = tuple(rows)
        self._pose_cache[key] = result
        return result

    def pose_at(self, timestamp_ns, source="gt", sensor=None):
        """Return the nearest pose from a manifest pose stream."""
        poses = self.poses(source=source, sensor=sensor)
        if not poses:
            return None
        stamps = [pose.timestamp_ns for pose in poses]
        stamp = int(timestamp_ns)
        index = bisect_left(stamps, stamp)
        if index == 0:
            return poses[0]
        if index == len(poses):
            return poses[-1]
        before, after = poses[index - 1], poses[index]
        if stamp - before.timestamp_ns <= after.timestamp_ns - stamp:
            return before
        return after
