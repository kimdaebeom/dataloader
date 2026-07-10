#!/usr/bin/env python3

import csv
import atexit
import os
import re
import shutil
import select
import struct
import sys
import termios
import time
import tty
from bisect import bisect_left
from bisect import bisect_right
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, MagneticField, NavSatFix, PointCloud2, PointField

try:
    from novatel_gps_msgs.msg import Inspva
except Exception:
    Inspva = None

try:
    from livox_ros_driver.msg import CustomMsg, CustomPoint
except Exception:
    CustomMsg = None
    CustomPoint = None

from dataloader.common import resolve_sequence_dir
from dataloader.pose_utils import matrix_to_quat, quat_to_matrix, timestamp_to_ns


AEVA_INTENSITY_THRESHOLD_NS = 1691936557946849179


POINT_FIELD_TYPES = {
    "float32": PointField.FLOAT32,
    "uint32": PointField.UINT32,
    "uint16": PointField.UINT16,
    "int32": PointField.INT32,
    "uint8": PointField.UINT8,
}


def _stamp_from_ns(timestamp_ns):
    return rospy.Time.from_sec(timestamp_ns * 1e-9)


def _read_timeline(path):
    events = []
    with path.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            events.append(
                {
                    "timestamp_ns": int(row["timestamp_ns"]),
                    "sensor": row["sensor"],
                    "relative_path": row["relative_path"],
                }
            )
    events.sort(key=lambda event: (event["timestamp_ns"], event["sensor"]))
    return events


def _load_csv_rows(path):
    rows = {}
    if not path.is_file():
        return rows
    with path.open("r", newline="") as handle:
        reader = csv.reader(handle)
        for raw in reader:
            if not raw:
                continue
            try:
                rows[int(raw[0])] = raw
            except ValueError:
                continue
    return rows


def _field(name, offset, datatype):
    return PointField(name=name, offset=offset, datatype=POINT_FIELD_TYPES[datatype], count=1)


def _quat_to_matrix(qx, qy, qz, qw):
    return quat_to_matrix(qx, qy, qz, qw)


def _matrix_to_quat(matrix):
    return matrix_to_quat(matrix)


def _load_tum_poses(path):
    poses = []
    with Path(path).expanduser().open("r") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 8:
                continue
            stamp_ns = timestamp_to_ns(parts[0])
            tx, ty, tz = (float(value) for value in parts[1:4])
            qx, qy, qz, qw = (float(value) for value in parts[4:8])
            transform = np.eye(4, dtype=np.float64)
            transform[:3, :3] = _quat_to_matrix(qx, qy, qz, qw)
            transform[:3, 3] = [tx, ty, tz]
            poses.append((stamp_ns, transform))
    poses.sort(key=lambda item: item[0])
    return poses


def _load_matrix_txt(path):
    values = []
    with Path(path).expanduser().open("r") as handle:
        for line in handle:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            values.extend(float(value) for value in line.replace(",", " ").split())
    if len(values) != 16:
        raise ValueError("4x4 matrix file must contain 16 numbers: {}".format(path))
    return np.array(values, dtype=np.float64).reshape((4, 4))


def _transform_pointcloud2(msg, transform):
    if msg.width * msg.height == 0:
        return msg
    if msg.is_bigendian:
        raise ValueError("big-endian PointCloud2 transform is not supported")
    offsets = {field.name: field.offset for field in msg.fields}
    if not {"x", "y", "z"} <= set(offsets):
        raise ValueError("PointCloud2 has no x/y/z fields")
    if offsets["x"] != 0 or offsets["y"] != 4 or offsets["z"] != 8:
        raise ValueError("PointCloud2 x/y/z offsets are not supported")
    data = bytearray(msg.data)
    count = msg.width * msg.height
    coords = np.ndarray(shape=(count, 3), dtype="<f4", buffer=data, strides=(msg.point_step, 4))
    transformed = coords.astype(np.float64) @ transform[:3, :3].T + transform[:3, 3]
    coords[:] = transformed.astype(np.float32)
    msg.data = bytes(data)
    return msg


def _transform_livox_msg(msg, transform):
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    for point in msg.points:
        xyz = rotation @ np.array([point.x, point.y, point.z], dtype=np.float64) + translation
        point.x = float(xyz[0])
        point.y = float(xyz[1])
        point.z = float(xyz[2])
    return msg


def _pose_stamped(timestamp_ns, transform, frame_id):
    msg = PoseStamped()
    msg.header.stamp = _stamp_from_ns(timestamp_ns)
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(transform[0, 3])
    msg.pose.position.y = float(transform[1, 3])
    msg.pose.position.z = float(transform[2, 3])
    qx, qy, qz, qw = _matrix_to_quat(transform)
    msg.pose.orientation.x = float(qx)
    msg.pose.orientation.y = float(qy)
    msg.pose.orientation.z = float(qz)
    msg.pose.orientation.w = float(qw)
    return msg


def _cloud_from_raw(path, timestamp_ns, frame_id, fields, point_step):
    data = path.read_bytes()
    point_count = len(data) // point_step
    data = data[: point_count * point_step]
    msg = PointCloud2()
    msg.header.stamp = _stamp_from_ns(timestamp_ns)
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = point_count
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * point_count
    msg.data = data
    msg.is_dense = True
    return msg


def _mulran_ouster(path, timestamp_ns, frame_id):
    raw = np.fromfile(str(path), dtype="<f4")
    point_count = raw.size // 4
    raw = raw[: point_count * 4].reshape((point_count, 4))
    cloud = np.empty(point_count, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("ring", "<i4")])
    cloud["x"] = raw[:, 0]
    cloud["y"] = raw[:, 1]
    cloud["z"] = raw[:, 2]
    cloud["intensity"] = raw[:, 3]
    cloud["ring"] = (np.arange(point_count, dtype=np.int32) % 64) + 1
    msg = PointCloud2()
    msg.header.stamp = _stamp_from_ns(timestamp_ns)
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = point_count
    msg.fields = [
        _field("x", 0, "float32"),
        _field("y", 4, "float32"),
        _field("z", 8, "float32"),
        _field("intensity", 12, "float32"),
        _field("ring", 16, "int32"),
    ]
    msg.is_bigendian = False
    msg.point_step = 20
    msg.row_step = msg.point_step * point_count
    msg.data = cloud.tobytes()
    msg.is_dense = True
    return msg


def _semantic_kitti_velodyne(path, timestamp_ns, frame_id):
    return _cloud_from_raw(
        path,
        timestamp_ns,
        frame_id,
        [
            _field("x", 0, "float32"),
            _field("y", 4, "float32"),
            _field("z", 8, "float32"),
            _field("intensity", 12, "float32"),
        ],
        16,
    )


def _helipr_livox_avia(path, timestamp_ns, frame_id):
    if CustomMsg is None or CustomPoint is None:
        raise RuntimeError("livox_ros_driver is required to publish livox_avia")
    point_step = 19
    data = path.read_bytes()
    msg = CustomMsg()
    msg.header.stamp = _stamp_from_ns(timestamp_ns)
    msg.header.frame_id = frame_id
    for offset in range(0, len(data) - point_step + 1, point_step):
        x, y, z, reflectivity, tag, line, offset_time = struct.unpack_from("<fffBBBI", data, offset)
        point = CustomPoint()
        point.x = x
        point.y = y
        point.z = z
        point.reflectivity = reflectivity
        point.tag = tag
        point.line = line
        point.offset_time = offset_time
        msg.points.append(point)
    msg.point_num = len(msg.points)
    return msg


POINTCLOUD_READERS = {
    "mulran_ouster": _mulran_ouster,
    "semantic_kitti_velodyne": _semantic_kitti_velodyne,
    "helipr_ouster": lambda path, stamp, frame: _cloud_from_raw(
        path,
        stamp,
        frame,
        [
            _field("x", 0, "float32"),
            _field("y", 4, "float32"),
            _field("z", 8, "float32"),
            _field("intensity", 12, "float32"),
            _field("t", 16, "uint32"),
            _field("reflectivity", 20, "uint16"),
            _field("ring", 22, "uint16"),
            _field("ambient", 24, "uint16"),
        ],
        26,
    ),
    "helipr_velodyne": lambda path, stamp, frame: _cloud_from_raw(
        path,
        stamp,
        frame,
        [
            _field("x", 0, "float32"),
            _field("y", 4, "float32"),
            _field("z", 8, "float32"),
            _field("intensity", 12, "float32"),
            _field("ring", 16, "uint16"),
            _field("time", 18, "float32"),
        ],
        22,
    ),
    "helipr_aeva": None,
}


def _helipr_aeva(path, timestamp_ns, frame_id):
    has_intensity = timestamp_ns > AEVA_INTENSITY_THRESHOLD_NS
    if has_intensity:
        fields = [
            _field("x", 0, "float32"),
            _field("y", 4, "float32"),
            _field("z", 8, "float32"),
            _field("reflectivity", 12, "float32"),
            _field("velocity", 16, "float32"),
            _field("time_offset_ns", 20, "int32"),
            _field("line_index", 24, "uint8"),
            _field("intensity", 25, "float32"),
        ]
        return _cloud_from_raw(path, timestamp_ns, frame_id, fields, 29)
    fields = [
        _field("x", 0, "float32"),
        _field("y", 4, "float32"),
        _field("z", 8, "float32"),
        _field("reflectivity", 12, "float32"),
        _field("velocity", 16, "float32"),
        _field("time_offset_ns", 20, "int32"),
        _field("line_index", 24, "uint8"),
    ]
    return _cloud_from_raw(path, timestamp_ns, frame_id, fields, 25)


POINTCLOUD_READERS["helipr_aeva"] = _helipr_aeva


class UnifiedDatasetPlayer:
    def __init__(self):
        data_root = rospy.get_param("~data_root", "")
        dataset = rospy.get_param("~dataset", "")
        sequence = rospy.get_param("~sequence", "")
        if not data_root or not dataset or not sequence:
            raise RuntimeError("~data_root, ~dataset, and ~sequence are required")

        self.sequence_dir = resolve_sequence_dir(data_root, dataset, sequence)
        with (self.sequence_dir / "manifest.yaml").open("r") as handle:
            self.manifest = yaml.safe_load(handle)
        self.events = _read_timeline(self.sequence_dir / self.manifest["timeline"])

        self.play_rate = float(rospy.get_param("~play_rate", 1.0))
        self.loop = bool(rospy.get_param("~loop", False))
        self.primary_lidar = rospy.get_param("~primary_lidar", self.manifest.get("primary_lidar", "ouster"))
        self.start_lidar_frame = int(rospy.get_param("~start_lidar_frame", 0))
        self.end_lidar_frame = int(rospy.get_param("~end_lidar_frame", -1))
        self.publish_flags = rospy.get_param("~publish", {})
        self.topic_overrides = rospy.get_param("~topics", {})
        self.frame_overrides = rospy.get_param("~frame_ids", {})
        self.transform_config = rospy.get_param("~transform", {})
        self.transform_enabled = bool(self.transform_config.get("enabled", False))
        self.progress_log_interval_sec = float(rospy.get_param("~progress_log_interval_sec", 2.0))
        self.progress_log_percent_step = float(rospy.get_param("~progress_log_percent_step", 5.0))
        self.terminal_color = bool(rospy.get_param("~terminal_color", True))
        self.terminal_direct_tty = bool(rospy.get_param("~terminal_direct_tty", True))
        self.progress_bar_width = int(rospy.get_param("~progress_bar_width", 28))
        self.term = self._open_terminal()
        self.keyboard_control = bool(rospy.get_param("~keyboard_control", True))
        self.start_paused = bool(rospy.get_param("~start_paused", True))
        self.paused = self.start_paused
        self.keyboard_fd = None
        self.keyboard_close_fd = False
        self.keyboard_attr = None
        self._open_keyboard()
        self.pose_stamps = []
        self.pose_mats = []
        self.static_transform = np.eye(4, dtype=np.float64)
        self.pose_pub = None

        self.csv_cache = {}
        self.publishers = {}
        self.clock_pub = None
        self.play_event_counts = {}
        self.published_counts = Counter()
        self.selected_lidar_stamps = []
        self.total_lidar_frames = 0
        self.range_start_index = 0
        self.range_end_index = 0
        self.last_progress_log_time = 0.0
        self.last_progress_percent = -1.0
        self.play_start_wall_time = 0.0
        self._prepare_transform()
        self._prepare_range()
        self._prepare_publishers()
        self._prepare_csv_cache()
        self._log_startup_summary()

    def _sensor_enabled(self, sensor):
        if sensor not in self.manifest.get("sensors", {}):
            return False
        return bool(self.publish_flags.get(sensor, False))

    def _prepare_range(self):
        lidar_events = [event for event in self.events if event["sensor"] == self.primary_lidar]
        if not lidar_events:
            raise RuntimeError("primary lidar '{}' has no frames".format(self.primary_lidar))
        self.total_lidar_frames = len(lidar_events)
        start_index = max(0, self.start_lidar_frame)
        end_index = self.end_lidar_frame if self.end_lidar_frame >= 0 else len(lidar_events) - 1
        end_index = min(end_index, len(lidar_events) - 1)
        if start_index > end_index:
            raise RuntimeError("invalid lidar frame range: {} > {}".format(start_index, end_index))
        self.range_start_index = start_index
        self.range_end_index = end_index
        selected_lidar_events = lidar_events[start_index : end_index + 1]
        selected_lidar_ids = {id(event) for event in selected_lidar_events}
        self.selected_lidar_stamps = [event["timestamp_ns"] for event in selected_lidar_events]
        self.start_stamp = lidar_events[start_index]["timestamp_ns"]
        self.end_stamp = lidar_events[end_index]["timestamp_ns"]
        enabled = {name for name in self.manifest.get("sensors", {}) if self._sensor_enabled(name)}
        self.play_events = [
            event
            for event in self.events
            if event["sensor"] in enabled
            and (
                id(event) in selected_lidar_ids
                or (event["sensor"] != self.primary_lidar and self.start_stamp <= event["timestamp_ns"] <= self.end_stamp)
            )
        ]
        self.play_event_counts = dict(Counter(event["sensor"] for event in self.play_events))

    def _format_counts(self, counts):
        if not counts:
            return "none"
        return ", ".join("{}={}".format(sensor, counts[sensor]) for sensor in sorted(counts))

    def _format_publishers(self):
        if not self.publishers:
            return "none"
        items = []
        for sensor in sorted(self.publishers):
            topic = getattr(self.publishers[sensor], "name", "")
            items.append("{}:{}".format(sensor, topic))
        return ", ".join(items)

    def _format_rate(self):
        text = "{:.3f}".format(self.play_rate).rstrip("0").rstrip(".")
        return text or "0"

    def _open_terminal(self):
        if self.terminal_direct_tty:
            try:
                return open("/dev/tty", "w", buffering=1)
            except OSError:
                pass
        return sys.stdout

    def _open_keyboard(self):
        if not self.keyboard_control:
            self.paused = False
            return
        candidates = []
        try:
            if sys.stdin.isatty():
                candidates.append((sys.stdin.fileno(), False))
        except OSError:
            pass
        try:
            candidates.append((os.open("/dev/tty", os.O_RDONLY), True))
        except OSError:
            pass
        for fd, close_fd in candidates:
            try:
                self.keyboard_fd = fd
                self.keyboard_close_fd = close_fd
                self.keyboard_attr = termios.tcgetattr(self.keyboard_fd)
                tty.setcbreak(self.keyboard_fd)
                atexit.register(self._restore_keyboard)
                return
            except OSError:
                if close_fd:
                    try:
                        os.close(fd)
                    except OSError:
                        pass
                self.keyboard_fd = None
                self.keyboard_close_fd = False
                self.keyboard_attr = None
        if self.paused:
            self.paused = False
            rospy.logwarn("keyboard control unavailable; starting playback immediately")

    def _restore_keyboard(self):
        if self.keyboard_fd is None:
            return
        try:
            if self.keyboard_attr is not None:
                termios.tcsetattr(self.keyboard_fd, termios.TCSADRAIN, self.keyboard_attr)
            if self.keyboard_close_fd:
                os.close(self.keyboard_fd)
        except OSError:
            pass
        self.keyboard_fd = None
        self.keyboard_close_fd = False
        self.keyboard_attr = None

    def _terminal_width(self):
        try:
            return max(40, os.get_terminal_size(self.term.fileno()).columns)
        except OSError:
            pass
        try:
            return max(40, shutil.get_terminal_size(fallback=(100, 20)).columns)
        except OSError:
            return 100

    def _strip_ansi(self, text):
        return re.sub(r"\033\[[0-9;]*m", "", text)

    def _fit_line(self, text):
        width = self._terminal_width() - 1
        if len(self._strip_ansi(text)) <= width:
            return text
        plain = self._strip_ansi(text)
        if len(plain) <= width:
            return plain
        return plain[: max(0, width - 1)] + "…"

    def _color(self, text, code):
        if not self.terminal_color:
            return text
        return "\033[{}m{}\033[0m".format(code, text)

    def _term_line(self, text=""):
        self.term.write(self._fit_line(text) + "\n")
        self.term.flush()

    def _term_status(self, text):
        self.term.write("\r\033[2K" + self._fit_line(text))
        self.term.flush()

    def _term_newline(self):
        self.term.write("\n")
        self.term.flush()

    def _progress_bar(self, percent):
        width = max(8, min(60, self.progress_bar_width))
        filled = int(round(width * max(0.0, min(100.0, percent)) / 100.0))
        return "{}{}".format("#" * filled, "-" * (width - filled))

    def _log_startup_summary(self):
        sensors = sorted(self.manifest.get("sensors", {}))
        enabled = sorted(sensor for sensor in sensors if self._sensor_enabled(sensor))
        duration_sec = max(0.0, (self.end_stamp - self.start_stamp) * 1e-9)
        title = self._color("dataloader playback", "1;36")
        dataset = "{}/{}".format(self.manifest.get("dataset", ""), self.manifest.get("sequence", ""))
        transform = "on" if self.transform_enabled else "off"
        clock = "on" if self.clock_pub is not None else "off"
        self._term_line("")
        self._term_line("{} | {} | {} | rate {}x | loop {}".format(title, dataset, self.manifest.get("storage_mode", ""), self._format_rate(), self.loop))
        self._term_line(
            "{} events | lidar {}:{}-{} ({}/{}) | {:.1f}s | clock {} | tf {}".format(
                self._color(str(len(self.play_events)), "1;32"),
                self.primary_lidar,
                self.range_start_index,
                self.range_end_index,
                len(self.selected_lidar_stamps),
                self.total_lidar_frames,
                duration_sec,
                clock,
                transform,
            )
        )
        self._term_line("sensors: {} | topics: {}".format(", ".join(enabled) if enabled else "none", self._format_publishers()))
        if self.keyboard_fd is not None:
            state = "paused" if self.paused else "playing"
            self._term_line("control: space=pause/resume, q=quit | start={}".format(state))
        if self.transform_enabled:
            pose_source = self.transform_config.get("pose_source", "custom")
            pose_topic = getattr(self.pose_pub, "name", "off") if self.pose_pub is not None else "off"
            self._term_line("transform: pose_source={} poses={} pose_topic={}".format(pose_source, len(self.pose_stamps), pose_topic))

    def _topic_for(self, sensor, spec):
        return self.topic_overrides.get(sensor, spec.get("topic", "/" + sensor))

    def _frame_for(self, sensor, spec):
        return self.frame_overrides.get(sensor, spec.get("frame_id", sensor))

    def _prepare_publishers(self):
        if bool(self.publish_flags.get("clock", True)):
            self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)
        if self.transform_enabled and bool(self.transform_config.get("publish_pose", False)):
            pose_topic = self.transform_config.get("pose_topic", "/dataloader/pose")
            self.pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)
        for sensor, spec in self.manifest.get("sensors", {}).items():
            if not self._sensor_enabled(sensor):
                continue
            topic = self._topic_for(sensor, spec)
            kind = spec["kind"]
            if kind == "pointcloud":
                self.publishers[sensor] = rospy.Publisher(topic, PointCloud2, queue_size=10)
            elif kind == "image":
                self.publishers[sensor] = rospy.Publisher(topic, Image, queue_size=10)
            elif kind == "livox_custom":
                if CustomMsg is None:
                    raise RuntimeError("livox_ros_driver is required to publish {}".format(sensor))
                self.publishers[sensor] = rospy.Publisher(topic, CustomMsg, queue_size=10)
            elif spec["format"] == "mulran_gps":
                self.publishers[sensor] = rospy.Publisher(topic, NavSatFix, queue_size=100)
            elif spec["format"] == "xsens_imu":
                self.publishers[sensor] = rospy.Publisher(topic, Imu, queue_size=100)
                if bool(self.publish_flags.get("imu_mag", False)):
                    self.publishers["imu_mag"] = rospy.Publisher(spec.get("mag_topic", "/imu/mag"), MagneticField, queue_size=100)
            elif spec["format"] == "novatel_inspva":
                if Inspva is None:
                    raise RuntimeError("novatel_gps_msgs is required to publish inspva")
                self.publishers[sensor] = rospy.Publisher(topic, Inspva, queue_size=100)

    def _prepare_csv_cache(self):
        for sensor, spec in self.manifest.get("sensors", {}).items():
            if spec["kind"] == "csv" and self._sensor_enabled(sensor):
                self.csv_cache[sensor] = _load_csv_rows(self.sequence_dir / spec["out_file"])

    def _prepare_transform(self):
        if not self.transform_enabled:
            return
        matrix_file = self.transform_config.get("static_matrix_file", "")
        if matrix_file:
            matrix_file = self._resolve_data_path(matrix_file)
            self.static_transform = _load_matrix_txt(matrix_file)
        pose_file = self.transform_config.get("pose_file", "")
        pose_source = self.transform_config.get("pose_source", "custom")
        if pose_source != "custom" and not pose_file:
            pose_file = self._manifest_pose_file(pose_source)
        if pose_file:
            pose_file = self._resolve_data_path(pose_file)
            pose_format = self.transform_config.get("pose_format", "tum")
            if pose_format != "tum":
                raise RuntimeError("unsupported pose_format '{}'; only 'tum' is supported".format(pose_format))
            poses = _load_tum_poses(pose_file)
            if not poses:
                raise RuntimeError("no TUM poses loaded from {}".format(pose_file))
            self.pose_stamps = [stamp for stamp, _ in poses]
            self.pose_mats = [mat for _, mat in poses]
            rospy.loginfo("loaded %d TUM poses from %s", len(poses), pose_file)

    def _resolve_data_path(self, path):
        path = Path(path).expanduser()
        if path.is_absolute():
            return path
        return self.sequence_dir / path

    def _manifest_pose_file(self, pose_source):
        poses = self.manifest.get("poses", {}).get(pose_source, {})
        pose_file = poses.get(self.primary_lidar) or poses.get("default")
        if not pose_file:
            raise RuntimeError("transform.pose_source is '{}', but no matching pose is recorded in manifest".format(pose_source))
        return pose_file

    def _nearest_pose(self, timestamp_ns):
        if not self.pose_stamps:
            return None
        index = bisect_left(self.pose_stamps, timestamp_ns)
        candidates = []
        if index < len(self.pose_stamps):
            candidates.append(index)
        if index > 0:
            candidates.append(index - 1)
        best_index = min(candidates, key=lambda item: abs(self.pose_stamps[item] - timestamp_ns))
        delta = abs(self.pose_stamps[best_index] - timestamp_ns)
        tolerance = int(self.transform_config.get("pose_timestamp_tolerance_ns", 50000000))
        if delta > tolerance:
            return None
        return self.pose_mats[best_index]

    def _event_transform(self, timestamp_ns):
        if not self.transform_enabled:
            return None
        pose = self._nearest_pose(timestamp_ns) if self.pose_stamps else np.eye(4, dtype=np.float64)
        if pose is None:
            return None
        order = self.transform_config.get("static_transform_order", "after_pose")
        if order == "before_pose":
            return pose @ self.static_transform
        return self.static_transform @ pose

    def _output_frame_for(self, sensor, spec):
        if self.transform_enabled and bool(self.transform_config.get("apply_to_pointcloud", True)):
            return self.transform_config.get("output_frame_id", self._frame_for(sensor, spec))
        return self._frame_for(sensor, spec)

    def _maybe_publish_pose(self, timestamp_ns, transform):
        if self.pose_pub is None or transform is None:
            return
        frame_id = self.transform_config.get("output_frame_id", "map")
        self.pose_pub.publish(_pose_stamped(timestamp_ns, transform, frame_id))

    def run(self):
        if not self.play_events:
            rospy.logwarn("no events selected for playback")
            return
        try:
            self.play_start_wall_time = time.monotonic()
            while not rospy.is_shutdown():
                self.play_start_wall_time = time.monotonic()
                self.last_progress_log_time = 0.0
                self.last_progress_percent = -1.0
                previous_stamp = None
                for index, event in enumerate(self.play_events, start=1):
                    if rospy.is_shutdown():
                        return
                    self._wait_if_paused(index, event)
                    if rospy.is_shutdown():
                        return
                    if previous_stamp is not None:
                        delay = (event["timestamp_ns"] - previous_stamp) * 1e-9 / max(self.play_rate, 1e-9)
                        if delay > 0:
                            self._controlled_sleep(delay, index, event)
                    self._wait_if_paused(index, event)
                    if rospy.is_shutdown():
                        return
                    self._publish_clock(event["timestamp_ns"])
                    self._publish_event(event)
                    self._maybe_log_progress(index, event)
                    previous_stamp = event["timestamp_ns"]
                self._maybe_log_progress(len(self.play_events), self.play_events[-1], force=True)
                self._term_newline()
                self._term_line("{} published {}".format(self._color("playback completed:", "1;32"), self._format_counts(self.published_counts)))
                if not self.loop:
                    return
                self._term_line(self._color("loop enabled: restarting playback", "1;33"))
                self.published_counts.clear()
        finally:
            self._restore_keyboard()

    def _poll_keyboard(self):
        if self.keyboard_fd is None:
            return
        try:
            while select.select([self.keyboard_fd], [], [], 0.0)[0]:
                char = os.read(self.keyboard_fd, 1)
                if char == b" ":
                    self.paused = not self.paused
                    return
                if char in (b"q", b"Q"):
                    rospy.signal_shutdown("keyboard quit")
                    return
        except OSError:
            return

    def _pause_status(self, event_index, event):
        total_events = len(self.play_events)
        event_percent = (event_index * 100.0) / total_events if total_events else 0.0
        bar = self._color(self._progress_bar(event_percent), "1;33")
        self._term_status(
            "{} [{}] {:5.1f}% | ev {}/{} | press SPACE to play | q quit"
            .format(self._color("paused", "1;33"), bar, event_percent, event_index, total_events)
        )

    def _wait_if_paused(self, event_index, event):
        self._poll_keyboard()
        pause_start = None
        while self.paused and not rospy.is_shutdown():
            if pause_start is None:
                pause_start = time.monotonic()
            self._pause_status(event_index, event)
            rospy.sleep(0.05)
            self._poll_keyboard()
        if pause_start is None:
            return 0.0
        return time.monotonic() - pause_start

    def _controlled_sleep(self, delay, event_index, event):
        end_time = time.monotonic() + delay
        while not rospy.is_shutdown():
            self._poll_keyboard()
            end_time += self._wait_if_paused(event_index, event)
            remaining = end_time - time.monotonic()
            if remaining <= 0:
                return
            rospy.sleep(min(0.05, remaining))

    def _maybe_log_progress(self, event_index, event, force=False):
        total_events = len(self.play_events)
        if total_events <= 0:
            return
        event_percent = (event_index * 100.0) / total_events
        if force and self.last_progress_percent >= event_percent:
            return
        now = time.monotonic()
        lidar_index = bisect_right(self.selected_lidar_stamps, event["timestamp_ns"])
        lidar_total = len(self.selected_lidar_stamps)
        should_log = force
        if not should_log and self.progress_log_interval_sec > 0:
            should_log = now - self.last_progress_log_time >= self.progress_log_interval_sec
        if not should_log and self.progress_log_percent_step > 0:
            should_log = event_percent - self.last_progress_percent >= self.progress_log_percent_step
        if not should_log:
            return
        self.last_progress_log_time = now
        self.last_progress_percent = event_percent
        rel_time = (event["timestamp_ns"] - self.start_stamp) * 1e-9
        bar = self._color(self._progress_bar(event_percent), "1;32")
        published_total = sum(self.published_counts.values())
        status = (
            "{} [{}] {:5.1f}% | ev {}/{} | lidar {}/{} | t+{:.2f}s | {} | pub {}"
            .format(
                self._color("play", "1;36"),
                bar,
                event_percent,
                event_index,
                total_events,
                lidar_index,
                lidar_total,
                rel_time,
                event["sensor"],
                published_total,
            )
        )
        self._term_status(status)

    def _publish_clock(self, timestamp_ns):
        if self.clock_pub is None:
            return
        msg = Clock()
        msg.clock = _stamp_from_ns(timestamp_ns)
        self.clock_pub.publish(msg)

    def _publish_event(self, event):
        sensor = event["sensor"]
        spec = self.manifest["sensors"][sensor]
        path = self.sequence_dir / event["relative_path"]
        try:
            if spec["kind"] == "pointcloud":
                reader = POINTCLOUD_READERS[spec["format"]]
                transform = self._event_transform(event["timestamp_ns"])
                msg = reader(path, event["timestamp_ns"], self._frame_for(sensor, spec))
                if transform is not None and bool(self.transform_config.get("apply_to_pointcloud", True)):
                    msg = _transform_pointcloud2(msg, transform)
                    msg.header.frame_id = self.transform_config.get("output_frame_id", msg.header.frame_id)
                if transform is not None:
                    self._maybe_publish_pose(event["timestamp_ns"], transform)
                elif self.transform_enabled:
                    rospy.logwarn_throttle(2.0, "no pose matched at %d; publishing raw %s", event["timestamp_ns"], sensor)
                self.publishers[sensor].publish(msg)
                self.published_counts[sensor] += 1
            elif spec["kind"] == "image":
                self.publishers[sensor].publish(self._load_image(path, event["timestamp_ns"], self._frame_for(sensor, spec)))
                self.published_counts[sensor] += 1
            elif spec["kind"] == "livox_custom":
                transform = self._event_transform(event["timestamp_ns"])
                msg = _helipr_livox_avia(path, event["timestamp_ns"], self._frame_for(sensor, spec))
                if transform is not None and bool(self.transform_config.get("apply_to_pointcloud", True)):
                    msg = _transform_livox_msg(msg, transform)
                    msg.header.frame_id = self.transform_config.get("output_frame_id", msg.header.frame_id)
                if transform is not None:
                    self._maybe_publish_pose(event["timestamp_ns"], transform)
                elif self.transform_enabled:
                    rospy.logwarn_throttle(2.0, "no pose matched at %d; publishing raw %s", event["timestamp_ns"], sensor)
                self.publishers[sensor].publish(msg)
                self.published_counts[sensor] += 1
            elif spec["format"] == "mulran_gps":
                msg = self._gps_msg(event["timestamp_ns"], self.csv_cache[sensor].get(event["timestamp_ns"]), self._frame_for(sensor, spec))
                if msg:
                    self.publishers[sensor].publish(msg)
                    self.published_counts[sensor] += 1
            elif spec["format"] == "xsens_imu":
                imu_msg, mag_msg = self._imu_msg(event["timestamp_ns"], self.csv_cache[sensor].get(event["timestamp_ns"]), self._frame_for(sensor, spec))
                if imu_msg:
                    self.publishers[sensor].publish(imu_msg)
                    self.published_counts[sensor] += 1
                if mag_msg and "imu_mag" in self.publishers:
                    self.publishers["imu_mag"].publish(mag_msg)
                    self.published_counts["imu_mag"] += 1
            elif spec["format"] == "novatel_inspva":
                msg = self._inspva_msg(event["timestamp_ns"], self.csv_cache[sensor].get(event["timestamp_ns"]), self._frame_for(sensor, spec))
                if msg:
                    self.publishers[sensor].publish(msg)
                    self.published_counts[sensor] += 1
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "failed to publish %s at %d: %s", sensor, event["timestamp_ns"], exc)

    def _load_image(self, path, timestamp_ns, frame_id):
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(str(path))
        msg = Image()
        msg.header.stamp = _stamp_from_ns(timestamp_ns)
        msg.header.frame_id = frame_id
        msg.height, msg.width = image.shape[:2]
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = msg.width
        msg.data = image.tobytes()
        return msg

    def _gps_msg(self, timestamp_ns, row, frame_id):
        if row is None or len(row) < 13:
            return None
        msg = NavSatFix()
        msg.header.stamp = _stamp_from_ns(timestamp_ns)
        msg.header.frame_id = frame_id
        msg.latitude = float(row[1])
        msg.longitude = float(row[2])
        msg.altitude = float(row[3])
        for index in range(9):
            msg.position_covariance[index] = float(row[4 + index])
        return msg

    def _imu_msg(self, timestamp_ns, row, frame_id):
        if row is None or len(row) < 8:
            return None, None
        msg = Imu()
        msg.header.stamp = _stamp_from_ns(timestamp_ns)
        msg.header.frame_id = frame_id
        msg.orientation.x = float(row[1])
        msg.orientation.y = float(row[2])
        msg.orientation.z = float(row[3])
        msg.orientation.w = float(row[4])
        mag_msg = None
        if len(row) >= 17:
            msg.angular_velocity.x = float(row[8])
            msg.angular_velocity.y = float(row[9])
            msg.angular_velocity.z = float(row[10])
            msg.linear_acceleration.x = float(row[11])
            msg.linear_acceleration.y = float(row[12])
            msg.linear_acceleration.z = float(row[13])
            for offset in (0, 4, 8):
                msg.orientation_covariance[offset] = 3.0
                msg.angular_velocity_covariance[offset] = 3.0
                msg.linear_acceleration_covariance[offset] = 3.0
            mag_msg = MagneticField()
            mag_msg.header.stamp = _stamp_from_ns(timestamp_ns)
            mag_msg.header.frame_id = frame_id
            mag_msg.magnetic_field.x = float(row[14])
            mag_msg.magnetic_field.y = float(row[15])
            mag_msg.magnetic_field.z = float(row[16])
        return msg, mag_msg

    def _inspva_msg(self, timestamp_ns, row, frame_id):
        if row is None or len(row) < 11 or Inspva is None:
            return None
        msg = Inspva()
        msg.header.stamp = _stamp_from_ns(timestamp_ns)
        msg.header.frame_id = frame_id
        msg.latitude = float(row[1])
        msg.longitude = float(row[2])
        msg.height = float(row[3])
        msg.north_velocity = float(row[4])
        msg.east_velocity = float(row[5])
        msg.up_velocity = float(row[6])
        msg.roll = float(row[7])
        msg.pitch = float(row[8])
        msg.azimuth = float(row[9])
        status = row[10].strip()
        if status.startswith("status:"):
            status = status.split(":", 1)[1].strip()
        msg.status = int(status)
        return msg


def main():
    rospy.init_node("dataloader_player")
    player = UnifiedDatasetPlayer()
    player.run()


if __name__ == "__main__":
    main()
