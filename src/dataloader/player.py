#!/usr/bin/env python3

import csv
import os
import struct
from pathlib import Path

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import yaml
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

        self.csv_cache = {}
        self.publishers = {}
        self.clock_pub = None
        self._prepare_range()
        self._prepare_publishers()
        self._prepare_csv_cache()

    def _sensor_enabled(self, sensor):
        if sensor not in self.manifest.get("sensors", {}):
            return False
        return bool(self.publish_flags.get(sensor, False))

    def _prepare_range(self):
        lidar_stamps = [event["timestamp_ns"] for event in self.events if event["sensor"] == self.primary_lidar]
        if not lidar_stamps:
            raise RuntimeError("primary lidar '{}' has no frames".format(self.primary_lidar))
        start_index = max(0, self.start_lidar_frame)
        end_index = self.end_lidar_frame if self.end_lidar_frame >= 0 else len(lidar_stamps) - 1
        end_index = min(end_index, len(lidar_stamps) - 1)
        if start_index > end_index:
            raise RuntimeError("invalid lidar frame range: {} > {}".format(start_index, end_index))
        self.start_stamp = lidar_stamps[start_index]
        self.end_stamp = lidar_stamps[end_index]
        enabled = {name for name in self.manifest.get("sensors", {}) if self._sensor_enabled(name)}
        self.play_events = [
            event
            for event in self.events
            if self.start_stamp <= event["timestamp_ns"] <= self.end_stamp and event["sensor"] in enabled
        ]
        rospy.loginfo(
            "dataloader playback: %s [%d:%d] -> %d events",
            self.sequence_dir,
            start_index,
            end_index,
            len(self.play_events),
        )

    def _topic_for(self, sensor, spec):
        return self.topic_overrides.get(sensor, spec.get("topic", "/" + sensor))

    def _frame_for(self, sensor, spec):
        return self.frame_overrides.get(sensor, spec.get("frame_id", sensor))

    def _prepare_publishers(self):
        if bool(self.publish_flags.get("clock", True)):
            self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)
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

    def run(self):
        if not self.play_events:
            rospy.logwarn("no events selected for playback")
            return
        while not rospy.is_shutdown():
            previous_stamp = None
            for event in self.play_events:
                if rospy.is_shutdown():
                    return
                if previous_stamp is not None:
                    delay = (event["timestamp_ns"] - previous_stamp) * 1e-9 / max(self.play_rate, 1e-9)
                    if delay > 0:
                        rospy.sleep(delay)
                self._publish_clock(event["timestamp_ns"])
                self._publish_event(event)
                previous_stamp = event["timestamp_ns"]
            if not self.loop:
                return

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
                self.publishers[sensor].publish(reader(path, event["timestamp_ns"], self._frame_for(sensor, spec)))
            elif spec["kind"] == "image":
                self.publishers[sensor].publish(self._load_image(path, event["timestamp_ns"], self._frame_for(sensor, spec)))
            elif spec["kind"] == "livox_custom":
                self.publishers[sensor].publish(_helipr_livox_avia(path, event["timestamp_ns"], self._frame_for(sensor, spec)))
            elif spec["format"] == "mulran_gps":
                msg = self._gps_msg(event["timestamp_ns"], self.csv_cache[sensor].get(event["timestamp_ns"]), self._frame_for(sensor, spec))
                if msg:
                    self.publishers[sensor].publish(msg)
            elif spec["format"] == "xsens_imu":
                imu_msg, mag_msg = self._imu_msg(event["timestamp_ns"], self.csv_cache[sensor].get(event["timestamp_ns"]), self._frame_for(sensor, spec))
                if imu_msg:
                    self.publishers[sensor].publish(imu_msg)
                if mag_msg and "imu_mag" in self.publishers:
                    self.publishers["imu_mag"].publish(mag_msg)
            elif spec["format"] == "novatel_inspva":
                msg = self._inspva_msg(event["timestamp_ns"], self.csv_cache[sensor].get(event["timestamp_ns"]), self._frame_for(sensor, spec))
                if msg:
                    self.publishers[sensor].publish(msg)
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

