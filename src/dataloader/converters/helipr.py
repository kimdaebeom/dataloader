"""HeLiPR raw-dataset converter."""

from .base import BaseConverter
from .utils import read_labeled_timeline, tum_rows_from_file, write_tum_rows


class HeLiPRConverter(BaseConverter):
    name = "helipr"

    def check_source(self, source):
        timeline = source / self.definition["timeline_file"]
        if not timeline.is_file():
            raise FileNotFoundError("source timeline not found: {}".format(timeline))

    def read_timeline(self, source):
        definition = self.definition
        return read_labeled_timeline(
            source / definition["timeline_file"],
            definition["label_to_sensor"],
        )

    def convert_poses(self, source, sequence_dir, present_sensors):
        lidar_gt_dir = source / "LiDAR_GT"
        sensor_to_file = {
            "ouster": "Ouster_gt.txt",
            "velodyne": "Velodyne_gt.txt",
            "livox_avia": "Avia_gt.txt",
            "aeva": "Aeva_gt.txt",
        }
        gt = {}
        gt_global = {}
        for sensor, filename in sensor_to_file.items():
            if sensor not in present_sensors:
                continue
            src = lidar_gt_dir / filename
            if src.is_file():
                out = sequence_dir / "poses" / "gt_{}.txt".format(sensor)
                if write_tum_rows(tum_rows_from_file(src), out):
                    gt[sensor] = str(out.relative_to(sequence_dir))
            global_src = lidar_gt_dir / "global_{}".format(filename)
            if global_src.is_file():
                out = sequence_dir / "poses" / "gt_global_{}.txt".format(sensor)
                if write_tum_rows(tum_rows_from_file(global_src), out):
                    gt_global[sensor] = str(out.relative_to(sequence_dir))
        if "ouster" in gt:
            gt["default"] = gt["ouster"]
        if "ouster" in gt_global:
            gt_global["default"] = gt_global["ouster"]
        poses = {}
        if gt:
            poses["gt"] = gt
        if gt_global:
            poses["gt_global"] = gt_global
        return poses
