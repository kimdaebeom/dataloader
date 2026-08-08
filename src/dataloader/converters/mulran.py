"""MulRan raw-dataset converter."""

from .base import BaseConverter
from .utils import read_labeled_timeline, tum_rows_from_global_pose_csv, write_tum_rows


class MulRanConverter(BaseConverter):
    name = "mulran"

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
        global_pose = source / "global_pose.csv"
        if not global_pose.is_file():
            return {}
        out = sequence_dir / "poses" / "gt.txt"
        if not write_tum_rows(tum_rows_from_global_pose_csv(global_pose), out):
            return {}
        relative = str(out.relative_to(sequence_dir))
        gt = {"default": relative}
        gt_global = {"default": relative}
        if "ouster" in present_sensors:
            gt["ouster"] = relative
            gt_global["ouster"] = relative
        return {"gt": gt, "gt_global": gt_global}
