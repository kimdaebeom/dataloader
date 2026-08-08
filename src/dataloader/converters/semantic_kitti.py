"""SemanticKITTI normalized-source converter."""

from .base import BaseConverter
from .utils import tum_rows_from_file, write_tum_rows


class SemanticKittiConverter(BaseConverter):
    name = "semantic_kitti"

    def check_source(self, source):
        pose_path = source / "odom_tum.txt"
        pcd_dir = source / "pcd"
        if not pose_path.is_file():
            raise FileNotFoundError(
                "semantic_kitti pose file not found: {}".format(pose_path)
            )
        if not pcd_dir.is_dir():
            raise FileNotFoundError(
                "semantic_kitti pcd directory not found: {}".format(pcd_dir)
            )

    def read_timeline(self, source):
        pose_rows = tum_rows_from_file(source / "odom_tum.txt")
        pcd_files = sorted((source / "pcd").glob("*.bin"))
        if len(pose_rows) != len(pcd_files):
            raise RuntimeError(
                "semantic_kitti pose/pcd count mismatch: {} poses, {} pcd files".format(
                    len(pose_rows), len(pcd_files)
                )
            )
        rows = [
            {
                "timestamp_ns": pose_row[0],
                "sensor": "velodyne",
                "source_label": "velodyne",
                "source_filename": pcd_file.name,
            }
            # ``zip(strict=True)`` is unavailable on the supported Python 3.8.
            for pose_row, pcd_file in zip(pose_rows, pcd_files)  # noqa: B905
        ]
        rows.sort(key=lambda item: (item["timestamp_ns"], item["source_filename"]))
        return rows

    def convert_poses(self, source, sequence_dir, present_sensors):
        pose_path = source / "odom_tum.txt"
        if not pose_path.is_file():
            return {}
        out = sequence_dir / "poses" / "gt.txt"
        if not write_tum_rows(tum_rows_from_file(pose_path), out):
            return {}
        relative = str(out.relative_to(sequence_dir))
        gt = {"default": relative}
        gt_global = {"default": relative}
        if "velodyne" in present_sensors:
            gt["velodyne"] = relative
            gt_global["velodyne"] = relative
        return {"gt": gt, "gt_global": gt_global}
