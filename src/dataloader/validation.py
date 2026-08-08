"""Validation for converted dataloader sequences."""

import argparse
import csv
import json
import os
from collections import Counter
from dataclasses import asdict, dataclass, field
from pathlib import Path

import yaml

from .common import FORMAT_VERSION
from .lidar import UnsupportedPointCloudFormat, point_step
from .pose_utils import timestamp_to_ns


@dataclass(frozen=True)
class ValidationIssue:
    level: str
    code: str
    message: str


@dataclass
class ValidationReport:
    path: Path
    issues: list = field(default_factory=list)
    stats: dict = field(default_factory=dict)

    @property
    def errors(self):
        return tuple(issue for issue in self.issues if issue.level == "error")

    @property
    def warnings(self):
        return tuple(issue for issue in self.issues if issue.level == "warning")

    @property
    def ok(self):
        return not self.errors

    def add(self, level, code, message):
        self.issues.append(ValidationIssue(level, code, message))

    def to_dict(self):
        return {
            "path": str(self.path),
            "ok": self.ok,
            "errors": len(self.errors),
            "warnings": len(self.warnings),
            "issues": [asdict(issue) for issue in self.issues],
            "stats": self.stats,
        }


def _validate_pose_file(path, report):
    count = 0
    previous = None
    try:
        with path.open("r") as handle:
            for line_number, line in enumerate(handle, start=1):
                line = line.split("#", 1)[0].strip()
                if not line:
                    continue
                parts = line.replace(",", " ").split()
                if len(parts) < 8:
                    report.add(
                        "error",
                        "invalid_pose_row",
                        "{}:{} has fewer than 8 values".format(path, line_number),
                    )
                    continue
                try:
                    stamp = timestamp_to_ns(parts[0])
                    [float(value) for value in parts[1:8]]
                except ValueError:
                    report.add(
                        "error",
                        "invalid_pose_value",
                        "{}:{} contains a non-numeric value".format(path, line_number),
                    )
                    continue
                if previous is not None and stamp < previous:
                    report.add(
                        "error",
                        "unsorted_pose",
                        "{}:{} timestamp is out of order".format(path, line_number),
                    )
                previous = stamp
                count += 1
    except OSError as exc:
        report.add("error", "pose_read_failed", "{}: {}".format(path, exc))
    return count


def validate_dataset(sequence_dir):
    """Validate one converted sequence without loading point-cloud payloads."""
    root = Path(sequence_dir).expanduser().resolve()
    report = ValidationReport(root)
    manifest_path = root / "manifest.yaml"
    if not root.is_dir():
        report.add("error", "directory_missing", "directory not found: {}".format(root))
        return report
    if not manifest_path.is_file():
        report.add("error", "manifest_missing", "manifest.yaml not found")
        return report

    try:
        with manifest_path.open("r") as handle:
            manifest = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError) as exc:
        report.add("error", "manifest_invalid", str(exc))
        return report
    if not isinstance(manifest, dict):
        report.add("error", "manifest_invalid", "manifest must contain a mapping")
        return report

    required = ("format_version", "dataset", "sequence", "timeline", "sensors")
    for name in required:
        if name not in manifest:
            report.add("error", "manifest_key_missing", "missing manifest key: {}".format(name))
    if manifest.get("format_version") != FORMAT_VERSION:
        report.add(
            "error",
            "unsupported_format_version",
            "format_version {} is not supported; expected {}".format(
                manifest.get("format_version"), FORMAT_VERSION
            ),
        )
    sensors = manifest.get("sensors", {})
    if not isinstance(sensors, dict):
        report.add("error", "invalid_sensors", "manifest sensors must be a mapping")
        sensors = {}

    timeline_name = Path(manifest.get("timeline", "timeline.csv"))
    if timeline_name.is_absolute():
        report.add("error", "timeline_path", "timeline path must be relative")
        return report
    timeline_path = (root / timeline_name).resolve()
    if timeline_path != root and root not in timeline_path.parents:
        report.add(
            "error",
            "timeline_path",
            "timeline path escapes the sequence directory",
        )
        return report
    if not timeline_path.is_file():
        report.add("error", "timeline_missing", "timeline not found: {}".format(timeline_path))
        return report

    storage_mode = manifest.get("storage_mode", "")
    previous_key = None
    event_counts = Counter()
    unique_paths = set()
    checked_point_files = set()
    start_stamp = None
    end_stamp = None
    try:
        with timeline_path.open("r", newline="") as handle:
            reader = csv.DictReader(handle)
            required_columns = {"timestamp_ns", "sensor", "relative_path"}
            if not required_columns <= set(reader.fieldnames or ()):
                report.add(
                    "error",
                    "timeline_columns",
                    "timeline must contain: {}".format(", ".join(sorted(required_columns))),
                )
                return report
            for line_number, row in enumerate(reader, start=2):
                try:
                    stamp = int(row["timestamp_ns"])
                except (TypeError, ValueError):
                    report.add(
                        "error",
                        "invalid_timestamp",
                        "timeline line {} has an invalid timestamp".format(line_number),
                    )
                    continue
                sensor = row.get("sensor", "")
                relative = row.get("relative_path", "")
                if sensor not in sensors:
                    report.add(
                        "error",
                        "unknown_sensor",
                        "timeline line {} references '{}'".format(line_number, sensor),
                    )
                key = (stamp, sensor)
                if previous_key is not None and key < previous_key:
                    report.add(
                        "error",
                        "unsorted_timeline",
                        "timeline line {} is out of order".format(line_number),
                    )
                previous_key = key
                start_stamp = stamp if start_stamp is None else min(start_stamp, stamp)
                end_stamp = stamp if end_stamp is None else max(end_stamp, stamp)
                event_counts[sensor] += 1

                if not relative:
                    report.add(
                        "error",
                        "empty_path",
                        "timeline line {} has an empty path".format(line_number),
                    )
                    continue
                raw_path = Path(relative)
                if raw_path.is_absolute():
                    path = raw_path
                    if storage_mode != "reference":
                        report.add(
                            "error",
                            "unexpected_absolute_path",
                            "absolute timeline path in '{}' storage: {}".format(
                                storage_mode, relative
                            ),
                        )
                else:
                    path = Path(os.path.abspath(str(root / raw_path)))
                    if path != root and root not in path.parents:
                        report.add(
                            "error",
                            "path_escape",
                            "timeline path escapes sequence directory: {}".format(relative),
                        )
                        continue
                unique_paths.add(str(path))
                if not path.is_file():
                    report.add("error", "file_missing", "file not found: {}".format(path))
                    continue
                spec = sensors.get(sensor, {})
                if (
                    spec.get("kind") in ("pointcloud", "livox_custom")
                    and str(path) not in checked_point_files
                ):
                    checked_point_files.add(str(path))
                    format_name = spec.get("format", "")
                    try:
                        step = point_step(format_name, stamp)
                    except (UnsupportedPointCloudFormat, ValueError) as exc:
                        report.add("error", "unsupported_point_format", str(exc))
                    else:
                        size = path.stat().st_size
                        if size % step:
                            report.add(
                                "error",
                                "invalid_point_file_size",
                                "{} size {} is not divisible by {}".format(path, size, step),
                            )
    except (OSError, csv.Error) as exc:
        report.add("error", "timeline_read_failed", str(exc))
        return report

    if not event_counts:
        report.add("error", "timeline_empty", "timeline has no events")
    if (root / "missing_files.txt").exists():
        report.add(
            "error",
            "missing_file_report",
            "missing_files.txt exists; conversion was incomplete",
        )

    primary = manifest.get("primary_lidar")
    if primary and primary not in sensors:
        report.add(
            "error",
            "primary_lidar_missing",
            "primary_lidar '{}' is not in sensors".format(primary),
        )
    declared_frames = manifest.get("primary_lidar_frames")
    if primary and declared_frames is not None and event_counts[primary] != declared_frames:
        report.add(
            "error",
            "primary_lidar_count",
            "manifest declares {} frames but timeline contains {}".format(
                declared_frames, event_counts[primary]
            ),
        )

    pose_files = set()
    pose_rows = 0
    for group in manifest.get("poses", {}).values():
        if not isinstance(group, dict):
            report.add("error", "invalid_poses", "pose groups must be mappings")
            continue
        for relative in group.values():
            path = Path(relative)
            if path.is_absolute():
                report.add(
                    "error",
                    "pose_path",
                    "pose path must be relative: {}".format(path),
                )
                continue
            path = (root / path).resolve()
            if path != root and root not in path.parents:
                report.add(
                    "error",
                    "pose_path",
                    "pose path escapes the sequence directory: {}".format(relative),
                )
                continue
            if path in pose_files:
                continue
            pose_files.add(path)
            if not path.is_file():
                report.add("error", "pose_missing", "pose file not found: {}".format(path))
            else:
                pose_rows += _validate_pose_file(path, report)

    report.stats = {
        "dataset": manifest.get("dataset", ""),
        "sequence": manifest.get("sequence", ""),
        "storage_mode": storage_mode,
        "events": sum(event_counts.values()),
        "event_counts": dict(sorted(event_counts.items())),
        "unique_files": len(unique_paths),
        "pose_files": len(pose_files),
        "pose_rows": pose_rows,
        "duration_ns": 0 if start_stamp is None else end_stamp - start_stamp,
    }
    return report


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Validate a converted dataset sequence.")
    parser.add_argument("sequence_dir")
    parser.add_argument("--json", action="store_true", help="Print a machine-readable report.")
    return parser


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    report = validate_dataset(args.sequence_dir)
    if args.json:
        print(json.dumps(report.to_dict(), indent=2, sort_keys=True))
    else:
        status = "OK" if report.ok else "FAILED"
        print("validation {}: {}".format(status, report.path))
        print(
            "events: {} | unique files: {} | errors: {} | warnings: {}".format(
                report.stats.get("events", 0),
                report.stats.get("unique_files", 0),
                len(report.errors),
                len(report.warnings),
            )
        )
        for issue in report.issues:
            print("{} [{}] {}".format(issue.level.upper(), issue.code, issue.message))
    return 0 if report.ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
