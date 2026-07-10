#!/usr/bin/env python3

import argparse
import csv
import os
import shutil
import sys
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

from dataloader.common import FORMAT_VERSION, dataset_definition
from dataloader.pose_utils import matrix_to_quat, timestamp_to_ns


def _read_source_timeline(path, label_to_sensor):
    rows = []
    with path.open("r", newline="") as handle:
        reader = csv.reader(handle)
        for raw in reader:
            if len(raw) < 2:
                continue
            try:
                stamp = int(raw[0])
            except ValueError:
                continue
            label = raw[1].strip()
            sensor = label_to_sensor.get(label)
            if sensor is None:
                continue
            rows.append({"timestamp_ns": stamp, "sensor": sensor, "source_label": label})
    rows.sort(key=lambda item: (item["timestamp_ns"], item["sensor"]))
    return rows


def _read_semantic_kitti_timeline(source):
    pose_path = source / "odom_tum.txt"
    pcd_dir = source / "pcd"
    if not pose_path.is_file():
        raise FileNotFoundError("semantic_kitti pose file not found: {}".format(pose_path))
    if not pcd_dir.is_dir():
        raise FileNotFoundError("semantic_kitti pcd directory not found: {}".format(pcd_dir))

    pose_rows = _tum_rows_from_tum_file(pose_path)
    pcd_files = sorted(pcd_dir.glob("*.bin"))
    if len(pose_rows) != len(pcd_files):
        raise RuntimeError(
            "semantic_kitti pose/pcd count mismatch: {} poses, {} pcd files".format(
                len(pose_rows), len(pcd_files)
            )
        )

    rows = []
    for pose_row, pcd_file in zip(pose_rows, pcd_files):
        rows.append(
            {
                "timestamp_ns": pose_row[0],
                "sensor": "velodyne",
                "source_label": "velodyne",
                "source_filename": pcd_file.name,
            }
        )
    rows.sort(key=lambda item: (item["timestamp_ns"], item["source_filename"]))
    return rows


def _ensure_parent(path):
    path.parent.mkdir(parents=True, exist_ok=True)


def _log(verbose, message):
    if verbose:
        print(message, flush=True)


class _ProgressPrinter:
    def __init__(self, label, total, verbose):
        self.label = label
        self.total = max(0, int(total))
        self.verbose = verbose
        self.last_percent = -1
        self.last_print_time = 0.0
        self.is_tty = sys.stdout.isatty()

    def update(self, current, detail=None, force=False):
        if not self.verbose or self.total == 0:
            return
        current = min(max(0, int(current)), self.total)
        percent = int((current * 100.0) / self.total)
        now = time.time()
        should_print = (
            force
            or current == self.total
            or percent != self.last_percent
            or now - self.last_print_time > 5.0
        )
        if not should_print:
            return
        self.last_percent = percent
        self.last_print_time = now
        detail_text = " | {}".format(detail) if detail else ""
        message = "{} {:3d}% ({}/{}){}".format(self.label, percent, current, self.total, detail_text)
        if self.is_tty:
            sys.stdout.write("\r{}".format(message))
            if current == self.total:
                sys.stdout.write("\n")
            sys.stdout.flush()
        elif force or current == self.total or percent % 5 == 0:
            print(message, flush=True)


def _place_file(src, dst, mode):
    if not src.is_file():
        return False
    if mode == "reference":
        return True
    _ensure_parent(dst)
    if dst.exists() or dst.is_symlink():
        return True
    if mode == "symlink":
        os.symlink(src, dst)
        return True
    if mode == "hardlink":
        os.link(src, dst)
        return True
    if mode == "hardlink_or_copy":
        try:
            os.link(src, dst)
            return True
        except OSError:
            shutil.copy2(src, dst)
            return True
    shutil.copy2(src, dst)
    return True


def _source_dir(source, spec):
    raw_dirs = spec.get("raw_dirs")
    if raw_dirs is None:
        raw_dirs = [spec["raw_dir"]]
    for raw_dir in raw_dirs:
        candidate = source / raw_dir
        if candidate.is_dir():
            return candidate
    return source / raw_dirs[0]


def _event_rel_path(definition, sensor, stamp, source_dir=None, link_mode="copy", source_filename=None):
    spec = definition["sensors"][sensor]
    if "out_file" in spec:
        if link_mode == "reference":
            return None
        return spec["out_file"]
    filename = source_filename or "{}{}".format(stamp, spec["suffix"])
    if link_mode == "reference":
        return str((source_dir / filename).resolve())
    return str(Path(spec["out_dir"]) / filename)


def _slice_by_lidar_frames(rows, primary_lidar, start_lidar_frame=None, end_lidar_frame=None):
    if start_lidar_frame is None and end_lidar_frame is None:
        return rows
    lidar_rows = [row for row in rows if row["sensor"] == primary_lidar]
    if not lidar_rows:
        return []
    start_index = 0 if start_lidar_frame is None else max(0, start_lidar_frame)
    end_index = len(lidar_rows) - 1 if end_lidar_frame is None or end_lidar_frame < 0 else end_lidar_frame
    end_index = min(end_index, len(lidar_rows) - 1)
    if start_index > end_index:
        return []
    selected_lidar_ids = {id(row) for row in lidar_rows[start_index : end_index + 1]}
    start_stamp = lidar_rows[start_index]["timestamp_ns"]
    end_stamp = lidar_rows[end_index]["timestamp_ns"]
    return [
        row
        for row in rows
        if id(row) in selected_lidar_ids
        or (row["sensor"] != primary_lidar and start_stamp <= row["timestamp_ns"] <= end_stamp)
    ]


def _write_tum_rows(rows, path):
    if not rows:
        return False
    _ensure_parent(path)
    with path.open("w") as handle:
        for stamp_ns, tx, ty, tz, qx, qy, qz, qw in rows:
            handle.write(
                "{} {:.12g} {:.12g} {:.12g} {:.12g} {:.12g} {:.12g} {:.12g}\n".format(
                    int(stamp_ns), tx, ty, tz, qx, qy, qz, qw
                )
            )
    return True


def _tum_rows_from_tum_file(path):
    rows = []
    with path.open("r", errors="replace") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 8:
                continue
            stamp_ns = timestamp_to_ns(parts[0])
            values = [float(value) for value in parts[1:8]]
            rows.append((stamp_ns, *values))
    return rows


def _tum_rows_from_global_pose_csv(path):
    rows = []
    with path.open("r", newline="", errors="replace") as handle:
        reader = csv.reader(handle)
        for raw in reader:
            if len(raw) < 13:
                continue
            try:
                stamp_ns = timestamp_to_ns(raw[0])
                values = [float(value) for value in raw[1:13]]
            except ValueError:
                continue
            transform = np.eye(4, dtype=np.float64)
            transform[0, :4] = values[0:4]
            transform[1, :4] = values[4:8]
            transform[2, :4] = values[8:12]
            qx, qy, qz, qw = matrix_to_quat(transform)
            tx, ty, tz = transform[:3, 3]
            rows.append((stamp_ns, tx, ty, tz, qx, qy, qz, qw))
    return rows


def _count_pose_files(poses):
    files = set()
    for group in poses.values():
        for value in group.values():
            files.add(value)
    return len(files)


def _convert_gt_poses(dataset, source, sequence_dir, present_sensors):
    gt = {}
    gt_global = {}
    if dataset == "mulran":
        global_pose = source / "global_pose.csv"
        if global_pose.is_file():
            out = sequence_dir / "poses" / "gt.txt"
            if _write_tum_rows(_tum_rows_from_global_pose_csv(global_pose), out):
                gt["default"] = str(out.relative_to(sequence_dir))
                gt_global["default"] = str(out.relative_to(sequence_dir))
                if "ouster" in present_sensors:
                    gt["ouster"] = str(out.relative_to(sequence_dir))
                    gt_global["ouster"] = str(out.relative_to(sequence_dir))
    elif dataset == "helipr":
        lidar_gt_dir = source / "LiDAR_GT"
        sensor_to_file = {
            "ouster": "Ouster_gt.txt",
            "velodyne": "Velodyne_gt.txt",
            "livox_avia": "Avia_gt.txt",
            "aeva": "Aeva_gt.txt",
        }
        for sensor, filename in sensor_to_file.items():
            if sensor not in present_sensors:
                continue
            src = lidar_gt_dir / filename
            if not src.is_file():
                continue
            out = sequence_dir / "poses" / "gt_{}.txt".format(sensor)
            if _write_tum_rows(_tum_rows_from_tum_file(src), out):
                gt[sensor] = str(out.relative_to(sequence_dir))
            global_src = lidar_gt_dir / "global_{}".format(filename)
            if global_src.is_file():
                global_out = sequence_dir / "poses" / "gt_global_{}.txt".format(sensor)
                if _write_tum_rows(_tum_rows_from_tum_file(global_src), global_out):
                    gt_global[sensor] = str(global_out.relative_to(sequence_dir))
        if "ouster" in gt:
            gt["default"] = gt["ouster"]
        if "ouster" in gt_global:
            gt_global["default"] = gt_global["ouster"]
    elif dataset == "semantic_kitti":
        pose_path = source / "odom_tum.txt"
        if pose_path.is_file():
            out = sequence_dir / "poses" / "gt.txt"
            if _write_tum_rows(_tum_rows_from_tum_file(pose_path), out):
                gt["default"] = str(out.relative_to(sequence_dir))
                gt_global["default"] = str(out.relative_to(sequence_dir))
                if "velodyne" in present_sensors:
                    gt["velodyne"] = str(out.relative_to(sequence_dir))
                    gt_global["velodyne"] = str(out.relative_to(sequence_dir))
    poses = {}
    if gt:
        poses["gt"] = gt
    if gt_global:
        poses["gt_global"] = gt_global
    return poses


def convert_dataset(
    dataset,
    source,
    output_root,
    sequence=None,
    link_mode="copy",
    overwrite=False,
    start_lidar_frame=None,
    end_lidar_frame=None,
    verbose=False,
):
    definition = dataset_definition(dataset)
    source = Path(source).expanduser().resolve()
    if sequence is None:
        sequence = source.name
    sequence_dir = Path(output_root).expanduser().resolve() / dataset / sequence

    _log(verbose, "")
    _log(verbose, "========================================")
    _log(verbose, "dataloader convert")
    _log(verbose, "dataset      : {}".format(dataset))
    _log(verbose, "sequence     : {}".format(sequence))
    _log(verbose, "source       : {}".format(source))
    _log(verbose, "output       : {}".format(sequence_dir))
    _log(verbose, "storage mode : {}".format(link_mode))
    if start_lidar_frame is not None or end_lidar_frame is not None:
        _log(
            verbose,
            "lidar range  : {} ~ {}".format(
                "start" if start_lidar_frame is None else start_lidar_frame,
                "end" if end_lidar_frame is None else end_lidar_frame,
            ),
        )
    _log(verbose, "========================================")

    if dataset == "semantic_kitti":
        timeline_path = None
    else:
        timeline_path = source / definition["timeline_file"]
        if not timeline_path.is_file():
            raise FileNotFoundError("source timeline not found: {}".format(timeline_path))

    if sequence_dir.exists():
        if not overwrite:
            raise FileExistsError("{} already exists; pass --overwrite to replace it".format(sequence_dir))
        _log(verbose, "[1/6] remove existing output: {}".format(sequence_dir))
        shutil.rmtree(sequence_dir)
    else:
        _log(verbose, "[1/6] prepare output directory")
    sequence_dir.mkdir(parents=True, exist_ok=True)

    _log(verbose, "[2/6] read source timeline")
    if dataset == "semantic_kitti":
        source_rows = _read_semantic_kitti_timeline(source)
    else:
        source_rows = _read_source_timeline(timeline_path, definition["label_to_sensor"])
    raw_event_count = len(source_rows)
    source_rows = _slice_by_lidar_frames(source_rows, definition["primary_lidar"], start_lidar_frame, end_lidar_frame)
    _log(verbose, "      raw events      : {}".format(raw_event_count))
    _log(verbose, "      selected events : {}".format(len(source_rows)))

    _log(verbose, "[3/6] detect source sensor directories")
    source_sensors = sorted({row["sensor"] for row in source_rows})
    source_dirs = {}
    for sensor in source_sensors:
        spec = definition["sensors"][sensor]
        if "raw_dir" not in spec and "raw_dirs" not in spec:
            continue
        candidate = _source_dir(source, spec)
        if candidate.is_dir():
            source_dirs[sensor] = candidate
            _log(verbose, "      {} -> {}".format(sensor, candidate))
        else:
            _log(verbose, "      {} -> missing directory".format(sensor))

    missing_files = []
    missing_by_sensor = Counter()
    skipped_by_sensor = Counter()
    placed_files = set()
    rows = []
    _log(verbose, "[4/6] place sensor files")
    progress = _ProgressPrinter("      progress", len(source_rows), verbose)
    progress.update(0, force=True)
    for index, row in enumerate(source_rows, start=1):
        sensor = row["sensor"]
        spec = definition["sensors"][sensor]
        if "raw_file" in spec:
            src = source / spec["raw_file"]
            if not src.is_file():
                missing_files.append(str(src))
                missing_by_sensor[sensor] += 1
                progress.update(index, detail=sensor)
                continue
        else:
            if sensor not in source_dirs:
                skipped_by_sensor[sensor] += 1
                progress.update(index, detail=sensor)
                continue
            filename = row.get("source_filename") or "{}{}".format(row["timestamp_ns"], spec["suffix"])
            src = source_dirs[sensor] / filename
            if not src.is_file():
                missing_files.append(str(src))
                missing_by_sensor[sensor] += 1
                progress.update(index, detail=sensor)
                continue

        row["relative_path"] = _event_rel_path(
            definition,
            sensor,
            row["timestamp_ns"],
            source_dir=source_dirs.get(sensor),
            link_mode=link_mode,
            source_filename=row.get("source_filename"),
        )
        if row["relative_path"] is None:
            row["relative_path"] = str((source / spec["raw_file"]).resolve())
        dst = sequence_dir / row["relative_path"]
        if _place_file(src, dst, link_mode):
            placed_files.add(str(dst))
        else:
            missing_files.append(str(src))
            missing_by_sensor[sensor] += 1
            progress.update(index, detail=sensor)
            continue
        rows.append(row)
        progress.update(index, detail=sensor)

    present_sensors = sorted({row["sensor"] for row in rows})
    _log(verbose, "[5/6] convert GT pose files")
    poses = _convert_gt_poses(dataset, source, sequence_dir, present_sensors)
    pose_file_count = _count_pose_files(poses)
    _log(verbose, "      pose files: {}".format(pose_file_count))
    for sensor in present_sensors:
        spec = definition["sensors"][sensor]
        if "raw_file" not in spec:
            continue
        src = source / spec["raw_file"]
        dst = sequence_dir / spec["out_file"]
        if _place_file(src, dst, link_mode):
            placed_files.add(str(dst))
        else:
            missing_files.append(str(src))
            missing_by_sensor[sensor] += 1

    _log(verbose, "[6/6] write timeline and manifest")
    timeline_out = sequence_dir / "timeline.csv"
    with timeline_out.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["timestamp_ns", "sensor", "relative_path"])
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "timestamp_ns": row["timestamp_ns"],
                    "sensor": row["sensor"],
                    "relative_path": row["relative_path"],
                }
            )

    sensors = {}
    for name, spec in definition["sensors"].items():
        if name not in present_sensors:
            continue
        sensor_spec = {
            key: value
            for key, value in spec.items()
            if key
            in {
                "kind",
                "format",
                "out_dir",
                "out_file",
                "raw_dirs",
                "suffix",
                "topic",
                "mag_topic",
                "frame_id",
            }
        }
        if link_mode == "reference" and "raw_file" in spec:
            sensor_spec["out_file"] = str((source / spec["raw_file"]).resolve())
        sensors[name] = sensor_spec

    primary_lidar = definition["primary_lidar"]
    lidar_count = sum(1 for row in rows if row["sensor"] == primary_lidar)
    manifest = {
        "format_version": FORMAT_VERSION,
        "dataset": dataset,
        "sequence": sequence,
        "source_path": str(source),
        "storage_mode": link_mode,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "time_unit": "nanoseconds",
        "timeline": "timeline.csv",
        "primary_lidar": primary_lidar,
        "primary_lidar_frames": lidar_count,
        "sensors": sensors,
    }
    if poses:
        manifest["poses"] = poses
    with (sequence_dir / "manifest.yaml").open("w") as handle:
        yaml.safe_dump(manifest, handle, sort_keys=False)

    if missing_files:
        missing_path = sequence_dir / "missing_files.txt"
        with missing_path.open("w") as handle:
            handle.write("\n".join(missing_files))
            handle.write("\n")

    return {
        "sequence_dir": sequence_dir,
        "timeline_path": timeline_out,
        "manifest_path": sequence_dir / "manifest.yaml",
        "dataset": dataset,
        "sequence": sequence,
        "storage_mode": link_mode,
        "events": len(rows),
        "sensors": present_sensors,
        "sensor_event_counts": dict(Counter(row["sensor"] for row in rows)),
        "primary_lidar": primary_lidar,
        "primary_lidar_frames": lidar_count,
        "placed_files": len(placed_files),
        "pose_files": pose_file_count,
        "missing_files": missing_files,
        "missing_by_sensor": dict(missing_by_sensor),
        "skipped_by_sensor": dict(skipped_by_sensor),
    }


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Convert raw datasets to the dataloader layout.")
    parser.add_argument("--dataset", required=True, choices=sorted(["mulran", "helipr", "semantic_kitti"]))
    parser.add_argument("--source", required=True, help="Raw sequence directory.")
    parser.add_argument("--output", required=True, help="Converted dataset root.")
    parser.add_argument("--sequence", default=None, help="Sequence name. Defaults to source directory name.")
    parser.add_argument(
        "--link-mode",
        default="copy",
        choices=["reference", "symlink", "hardlink", "hardlink_or_copy", "copy"],
        help="How to place large sensor files. copy is the default and creates a self-contained converted dataset.",
    )
    parser.add_argument("--overwrite", action="store_true", help="Replace an existing converted sequence.")
    parser.add_argument("--start-lidar-frame", type=int, default=None, help="Optional 0-based primary LiDAR start frame for partial conversion.")
    parser.add_argument("--end-lidar-frame", type=int, default=None, help="Optional 0-based primary LiDAR end frame for partial conversion.")
    return parser


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    result = convert_dataset(
        dataset=args.dataset,
        source=args.source,
        output_root=args.output,
        sequence=args.sequence,
        link_mode=args.link_mode,
        overwrite=args.overwrite,
        start_lidar_frame=args.start_lidar_frame,
        end_lidar_frame=args.end_lidar_frame,
        verbose=True,
    )
    print("")
    print("========================================")
    print("convert summary")
    print("dataset              : {}".format(result["dataset"]))
    print("sequence             : {}".format(result["sequence"]))
    print("output               : {}".format(result["sequence_dir"]))
    print("storage mode         : {}".format(result["storage_mode"]))
    print("timeline             : {}".format(result["timeline_path"]))
    print("manifest             : {}".format(result["manifest_path"]))
    print("events               : {}".format(result["events"]))
    print("primary lidar        : {}".format(result["primary_lidar"]))
    print("primary lidar frames : {}".format(result["primary_lidar_frames"]))
    print("unique sensor files  : {}".format(result["placed_files"]))
    print("pose files           : {}".format(result["pose_files"]))
    print("sensors              : {}".format(", ".join(result["sensors"])))
    if result["sensor_event_counts"]:
        print("")
        print("events by sensor")
        for sensor in sorted(result["sensor_event_counts"]):
            print("  - {:<12} {}".format(sensor, result["sensor_event_counts"][sensor]))
    if result["skipped_by_sensor"]:
        print("")
        print("skipped events by sensor")
        for sensor in sorted(result["skipped_by_sensor"]):
            print("  - {:<12} {}".format(sensor, result["skipped_by_sensor"][sensor]))
    if result["missing_files"]:
        print("")
        print("missing files       : {} (see missing_files.txt)".format(len(result["missing_files"])))
        if result["missing_by_sensor"]:
            print("missing by sensor")
            for sensor in sorted(result["missing_by_sensor"]):
                print("  - {:<12} {}".format(sensor, result["missing_by_sensor"][sensor]))
        return 2
    print("status               : OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
