#!/usr/bin/env python3

import argparse
import csv
import os
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml

from dataloader.common import FORMAT_VERSION, dataset_definition


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


def _ensure_parent(path):
    path.parent.mkdir(parents=True, exist_ok=True)


def _place_file(src, dst, mode):
    if not src.is_file():
        return False
    _ensure_parent(dst)
    if dst.exists() or dst.is_symlink():
        return True
    if mode == "symlink":
        os.symlink(src, dst)
        return True
    if mode == "hardlink":
        try:
            os.link(src, dst)
            return True
        except OSError:
            shutil.copy2(src, dst)
            return True
    shutil.copy2(src, dst)
    return True


def _event_rel_path(definition, sensor, stamp):
    spec = definition["sensors"][sensor]
    if "out_file" in spec:
        return spec["out_file"]
    return str(Path(spec["out_dir"]) / "{}{}".format(stamp, spec["suffix"]))


def convert_dataset(dataset, source, output_root, sequence=None, link_mode="hardlink", overwrite=False):
    definition = dataset_definition(dataset)
    source = Path(source).expanduser().resolve()
    if sequence is None:
        sequence = source.name
    sequence_dir = Path(output_root).expanduser().resolve() / dataset / sequence

    timeline_path = source / definition["timeline_file"]
    if not timeline_path.is_file():
        raise FileNotFoundError("source timeline not found: {}".format(timeline_path))

    if sequence_dir.exists():
        if not overwrite:
            raise FileExistsError("{} already exists; pass --overwrite to replace it".format(sequence_dir))
        shutil.rmtree(sequence_dir)
    sequence_dir.mkdir(parents=True, exist_ok=True)

    rows = _read_source_timeline(timeline_path, definition["label_to_sensor"])
    present_sensors = sorted({row["sensor"] for row in rows})

    missing_files = []
    linked_files = 0
    for sensor in present_sensors:
        spec = definition["sensors"][sensor]
        if "raw_file" in spec:
            src = source / spec["raw_file"]
            dst = sequence_dir / spec["out_file"]
            if _place_file(src, dst, link_mode):
                linked_files += 1
            else:
                missing_files.append(str(src))

    for row in rows:
        sensor = row["sensor"]
        spec = definition["sensors"][sensor]
        row["relative_path"] = _event_rel_path(definition, sensor, row["timestamp_ns"])
        if "raw_dir" not in spec:
            continue
        src = source / spec["raw_dir"] / "{}{}".format(row["timestamp_ns"], spec["suffix"])
        dst = sequence_dir / row["relative_path"]
        if _place_file(src, dst, link_mode):
            linked_files += 1
        else:
            missing_files.append(str(src))

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
        sensors[name] = {
            key: value
            for key, value in spec.items()
            if key
            in {
                "kind",
                "format",
                "out_dir",
                "out_file",
                "suffix",
                "topic",
                "mag_topic",
                "frame_id",
            }
        }

    primary_lidar = definition["primary_lidar"]
    lidar_count = sum(1 for row in rows if row["sensor"] == primary_lidar)
    manifest = {
        "format_version": FORMAT_VERSION,
        "dataset": dataset,
        "sequence": sequence,
        "source_path": str(source),
        "created_at": datetime.now(timezone.utc).isoformat(),
        "time_unit": "nanoseconds",
        "timeline": "timeline.csv",
        "primary_lidar": primary_lidar,
        "primary_lidar_frames": lidar_count,
        "sensors": sensors,
    }
    with (sequence_dir / "manifest.yaml").open("w") as handle:
        yaml.safe_dump(manifest, handle, sort_keys=False)

    if missing_files:
        missing_path = sequence_dir / "missing_files.txt"
        with missing_path.open("w") as handle:
            handle.write("\n".join(missing_files))
            handle.write("\n")

    return {
        "sequence_dir": sequence_dir,
        "events": len(rows),
        "sensors": present_sensors,
        "linked_files": linked_files,
        "missing_files": missing_files,
    }


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Convert raw MulRan/HeLiPR data to the dataloader layout.")
    parser.add_argument("--dataset", required=True, choices=sorted(["mulran", "helipr"]))
    parser.add_argument("--source", required=True, help="Raw sequence directory.")
    parser.add_argument("--output", required=True, help="Converted dataset root.")
    parser.add_argument("--sequence", default=None, help="Sequence name. Defaults to source directory name.")
    parser.add_argument(
        "--link-mode",
        default="hardlink",
        choices=["hardlink", "symlink", "copy"],
        help="How to place large sensor files. hardlink falls back to copy across filesystems.",
    )
    parser.add_argument("--overwrite", action="store_true", help="Replace an existing converted sequence.")
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
    )
    print("converted sequence: {}".format(result["sequence_dir"]))
    print("events: {}".format(result["events"]))
    print("sensors: {}".format(", ".join(result["sensors"])))
    if result["missing_files"]:
        print("missing files: {} (see missing_files.txt)".format(len(result["missing_files"])), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())

