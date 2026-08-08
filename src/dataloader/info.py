"""Human- and machine-readable summaries of converted sequences."""

import argparse
import json
from collections import Counter

from .dataset import Dataset


def dataset_info(sequence_dir):
    dataset = Dataset(sequence_dir)
    counts = Counter(event.sensor for event in dataset)
    sensor_events = {
        name: dataset.sensor_events(name)
        for name in dataset.sensors
    }
    formats = {
        name: {
            "kind": spec.get("kind", ""),
            "format": spec.get("format", ""),
            "events": counts.get(name, 0),
            "rate_hz": (
                0.0
                if len(sensor_events[name]) < 2
                or sensor_events[name][-1].timestamp_ns
                == sensor_events[name][0].timestamp_ns
                else (len(sensor_events[name]) - 1)
                * 1e9
                / (
                    sensor_events[name][-1].timestamp_ns
                    - sensor_events[name][0].timestamp_ns
                )
            ),
        }
        for name, spec in sorted(dataset.sensors.items())
    }
    pose_streams = {
        source: sorted(group)
        for source, group in sorted(dataset.manifest.get("poses", {}).items())
        if isinstance(group, dict)
    }
    return {
        "path": str(dataset.root),
        "dataset": dataset.dataset,
        "sequence": dataset.sequence,
        "storage_mode": dataset.manifest.get("storage_mode", ""),
        "events": len(dataset),
        "duration_ns": dataset.duration_ns,
        "duration_sec": dataset.duration_ns / 1e9,
        "primary_lidar": dataset.primary_lidar,
        "sensors": formats,
        "pose_streams": pose_streams,
    }


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Show converted dataset information.")
    parser.add_argument("sequence_dir")
    parser.add_argument("--json", action="store_true")
    return parser


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    info = dataset_info(args.sequence_dir)
    if args.json:
        print(json.dumps(info, indent=2, sort_keys=True))
        return 0
    print("{}/{}".format(info["dataset"], info["sequence"]))
    print("path     : {}".format(info["path"]))
    print("storage  : {}".format(info["storage_mode"]))
    print("events   : {}".format(info["events"]))
    print("duration : {:.3f} sec".format(info["duration_sec"]))
    print("primary  : {}".format(info["primary_lidar"]))
    print("sensors")
    for name, spec in info["sensors"].items():
        print(
            "  - {:<14} {:>8} events | {:>8.2f} Hz | {:<12} | {}".format(
                name, spec["events"], spec["rate_hz"], spec["kind"], spec["format"]
            )
        )
    if info["pose_streams"]:
        print("poses")
        for source, sensors in info["pose_streams"].items():
            print("  - {}: {}".format(source, ", ".join(sensors)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
