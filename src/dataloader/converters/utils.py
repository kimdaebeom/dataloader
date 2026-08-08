"""Shared parsing helpers used by dataset-specific converters."""

import csv

import numpy as np

from dataloader.pose_utils import matrix_to_quat, timestamp_to_ns


def read_labeled_timeline(path, label_to_sensor):
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
            if sensor is not None:
                rows.append(
                    {
                        "timestamp_ns": stamp,
                        "sensor": sensor,
                        "source_label": label,
                    }
                )
    rows.sort(key=lambda item: (item["timestamp_ns"], item["sensor"]))
    return rows


def tum_rows_from_file(path):
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


def tum_rows_from_global_pose_csv(path):
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


def write_tum_rows(rows, path):
    if not rows:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as handle:
        for stamp_ns, tx, ty, tz, qx, qy, qz, qw in rows:
            handle.write(
                "{} {:.12g} {:.12g} {:.12g} {:.12g} {:.12g} {:.12g} {:.12g}\n".format(
                    int(stamp_ns), tx, ty, tz, qx, qy, qz, qw
                )
            )
    return True
