#!/usr/bin/env python3

from pathlib import Path


FORMAT_VERSION = 1


DATASET_DEFINITIONS = {
    "mulran": {
        "timeline_file": "sensor_data/data_stamp.csv",
        "primary_lidar": "ouster",
        "label_to_sensor": {
            "ouster": "ouster",
            "radar": "radar",
            "gps": "gps",
            "imu": "imu",
        },
        "sensors": {
            "ouster": {
                "kind": "pointcloud",
                "format": "mulran_ouster",
                "raw_dir": "sensor_data/Ouster",
                "out_dir": "sensors/lidar/ouster",
                "suffix": ".bin",
                "topic": "/os1_points",
                "frame_id": "ouster",
            },
            "radar": {
                "kind": "image",
                "format": "mono8_png",
                "raw_dirs": ["sensor_data/radar/polar", "sensor_data/radara/polar"],
                "out_dir": "sensors/radar/polar",
                "suffix": ".png",
                "topic": "/radar/polar",
                "frame_id": "radar_polar",
            },
            "gps": {
                "kind": "csv",
                "format": "mulran_gps",
                "raw_file": "sensor_data/gps.csv",
                "out_file": "sensors/gps/gps.csv",
                "topic": "/gps/fix",
                "frame_id": "gps",
            },
            "imu": {
                "kind": "csv",
                "format": "xsens_imu",
                "raw_file": "sensor_data/xsens_imu.csv",
                "out_file": "sensors/imu/xsens_imu.csv",
                "topic": "/imu/data_raw",
                "mag_topic": "/imu/mag",
                "frame_id": "imu",
            },
        },
    },
    "helipr": {
        "timeline_file": "stamp.csv",
        "primary_lidar": "ouster",
        "label_to_sensor": {
            "inspva": "inspva",
            "imu": "imu",
            "ouster": "ouster",
            "velodyne": "velodyne",
            "livox_avia": "livox_avia",
            "aeva": "aeva",
        },
        "sensors": {
            "inspva": {
                "kind": "csv",
                "format": "novatel_inspva",
                "raw_file": "Inertial_data/inspva.csv",
                "out_file": "sensors/inertial/inspva.csv",
                "topic": "/inspva",
                "frame_id": "inspva",
            },
            "imu": {
                "kind": "csv",
                "format": "xsens_imu",
                "raw_file": "Inertial_data/xsens_imu.csv",
                "out_file": "sensors/imu/xsens_imu.csv",
                "topic": "/imu/data_raw",
                "mag_topic": "/imu/mag",
                "frame_id": "imu",
            },
            "ouster": {
                "kind": "pointcloud",
                "format": "helipr_ouster",
                "raw_dirs": ["LiDAR/Ouster", "LiDAR/Oustera"],
                "out_dir": "sensors/lidar/ouster",
                "suffix": ".bin",
                "topic": "/ouster/points",
                "frame_id": "ouster",
            },
            "velodyne": {
                "kind": "pointcloud",
                "format": "helipr_velodyne",
                "raw_dirs": ["LiDAR/Velodyne", "LiDAR/Velodynea"],
                "out_dir": "sensors/lidar/velodyne",
                "suffix": ".bin",
                "topic": "/velodyne/points",
                "frame_id": "velodyne",
            },
            "livox_avia": {
                "kind": "livox_custom",
                "format": "helipr_livox_avia",
                "raw_dirs": ["LiDAR/Avia", "LiDAR/Aviaa"],
                "out_dir": "sensors/lidar/avia",
                "suffix": ".bin",
                "topic": "/avia/points",
                "frame_id": "livox_avia",
            },
            "aeva": {
                "kind": "pointcloud",
                "format": "helipr_aeva",
                "raw_dirs": ["LiDAR/Aeva", "LiDAR/Aevaa"],
                "out_dir": "sensors/lidar/aeva",
                "suffix": ".bin",
                "topic": "/aeva/points",
                "frame_id": "aeva",
            },
        },
    },
}


def dataset_definition(dataset):
    try:
        return DATASET_DEFINITIONS[dataset]
    except KeyError as exc:
        names = ", ".join(sorted(DATASET_DEFINITIONS))
        raise ValueError("unknown dataset '{}'; expected one of: {}".format(dataset, names)) from exc


def resolve_sequence_dir(data_root, dataset, sequence):
    root = Path(data_root).expanduser()
    candidates = [
        root / dataset / sequence,
        root / sequence,
        root,
    ]
    for candidate in candidates:
        if (candidate / "manifest.yaml").is_file():
            return candidate.resolve()
    raise FileNotFoundError(
        "manifest.yaml not found for dataset='{}', sequence='{}' under '{}'".format(
            dataset, sequence, root
        )
    )
