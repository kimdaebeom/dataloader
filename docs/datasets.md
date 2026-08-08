# Dataset layouts

Each converter accepts one raw sequence directory. The examples below show the paths used by the adapters; unrelated files may remain alongside them.

<a id="mulran"></a>
<details>
<summary><strong>MulRan</strong></summary>

<br>

```text
KAIST01/
├── global_pose.csv                 # optional ground truth
└── sensor_data/
    ├── data_stamp.csv
    ├── Ouster/*.bin
    ├── radar/polar/*.png
    ├── gps.csv
    └── xsens_imu.csv
```

`data_stamp.csv` contains `<timestamp_ns>,<sensor_name>` rows. The converter also accepts the local `radara/polar` spelling. A sensor listed in the timeline but missing on disk is excluded from the converted manifest.

Ground truth in `global_pose.csv` is converted to `poses/gt.txt`.

</details>

<a id="helipr"></a>
<details>
<summary><strong>HeLiPR</strong></summary>

<br>

```text
DCC01/
├── stamp.csv
├── Inertial_data/
│   ├── inspva.csv
│   └── xsens_imu.csv
├── LiDAR/
│   ├── Ouster/*.bin
│   ├── Velodyne/*.bin
│   ├── Avia/*.bin
│   └── Aeva/*.bin
└── LiDAR_GT/
    ├── Ouster_gt.txt
    ├── Velodyne_gt.txt
    └── global_*_gt.txt
```

`stamp.csv` contains `<timestamp_ns>,<sensor_name>` rows. The converter also accepts the local folder spellings `Oustera`, `Velodynea`, `Aviaa`, and `Aevaa`.

LiDAR-specific ground truth is copied into `poses/gt_<sensor>.txt`; global tracks are stored as `poses/gt_global_<sensor>.txt`.

</details>

<a id="semantickitti"></a>
<details>
<summary><strong>SemanticKITTI</strong></summary>

<br>

The current adapter targets a preprocessed sequence rather than native KITTI files:

```text
00/
├── pcd/
│   ├── 000000.bin
│   ├── 000001.bin
│   └── ...
├── odom_tum.txt
└── manifest.json                  # optional metadata, not required
```

Each `pcd/*.bin` file stores little-endian `float32 [x, y, z, intensity]` records. `odom_tum.txt` supplies both LiDAR timestamps and poses in TUM format. Its row order must match the sorted PCD filenames one-to-one; conversion stops when the counts differ.

Native KITTI/KITTI Odometry layouts are not currently supported.

</details>
