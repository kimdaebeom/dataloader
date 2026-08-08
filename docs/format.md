# Converted format

All adapters produce the same top-level structure:

```text
<converted_root>/
└── <dataset>/
    └── <sequence>/
        ├── manifest.yaml
        ├── timeline.csv
        ├── sensors/
        └── poses/
```

<a id="manifest"></a>
<details open>
<summary><strong>Manifest</strong></summary>

<br>

`manifest.yaml` records the format version, dataset and sequence names, storage mode, primary LiDAR, timeline path, sensor definitions, default topics and frame IDs, and available pose tracks.

Example:

```yaml
format_version: 1
dataset: mulran
sequence: KAIST01
storage_mode: copy
timeline: timeline.csv
primary_lidar: ouster
sensors:
  ouster:
    kind: pointcloud
    format: mulran_ouster
    topic: /ouster/points
    frame_id: ouster
poses:
  gt:
    default: poses/gt.txt
```

Consumers should use the declared sensor `format`; binary records differ by sensor and dataset.

</details>

<a id="timeline"></a>
<details>
<summary><strong>Timeline</strong></summary>

<br>

`timeline.csv` is globally ordered by timestamp and sensor:

```csv
timestamp_ns,sensor,relative_path
1561000000000000000,ouster,sensors/lidar/ouster/1561000000000000000.bin
```

In self-contained modes, `relative_path` stays inside the converted sequence. `reference` mode may contain absolute paths to raw files.

</details>

<a id="lidar-data"></a>
<details>
<summary><strong>LiDAR data</strong></summary>

<br>

`Dataset.lidar(...).structured()` preserves the sensor-specific packed fields. `numpy()` returns the common `float32 [N, 4]` view containing `x`, `y`, `z`, and `intensity` (or the closest declared reflectivity field where applicable).

The validator checks that each binary file size is divisible by its format's packed record size.

</details>

<a id="poses"></a>
<details>
<summary><strong>Poses</strong></summary>

<br>

Converted poses use TUM syntax:

```text
timestamp tx ty tz qx qy qz qw
```

Timestamps may be expressed in seconds or nanoseconds when read. Converter output uses nanoseconds. Pose keys are recorded in the manifest so consumers do not need dataset-specific filename rules.

</details>

<a id="storage-modes"></a>
<details>
<summary><strong>Storage modes</strong></summary>

<br>

| Mode | Behavior | Recommended use |
| --- | --- | --- |
| `copy` | Copies every sensor file | Portable output, sharing, or raw-data cleanup |
| `reference` | Stores absolute paths to raw files | Fast local conversion with minimal disk use |
| `symlink` | Creates links in the unified tree | Local use on filesystems with symlink support |
| `hardlink` | Creates hard links | Avoid duplicates on one filesystem |
| `hardlink_or_copy` | Tries a hard link, then copies | Portable fallback across filesystems |

Only `copy` should be used when the raw sequence will be deleted. References, symbolic links, and hard links can depend on the original path or filesystem state.

</details>

<a id="safe-raw-data-cleanup"></a>
<details>
<summary><strong>Safe raw-data cleanup</strong></summary>

<br>

The legacy ROS 1 batch wrapper supports `--delete-source-after-success`:

```bash
rosrun dataloader convert_sequences.sh \
  --dataset helipr \
  --source-root /data/raw/helipr \
  --output /data/converted \
  --delete-source-after-success
```

Deletion is allowed only for a newly converted `copy` result that passes validation, has no `missing_files.txt`, resolves every timeline entry inside the output sequence, and targets a child of the declared source root. Skipped outputs are never deleted.

Source deletion cannot be combined with a partial LiDAR frame range or a non-`copy` storage mode.

</details>
