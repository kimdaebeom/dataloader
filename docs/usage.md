# Usage

Conversion, reading, inspection, and validation do not require ROS. Replace the example paths below with your dataset locations.

## Command-line tools

A Python installation provides four commands:

| Command | Purpose |
| --- | --- |
| `dataloader-convert` | Convert one sequence |
| `dataloader-convert-many` | Convert multiple sequences |
| `dataloader-info` | Summarize a converted sequence |
| `dataloader-player` | Run playback from a sourced ROS environment |
| `dataloader-validate` | Validate format and referenced files |

`python -m dataloader` is equivalent to `dataloader-convert`.

## Convert one sequence

```bash
dataloader-convert \
  --dataset helipr \
  --source /data/raw/helipr/DCC01 \
  --output /data/converted \
  --sequence DCC01
```

The default `copy` mode produces a self-contained sequence. Use `--overwrite` to replace an existing output or select a LiDAR frame range:

```bash
dataloader-convert \
  --dataset semantic_kitti \
  --source /data/raw/semantic_kitti/00 \
  --output /data/converted \
  --start-lidar-frame 100 \
  --end-lidar-frame 200
```

The equivalent Python API is:

```python
from dataloader import convert_dataset

result = convert_dataset(
    dataset="mulran",
    source="/data/raw/mulran/KAIST01",
    output_root="/data/converted",
    link_mode="copy",
)
print(result["manifest_path"])
```

## Read a converted sequence

```python
from dataloader import Dataset

dataset = Dataset("/data/converted/helipr/DCC01")

for event in dataset:
    print(event.timestamp_ns, event.sensor, event.path)

frame = dataset.lidar(sensor="ouster", frame=100)
points = frame.numpy()       # float32 [N, 4]: x, y, z, intensity
raw = frame.structured()     # sensor-specific fields such as ring or velocity
```

The LiDAR adapter is selected from the `format` value in `manifest.yaml`. Unknown formats fail explicitly instead of being guessed.

Time and pose queries:

```python
nearest_imu = dataset.nearest("imu", frame.timestamp_ns)
imu_window = dataset.between(
    "imu",
    frame.timestamp_ns - 50_000_000,
    frame.timestamp_ns,
)
pose = dataset.pose_at(frame.timestamp_ns, source="gt", sensor="ouster")
```

## Validate and inspect

```bash
dataloader-validate /data/converted/helipr/DCC01
dataloader-info /data/converted/helipr/DCC01
```

Both commands accept `--json`. Validation checks the manifest, timeline, path safety, missing files, LiDAR record sizes, and pose syntax.

```python
from dataloader import dataset_info, validate_dataset

report = validate_dataset("/data/converted/helipr/DCC01")
print(report.ok, report.errors)

info = dataset_info("/data/converted/helipr/DCC01")
print(info["sensors"])
```

## Batch conversion

Convert every child directory under a raw dataset root:

```bash
dataloader-convert-many \
  --dataset mulran \
  --source-root /data/raw/mulran \
  --output /data/converted \
  --workers 2
```

Append sequence names to convert only a subset:

```bash
dataloader-convert-many \
  --dataset mulran \
  --source-root /data/raw/mulran \
  --output /data/converted \
  KAIST01 KAIST02
```

```python
from dataloader import convert_many

result = convert_many(
    dataset="mulran",
    source_root="/data/raw/mulran",
    output_root="/data/converted",
    sequences=["KAIST01", "KAIST02"],
    workers=2,
)
print(result.successful, result.skipped, result.failed)
```

Existing outputs are skipped unless `overwrite=True`. Use `workers=1` when you want sequence logs to remain strictly ordered.

## Storage modes

`copy`, `reference`, `symlink`, `hardlink`, and `hardlink_or_copy` are supported. Use `copy` for portable results and whenever the raw sequence may be removed. See [Converted format](format.md#storage-modes) for the trade-offs.

## ROS playback

Build the package in a ROS workspace, choose a config under `config/`, and use the ROS-version-specific launch command. Playback starts paused by default: press `Space` to play or pause and `Q` to quit.

See [Installation](installation.md) and [ROS playback](ros-playback.md) for complete commands and configuration options.
