# Dataloader

[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![ROS](https://img.shields.io/badge/ROS-Noetic%20%7C%20ROS%202-22314E?logo=ros&logoColor=white)](docs/installation.md)
[![CI](https://github.com/kimdaebeom/dataloader/actions/workflows/python-package.yml/badge.svg)](https://github.com/kimdaebeom/dataloader/actions/workflows/python-package.yml)
[![ROS CI](https://github.com/kimdaebeom/dataloader/actions/workflows/ros.yml/badge.svg)](https://github.com/kimdaebeom/dataloader/actions/workflows/ros.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Convert MulRan, HeLiPR, and SemanticKITTI sequences into one timestamped layout, then read them through the same Python API or replay them in ROS 1 and ROS 2.

> Conversion, validation, and reading are ROS-independent. ROS is needed only for playback.

## Highlights

| Capability | Included |
| --- | --- |
| Unified data | `manifest.yaml`, `timeline.csv`, `sensors/`, and `poses/` |
| Dataset adapters | MulRan, HeLiPR, SemanticKITTI |
| Python tools | Converter, batch conversion, reader, validator, and summary CLI |
| LiDAR readers | Ouster, Velodyne, Livox Avia, and Aeva |
| ROS playback | ROS 1 catkin and ROS 2 colcon, with time, topics, and pose transforms |

## Quick start

```bash
git clone https://github.com/kimdaebeom/dataloader.git
cd dataloader
python3 -m pip install .

dataloader-convert \
  --dataset mulran \
  --source /data/raw/mulran/KAIST01 \
  --output /data/converted
```

```python
from dataloader import Dataset

dataset = Dataset("/data/converted/mulran/KAIST01")
points = dataset.lidar(frame=0).numpy()  # float32 [N, 4]
```

ROS playback:

```bash
# ROS 1
roslaunch dataloader player.launch config:=/path/to/mulran.yaml

# ROS 2
ros2 launch dataloader player.launch.py config:=/path/to/mulran.yaml
```

## Documentation

- [Installation](docs/installation.md) — Python, ROS 1, and ROS 2
- [Usage](docs/usage.md) — CLI and Python API
- [Dataset layouts](docs/datasets.md) — expected raw input structures
- [Converted format](docs/format.md) — manifest, timeline, poses, and storage modes
- [ROS playback](docs/ros-playback.md) — configuration, topics, and transforms
- [Development](docs/development.md) · [Release checklist](docs/releasing.md)

## Support matrix

| Dataset | Converter | LiDAR formats |
| --- | --- | --- |
| MulRan | Yes | Ouster |
| HeLiPR | Yes | Ouster, Velodyne, Livox Avia, Aeva |
| SemanticKITTI | Yes | Velodyne |

SemanticKITTI currently expects the preprocessed `pcd/` plus `odom_tum.txt` layout described in [Dataset layouts](docs/datasets.md#semantickitti). Native KITTI/KITTI Odometry input is not yet supported.

Released under the [MIT License](LICENSE).
