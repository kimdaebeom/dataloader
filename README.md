# AutoDataloader

Convert autonomous-driving datasets into one timestamped layout, then use the same Python API and ROS player across datasets.

> Conversion, validation, and reading are ROS-independent. ROS is required only for playback.

## Highlights

| Capability | Included |
| --- | --- |
| Unified data | `manifest.yaml`, `timeline.csv`, `sensors/`, and `poses/` |
| Dataset adapters | [MulRan](https://sites.google.com/view/mulran-pr), [HeLiPR](https://sites.google.com/view/heliprdataset), [SemanticKITTI](https://www.semantic-kitti.org/) |
| Python tools | Converter, batch conversion, reader, validator, and summary CLI |
| LiDAR readers | Ouster, Velodyne, Livox Avia, and Aeva |
| ROS playback | ROS 1 catkin and ROS 2 colcon, with time, topics, and pose transforms |

## Quick start

<details open>
<summary><strong>Install</strong></summary>

<br>

#### Python

Install the public package from PyPI:

```bash
python3 -m pip install autodataloader
```

The distribution is named `autodataloader`; import it as `dataloader`. The PyPI package installs only the ROS-independent conversion, reading, validation, and inspection tools.

#### ROS 1

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kimdaebeom/dataloader.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

#### ROS 2

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/kimdaebeom/dataloader.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select dataloader
source install/setup.bash
```

</details>

<details>
<summary><strong>Convert</strong></summary>

<br>

#### Python CLI

```bash
dataloader-convert \
  --dataset mulran \
  --source /data/raw/mulran/KAIST01 \
  --output /data/converted
```

#### Python API

```python
from dataloader import Dataset, convert_dataset

result = convert_dataset(
    dataset="mulran",
    source="/data/raw/mulran/KAIST01",
    output_root="/data/converted",
)
dataset = Dataset(result["sequence_dir"])
points = dataset.lidar(frame=0).numpy()  # float32 [N, 4]
```

#### ROS 1

```bash
rosrun dataloader dataloader_convert.py \
  --dataset mulran \
  --source /data/raw/mulran/KAIST01 \
  --output /data/converted
```

#### ROS 2

```bash
ros2 run dataloader dataloader_convert \
  --dataset mulran \
  --source /data/raw/mulran/KAIST01 \
  --output /data/converted
```

</details>

<details>
<summary><strong>Playback</strong></summary>

<br>

Choose a config from `config/` and set its dataset root and sequence.

#### ROS 1

```bash
roslaunch dataloader player.launch \
  config:=$(rospack find dataloader)/config/mulran.yaml
```

#### ROS 2

```bash
ros2 launch dataloader player.launch.py \
  config:=$(ros2 pkg prefix dataloader)/share/dataloader/config/mulran.yaml
```

</details>

## Documentation

- [Installation](https://github.com/kimdaebeom/dataloader/blob/master/docs/installation.md) — Python, ROS 1, and ROS 2
- [Usage](https://github.com/kimdaebeom/dataloader/blob/master/docs/usage.md) — CLI and Python API
- [Dataset layouts](https://github.com/kimdaebeom/dataloader/blob/master/docs/datasets.md) — expected raw input structures
- [Converted format](https://github.com/kimdaebeom/dataloader/blob/master/docs/format.md) — manifest, timeline, poses, and storage modes
- [ROS playback](https://github.com/kimdaebeom/dataloader/blob/master/docs/ros-playback.md) — configuration, topics, and transforms
- [Development](https://github.com/kimdaebeom/dataloader/blob/master/docs/development.md) · [Release checklist](https://github.com/kimdaebeom/dataloader/blob/master/docs/releasing.md)

<details>
<summary><strong>Supported datasets</strong></summary>

<br>

| Dataset | Converter | LiDAR formats |
| --- | --- | --- |
| [MulRan](https://sites.google.com/view/mulran-pr) | Yes | Ouster |
| [HeLiPR](https://sites.google.com/view/heliprdataset) | Yes | Ouster, Velodyne, Livox Avia, Aeva |
| [SemanticKITTI](https://www.semantic-kitti.org/) | Yes | Velodyne |

SemanticKITTI currently expects the preprocessed `pcd/` plus `odom_tum.txt` layout described in [Dataset layouts](https://github.com/kimdaebeom/dataloader/blob/master/docs/datasets.md#semantickitti). Native KITTI/KITTI Odometry input is not yet supported.

</details>

Released under the [MIT License](https://github.com/kimdaebeom/dataloader/blob/master/LICENSE).
