# Installation

<a id="requirements"></a>
<details open>
<summary><strong>Requirements</strong></summary>

<br>

- Python 3.8 or newer
- NumPy and PyYAML for conversion and reading
- OpenCV plus ROS message packages for playback
- ROS 1 Noetic or a supported ROS 2 distribution for ROS playback

The Python API and CLI tools do not require ROS.

</details>

<a id="python-and-pip"></a>
<details open>
<summary><strong>Python and pip</strong></summary>

<br>

Install the public package from PyPI:

```bash
python3 -m pip install autodataloader
```

The distribution name is `autodataloader`; the import package remains `dataloader`. This installation provides conversion, batch conversion, reading, validation, and inspection without ROS.

Verify the command-line entry points:

```bash
dataloader-convert --help
dataloader-convert-many --help
dataloader-info --help
dataloader-validate --help
```

The PyPI distribution does not install ROS dependencies or expose a playback command. Build the repository as a ROS package when playback is required.

For development from a source checkout:

```bash
git clone https://github.com/kimdaebeom/dataloader.git
cd dataloader
python3 -m pip install -e .
```

</details>

<a id="ros-1"></a>
<details>
<summary><strong>ROS 1</strong></summary>

<br>

Clone the repository into a catkin workspace and build it after sourcing ROS:

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kimdaebeom/dataloader.git
cd ..
catkin_make
source devel/setup.bash
```

Verify and launch:

```bash
rosrun dataloader dataloader_convert.py --help
roslaunch dataloader player.launch \
  config:=$(rospack find dataloader)/config/mulran.yaml
```

</details>

<a id="ros-2"></a>
<details>
<summary><strong>ROS 2</strong></summary>

<br>

Clone the same repository into a colcon workspace:

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

Verify and launch:

```bash
ros2 run dataloader dataloader_convert --help
ros2 launch dataloader player.launch.py \
  config:=$(ros2 pkg prefix dataloader)/share/dataloader/config/mulran.yaml
```

The package uses the sourced `ROS_VERSION` to select catkin or ament from the same `package.xml` and `CMakeLists.txt`.

</details>

<a id="optional-playback-messages"></a>
<details>
<summary><strong>Optional playback messages</strong></summary>

<br>

Most sensors use standard messages from `sensor_msgs` and `geometry_msgs`. HeLiPR has two optional publishers:

- `livox_avia`: `livox_ros_driver` on ROS 1 or `livox_ros_driver2` on ROS 2
- `inspva`: `novatel_gps_msgs`

These packages are needed only when the corresponding sensor is enabled in the playback config. Source their workspace before launching the player.

</details>

<a id="troubleshooting"></a>
<details>
<summary><strong>Troubleshooting</strong></summary>

<br>

- `ROS_VERSION is not set`: source exactly one ROS environment before playback.
- `package 'dataloader' not found`: source the workspace's `devel/setup.bash` or `install/setup.bash` after building.
- Missing Livox or NovAtel message errors: disable that sensor or install and source its message package.
- No keyboard controls: launch from an interactive terminal or set `keyboard_control: false` and `start_paused: false`.

</details>
