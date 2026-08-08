# ROS playback

ROS 1 and ROS 2 use the same converted data and YAML config. Only the launch command differs.

<a id="launch"></a>
<details open>
<summary><strong>Launch</strong></summary>

<br>

```bash
# ROS 1
roslaunch dataloader player.launch \
  config:=$(rospack find dataloader)/config/mulran.yaml

# ROS 2
ros2 launch dataloader player.launch.py \
  config:=$(ros2 pkg prefix dataloader)/share/dataloader/config/mulran.yaml
```

Copy a config before editing it when you need reproducible experiment settings.

</details>

<a id="required-selection"></a>
<details>
<summary><strong>Required selection</strong></summary>

<br>

```yaml
data_root: /data/converted
dataset: mulran
sequence: KAIST01

primary_lidar: ouster
start_lidar_frame: 0
end_lidar_frame: -1
```

The player selects the time interval from the primary LiDAR frame indices, then publishes enabled sensor events that fall inside it. `-1` selects the final frame.

</details>

<a id="playback-controls"></a>
<details>
<summary><strong>Playback controls</strong></summary>

<br>

```yaml
play_rate: 1.0
loop: false
keyboard_control: true
start_paused: true
```

Press `Space` to play or pause and `Q` to stop. Disable keyboard control for non-interactive launches and set `start_paused: false` if playback should begin immediately.

</details>

<a id="sensors-topics-and-frames"></a>
<details>
<summary><strong>Sensors, topics, and frames</strong></summary>

<br>

Enable only the streams you need:

```yaml
publish:
  ouster: true
  radar: false
  gps: true
  imu: true
  imu_mag: false
  clock: true
```

Override defaults from the manifest when necessary:

```yaml
topics:
  ouster: /points_raw
  imu: /imu/data

frame_ids:
  ouster: lidar
  imu: imu_link
```

`/clock` is published by default. Set `use_sim_time` in downstream ROS nodes when they should follow dataset time.

</details>

<a id="pose-based-point-transforms"></a>
<details>
<summary><strong>Pose-based point transforms</strong></summary>

<br>

Leave transforms disabled when an algorithm expects scans in their original sensor frame:

```yaml
transform:
  enabled: false
```

Use a pose track recorded in the manifest:

```yaml
transform:
  enabled: true
  pose_source: gt
  output_frame_id: map
  apply_to_pointcloud: true
  publish_pose: true
  pose_topic: /dataloader/pose
  pose_timestamp_tolerance_ns: 50000000
```

Or provide an external TUM pose file and an optional 4-by-4 static transform:

```yaml
transform:
  enabled: true
  pose_source: custom
  pose_file: /data/poses/slam.txt
  pose_format: tum
  static_matrix_file: /data/poses/submap_transform.txt
  static_transform_order: after_pose
  output_frame_id: map
```

`after_pose` computes `T_static @ T_pose`; `before_pose` computes `T_pose @ T_static`. A scan is published in its raw frame when no pose is found within the configured tolerance.

</details>

<a id="optional-helipr-message-packages"></a>
<details>
<summary><strong>Optional HeLiPR message packages</strong></summary>

<br>

`livox_avia` requires `livox_ros_driver` on ROS 1 or `livox_ros_driver2` on ROS 2. `inspva` requires `novatel_gps_msgs`. These dependencies are checked only when the corresponding stream is enabled.

</details>
