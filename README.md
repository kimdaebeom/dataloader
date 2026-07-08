# dataloader

Unified dataset converter and ROS file player for MulRan, HeLiPR, and future datasets.

## Convert

```bash
rosrun dataloader dataloader_convert.py --dataset mulran --source /path/to/raw_sequence --output /path/to/converted_root --sequence KAIST01
```

The default converter mode is `reference`, so converted datasets stay small by storing metadata and absolute paths to the original files.

## Play

```bash
roslaunch dataloader player.launch config:=$(rospack find dataloader)/config/mulran.yaml
```

For HeLiPR `livox_avia`, source the Livox workspace first:

```bash
source /home/beom/livox_ws/devel/setup.bash
```

Playback can optionally transform LiDAR points using a TUM pose file and an extra 4x4 matrix. See `transform` in `config/mulran.yaml` or `config/helipr.yaml`.

Dataset folder requirements are documented in `docs/dataset_layout.html`.
