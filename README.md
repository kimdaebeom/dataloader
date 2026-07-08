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

Dataset folder requirements are documented in `docs/dataset_layout.html`.
