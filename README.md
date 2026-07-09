# dataloader

MulRan, HeLiPR, SemanticKITTI, and future datasets를 같은 방식으로 변환하고 ROS topic으로 재생하기 위한 패키지입니다.

## Convert

```bash
rosrun dataloader dataloader_convert.py --dataset mulran --source /path/to/raw_sequence --output /path/to/converted_root --sequence KAIST01
```

기본 storage mode는 `copy`입니다. 실제 sensor file까지 converted folder 안으로 복사하므로, 변환이 끝난 뒤에는 원본 dataset 없이도 재생하거나 offline algorithm에서 읽을 수 있습니다.

용량을 아끼는 local test가 필요할 때만 `--link-mode reference`를 명시하면 됩니다. 이 경우 sensor file은 복사하지 않고 원본 absolute path만 참조하므로 원본 dataset을 지우면 안 됩니다.

원본 dataset에 GT pose가 있으면 converter가 TUM format으로 자동 저장합니다.

- MulRan: `global_pose.csv` -> `poses/gt.tum`
- HeLiPR: `LiDAR_GT/*_gt.txt` -> `poses/gt_<lidar>.tum`
- HeLiPR global GT: `LiDAR_GT/global_*_gt.txt` -> `poses/gt_global_<lidar>.tum`
- SemanticKITTI: `odom_tum.txt` -> `poses/gt.tum`

SemanticKITTI 예시:

```bash
rosrun dataloader dataloader_convert.py --dataset semantic_kitti --source /media/beom/T71/dataset/semantic_kitti/00 --output /path/to/converted_root --sequence 00
```

## Play

```bash
roslaunch dataloader player.launch config:=$(rospack find dataloader)/config/mulran.yaml
```

SemanticKITTI는 `config/semantic_kitti.yaml`을 사용합니다.

HeLiPR에서 `livox_avia`를 재생하려면 먼저 Livox workspace를 source해야 합니다.

```bash
source /home/beom/livox_ws/devel/setup.bash
```

## Pose Transform Playback

SLAM을 새로 돌릴 때는 raw LiDAR frame 그대로 쓰는 것이 일반적이므로:

```yaml
transform:
  enabled: false
```

이미 구한 SLAM pose, GT pose, 또는 submap 정합 matrix를 활용할 때는 transform을 켭니다.

직접 만든 TUM pose 사용:

```yaml
transform:
  enabled: true
  pose_source: custom
  pose_file: /path/to/poses.tum
  static_matrix_file: /path/to/submap_transform.txt
  output_frame_id: map
```

converter가 저장한 GT pose 사용:

```yaml
transform:
  enabled: true
  pose_source: gt
  pose_file: ""
  output_frame_id: map
```

HeLiPR의 `global_*_gt.txt` 또는 MulRan global pose를 쓰려면:

```yaml
transform:
  enabled: true
  pose_source: gt_global
```

기본 적용 순서는 `T_publish = T_static * T_pose`입니다. 자세한 옵션은 `config/mulran.yaml`, `config/helipr.yaml`, `docs/playback_config.html`을 참고하세요.

dataset 폴더 구성과 convert 결과 구조는 `docs/index.html`, `docs/converted_format.html`에 정리되어 있습니다.
