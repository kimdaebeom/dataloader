# dataloader

MulRan, HeLiPR, SemanticKITTI raw dataset을 공통 format으로 변환하고, 변환된 데이터를 같은 방식으로 ROS playback하기 위한 패키지입니다.

## 한눈에 보기

- raw dataset을 먼저 `copy` mode로 공통 format에 변환
- 변환 결과는 `manifest.yaml`, `timeline.csv`, `sensors/`, `poses/` 구조로 통일
- 여러 sequence batch convert 지원
- 변환 검증 후 raw sequence 자동 삭제 옵션 지원
- playback은 시작 시 pause 상태이고 `spacebar`로 재생/정지
- playback 중 한 줄 progress bar로 현재 진행률 표시

## 문서

HTML 문서가 기본 문서입니다.

- `docs/index.html`: 원본 dataset 배치 구조
- `docs/converted_format.html`: 변환 결과 구조, batch convert, 원본 삭제 옵션
- `docs/playback_config.html`: playback config, topic 선택, pose transform

## 준비

```bash
source /opt/ros/noetic/setup.zsh
source /home/beom/dynamic_ws/devel/setup.zsh
```

HeLiPR에서 `livox_avia`를 playback하려면 Livox workspace도 source합니다.

```bash
source /home/beom/livox_ws/devel/setup.zsh
```

## Convert

sequence 하나를 변환할 때는 raw sequence 폴더를 `--source`로 넘깁니다.

```bash
rosrun dataloader dataloader_convert.py \
  --dataset mulran \
  --source /media/beom/ux_dataset1/dataset/mulran/DCC01 \
  --output /media/beom/ux_dataset1/converted_dataset \
  --sequence DCC01
```

기본 storage mode는 `copy`입니다. sensor file까지 converted folder 안으로 복사하므로, 변환 결과만 있어도 playback과 offline algorithm에서 사용할 수 있습니다.

지원 dataset:

- `mulran`
- `helipr`
- `semantic_kitti`

특정 LiDAR frame 구간만 변환:

```bash
rosrun dataloader dataloader_convert.py \
  --dataset semantic_kitti \
  --source /media/beom/T71/dataset/semantic_kitti/00 \
  --output /media/beom/ux_dataset1/converted_dataset \
  --sequence 00_100_200 \
  --start-lidar-frame 100 \
  --end-lidar-frame 200
```

이미 같은 sequence가 있으면 `--overwrite`를 붙여 다시 만듭니다.

## Batch Convert

여러 sequence를 자동으로 순서대로 변환할 때는 batch script를 사용합니다. sequence 이름을 생략하면 `--source-root` 아래의 모든 폴더를 sequence로 보고 변환합니다.

```bash
rosrun dataloader convert_sequences.sh \
  --dataset helipr \
  --source-root /media/beom/ux_dataset1/dataset/helipr \
  --output /media/beom/ux_dataset1/converted_dataset
```

일부 sequence만 변환:

```bash
rosrun dataloader convert_sequences.sh \
  --dataset semantic_kitti \
  --source-root /media/beom/T71/dataset/semantic_kitti \
  --output /media/beom/ux_dataset1/converted_dataset \
  00 01 02 05 07
```

주요 옵션:

- `--overwrite`: 이미 변환된 sequence를 삭제하고 다시 변환
- `--continue-on-error`: 중간에 실패해도 다음 sequence 계속 진행
- `--link-mode reference`: 빠른 local test용, 원본 파일을 참조만 함
- `--delete-source-after-success`: 변환 검증 후 raw sequence 폴더 삭제

## 원본 자동 삭제

용량 때문에 sequence 하나를 변환한 뒤 바로 원본을 지우려면 batch script에 아래 옵션을 붙입니다.

```bash
rosrun dataloader convert_sequences.sh \
  --dataset helipr \
  --source-root /media/beom/ux_dataset1/dataset/helipr \
  --output /media/beom/ux_dataset1/converted_dataset \
  --delete-source-after-success
```

삭제는 아래 조건을 모두 만족할 때만 실행됩니다.

- convert command 성공
- `manifest.yaml`의 `storage_mode: copy`
- `missing_files.txt` 없음
- `timeline.csv`의 모든 `relative_path`가 converted folder 내부 파일
- timeline에 적힌 모든 파일이 실제 존재
- 삭제 대상이 `--source-root` 아래의 sequence 폴더

`--link-mode reference`처럼 원본을 참조하는 mode에서는 삭제가 거부됩니다. 이미 변환되어 skip된 sequence도 삭제하지 않습니다.

## Convert 결과

변환 결과는 dataset 종류와 상관없이 같은 큰 구조를 가집니다.

```text
<converted_root>/
  <dataset>/
    <sequence>/
      manifest.yaml
      timeline.csv
      sensors/
      poses/
```

공통 파일:

- `manifest.yaml`: dataset, sequence, sensor, topic, frame, pose 정보
- `timeline.csv`: `timestamp_ns,sensor,relative_path`
- `sensors/`: 실제 sensor file
- `poses/*.txt`: TUM format pose

LiDAR bin 내부 format은 dataset마다 다르며, reader는 `manifest.yaml`의 `format` 값을 기준으로 선택됩니다.

## Pose

원본 dataset에 GT pose가 있으면 converter가 TUM format `.txt`로 저장합니다.

- MulRan: `global_pose.csv` -> `poses/gt.txt`
- HeLiPR: `LiDAR_GT/*_gt.txt` -> `poses/gt_<lidar>.txt`
- HeLiPR global GT: `LiDAR_GT/global_*_gt.txt` -> `poses/gt_global_<lidar>.txt`
- SemanticKITTI: `odom_tum.txt` -> `poses/gt.txt`

TUM format:

```text
timestamp tx ty tz qx qy qz qw
```

## Playback

config 파일에서 dataset, sequence, publish할 topic, LiDAR frame range를 고른 뒤 실행합니다.

```bash
roslaunch dataloader player.launch \
  config:=$(rospack find dataloader)/config/mulran.yaml
```

SemanticKITTI:

```bash
roslaunch dataloader player.launch \
  config:=$(rospack find dataloader)/config/semantic_kitti.yaml
```

Topic 선택은 config의 `publish` block에서 true/false로 관리합니다.

```yaml
publish:
  ouster: true
  radar: false
  gps: true
  imu: true
```

LiDAR frame range:

```yaml
start_lidar_frame: 100
end_lidar_frame: 200
```

기본 playback은 시작 직후 멈춘 상태입니다. 터미널에서 `spacebar`를 누르면 재생하고, 다시 `spacebar`를 누르면 pause됩니다. `q`를 누르면 종료합니다.

```yaml
keyboard_control: true
start_paused: true
```

playback 시작 시에는 dataset, sequence, selected event, LiDAR frame range, enabled sensor, topic만 간단히 출력됩니다. 재생 중에는 한 줄 progress bar가 계속 갱신됩니다.

```text
dataloader playback | mulran/DCC01 | copy | rate 1x | loop False
66205 events | lidar ouster:0-5541 (5542/5542) | 554.1s | clock on | tf off
sensors: gps, imu, ouster, radar | topics: gps:/gps/fix, imu:/imu/data_raw

play [##############--------------]  50.0% | ev 33103/66205 | lidar 2771/5542 | t+277.0s | imu | pub 33103
```

```yaml
progress_log_interval_sec: 2.0
progress_log_percent_step: 5.0
progress_bar_width: 28
terminal_color: true
terminal_direct_tty: true
```

## Pose Transform Playback

SLAM을 새로 돌릴 때는 보통 raw LiDAR frame 그대로 사용합니다.

```yaml
transform:
  enabled: false
```

GT pose, SLAM pose, submap 정합 matrix를 playback에 반영하려면 transform을 켭니다.

```yaml
transform:
  enabled: true
  pose_source: custom
  pose_file: /path/to/poses.txt
  static_matrix_file: /path/to/submap_transform.txt
  output_frame_id: map
  publish_pose: true
```

converter가 저장한 GT pose를 쓰려면:

```yaml
transform:
  enabled: true
  pose_source: gt
  output_frame_id: map
```

자세한 config 설명은 `docs/playback_config.html`을 참고하세요.
