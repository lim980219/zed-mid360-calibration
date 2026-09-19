# ZED2–MID-360 자동 캘리브레이션 (ROS 1 Noetic)

여러 정지 장면의 ROS1 `.bag`을 입력해 영상·PCD 추출, 내부/외부 초기값 설정,
HKU `livox_camera_calib` 실행, 결과 저장까지 한 명령으로 처리합니다.

**카메라 위치는 반드시 직접 설정해야 합니다. 설치 위치의 기본값은 없습니다.**
입력 위치와 각도는 최적화 초기값이며, 최적화는 회전과 이동을 함께 추정합니다.
어떤 장면에서도 정확도를 보장하는 전역 탐색 방식은 아닙니다.

## 1. 설치

실제 최적화 실행 환경: **Ubuntu 20.04 / ROS1 Noetic / Python 3.8**.
기존에 빌드한 `hku-mars/livox_camera_calib`의 `lidar_camera_multi_calib`를 사용합니다.

아래 명령은 **로봇의 Ubuntu 터미널**에서 실행합니다. 이 패키지 폴더를
`~/ws_calibration/src/zed_mid360_calibration`에 복사했다고 가정합니다.
워크스페이스 경로는 실제 설치 위치로 바꾸세요.

```bash
source /opt/ros/noetic/setup.bash
# 기존 livox_camera_calib이 다른 워크스페이스에 있다면 그 devel/setup.bash도 source
sudo apt update
sudo apt install -y python3-numpy python3-scipy python3-opencv python3-yaml \
  python3-setuptools ros-noetic-rosbag ros-noetic-roslaunch \
  ros-noetic-sensor-msgs ros-noetic-tf2-ros xvfb xauth

cd ~/ws_calibration
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
rospack find livox_camera_calib
rosrun zed_mid360_calibration calibrate_bags.py --help
```

이 패키지 자체에는 C++ 빌드가 없습니다. 기존 HKU 패키지의 OpenCV/PCL/Ceres 설정은
그대로 사용합니다. HKU 패키지가 아직 없다면
[공식 저장소](https://github.com/hku-mars/livox_camera_calib)의 설치 절차를 먼저 완료해야 합니다.
HKU C++ 프로그램의 빌드 호환성 수정은 이 패키지에서 자동으로 수행하지 않습니다.

## 2. 설치 위치 설정 — 필수

설정 파일을 복사합니다.

```bash
cp "$(rospack find zed_mid360_calibration)/config/calibration.yaml" ~/zed_mid360.yaml
```

`~/zed_mid360.yaml`의 아래 세 `null`을 **실측한 미터 값**으로 바꿉니다.

```yaml
camera_mount:
  forward_m: null       # 라이다 원점에서 ZED 왼쪽 렌즈 중심까지 앞쪽 거리
  left_m: null          # 왼쪽 거리
  down_m: null          # 아래쪽 거리
  pitch_down_deg: 10.0  # 아래를 향하는 기울기
  yaw_left_deg: 0.0     # 위에서 보아 왼쪽으로 향하는 회전
  roll_deg: 0.0         # 카메라 전방 축 기준 오른손 회전
```

예를 들어 센티미터로 측정했다면 100으로 나눠 입력합니다.
앞·왼쪽·아래는 양수, 반대 방향은 음수입니다. 위치를 미입력하거나 NaN을 넣으면 실행이 중단됩니다.
거리의 기준은 **라이다 포인트 좌표의 원점**과 **왼쪽 카메라 광학 중심**입니다.
브래킷이나 센서 하우징 끝점까지의 거리가 아닙니다.

- 라이다: 앞 +X, 왼쪽 +Y, 위 +Z (FLU).
- 카메라 optical: 오른쪽 +X, 아래 +Y, 렌즈 전방 +Z.
- 자세는 `Rz(yaw) Ry(pitch_down) Rx(roll)` 순서로 구성합니다.
- `C_lidar = [forward_m, left_m, -down_m]`.
- 초기 `T_camera_lidar`의 이동벡터는 `-R_camera_lidar @ C_lidar`입니다.
  설치 위치를 그대로 이동벡터에 복사하면 안 됩니다.
- 드라이버가 포인트를 다른 좌표계로 변환해 저장한 bag에는 이 FLU 가정을 그대로 적용할 수 없습니다.
  bag의 `header.frame_id`와 드라이버 설정을 확인하세요.

## 3. 입력 토픽 확인

```bash
rosrun zed_mid360_calibration calibrate_bags.py ~/ws_calibration/bags --inspect
```

기본 토픽은 다음과 같습니다. 실제 bag에 맞게 설정을 바꾸면 됩니다.

```yaml
topics:
  image: /zed2/zed_node/left/image_rect_color
  camera_info: /zed2/zed_node/left/camera_info
  points: /livox/lidar
```

지원 입력:

- ROS1 `.bag`, 폴더 안의 여러 `.bag`, 또는 따옴표로 감싼 glob.
- 왼쪽 rectified `sensor_msgs/Image` 또는 JPEG/PNG `CompressedImage`.
- 해당 해상도의 `sensor_msgs/CameraInfo`.
- `sensor_msgs/PointCloud2` 또는 Livox `CustomMsg` (bag에 포함된 메시지 정의 사용).
- PointCloud2는 행/점 padding, big endian, intensity/reflectivity를 처리합니다.

현재 패키지는 **왼쪽 rectified 영상 전용**입니다. raw 영상, 좌우 합쳐진 영상,
depth 영상, crop/binning이 적용된 CameraInfo는 지원하지 않습니다.
보정된 투영행렬 `P[:3,:3]`를 사용하고 왜곡계수는 0으로 설정합니다.
좌우 영상 혼동을 막기 위해 P의 stereo translation 열은 0인지 확인합니다.
CameraInfo가 없거나 해상도/프레임/내부 파라미터가 bag 간에 다르면 중단합니다.

## 4. 실행

```bash
rosrun zed_mid360_calibration calibrate_bags.py \
  ~/ws_calibration/bags \
  --config ~/zed_mid360.yaml \
  --output ~/calibration_results/run_01
```

파일을 직접 여러 개 넘겨도 됩니다.

```bash
rosrun zed_mid360_calibration calibrate_bags.py \
  ~/bags/scene_2.bag ~/bags/scene_8.bag ~/bags/scene_12.bag \
  --config ~/zed_mid360.yaml --output ~/calibration_results/run_02
```

출력 폴더는 **새 경로**여야 합니다. 기존 결과를 덮어쓰거나 이전 결과를 재사용하지 않습니다.
bag 파일명 번호가 연속일 필요는 없습니다. 내부에서 0, 1, 2…로 재배열합니다.

BMP·PCD와 설정 파일만 만들려면 `--prepare-only`를 추가합니다.
이 모드에서는 최적화를 실행하지 않으며 extrinsics.yaml을 생성하지 않습니다.
전체 실행에는 `--prepare-only`를 빼고 새 출력 폴더를 사용하세요.

센서 실행, rosbag play, 수동 대응점 클릭은 필요하지 않습니다.
자체 ROS master를 사용하며 기존 로봇 ROS 파라미터에 쓰지 않습니다.
HKU의 OpenCV 창은 기본적으로 Xvfb에서 실행됩니다.
완료 후 `push enter to publish again` 대기를 감지하면 소유한 프로세스를 종료합니다.
중단·실패·시간 초과 때도 같은 정리를 수행합니다. TF는 자동으로 게시하지 않습니다.

## 5. 데이터 선택과 한계

각 bag을 하나의 장면으로 취급합니다. **bag 중앙의 최대 5초 구간**에서
가운데 영상 한 장과 누적 포인트클라우드를 사용합니다.
중앙 구간이 움직이면 다른 구간을 자동 탐색하지 않고 해당 실행을 중단합니다.
정지 구간만 잘라 저장하거나 다시 촬영하세요.

영상 11개 표본 사이의 광학 흐름으로 움직임을 검사합니다.
추적 가능한 특징이 부족하거나 움직임이 크면 실패 처리합니다.
이는 정지 여부의 보조 검사이며, LiDAR deskew/SLAM 또는 움직이는 물체의 완전한 제거가 아닙니다.
센서·환경이 정지한 장면, 여러 방향의 문틀/박스/기둥 등 입체 경계를 포함한
여러 bag을 사용하는 것이 적합합니다. 동일 평면만 있는 장면은 변환을 충분히 구속하지 못합니다.

`header.stamp` 기준 최근접 영상–라이다 시간 차이를 검사합니다.
시간 오프셋 자동 추정은 하지 않습니다. 이미 알고 있는 보정값만 다음 항목에 입력하세요.

```yaml
data:
  accumulation_seconds: 5.0
  max_sync_seconds: 0.10
  camera_time_offset_seconds: 0.0
  voxel_size_m: 0.02
  max_motion_pixels: 2.0
```

보정식은 `t_image_corrected = t_image_header + camera_time_offset_seconds`입니다.
bag 기록 시각은 중앙 구간 선택에 사용하고, 헤더 시각은 동기 검사에 사용합니다.
두 센서 헤더가 다른 clock domain이면 먼저 기록 환경을 점검해야 합니다.
누적 포인트 수가 제한을 넘으면 voxel 크기를 키우거나 누적 시간을 줄이세요.

## 6. 결과

| 파일 | 내용 |
|---|---|
| `extrinsics.yaml` | 양방향 행렬, quaternion, 프레임, 변화량, 검사 상태 |
| `T_camera_lidar.txt` | `p_camera_optical = T_camera_lidar @ p_lidar` |
| `T_lidar_camera.txt` | 역변환, 카메라의 라이다 기준 자세 |
| `static_tf.launch` | parent=라이다, child=왼쪽 optical인 ROS1 tf2 설정 |
| `overlays/*_initial.png` | 초기 파라미터 투영 |
| `overlays/*_result.png` | 최종 파라미터 투영 |
| `report.json` | 대응점 수, 투영점 수, 초기값 대비 변화, 검사 결과 |
| `manifest.json` | 원본 bag–장면 번호 매핑, 시각, 카메라 정보 |
| `effective_config.yaml` | 실제 사용한 설정 |
| `backend.log`, `ros_logs/` | HKU/ROS 실행 로그 |
| `backend_extrinsic.txt` | HKU가 출력한 rectified 카메라 기준 원본 행렬 |
| `failure.json` | 실패한 경우 이유와 traceback |

모든 거리는 m, quaternion은 **qx, qy, qz, qw** 순서입니다.
CameraInfo.R이 단위행렬이 아닌 경우 backend 결과를 원래 optical 좌표로 되돌려 저장합니다.
보정된 영상에 투영할 때는 `T_rectified_camera_lidar`를 사용합니다.

상태와 종료 코드:

- `estimated_unvalidated` / 0: 최적화 출력과 기본 검사를 통과한 **미검증 추정값**.
- `needs_review` / 2: 초기값 대비 변화나 영상 겹침 검사에서 문제가 발견됨. 결과는 검토용으로 저장.
- 실패 / 1: 대응점 부족, 잘못된 행렬, 입력 오류, 프로세스 실패 등. 정상 결과로 내보내지 않음.
- 사용자 중단 / 130.

투영점이 많다는 것만으로 정확하다고 판단할 수 없습니다.
학습에 사용한 장면의 투영 이미지는 정량 정확도 검증이나 독립 검증을 대체하지 않습니다.
별도로 촬영한 장면의 경계 정합을 확인하고, 필요하면 다른 장면 묶음으로 다시 추정해 비교하세요.

확인한 결과만 다음처럼 게시합니다. 기존 ZED TF 트리에서 같은 child 프레임에
이미 parent가 있다면 로봇의 URDF/TF 구성을 먼저 정리해야 합니다.

```bash
roslaunch ~/calibration_results/run_01/static_tf.launch
```

## 7. 검증 범위와 개발 테스트

**실제 센서 캘리브레이션 정확도와 Ubuntu catkin/HKU 통합 실행은 아직 검증되지 않았습니다.**
테스트는 ROS1 형식으로 직렬화한 합성 bag의 추출, 좌표축/역변환, 시간·움직임 거절,
결과/TF 저장을 검증합니다. 전체 파이프라인 테스트의 최적화 단계는 대역을 사용합니다.

Noetic 런타임은 기본 ROS `rosbag`을 사용합니다.
ROS가 없는 Python 3.10+ 환경에서는 선택 의존성 `rosbags>=0.11,<0.12`로
추출 및 합성 테스트만 실행할 수 있습니다.

```bash
# 개발용 Python 3.10+ 가상환경에서만 실행. Noetic의 cv2를 pip로 교체하지 마세요.
python -m pip install -r requirements-dev.txt
python -m pytest -q
```

구현 인터페이스는 HKU의
[multi calibration 소스](https://github.com/hku-mars/livox_camera_calib/blob/master/src/lidar_camera_multi_calib.cpp)와
[설정 파일](https://github.com/hku-mars/livox_camera_calib/blob/master/config/multi_calib.yaml)을 참고했습니다.
설치된 HKU 버전에서 로그 형식이 바뀌면 대응점 검사가 실패할 수 있으며,
그 경우 `backend.log`를 확인해 해당 버전에 맞게 조정해야 합니다.


