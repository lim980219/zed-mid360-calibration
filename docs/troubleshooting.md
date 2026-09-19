# 문제 해결

| 현상 | 확인할 내용 |
|---|---|
| `Set a finite number for camera_mount...` | 설정 파일의 세 위치값 null을 실측 m 값으로 교체 |
| 토픽 없음 | `--inspect`로 실제 이름·타입 확인 후 topics 수정 |
| CameraInfo 없음/불일치 | 왼쪽 영상과 일치하는 CameraInfo를 같은 bag에 기록 |
| `Scene is moving` | bag 중앙 정지 구간을 확인. 현재 자동 구간 재탐색은 지원하지 않음 |
| 특징점 부족 | 경계와 텍스처가 보이는 정지 장면 사용 |
| `time gap` | 두 센서 header.stamp의 clock domain과 알려진 시간 오프셋 확인 |
| 포인트 수 부족/초과 | 거리 필터·누적 시간·voxel 크기 확인 |
| HKU 실행 파일 없음 | HKU 빌드 및 devel/setup.bash source 확인 |
| `xvfb-run` 없음 | `sudo apt install xvfb xauth` |
| 대응점 부족 | 초기 설치 위치·각도, 다방향 입체 경계, edge 설정 확인 |
| `needs_review` | report.json과 초기/최종 투영 이미지를 비교 |
| 출력 폴더 이미 존재 | 새 --output 경로 사용 |
| timeout/프로세스 실패 | backend.log, ros_logs, failure.json 확인 |

설정 예외를 해결하기 위해 임계값부터 크게 완화하면 잘못된 데이터가 통과할 수 있습니다.
먼저 센서 프레임, 시간, 입력 영상과 정지 조건을 확인하세요.

