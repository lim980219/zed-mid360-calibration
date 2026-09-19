# 개발 및 테스트

런타임은 Ubuntu 20.04 / ROS1 Noetic / Python 3.8입니다.
합성 테스트는 ROS가 없는 Python 3.10+ 환경에서 rosbags를 사용합니다.

```bash
python -m venv .venv
# Linux/macOS
source .venv/bin/activate
# Windows PowerShell: .venv\Scripts\Activate.ps1
python -m pip install -r requirements-dev.txt
python -m pytest -q
```

테스트는 PointCloud2/CustomMsg bag 읽기, 영상·시간 검사, 좌표 변환,
최적화 프로세스 감시, 결과 파일과 TF 생성을 다룹니다.
최적화 테스트는 대역을 사용하므로 HKU의 Ceres 실행이나 실제 센서 정확도를 검증하지 않습니다.

변경 시 지킬 사항:

- 런타임 모듈은 Python 3.8 문법과 Noetic apt 패키지에 맞춥니다.
- 설치 위치 기본값은 null을 유지하고 실측값을 입력받습니다.
- 양방향 변환식, quaternion 순서, 광학/rectified 프레임 구분을 유지합니다.
- 실측 bag이나 센서 영상, 로컬 경로와 개인 설정은 커밋하지 않습니다.
- 기능 변경에 해당하는 테스트와 사용자 문서를 함께 갱신합니다.

GitHub Actions의 합성 테스트 통과와 실제 ROS 통합 검증은 별개입니다.
실제 장비 검증을 보고할 때는 ROS·HKU 버전, 토픽 타입, 해상도,
정지 장면 구성 및 검증 방법을 함께 기록하세요.

