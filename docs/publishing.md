# GitHub 업로드

저장소 루트에는 README.md, package.xml, CMakeLists.txt와 Python 패키지가 위치합니다.
로컬 작업 폴더에는 가상환경이나 개인 참고 자료가 있을 수 있으므로
다음 명령으로 **공개 파일만 포함한 ZIP**을 만드세요.

```bash
python scripts/package_release.py
```

출력: `dist/zed_mid360_calibration-github.zip`.
압축을 풀면 저장소 루트용 파일들이 나옵니다. ZIP 자체를 소스 대신 올리지 말고
압축을 푼 폴더의 내용(숨김 파일 포함)을 올립니다.

Git이 있는 환경에서 새 빈 GitHub 저장소에 게시하는 예시:

```bash
# 압축을 푼 소스 루트에서 실행
git init -b main
git add .
git status --short
git commit -m "Add ROS1 ZED2 MID-360 calibration package"
git remote add origin https://github.com/OWNER/REPOSITORY.git
git push -u origin main
```

OWNER/REPOSITORY는 실제 저장소로 바꿉니다.
이미 파일이 있는 저장소라면 먼저 clone한 뒤 파일을 복사해 변경 내용을 검토합니다.
기존 원격 이력을 덮어쓰기 위한 force push는 필요하지 않습니다.

최초 공개 시 package.xml의 maintainer 이름·이메일을 실제 유지관리자 정보로 바꾸세요.
현재 값은 템플릿입니다. 이 패키지의 선언된 라이선스는 MIT이며,
외부 HKU 프로젝트는 해당 저장소의 라이선스를 따릅니다.

게시 후 Actions 탭에서 합성 테스트 실행을 확인합니다.
README의 실제 센서/ROS 통합 미검증 상태는 해당 검증을 수행하기 전까지 유지하세요.

