
# Husky Adopter

## 개요

`husky_adopter`는 ROS 2 패키지로, Clearpath의 Husky 플랫폼에 대해 MAVLInk 프로토콜 기반 웹 관제를 위한 어댑터 기능을 포함하고 있습니다.

## 주요 기능

- huskydatahandlernode
  - Husky 내 GPS, IMU 등 ROS2 데이터의 MAVLink 프로토콜을 통한 서버로의 전송
- gcsdatahandlernode
  - 웹 관제 시스템에서 MAVLink로 보낸 메시지를 ROS로 변환하여 로봇 제어

## 디렉토리 구조

```
husky_adopter-master/
├── .gitignore
├── husky_adopter/
│   ├── husky_adopter/      # 패키지 소스 코드
│   ├── package.xml         # ROS 패키지 매니페스트
│   ├── resource/           # 패키지 설정 리소스
│   ├── setup.cfg           # Python 패키지 구성 파일
│   ├── setup.py            # 패키지 설치 스크립트
│   ├── test/               # 테스트 케이스 및 유틸리티
```

## 설치 방법

1. 레포지토리 클론:
   ```bash
   git clone <repository-url>
   cd husky_adopter-master
   ```
2. 패키지 빌드 및 설치:
   ```bash
   colcon build
   source install/setup.bash
   ```

## 의존성

다음 ROS 2 의존성이 설치되어 있는지 확인하세요:

- `rclpy`
- `sensor_msgs`
- `mavros_msgs`
- `builtin_interfaces`

누락된 의존성은 `rosdep`을 사용해 설치할 수 있습니다:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 사용법

패키지 실행 (예제 명령어):
```bash
ros2 run husky_adopter huskydatahandlernode &
ros2 run husky_adopter gcsdatahandlernode &
```

## 기여 방법

1. 레포지토리를 포크합니다.
2. 새 기능 브랜치를 생성합니다:
   ```bash
   git checkout -b feature/your-feature
   ```
3. Pull Request를 제출합니다.
