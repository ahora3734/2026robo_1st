# 2026 시각장애인용 자율주행 보조 카트 프로젝트
ㅇ
본 프로젝트는 **NVIDIA Jetson Orin Nano Super (8GB)**와 **ESP32**를 활용한 시각장애인용 4륜 메카넘 자율주행 보조 카트 개발 워크스페이스입니다.

---

## 📁 디렉터리 구조

```text
2026_ROBO_1/
├── BLUEPRINT.md                 # 🌟 [필독] 전체 시스템 작업설계도 및 규격서
├── README.md                    # 프로젝트 퀵스타트 안내
├── tools/                       # udev 규칙 및 하드웨어 점검 스크립트
│   └── 99-cart-sensors.rules    # USB 센서 포트 고정 udev 룰
├── esp32_firmware/              # ESP32 4륜 메카넘 모터 제어 펌웨어 (C++/Arduino)
└── cart_ws/                     # Jetson Orin Nano ROS 2 Humble 워크스페이스
    └── src/
        ├── cart_drivers/        # GPS, IMU, LiDAR, Camera 드라이버 패키지
        ├── cart_navigation/     # TMAP 보행자 API 연동 및 전역 경로 계획
        ├── cart_vision/         # YOLO 비전 객체 인식 패키지
        ├── cart_control/        # 장애물 회피 및 주행 제어기 (Local Planner)
        ├── cart_tts/            # 시각장애인 음성 안내 (TTS) 패키지
        └── cart_bringup/        # 전체 시스템 원클릭 통합 Launch 패키지
```

---

## 🚀 빠른 시작 (Quick Start)

1. **설계도 열기**: [BLUEPRINT.md](file:///home/pogo/2026_ROBO_1/README.md)를 열어 전체 아키텍처와 핀맵, ROS 2 토픽 규격을 확인합니다.
2. **안티그래비티 IDE와 협업**:
   - 구현하고자 하는 모듈(예: ESP32 모터 제어, TMAP API 연동 노드 등)에 대해 [BLUEPRINT.md](file:///home/pogo/2026_ROBO_1/README.md)의 명세를 참조하여 Antigravity AI에게 질문하고 코드를 함께 작성합니다.
3. **단계별 검증**:
   - 하드웨어 단품 테스트 ➔ ESP32 모터 구동 ➔ 센서 드라이버 ➔ 전역/로컬 내비게이션 ➔ 음성 피드백 통합 순으로 개발을 진행합니다.

---

## 🤖 ROS 2 & ESP32(micro-ROS) 동작 원리 (입문자 가이드)

ROS 2가 처음이라도 **"ESP32는 `/cmd_vel`만 듣고 움직인다"**는 한 가지만 이해하면 전체 구조가 쉽게 보입니다.

### 1. 토픽(Topic)과 `/cmd_vel`이 뭔가요?
- **토픽(Topic)**은 프로그램들이 서로 데이터를 주고받는 **공용 게시판(또는 라디오 주파수)**입니다.
  - 명령을 올리는 쪽을 **발행자(Publisher)**, 명령을 받아서 실행하는 쪽을 **구독자(Subscriber)**라고 합니다.
- **`/cmd_vel`**은 ROS 2에서 로봇의 이동 속도를 전달할 때 전 세계 공통으로 쓰는 토픽 이름입니다. (`geometry_msgs/msg/Twist` 형식)
  - `linear.x`: 앞(+) / 뒤(-) 속도 (m/s)
  - `linear.y`: 왼쪽(+) / 오른쪽(-) 게걸음 속도 (m/s, 메카넘 휠 전용)
  - `angular.z`: 좌회전(+) / 우회전(-) 회전 속도 (rad/s)

### 2. 왜 키보드(`wsadqe`)와 자율주행(Nav2)이 똑같이 작동하나요?
```text
  [키보드 조종 (wsadqe)] ──┐
                           ├──▶  /cmd_vel 토픽  ──▶ [micro-ROS Agent] ──(USB)──▶ [ESP32 모터 제어]
  [자율주행 (Nav2)]      ──┘
```
- **ESP32 펌웨어**는 "누가 보냈는지(사람 키보드인지, AI 자율주행인지)" 신경 쓰지 않고, 오직 **`/cmd_vel` 토픽에 들어오는 숫자**만 읽어서 4개 바퀴를 계산해 굴립니다.
- 따라서 Jetson에서 **키보드 프로그램**을 켜든 **Nav2 자율주행**을 켜든, 둘 다 `/cmd_vel`로 명령을 보내기 때문에 **ESP32 코드를 바꿀 필요 없이 동일하게 움직입니다.**

| 키보드 입력 | 동작 | `/cmd_vel`에 전달되는 값 예시 |
|---|---|---|
| `W` / `S` | 전진 / 후진 | `linear.x` (`+` / `-`) |
| `A` / `D` | 좌 / 우 평행이동(게걸음) | `linear.y` (`+` / `-`) |
| `Q` / `E` | 제자리 좌회전 / 우회전 | `angular.z` (`+` / `-`) |

> **참고:** 키보드 조종과 Nav2를 동시에 켜면 서로 다른 `/cmd_vel` 명령이 섞여 로봇이 흔들릴 수 있으므로, 한 번에 하나만 실행하거나 Jetson에서 우선순위(`twist_mux`)를 두어 제어합니다.

### 3. ESP32가 Jetson과 통신할 때 꼭 필요한 것 (`micro-ROS Agent`)
- ESP32와 Jetson을 USB 케이블로 꽂은 뒤, Jetson 터미널에서 **통신 다리 역할(`micro_ros_agent`)**을 실행해야 `/cmd_vel`과 센서 데이터(`/wheel/odom`, `/ultrasonic/*`)가 오갑니다.
  ```bash
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
  ```
- 연결에 성공하면 ESP32 보드의 파란색 LED(`GPIO2`)가 깜빡임에서 **상시 점등**으로 바뀝니다. (만약 USB 연결이 끊기거나 0.5초 이상 `/cmd_vel` 명령이 없으면 ESP32가 안전을 위해 자동으로 모터를 정지합니다.)
