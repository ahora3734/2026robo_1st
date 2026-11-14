# 2026 시각장애인용 자율주행 보조 카트 프로젝트

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
