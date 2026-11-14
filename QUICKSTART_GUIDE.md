# 🚀 2026 자율주행 스마트 카트 개발 퀵스타트 가이드

본 문서는 사용자가 요청한 **4단계 개발 프로세스 (부품점검 ➔ 수동 주행 ➔ 자율주행 ➔ 4분할 실시간 대시보드)**의 실행 방법과 파일 구성을 안내합니다.

---

## 📂 파일 및 패키지 구성 요약

```text
2026_ROBO_1/
├── BLUEPRINT.md                        # 전체 하드웨어/소프트웨어 상세 작업설계도
├── esp32_firmware/
│   └── esp32_mecanum_controller/      # ESP32 메카넘 역기구학 & Cytron MD30C 펌웨어 (.ino)
├── tools/                              # [1단계 & 2단계] 부품별 단품 점검 도구
│   ├── check_all_devices.py            # 하드웨어 종합 연결 & udev 상태 진단기
│   ├── test_motor_esp32.py             # W,A,S,D 무선키보드 모터 구동 & 엔코더 테스트
│   ├── test_lidar.py                   # D500 LiDAR 360도 거리 데이터 수신 테스트
│   ├── test_gps.py                     # ZED-F9P GNSS 위도/경도/RTK 상태 테스트
│   ├── test_camera.py                  # C920 웹캠 실시간 영상 스트림 테스트
│   └── test_imu.py                     # WT901 9축 IMU 자세/방위각 테스트
└── cart_ws/src/cart_dashboard/         # [3단계 & 4단계] 4분할 실시간 GUI & 주행 시스템
    ├── cart_dashboard_gui.py           # 🌟 메인 4분할 대시보드 & 수동/자율 통합 제어기
    ├── tmap_engine.py                  # TMAP 보행자 API 연동 및 경로 생성 엔진
    ├── lidar_engine.py                 # LiDAR 데이터 파서 및 0.6m/1.5m 안전 평가
    └── vision_engine.py                # 카메라 스트림 및 YOLO AI 객체 검출 엔진
```

---

## 🛠️ 단계별 실행 및 개발 가이드

### 1️⃣ [1단계] 각 부품별 작동상태 확인 (단품 진단)

Jetson 터미널에서 아래 스크립트를 실행하여 각 하드웨어의 연결과 통신을 즉시 검증합니다.

```bash
# 1. 전체 센서 및 디바이스 종합 진단 (포트 및 권한 점검)
python3 tools/check_all_devices.py

# 2. 개별 센서 단품 정밀 테스트
python3 tools/test_lidar.py      # D500 라이다 거리 측정 및 장애물 감지
python3 tools/test_gps.py        # ZED-F9P GPS 위도/경도/위성수 확인
python3 tools/test_camera.py     # C920 웹캠 640x480 영상 확인 (창에서 'q' 누르면 종료)
python3 tools/test_imu.py        # WT901 9축 IMU 자세(Roll, Pitch, Yaw) 확인
```

---

### 2️⃣ [2단계] 무선 키보드 수동 주행 테스트 (CLI 모드)

ESP32에 [esp32_mecanum_controller.ino](file:///c:/Users/포항고/Desktop/2026_ROBO_1/esp32_firmware/esp32_mecanum_controller/esp32_mecanum_controller.ino) 펌웨어를 업로드한 후 실행합니다.

```bash
python3 tools/test_motor_esp32.py
```

* **조작 키 안내**:
  * `W` : 전진 (+Vx)
  * `S` : 후진 (-Vx)
  * `A` : 좌측 게걸음 (+Vy)
  * `D` : 우측 게걸음 (-Vy)
  * `Q` / `E` : 좌회전 / 우회전 ($\pm\omega_z$)
  * `SPACE` : **즉각 긴급 정지 (E-STOP)**

---

### 3️⃣ & 4️⃣ [3, 4단계] 4분할 실시간 GUI 대시보드 & 수동/자율 통합 주행

Jetson에 연결된 모니터에 **4분할 HUD 대시보드**를 띄우고, 키보드로 수동 주행하거나 `M` 키를 눌러 자율주행 모드로 전환합니다.

```bash
python3 cart_ws/src/cart_dashboard/cart_dashboard_gui.py
```

### 🖥️ 4분할 화면 구성
1. **[좌상단 / 1사분면] TMAP 지도 & 내비게이션**:
   - 실시간 현재 GPS 위치 마커, 목적지까지의 안내 경로(Polyline), TMAP 보행자 안내 문구
2. **[우상단 / 2사분면] LiDAR 센싱 레이더 GUI**:
   - 360° 다크 테마 레이더 뷰, 360도 점군(Point Cloud), 1.5m 감속구역(노란색), 0.6m 긴급정지구역(빨간색), 최단거리 OSD
3. **[좌하단 / 3사분면] 속도계, 텔레메트리 & 모드 제어**:
   - 현재 속도 ($m/s, km/h$), 벡터 $(v_x, v_y, \omega_z)$, 배터리 전압, `[MANUAL]` / `[AUTO]` / `[E-STOP]` 상태 배너
4. **[우하단 / 4사분면] AI 비전 & 카메라 스트림**:
   - C920 웹캠 실시간 영상, YOLO AI 객체 검출(보행자, 신호등, 점자블록) 바운딩 박스 오버레이

### 🎮 대시보드 단축키 & 제어
* **`M` 키 (또는 모드 전환 버튼 클릭)**: `MANUAL (수동)` ↔ `AUTO (자율)` 모드 실시간 전환
* **`W, A, S, D, Q, E` 키**: 수동 모드 시 실시간 방향/속도 제어
* **`SPACE` 키 (또는 긴급정지 버튼 클릭)**: 어떤 상태에서든 즉시 **비상 정지 (E-STOP)**
