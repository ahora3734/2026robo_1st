# 🧭 시각장애인용 자율주행 메카넘 카트 프로젝트 작업설계도 (BLUEPRINT)

> **프로젝트명**: 시각장애인을 위한 고정밀 GNSS 및 AI 비전 기반 4륜 메카넘 자율주행 보조 카트  
> **개발 플랫폼**: NVIDIA Jetson Orin Nano Super DevKit (8GB) & ESP32  
> **미들웨어**: ROS 2 Humble Hawksbill (Ubuntu 22.04 LTS)  
> **버전**: v1.0.0

---

## 1. 프로젝트 개요 (Project Overview)

본 프로젝트는 시각장애인의 안전하고 편리한 보행 이동을 보조하는 **자율주행 스마트 카트**를 개발하는 것을 목표로 합니다.
- **두뇌 (Jetson Orin Nano)**: TMAP 보행자 API와 고정밀 GPS(ZED-F9P)를 통한 전역 경로 안내, D500 라이다 및 로지텍 웹캠(YOLO)을 통한 장애물 인지 및 실시간 국소 경로 계획, 사용자 음성 피드백(TTS)을 담당합니다.
- **신체 (ESP32)**: 4륜 메카넘 휠의 역기구학 연산, Cytron MD30C 모터 드라이버 및 엔코더 기반 정밀 PID 속도 제어, 하드웨어 안전 Fail-Safe를 수행합니다.

```mermaid
flowchart TB
    subgraph SENSORS["센서 및 입력 계통 (USB 연결)"]
        GPS["ZED-F9P GNSS<br>(위도/경도 RTK)"]
        IMU["WT901 IMU<br>(방위각/자세 9축)"]
        LIDAR["D500 (STL-19P) LiDAR<br>(2D 360도 거리측정)"]
        CAM["Logitech C920 웹캠<br>(RGB 영상)"]
    end

    subgraph JETSON["NVIDIA Jetson Orin Nano (ROS 2 Humble)"]
        TMAP_NODE["TMAP 내비게이션 노드<br>(SK Open API 보행자 경로)"]
        YOLO_NODE["YOLO 비전 인식 노드<br>(보행자, 신호등, 점자블록)"]
        FUSION_NODE["센서 퓨전 & 전역/로컬 플래너<br>(Costmap / 장애물 회피)"]
        TTS_NODE["TTS 음성 안내 노드<br>(스피커 음성 안내)"]
        BRIDGE_NODE["ESP32 통신 브릿지 노드<br>(cmd_vel 송신 / odom 수신)"]
    end

    subgraph ACTUATION["하위 구동 계통 (ESP32 & 하드웨어)"]
        ESP32["ESP32 마이크로컨트롤러<br>(메카넘 역기구학 & PID 제어)"]
        DRIVERS["Cytron MD30C x 4개<br>(PWM / DIR 제어)"]
        MOTORS["PG42-4266 모터 (1/49) x 4개<br>+ 엔코더 피드백"]
        WHEELS["4륜 메카넘 휠<br>(전/후/좌/우/회전 전방향 주행)"]
    end

    GPS -->|USB NMEA/UBX| TMAP_NODE
    IMU -->|USB Serial| FUSION_NODE
    LIDAR -->|USB LaserScan| FUSION_NODE
    CAM -->|USB V4L2| YOLO_NODE

    TMAP_NODE --> FUSION_NODE
    YOLO_NODE --> FUSION_NODE
    FUSION_NODE -->|안내 메시지| TTS_NODE
    FUSION_NODE -->|geometry_msgs/Twist| BRIDGE_NODE

    BRIDGE_NODE <-->|USB Serial (115200 bps)| ESP32
    ESP32 --> DRIVERS --> MOTORS --> WHEELS
    MOTORS -.->|엔코더 펄스| ESP32
```

---

## 2. 하드웨어 사양 및 핀맵 (Hardware Specifications & Pinout)

### 2.1 하드웨어 상세 목록

| 부품 구분 | 모델명 | 주요 스펙 및 특징 | 연결 인터페이스 |
| :--- | :--- | :--- | :--- |
| **메인 프로세서** | Jetson Orin Nano Super (8GB) | 40 TOPS AI 연산, Ubuntu 22.04 + ROS2 Humble | - |
| **MCU** | ESP32 DevKit V1 (38Pin/30Pin) | Dual Core 240MHz, 4채널 엔코더 인터럽트 처리 | USB (to Jetson) |
| **라이다 (LiDAR)** | D500 (STL-19P) | 2D 360° 레이저 스캐너, 측정 반경 12m, 10Hz | USB to Serial |
| **고정밀 GPS** | u-blox ZED-F9P-01B | Multi-band RTK GNSS (cm급 정밀도 지원) | USB Type-C |
| **IMU 센서** | WitMotion WT901 | 9축 자세 센서 (Roll, Pitch, Yaw, 각속도, 가속도) | USB to UART |
| **카메라** | Logitech C920 PRO HD | 1080p 30fps, 광각 시야각, 자동 초점 | USB 3.0 |
| **모터 드라이버** | Cytron MD30C (4개) | 연속 30A (피크 80A), 5V~30V 입력, PWM/DIR 제어 | ESP32 GPIO |
| **모터** | Motorbank PG42-4266-1270NE (4개) | DC 12V 기어드 모터, 감속비 1/49, 듀얼 엔코더 | MD30C & ESP32 |
| **휠** | 4륜 메카넘 휠 (Mecanum) | 전방향(Holonomic) 이동 (전진, 횡이동, 제자리 회전) | 모터 축 직결 |
| **전원** | 12V 대용량 파워뱅크 / Li-Po | Jetson (19V/PD or 12V 5A), 모터 (12V) 전원 공급 | 공통 GND 필수 |

---

### 2.2 ESP32 ↔ 모터 드라이버(Cytron MD30C) 및 엔코더 핀맵

> [!IMPORTANT]
> **모터 배치 기준**: 카트 위에서 내려다보았을 때
> - **FL (Front-Left, 앞-왼쪽)** / **FR (Front-Right, 앞-오른쪽)**
> - **RL (Rear-Left, 뒤-왼쪽)** / **RR (Rear-Right, 뒤-오른쪽)**

| 모터 위치 | 드라이버 PWM 핀 | 드라이버 DIR 핀 | 엔코더 A상 (인터럽트) | 엔코더 B상 (인터럽트) |
| :---: | :---: | :---: | :---: | :---: |
| **FL (앞-왼쪽)** | `GPIO 25` | `GPIO 26` | `GPIO 34` | `GPIO 35` |
| **FR (앞-오른쪽)** | `GPIO 27` | `GPIO 14` | `GPIO 36` (VP) | `GPIO 39` (VN) |
| **RL (뒤-왼쪽)** | `GPIO 12` | `GPIO 13` | `GPIO 32` | `GPIO 33` |
| **RR (뒤-오른쪽)** | `GPIO 15` | `GPIO 2` | `GPIO 18` | `GPIO 19` |

> [!WARNING]
> ESP32의 `GPIO 34, 35, 36, 39`는 입력 전용(Input Only) 핀이므로 반드시 **엔코더 입력 핀**으로만 사용해야 하며, 풀업 저항 설정에 유의해야 합니다.

---

### 2.3 Jetson Orin Nano USB 포트 고정 규칙 (`udev` Rules)

센서들이 모두 USB로 연결되므로 포트 순서가 바뀌지 않도록 `/etc/udev/rules.d/99-cart-sensors.rules`를 설정합니다.

| 센서 / 기기 | 벤더/기기 ID (예시) | 심볼릭 링크 이름 | 통신 속도 (Baudrate) |
| :--- | :--- | :--- | :--- |
| **ESP32 MCU** | CP2102 / CH340 (`0x10c4:0xea60`) | `/dev/ttyCartESP32` | 115200 bps |
| **WT901 IMU** | CH340 (`0x1a86:0x7523`) | `/dev/ttyCartIMU` | 115200 bps (또는 9600) |
| **ZED-F9P GPS** | u-blox AG (`0x1546:0x01a9`) | `/dev/ttyCartGPS` | 115200 / 38400 bps |
| **D500 LiDAR** | CP2102 (`0x10c4:0xea60`) | `/dev/ttyCartLiDAR` | 230400 bps |
| **C920 웹캠** | Logitech (`0x046d:0x082d`) | `/dev/video_cam` | V4L2 |

---

## 3. 메카넘 휠 역기구학 공식 (Mecanum Kinematics)

카트의 목표 선속도 $v_x$ (전진/후진), $v_y$ (좌/우 횡이동), 각속도 $\omega_z$ (회전)가 주어졌을 때 각 바퀴의 회전 각속도 ($\omega_{FL}, \omega_{FR}, \omega_{RL}, \omega_{RR}$) 계산 공식:

$$
\begin{cases}
\omega_{FL} = \frac{1}{R} \left( v_x - v_y - (L_x + L_y)\,\omega_z \right) \\
\omega_{FR} = \frac{1}{R} \left( v_x + v_y + (L_x + L_y)\,\omega_z \right) \\
\omega_{RL} = \frac{1}{R} \left( v_x + v_y - (L_x + L_y)\,\omega_z \right) \\
\omega_{RR} = \frac{1}{R} \left( v_x - v_y + (L_x + L_y)\,\omega_z \right)
\end{cases}
$$

* $R$: 메카넘 바퀴의 반지름 ($m$)
* $L_x$: 카트 중심축에서 바퀴 중심까지의 전후 거리 ($m$)
* $L_y$: 카트 중심축에서 바퀴 중심까지의 좌우 거리 ($m$)

---

## 4. Jetson ↔ ESP32 시리얼 통신 프로토콜 규격

### 4.1 Jetson ➔ ESP32 제어 명령 패킷 (10Hz ~ 20Hz)
형식: ASCII 기반 CSV 문자열 (줄바꿈 `\n` 종료)
```text
$CMD,vx,vy,wz,checksum\n
```
* **예시**: `$CMD,0.30,0.00,0.15,48\n` (전진 0.3m/s, 횡이동 0m/s, 반시계 회전 0.15rad/s)
* **비상 정지**: `$CMD,0.00,0.00,0.00,00\n` 또는 통신 단절 0.5초 경과 시 모터 강제 브레이크 (Watchdog)

### 4.2 ESP32 ➔ Jetson 오도메트리 피드백 패킷 (20Hz)
```text
$ODOM,enc_fl,enc_fr,enc_rl,enc_rr,battery_volt\n
```
* **예시**: `$ODOM,1240,1238,1242,1239,12.4\n` (각 모터 누적 엔코더 틱 카운트 및 배터리 전압)

---

## 5. ROS 2 소프트웨어 노드 아키텍처

```mermaid
graph LR
    subgraph Driver_Nodes["드라이버 노드"]
        N_GPS["gps_driver_node<br>(/dev/ttyCartGPS)"]
        N_IMU["imu_driver_node<br>(/dev/ttyCartIMU)"]
        N_LIDAR["lidar_driver_node<br>(/dev/ttyCartLiDAR)"]
        N_CAM["camera_driver_node<br>(/dev/video_cam)"]
    end

    subgraph Core_Nodes["인식 및 계획 노드"]
        N_TMAP["tmap_navigation_node<br>(SK API 연동)"]
        N_YOLO["yolo_vision_node<br>(YOLOv26/v8 TensorRT)"]
        N_PLANNER["cart_planner_node<br>(장애물 회피 & 경로 추종)"]
        N_TTS["tts_audio_node<br>(음성 안내)"]
    end

    subgraph Control_Nodes["하위 제어 노드"]
        N_BRIDGE["esp32_bridge_node<br>(/dev/ttyCartESP32)"]
    end

    N_GPS -->|/fix [NavSatFix]| N_TMAP
    N_IMU -->|/imu/data [Imu]| N_PLANNER
    N_LIDAR -->|/scan [LaserScan]| N_PLANNER
    N_CAM -->|/camera/image_raw [Image]| N_YOLO

    N_TMAP -->|/nav/global_path [Path]| N_PLANNER
    N_TMAP -->|/tts/guide_text [String]| N_TTS
    N_YOLO -->|/vision/detections [Detection2DArray]| N_PLANNER
    N_YOLO -->|/tts/warning_text [String]| N_TTS

    N_PLANNER -->|/cmd_vel [Twist]| N_BRIDGE
    N_BRIDGE -->|/wheel_odom [Odometry]| N_PLANNER
```

### 5.1 ROS 2 주요 토픽 명세

| 토픽 이름 | 메시지 타입 | 발행 주기 | 설명 |
| :--- | :--- | :---: | :--- |
| `/fix` | `sensor_msgs/msg/NavSatFix` | 5Hz | ZED-F9P 고정밀 위도/경도/고도 |
| `/imu/data` | `sensor_msgs/msg/Imu` | 50Hz | WT901 쿼터니언 자세, 각속도, 가속도 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 10Hz | D500 라이다 360도 거리 데이터 (0.15m ~ 12m) |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 30Hz | 로지텍 C920 RGB 프레임 |
| `/vision/detections` | `vision_msgs/msg/Detection2DArray` | 15Hz | YOLO 객체 인식 결과 (보행자, 신호등, 점자블록 등) |
| `/nav/target_waypoint` | `geometry_msgs/msg/PoseStamped` | 1Hz | TMAP 기반 현재 목표 보행자 웨이포인트 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 20Hz | 카트 최종 목표 선속도($v_x, v_y$) 및 각속도($\omega_z$) |
| `/tts/voice_cmd` | `std_msgs/msg/String` | 이벤트 | 시각장애인 스피커 음성 안내 텍스트 |

---

## 6. SK TMAP 보행자 길찾기 API 연동 가이드

* **엔드포인트 URL**: `https://apis.openapi.sk.com/tmap/routes/pedestrian?version=1`
* **인증 키 (AppKey)**: `Y1SXBnTMciaDlRMu3rsyU2TFuN3kXKvC6X0zGFwY`
* **요청 방식**: `POST` (JSON Payload)
* **주요 파라미터**:
  - `startX`, `startY`: 출발지 경도/위도 (ZED-F9P 현재 좌표)
  - `endX`, `endY`: 목적지 경도/위도
  - `reqCoordType`: `WGS84GEO`
  - `resCoordType`: `WGS84GEO`
  - `startName`: 출발지 명칭 (URL 인코딩)
  - `endName`: 도착지 명칭 (URL 인코딩)
* **응답 데이터 활용**:
  - `features[].geometry.coordinates`: 보행자 전역 경로 폴리라인 좌표 리스트
  - `features[].properties.description`: 음성 안내 문구 ("10m 앞 횡단보도를 건너세요" 등)

---

## 7. 학생 단계별 개발 로드맵 (Step-by-Step Roadmap)

```text
[Phase 1] 하드웨어 단품 점검 & udev 설정
   ↓
[Phase 2] ESP32 펌웨어 (메카넘 역기구학 + Cytron 모터 PID)
   ↓
[Phase 3] Jetson 센서 ROS 2 드라이버 노드 구현 (GPS, IMU, LiDAR, Camera)
   ↓
[Phase 4] TMAP 보행자 API 연동 및 글로벌 웨이포인트 추종
   ↓
[Phase 5] YOLO 비전 인식 + LiDAR 장애물 회피 로컬 플래너
   ↓
[Phase 6] 시각장애인 음성 피드백(TTS) 및 통합 자율주행 시험
```

### 📍 Phase 1: 하드웨어 단품 점검 및 udev 설정
- [ ] USB 포트별 디바이스 VID/PID 확인 (`lsusb`, `dmesg`)
- [ ] `/etc/udev/rules.d/99-cart-sensors.rules` 작성 및 심볼릭 링크 생성 확인
- [ ] 12V 배터리 전원 및 공통 접지(GND) 결선 확인

### 📍 Phase 2: ESP32 펌웨어 개발 (`esp32_firmware/`)
- [ ] Cytron MD30C 4채널 PWM/DIR 구동 테스트
- [ ] 4채널 엔코더 인터럽트 기반 속도 계측 및 방향 확인
- [ ] 메카넘 역기구학 알고리즘 적용 (전진, 후진, 좌우 게걸음, 회전)
- [ ] 속도 폐루프 PID 제어기 구현
- [ ] Jetson 시리얼 수신 파서 및 0.5초 통신 두절 시 자동 정지(Watchdog) 구현

### 📍 Phase 3: Jetson 센서 노드 개발 (`cart_ws/src/cart_drivers/`)
- [ ] `zed_f9p_gps_node.py`: NMEA 파싱 후 `/fix` 토픽 발행
- [ ] `wt901_imu_node.py`: WT901 시리얼 파싱 후 `/imu/data` 토픽 발행
- [ ] `d500_lidar_node`: D500 드라이버 실행 및 `/scan` 토픽 확인 (Rviz2 시각화)
- [ ] `c920_camera_node.py`: OpenCV V4L2 기반 `/camera/image_raw` 발행

### 📍 Phase 4: TMAP 보행자 경로 생성 노드 (`cart_ws/src/cart_navigation/`)
- [ ] `tmap_pedestrian_client.py`: SK Open API 호출 및 GeoJSON 응답 파싱
- [ ] GPS 현재 위치와 목표 웨이포인트 간의 거리 및 방위각($\theta$) 오차 계산
- [ ] `Pure Pursuit` 또는 비례 제어를 통한 목표 각속도/선속도 생성

### 📍 Phase 5: AI 비전(YOLO) 및 안전 회피 로컬 플래너 (`cart_ws/src/cart_vision/`, `cart_control/`)
- [ ] `yolo_detector_node.py`: C920 영상에서 보행자, 신호등(적색/녹색), 점자블록 감지
- [ ] `obstacle_avoidance_node.py`: LiDAR 전방 120° 부채꼴 영역 거리 감지
  - 1.5m 이내 장애물: 감속
  - 0.6m 이내 장애물: 즉각 정지 후 좌/우 회피 기동
- [ ] 비전 + 라이다 + TMAP 제어 명령의 우선순위 중재기(Arbiter) 구현

### 📍 Phase 6: 시각장애인 음성 피드백 & 전체 통합
- [ ] `tts_voice_node.py`: Python `gTTS` 또는 `pyttsx3`/`edge-tts` 기반 음성 합성
- [ ] 실시간 음성 안내 ("10m 앞 우회전입니다", "전방에 보행자가 있어 서행합니다")
- [ ] 전체 ROS 2 launch 파일 작성 및 실외 보행로 주행 테스트

---

## 8. 안티그래비티 IDE (Antigravity IDE) 학생 실습 활용 팁

학생들은 안티그래비티 IDE의 AI 페어 프로그래밍 기능을 활용하여 코드를 작성할 수 있습니다.

### 💡 추천 프롬프트 예시

1. **ESP32 메카넘 제어 펌웨어 작성 시:**
   > *"BLUEPRINT.md의 2.2절 핀맵과 3절 메카넘 역기구학 공식을 바탕으로, Cytron MD30C 4개를 구동하고 엔코더 속도 측정을 수행하는 ESP32 아두이노 C++ 코드를 작성해줘."*

2. **TMAP 보행자 API 연동 노드 작성 시:**
   > *"BLUEPRINT.md 6절의 SK Open API 명세와 AppKey를 사용하여, 현재 GPS 좌표와 목적지 좌표를 받아 보행자 경로 웨이포인트 리스트와 음성 안내 문구를 추출하는 ROS 2 Humble Python 노드를 작성해줘."*

3. **라이다 장애물 감지 노드 작성 시:**
   > *"ROS 2 `sensor_msgs/msg/LaserScan` 토픽을 구독하여 전방 ±45도 범위 내에 0.8m 이하의 장애물이 감지되면 긴급 정지 플래그를 발행하는 파이썬 노드를 작성해줘."*

---

## 9. 안전 수칙 및 주의사항 (Safety Rules)

1. **공통 접지(Common Ground)**: Jetson, ESP32, Cytron MD30C, 배터리의 모든 **GND(-) 선은 반드시 하나로 묶여 있어야** 통신 노이즈와 오작동을 방지할 수 있습니다.
2. **첫 구동 시 공중 부양 테스트**: 메카넘 휠이 바닥에 닿은 상태로 처음 모터를 돌리면 급발진 위험이 있습니다. 반드시 **카트 바퀴를 공중에 띄운 상태에서 전후/좌우/회전 방향을 확인**하세요.
3. **비상 정지 스위치(E-Stop)**: 배터리 전원 라인에 수동 비상 정지 킬 스위치를 직렬로 장착하여 비상시 즉각 전원을 차단할 수 있도록 하세요.
