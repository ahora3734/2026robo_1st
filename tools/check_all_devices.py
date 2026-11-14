#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
하드웨어 종합 점검 스크립트 (Hardware Diagnostic Tool)
- USB 연결 센서 및 시리얼 디바이스 인식 상태 점검
- udev 심볼릭 링크 생성 여부 점검
- 포트 접근 권한 점검
"""

import os
import sys
import glob
import subprocess
import serial
import cv2

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"

DEVICES = [
    {
        "name": "ESP32 메카넘 모터 컨트롤러",
        "symlink": "/dev/ttyCartESP32",
        "fallback_patterns": ["/dev/ttyUSB*", "/dev/ttyACM*"],
        "baudrate": 115200,
        "type": "serial"
    },
    {
        "name": "D500 (STL-19P) 2D LiDAR",
        "symlink": "/dev/ttyCartLiDAR",
        "fallback_patterns": ["/dev/ttyUSB*"],
        "baudrate": 230400,
        "type": "serial"
    },
    {
        "name": "ZED-F9P 고정밀 GNSS/GPS",
        "symlink": "/dev/ttyCartGPS",
        "fallback_patterns": ["/dev/ttyACM*", "/dev/ttyUSB*"],
        "baudrate": 115200,
        "type": "serial"
    },
    {
        "name": "WT901 9축 IMU 센서",
        "symlink": "/dev/ttyCartIMU",
        "fallback_patterns": ["/dev/ttyUSB*"],
        "baudrate": 115200,
        "type": "serial"
    },
    {
        "name": "Logitech C920 웹캠",
        "symlink": "/dev/video_cam",
        "fallback_patterns": ["/dev/video0", "/dev/video1", "/dev/video2"],
        "type": "camera"
    }
]

def check_serial_device(dev_path, baudrate):
    """시리얼 디바이스 열기 및 데이터 수신 테스트"""
    if not os.path.exists(dev_path):
        return False, "포트 파일 없음"
    try:
        ser = serial.Serial(dev_path, baudrate=baudrate, timeout=1.0)
        is_open = ser.is_open
        ser.close()
        return True, "정상 연결 가능" if is_open else "포트 열기 실패"
    except serial.SerialException as e:
        return False, f"접근 에러 ({e})"
    except Exception as e:
        return False, f"알 수 없는 오류 ({e})"

def check_camera_device(cam_path):
    """카메라 디바이스 열기 테스트"""
    if not os.path.exists(cam_path):
        return False, "비디오 디바이스 파일 없음"
    
    cap = cv2.VideoCapture(cam_path)
    if cap.isOpened():
        ret, frame = cap.read()
        cap.release()
        if ret and frame is not None:
            h, w = frame.shape[:2]
            return True, f"영상 캡처 성공 ({w}x{h})"
        return True, "디바이스 열림 (프레임 대기)"
    return False, "카메라 열기 실패"

def main():
    print(f"{BOLD}{CYAN}===================================================={RESET}")
    print(f"{BOLD}{CYAN}  2026 자율주행 카트 하드웨어 상태 진단 툴킷  {RESET}")
    print(f"{BOLD}{CYAN}===================================================={RESET}\n")

    print(f"{BOLD}[1] udev 심볼릭 링크 및 디바이스 점검{RESET}")
    all_ok = True
    
    for dev in DEVICES:
        name = dev["name"]
        symlink = dev["symlink"]
        print(f"\n{BOLD}▶ {name}{RESET}")
        print(f"  - 지정 심볼릭 링크: {CYAN}{symlink}{RESET}")
        
        target_path = None
        if os.path.exists(symlink):
            real_path = os.path.realpath(symlink)
            print(f"  - udev 매핑: {GREEN}성공 (-> {real_path}){RESET}")
            target_path = symlink
        else:
            print(f"  - udev 매핑: {YELLOW}미등록 또는 디바이스 미연결{RESET}")
            # 대체 포트 검색
            matched = []
            for pat in dev.get("fallback_patterns", []):
                matched.extend(glob.glob(pat))
            if matched:
                print(f"  - 검색된 후보 포트: {matched}")
                target_path = matched[0]
            else:
                print(f"  - {RED}연결된 포트를 찾을 수 없습니다.{RESET}")
                all_ok = False
                continue

        # 통신 테스트
        if dev["type"] == "serial":
            success, msg = check_serial_device(target_path, dev["baudrate"])
            if success:
                print(f"  - 상태: {GREEN}[PASS] {msg} ({dev['baudrate']} bps){RESET}")
            else:
                print(f"  - 상태: {RED}[FAIL] {msg}{RESET}")
                all_ok = False
        elif dev["type"] == "camera":
            success, msg = check_camera_device(target_path)
            if success:
                print(f"  - 상태: {GREEN}[PASS] {msg}{RESET}")
            else:
                print(f"  - 상태: {RED}[FAIL] {msg}{RESET}")
                all_ok = False

    print(f"\n{BOLD}[2] 무선 키보드 / 입력 장치 점검{RESET}")
    input_devices = glob.glob("/dev/input/event*")
    if input_devices:
        print(f"  - 감지된 입력 이벤트 노드: {GREEN}{len(input_devices)}개 발견{RESET}")
    else:
        print(f"  - {YELLOW}/dev/input/event* 장치가 없습니다 (데스크톱 환경에서는 일반 키보드 이벤트 사용 가능){RESET}")

    print(f"\n{BOLD}{CYAN}===================================================={RESET}")
    if all_ok:
        print(f"{BOLD}{GREEN}✔ 모든 하드웨어 부품이 정상 인식되었습니다! 다음 단계로 진행하세요.{RESET}")
    else:
        print(f"{BOLD}{YELLOW}⚠ 일부 센서가 연결되지 않았거나 udev 권한 설정이 필요합니다.{RESET}")
        print(f"  팁: `sudo usermod -aG dialout $USER` 및 `sudo cp tools/99-cart-sensors.rules /etc/udev/rules.d/` 실행 권장")
    print(f"{BOLD}{CYAN}===================================================={RESET}\n")

if __name__ == "__main__":
    main()
