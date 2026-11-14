#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WT901 9축 IMU 센서 단품 테스트 도구
- Roll, Pitch, Yaw 방위각 및 자이로 각속도 파싱
"""

import os
import sys
import time
import serial
import serial.tools.list_ports

PORT_DEFAULT = "/dev/ttyCartIMU"
BAUDRATE_DEFAULT = 115200

def test_imu():
    port = PORT_DEFAULT
    if not os.path.exists(port):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports:
            port = ports[0]
            print(f"[*] 지정 포트 없음 -> 대체 포트 사용: {port}")
        else:
            print("[!] 연결된 시리얼 포트가 없습니다.")
            return

    try:
        ser = serial.Serial(port, BAUDRATE_DEFAULT, timeout=1.0)
        print(f"[*] WT901 IMU 포트 열기 성공: {port} ({BAUDRATE_DEFAULT} bps)")
        print("[*] IMU 자세 데이터 수신 중... (Ctrl+C 로 종료)\n")

        while True:
            # WitMotion 0x55 헤더 검색
            b = ser.read(1)
            if len(b) == 0 or b[0] != 0x55:
                continue
            
            flag = ser.read(1)
            if len(flag) == 0:
                continue

            # 0x53: 각도 (Roll, Pitch, Yaw)
            if flag[0] == 0x53:
                data = ser.read(9)
                if len(data) == 9:
                    roll = ((data[1] << 8) | data[0]) / 32768.0 * 180.0
                    pitch = ((data[3] << 8) | data[2]) / 32768.0 * 180.0
                    yaw = ((data[5] << 8) | data[4]) / 32768.0 * 180.0
                    
                    if roll > 180.0: roll -= 360.0
                    if pitch > 180.0: pitch -= 360.0
                    if yaw > 180.0: yaw -= 360.0

                    print(f"\r[IMU 자세] Roll: {roll:+6.1f}° | Pitch: {pitch:+6.1f}° | Yaw(방위): {yaw:+6.1f}°    ", end="", flush=True)

    except serial.SerialException as e:
        print(f"\n[!] IMU 통신 에러: {e}")
    except KeyboardInterrupt:
        print("\n[*] IMU 테스트를 종료합니다.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    test_imu()
