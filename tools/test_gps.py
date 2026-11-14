#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
u-blox ZED-F9P 고정밀 GNSS/GPS 단품 테스트 도구
- NMEA $GNGGA, $GNRMC 문장 파싱
- 위도, 경도, 위성 수, RTK Fix 상태(Float/Fix) 실시간 출력
"""

import os
import sys
import time
import serial
import serial.tools.list_ports

PORT_DEFAULT = "/dev/ttyCartGPS"
BAUDRATE_DEFAULT = 115200

def parse_nmea_lat_lon(raw_val, direction):
    if not raw_val:
        return 0.0
    try:
        dot_idx = raw_val.find('.')
        deg_len = dot_idx - 2
        deg = float(raw_val[:deg_len])
        minutes = float(raw_val[deg_len:])
        dec = deg + (minutes / 60.0)
        if direction in ['S', 'W']:
            dec = -dec
        return dec
    except Exception:
        return 0.0

def test_gps():
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
        print(f"[*] ZED-F9P GPS 포트 열기 성공: {port} ({BAUDRATE_DEFAULT} bps)")
        print("[*] GNSS 위성 신호 수신 중... (Ctrl+C 로 종료)\n")

        fix_status_map = {
            '0': 'No Fix',
            '1': 'Single GPS (일반)',
            '2': 'DGPS (보정)',
            '4': 'RTK Fixed (cm급 고정밀)',
            '5': 'RTK Float (수십cm급)'
        }

        while True:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                parts = line.split(',')
                if len(parts) >= 10:
                    raw_lat, lat_dir = parts[2], parts[3]
                    raw_lon, lon_dir = parts[4], parts[5]
                    qual = parts[6]
                    sats = parts[7]
                    alt = parts[9]
                    
                    lat = parse_nmea_lat_lon(raw_lat, lat_dir)
                    lon = parse_nmea_lat_lon(raw_lon, lon_dir)
                    fix_type = fix_status_map.get(qual, f"Code {qual}")

                    print(f"\r[GPS] 위도: {lat:.7f}° | 경도: {lon:.7f}° | 위성수: {sats}개 | 고도: {alt}m | 상태: {fix_type}    ", end="", flush=True)

    except serial.SerialException as e:
        print(f"\n[!] GPS 통신 에러: {e}")
    except KeyboardInterrupt:
        print("\n[*] GPS 테스트를 종료합니다.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    test_gps()
