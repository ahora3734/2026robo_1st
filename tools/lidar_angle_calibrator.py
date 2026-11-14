#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
D500 (STL-19P) 라이다 전방 0도 방향 검증 & 자동 보정 스크립트
=============================================================================
사용법:
  1. 카트에 라이다를 장착한다.
  2. 카트 정면(전방)에서 약 30~50cm 거리에 손 또는 상자를 놓는다.
  3. 이 스크립트를 실행한다:
       python3 lidar_angle_calibrator.py
  4. 화면에 가장 가까운 물체의 각도가 표시된다.
  5. 그 각도가 0°에서 얼마나 떨어져 있는지 자동 계산하여
     lidar_engine.py에 넣어야 할 angle_offset_deg 값을 알려준다.

결과 예시:
  ✅ 전방 장애물이 3.2°에서 감지됨
  → angle_offset_deg = -3.2 로 설정하면 보정 완료!
=============================================================================
"""

import sys
import time
import math
import serial
import serial.tools.list_ports
import os

# ──────────── 설정 ────────────
BAUDRATE = 230400
HEADER = 0x54
VERLEN = 0x2C
PACKET_SIZE = 47
SCAN_SECONDS = 3         # 3초간 수집 후 판정
NEAR_THRESHOLD_M = 1.0   # 이 거리 이내의 점만 "전방 물체" 후보로 인정

# ANSI 색상
RESET  = "\033[0m"
BOLD   = "\033[1m"
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"

# ──────────── CRC-8 ────────────
def _init_crc_table():
    poly = 0x4D
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)
    return table

CRC_TABLE = _init_crc_table()

def cal_crc8(data):
    crc = 0
    for b in data:
        crc = CRC_TABLE[crc ^ b]
    return crc


def find_port():
    """라이다 시리얼 포트 자동 탐색"""
    # udev 심볼릭 링크 우선
    if os.path.exists("/dev/ttyCartLiDAR"):
        return "/dev/ttyCartLiDAR"
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "CP210" in (p.description or "") or "10C4:EA60" in (p.hwid or ""):
            return p.device
    for p in ports:
        if "ttyUSB" in p.device or "ttyACM" in p.device:
            return p.device
    return "/dev/ttyUSB0"


def collect_scan_data(port, duration_s):
    """
    지정된 시간 동안 라이다 패킷을 수신하고,
    (각도(deg), 거리(m), 신뢰도) 튜플의 리스트를 반환한다.
    """
    ser = serial.Serial(port, BAUDRATE, timeout=0.5)
    ser.reset_input_buffer()

    points = []
    rx_buf = bytearray()
    start_time = time.time()
    pkt_count = 0

    print(f"\n{CYAN}📡 라이다 데이터 수집 중... ({duration_s}초){RESET}")
    print(f"{CYAN}   ▶ 카트 전방(정면)에 손이나 상자를 30~50cm 거리에 놓아주세요!{RESET}\n")

    while time.time() - start_time < duration_s:
        waiting = ser.in_waiting
        if waiting > 0:
            rx_buf.extend(ser.read(waiting))
        else:
            rx_buf.extend(ser.read(47))

        while len(rx_buf) >= 47:
            # 헤더 찾기
            idx = -1
            for i in range(len(rx_buf) - 1):
                if rx_buf[i] == HEADER and rx_buf[i + 1] == VERLEN:
                    idx = i
                    break
            if idx < 0:
                rx_buf = rx_buf[-1:]
                break
            if idx > 0:
                del rx_buf[:idx]
            if len(rx_buf) < 47:
                break

            packet = bytes(rx_buf[:47])
            del rx_buf[:47]

            # CRC 검증
            if cal_crc8(packet[:46]) != packet[46]:
                continue

            pkt_count += 1

            # 각도 파싱
            start_angle = ((packet[5] << 8) | packet[4]) / 100.0
            end_angle   = ((packet[43] << 8) | packet[42]) / 100.0
            if end_angle < start_angle:
                end_angle += 360.0
            step = (end_angle - start_angle) / 11.0

            for i in range(12):
                offset = 6 + i * 3
                dist_mm = (packet[offset + 1] << 8) | packet[offset]
                conf = packet[offset + 2]
                if conf > 15 and dist_mm >= 30:
                    angle = (start_angle + step * i) % 360.0
                    dist_m = dist_mm / 1000.0
                    points.append((angle, dist_m, conf))

        # 진행률 표시
        elapsed = time.time() - start_time
        pct = min(100, int(elapsed / duration_s * 100))
        bar = "█" * (pct // 5) + "░" * (20 - pct // 5)
        print(f"\r   [{bar}] {pct:3d}%  패킷: {pkt_count}", end="", flush=True)

    ser.close()
    print(f"\n\n{GREEN}✅ 수집 완료! 총 {pkt_count}개 패킷, {len(points)}개 포인트{RESET}\n")
    return points


def analyze_front_direction(points):
    """
    수집된 점군 중 가장 가까운 물체 클러스터의 중심 각도를 찾아
    센서의 실제 0°와 카트 전방의 차이(offset)를 계산한다.
    """
    if not points:
        print(f"{RED}❌ 수집된 데이터가 없습니다. 센서 연결을 확인하세요.{RESET}")
        return None

    # 1m 이내의 가까운 점만 필터링
    near_points = [(a, d, c) for a, d, c in points if d < NEAR_THRESHOLD_M]

    if not near_points:
        print(f"{YELLOW}⚠️ {NEAR_THRESHOLD_M}m 이내에 물체가 감지되지 않았습니다.{RESET}")
        print(f"   전방에 손이나 상자를 30~50cm 거리에 놓고 다시 시도하세요.")
        return None

    # 각도 히스토그램 (1° 해상도)
    angle_bins = {}
    for a, d, c in near_points:
        deg_key = int(round(a)) % 360
        if deg_key not in angle_bins:
            angle_bins[deg_key] = []
        angle_bins[deg_key].append(d)

    # 가장 많은 포인트가 몰려 있는 ±10° 윈도우 찾기
    best_center = 0
    best_count = 0
    for center in range(360):
        count = 0
        for offset in range(-10, 11):
            key = (center + offset) % 360
            count += len(angle_bins.get(key, []))
        if count > best_count:
            best_count = count
            best_center = center

    # 해당 윈도우 내 포인트들의 가중평균 각도 계산 (거리가 가까울수록 가중치 높음)
    sum_sin = 0.0
    sum_cos = 0.0
    total_weight = 0.0
    for offset in range(-10, 11):
        key = (best_center + offset) % 360
        for d in angle_bins.get(key, []):
            weight = 1.0 / max(d, 0.05)  # 거리 역수 가중치
            rad = math.radians(key)
            sum_sin += weight * math.sin(rad)
            sum_cos += weight * math.cos(rad)
            total_weight += weight

    if total_weight < 1e-6:
        print(f"{RED}❌ 분석 실패: 유효 포인트 부족{RESET}")
        return None

    detected_angle = math.degrees(math.atan2(sum_sin, sum_cos)) % 360.0

    # 최근접 포인트
    min_d_point = min(near_points, key=lambda x: x[1])

    return detected_angle, min_d_point, best_count, len(near_points)


def display_360_overview(points):
    """간단한 360도 레이더 텍스트 표시 (어느 방향에 물체가 있는지 한눈에 파악)"""
    # 8방향 최소 거리 계산
    dirs = [
        ("  전방  (0°) ", 0),
        ("전방우측(45°)", 45),
        ("  우측 (90°) ", 90),
        ("후방우측(135°)", 135),
        ("  후방 (180°)", 180),
        ("후방좌측(225°)", 225),
        ("  좌측 (270°)", 270),
        ("전방좌측(315°)", 315),
    ]

    print(f"{BOLD}{'─' * 55}{RESET}")
    print(f"{BOLD}  📡 8방향 최근접 거리 요약{RESET}")
    print(f"{BOLD}{'─' * 55}{RESET}")

    for name, center in dirs:
        sector_points = [
            d for a, d, c in points
            if abs(((a - center + 180) % 360) - 180) <= 22
        ]
        if sector_points:
            min_d = min(sector_points)
            bar_len = min(20, int(min_d / 0.25))
            bar = "█" * bar_len + "░" * (20 - bar_len)
            if min_d < 0.6:
                color = RED + BOLD
            elif min_d < 1.5:
                color = YELLOW
            else:
                color = GREEN
            print(f"  {name}  {color}[{bar}] {min_d:5.2f}m{RESET}")
        else:
            print(f"  {name}  {CYAN}[{'░' * 20}]  --  {RESET}")

    print(f"{BOLD}{'─' * 55}{RESET}")


def main():
    print(f"\n{'═' * 60}")
    print(f"{BOLD}{CYAN}  🔧 D500 (STL-19P) 라이다 전방 방향 검증 & 보정 도구{RESET}")
    print(f"{'═' * 60}")

    # 포트 탐색
    port = find_port()
    print(f"\n  📍 사용할 포트: {BOLD}{port}{RESET}")

    if not os.path.exists(port):
        print(f"\n{RED}❌ 포트 [{port}]를 찾을 수 없습니다.{RESET}")
        print(f"   라이다 USB 케이블 연결을 확인하세요.")
        sys.exit(1)

    # 데이터 수집
    points = collect_scan_data(port, SCAN_SECONDS)

    if not points:
        print(f"{RED}❌ 데이터를 수집하지 못했습니다. 센서 연결/전원을 확인하세요.{RESET}")
        sys.exit(1)

    # 360도 개요 표시
    display_360_overview(points)

    # 전방 분석
    result = analyze_front_direction(points)
    if result is None:
        sys.exit(1)

    detected_angle, closest_point, cluster_count, total_near = result

    # 오프셋 계산 (0°에서의 편차)
    # 센서가 카트 전방을 detected_angle로 인식하고 있으므로,
    # 이것을 0°로 맞추려면 -detected_angle 만큼 보정해야 함.
    # 단, ±180° 범위로 정규화
    offset = detected_angle
    if offset > 180:
        offset -= 360.0

    print(f"\n{'═' * 60}")
    print(f"{BOLD}  📊 전방 방향 분석 결과{RESET}")
    print(f"{'═' * 60}")
    print(f"  • 전방 물체 감지 각도 : {BOLD}{CYAN}{detected_angle:.1f}°{RESET}")
    print(f"  • 최근접 포인트       : {closest_point[1]:.3f}m @ {closest_point[0]:.1f}° (신뢰도 {closest_point[2]})")
    print(f"  • 분석에 사용된 포인트: {total_near}개 (클러스터 {cluster_count}개)")

    if abs(offset) < 2.0:
        print(f"\n  {GREEN}{BOLD}✅ 센서 방향이 거의 정확합니다! (오차 {offset:+.1f}°){RESET}")
        print(f"  → angle_offset_deg = 0.0 (보정 불필요)")
        recommended_offset = 0.0
    else:
        recommended_offset = -offset
        print(f"\n  {YELLOW}{BOLD}⚠️  센서 0°가 카트 전방에서 {offset:+.1f}° 어긋나 있습니다.{RESET}")
        print(f"  {BOLD}→ angle_offset_deg = {recommended_offset:+.1f}{RESET} 로 설정하면 보정 완료!")

    print(f"\n{'─' * 60}")
    print(f"  {BOLD}📝 적용 방법:{RESET}")
    print(f"  lidar_engine.py에서 LidarEngine 생성 시:")
    print(f"    {CYAN}LidarEngine(angle_offset_deg={recommended_offset:.1f}){RESET}")
    print(f"  또는 대시보드 코드에서:")
    print(f"    {CYAN}self.lidar = LidarEngine(angle_offset_deg={recommended_offset:.1f}){RESET}")
    print(f"{'═' * 60}\n")

    return recommended_offset


if __name__ == "__main__":
    main()

