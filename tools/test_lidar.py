#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D500 (STL-19P) 2D LiDAR 단품 종합 진단 및 실시간 터미널 모니터
- 시리얼 포트 자동 탐색 및 패킷 무결성 검증
- 실시간 360도 점군 버퍼링 & 8방향 거리 계측
- ANSI 기반 실시간 ASCII 레이더 & 장애물 안전 경보 표시
"""

import os
import sys
import time
import math
import argparse
import statistics
from collections import deque
import serial
import serial.tools.list_ports

# ANSI 색상 코드
RESET = "\033[0m"
BOLD = "\033[1m"
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
CYAN = "\033[96m"
WHITE = "\033[97m"
CLEAR_SCREEN = "\033[H\033[J"

def find_lidar_port(preferred_port="/dev/ttyCartLiDAR"):
    """라이다 포트 자동 탐색"""
    if os.path.exists(preferred_port):
        return preferred_port
    
    # CP2102 (10C4:EA60) 등 라이다로 사용되는 USB 시리얼 포트 우선 탐색
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "CP210" in (p.description or "") or "10C4:EA60" in (p.hwid or ""):
            return p.device
            
    # 그 외 첫 번째 ttyUSB 또는 ttyACM
    for p in ports:
        if "ttyUSB" in p.device or "ttyACM" in p.device:
            return p.device
            
    if ports:
        return ports[0].device
    return None

def render_ascii_radar(ranges, max_range=3.0, size=17, mask_radius=0.20):
    """
    360도 거리 배열을 기반으로 ASCII 원형 레이더를 렌더링
    size: 홀수 권장 (기본 17x17)
    """
    center = size // 2
    grid = [[" " for _ in range(size)] for _ in range(size)]
    
    # 안전 반경 동심원 가이드라인
    # 0.6m (위험 반경)과 1.5m (주의 반경)
    r_danger_grid = int(round((0.6 / max_range) * (center - 1)))
    r_caution_grid = int(round((1.5 / max_range) * (center - 1)))
    
    for y in range(size):
        for x in range(size):
            dist_grid = math.sqrt((x - center)**2 + (y - center)**2)
            if abs(dist_grid - center + 1) < 0.6:
                grid[y][x] = "."  # 최대 탐지 경계 (3m)
            elif abs(dist_grid - r_caution_grid) < 0.5:
                grid[y][x] = "~"  # 1.5m 감속 경계
            elif abs(dist_grid - r_danger_grid) < 0.5:
                grid[y][x] = "-"  # 0.6m 비상정지 경계

    # 십자 기준선
    grid[0][center] = "^"
    grid[size-1][center] = "v"
    grid[center][0] = "<"
    grid[center][size-1] = ">"

    # 측정된 360도 포인트 투영
    for deg in range(360):
        d = ranges[deg]
        if d is not None and 0.03 <= d <= max_range:
            rad = math.radians(deg)
            # 0도는 상단(전방), 90도는 우측, 180도는 하단(후방), 270도는 좌측
            gx = int(round(center + (d / max_range) * (center - 1) * math.sin(rad)))
            gy = int(round(center - (d / max_range) * (center - 1) * math.cos(rad)))
            if 0 <= gx < size and 0 <= gy < size:
                if d < mask_radius:
                    grid[gy][gx] = f"{CYAN}·{RESET}"  # 본체 마스킹 영역
                elif d < 0.6:
                    grid[gy][gx] = f"{RED}X{RESET}"
                elif d < 1.5:
                    grid[gy][gx] = f"{YELLOW}*{RESET}"
                else:
                    grid[gy][gx] = f"{GREEN}o{RESET}"

    # 카트 본체(중앙) 표시
    grid[center][center] = f"{CYAN}▲{RESET}"

    radar_lines = []
    border = "+" + "-" * size + "+"
    radar_lines.append(border)
    for row in grid:
        # ANSI 코드가 섞여 있으므로 문자열 조합
        line_str = "".join(row)
        radar_lines.append(f"|{line_str}|")
    radar_lines.append(border)
    return radar_lines

def get_sector_distance(ranges, center_deg, width=15, min_valid=0.20):
    """지정 각도 주변의 최소 거리 계산 (본체 마스킹 min_valid 이상 유효값만)"""
    vals = []
    for offset in range(-width, width + 1):
        deg = (center_deg + offset) % 360
        if ranges[deg] is not None and ranges[deg] >= min_valid:
            vals.append(ranges[deg])
    if vals:
        return min(vals)
    return float('inf')

def run_lidar_monitor(port, baudrate=230400, max_range=3.0, mask_radius=0.20):
    if not port or not os.path.exists(port):
        print(f"{RED}[!] 에러: 지정된 포트 '{port}'를 찾을 수 없습니다.{RESET}")
        return

    print(f"{CYAN}[*] LiDAR 시리얼 포트 여는 중: {port} ({baudrate} bps)...{RESET}")
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1.0)
    except Exception as e:
        print(f"{RED}[!] 시리얼 포트 열기 실패: {e}{RESET}")
        print(f"{YELLOW}    팁: 현재 사용자가 'dialout' 그룹에 속해 있는지 확인하세요: `sudo usermod -aG dialout $USER`{RESET}")
        return

    ser.reset_input_buffer()
    
    # 360도 각도별 거리(m) 롤링 버퍼 (최근 5개값 중앙값으로 스무딩)
    SMOOTH_N = 5
    scan_buffers = [deque(maxlen=SMOOTH_N) for _ in range(360)]  # 각도별 최근 N개 측정값
    scan_ranges = [None] * 360   # 스무딩 완료된 최종값
    point_timestamps = [0.0] * 360
    
    packet_count = 0
    fps_counter = 0
    fps = 0.0
    last_fps_time = time.time()
    last_render_time = 0.0
    rotation_speed_dps = 0
    # 안전상태 히스테리시스: 연속 N프레임 같은 상태여야 전환
    safety_state_buffer = deque(maxlen=3)
    safety_display_state = "SAFE"

    print(f"{GREEN}[✔] 연결 성공! 데이터를 수신하여 화면을 초기화합니다...{RESET}")
    time.sleep(0.3)

    try:
        while True:
            # 패킷 헤더 0x54 0x2C 탐색
            header = ser.read(2)
            if len(header) < 2:
                continue
            if header[0] == 0x54 and header[1] == 0x2C:
                data = ser.read(45)
                if len(data) != 45:
                    continue

                packet_count += 1
                fps_counter += 1
                now = time.time()

                # 회전 속도 (도/초)
                rotation_speed_dps = (data[1] << 8) | data[0]
                
                # 시작 각도 및 끝 각도 (0.01도 단위)
                start_angle = ((data[3] << 8) | data[2]) / 100.0
                end_angle = ((data[41] << 8) | data[40]) / 100.0
                if end_angle < start_angle:
                    end_angle += 360.0
                
                step = (end_angle - start_angle) / 11.0

                # 12개 포인트 추출 → 롤링 버퍼에 추가 후 중앙값(median) 스무딩
                for i in range(12):
                    dist_mm = (data[5 + i*3] << 8) | data[4 + i*3]
                    conf = data[6 + i*3]
                    
                    if conf > 10 and dist_mm >= 30:  # 신뢰도 유효 & 최소 3cm (D500 스펙 하한)
                        ang = int(round(start_angle + step * i)) % 360
                        scan_buffers[ang].append(dist_mm / 1000.0)
                        # 버퍼에 값이 2개 이상이면 중앙값, 1개면 그 값 그대로 사용
                        scan_ranges[ang] = statistics.median(scan_buffers[ang])
                        point_timestamps[ang] = now

                # 1초마다 FPS 갱신
                if now - last_fps_time >= 1.0:
                    fps = fps_counter / (now - last_fps_time)
                    fps_counter = 0
                    last_fps_time = now

                # 0.15초마다 터미널 화면 갱신 (~6.6 FPS)
                if now - last_render_time >= 0.15:
                    last_render_time = now
                    
                    # 오래된 포인트 만료 처리 (0.6초 초과)
                    active_ranges = [
                        scan_ranges[i] if (now - point_timestamps[i] <= 0.6) else None
                        for i in range(360)
                    ]

                    # 8방향 거리 계산 (본체 마스킹 거리 이상만)
                    d_front  = get_sector_distance(active_ranges, 0, width=15, min_valid=mask_radius)
                    d_fr     = get_sector_distance(active_ranges, 45, width=15, min_valid=mask_radius)
                    d_right  = get_sector_distance(active_ranges, 90, width=15, min_valid=mask_radius)
                    d_br     = get_sector_distance(active_ranges, 135, width=15, min_valid=mask_radius)
                    d_back   = get_sector_distance(active_ranges, 180, width=15, min_valid=mask_radius)
                    d_bl     = get_sector_distance(active_ranges, 225, width=15, min_valid=mask_radius)
                    d_left   = get_sector_distance(active_ranges, 270, width=15, min_valid=mask_radius)
                    d_fl     = get_sector_distance(active_ranges, 315, width=15, min_valid=mask_radius)

                    # 전체 최단 거리 장애물 탐색 (본체 마스킹 반경 밖)
                    valid_items = [
                        (active_ranges[deg], deg) for deg in range(360) 
                        if active_ranges[deg] is not None and active_ranges[deg] >= mask_radius
                    ]
                    if valid_items:
                        min_dist, min_ang = min(valid_items, key=lambda x: x[0])
                    else:
                        min_dist, min_ang = 99.0, 0

                    # 안전 상태 판정 (전방 90도 부채꼴 중심, 본체 마스킹 반경 밖)
                    front_items = [
                        active_ranges[deg] for deg in range(360) 
                        if (deg <= 45 or deg >= 315) and active_ranges[deg] is not None and active_ranges[deg] >= mask_radius
                    ]
                    front_min = min(front_items) if front_items else 99.0

                    # 이번 프레임의 원시 상태 판정
                    if front_min < 0.6:
                        raw_state = "DANGER"
                    elif front_min < 1.5:
                        raw_state = "CAUTION"
                    else:
                        raw_state = "SAFE"

                    # 연속 3프레임 같은 상태일 때만 표시 상태 전환 (깜빡임 방지)
                    safety_state_buffer.append(raw_state)
                    if len(safety_state_buffer) == 3 and len(set(safety_state_buffer)) == 1:
                        safety_display_state = raw_state

                    if safety_display_state == "DANGER":
                        status_badge = f"{RED}{BOLD}🚨 [위험 / 비상 정지 대상]{RESET}"
                    elif safety_display_state == "CAUTION":
                        status_badge = f"{YELLOW}{BOLD}⚠️ [주의 / 감속 주행 구간]{RESET}"
                    else:
                        status_badge = f"{GREEN}{BOLD}🟢 [안전 / 정상 주행 가능]{RESET}"

                    # 회전 속도 (Hz 및 RPM 계산)
                    hz = rotation_speed_dps / 360.0
                    rpm = hz * 60.0
                    valid_point_count = len(valid_items)

                    # ASCII 레이더 생성
                    radar_lines = render_ascii_radar(active_ranges, max_range=max_range, size=19, mask_radius=mask_radius)

                    # 텍스트 정보 라인 구성 (m와 cm 동시 표기)
                    def fmt_d(val):
                        if val == float('inf') or val > 10.0:
                            return f"{WHITE} --.--m ( --cm){RESET}"
                        cm_val = int(round(val * 100))
                        if val < 0.6:
                            return f"{RED}{BOLD}{val:4.2f}m ({cm_val:3d}cm){RESET}"
                        elif val < 1.5:
                            return f"{YELLOW}{BOLD}{val:4.2f}m ({cm_val:3d}cm){RESET}"
                        else:
                            return f"{GREEN}{val:4.2f}m ({cm_val:3d}cm){RESET}"

                    info_lines = [
                        f"{BOLD}{CYAN}=== 🛰️ D500 (STL-19P) 2D LiDAR 실시간 진단 모니터 ==={RESET}",
                        f"• 연결 포트 : {WHITE}{port}{RESET} ({baudrate} bps) | 수신 상태 : {GREEN}정상{RESET}",
                        f"• 회전 속도 : {WHITE}{hz:.1f} Hz{RESET} ({rpm:.0f} RPM) | 패킷 레이트 : {WHITE}{fps:.0f} pkts/s{RESET}",
                        f"• 유효 점군 : {WHITE}{valid_point_count:3d} / 360 도{RESET} (커버리지: {valid_point_count/3.6:.0f}%)",
                        "",
                        f"• 전방 안전 상태 : {status_badge}",
                        f"• 최근접 장애물  : {fmt_d(min_dist)} (방위각: {min_ang:3d}°)",
                        "",
                        f"{BOLD}[🧭 8방향 실시간 거리 계측]{RESET}",
                        f"  ↖️ 전좌 (315°): {fmt_d(d_fl)}  |  ⬆️ 전방 (  0°): {fmt_d(d_front)}  |  ↗️ 전우 ( 45°): {fmt_d(d_fr)}",
                        f"  ⬅️ 좌측 (270°): {fmt_d(d_left)}  |  ▲ [카트 중심]  |  ➡️ 우측 ( 90°): {fmt_d(d_right)}",
                        f"  ↙️ 후좌 (225°): {fmt_d(d_bl)}  |  ⬇️ 후방 (180°): {fmt_d(d_back)}  |  ↘️ 후우 (135°): {fmt_d(d_br)}",
                        "",
                        f"{BOLD}[📡 실시간 2D 레이더 범례]{RESET}",
                        f"  ▲: 카트 본체  |  {CYAN}·{RESET}: 본체마스킹({mask_radius*100:.0f}cm)  |  {RED}X{RESET}: <0.6m 위험  |  {YELLOW}*{RESET}: <1.5m 주의  |  {GREEN}o{RESET}: 안전",
                        f"  -: 0.6m 경계선  |  ~: 1.5m 경계선  |  .: {max_range:.1f}m 최대 탐지 외곽선",
                        f"{WHITE}  (종료하려면 Ctrl+C 를 누르세요){RESET}"
                    ]

                    # 레이더와 정보창 나란히 병합 출력
                    output = [CLEAR_SCREEN]
                    max_rows = max(len(radar_lines), len(info_lines))
                    for r in range(max_rows):
                        r_part = radar_lines[r] if r < len(radar_lines) else " " * 21
                        i_part = info_lines[r] if r < len(info_lines) else ""
                        output.append(f"{r_part}   {i_part}")

                    sys.stdout.write("\n".join(output) + "\n")
                    sys.stdout.flush()

    except KeyboardInterrupt:
        print(f"\n\n{CYAN}[*] 사용자에 의해 테스트가 중단되었습니다.{RESET}")
    finally:
        ser.close()
        print(f"{GREEN}[*] LiDAR 시리얼 포트가 안전하게 닫혔습니다.{RESET}")
        print(f"{GREEN}[*] 총 수신된 패킷: {packet_count:,} 개{RESET}\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="D500 2D LiDAR 단품 실시간 테스트")
    parser.add_argument("--port", type=str, default=None, help="시리얼 포트 경로 (미지정시 자동 탐색)")
    parser.add_argument("--baud", type=int, default=230400, help="통신 보드레이트 (기본: 230400)")
    parser.add_argument("--range", type=float, default=8.0, help="레이더 최대 반경 (m, 기본: 8.0)")
    parser.add_argument("--mask", type=float, default=0.20, help="카트 본체/거치대 마스킹 반경 (m, 기본: 0.20m = 20cm)")
    args = parser.parse_args()

    selected_port = args.port if args.port else find_lidar_port()
    run_lidar_monitor(selected_port, args.baud, args.range, args.mask)
