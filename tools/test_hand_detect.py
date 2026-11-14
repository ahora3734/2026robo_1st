#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D500 (STL-19P) 실시간 8방향 주행 장애물 감지 모니터
- 감지 거리: 최대 8.0m (800cm, 성능 저하 없는 D500 최적 유효 거리)
- 8방향 (N, NE, E, SE, S, SW, W, NW) 실시간 정밀 계측
- 20cm 차체 마스킹 필터 유지 (카트 본체/브라켓 간섭 배제)
- 3단계 안전 상태 판정 (0.6m 위험 정지 / 1.5m 주의 감속 / 안전 주행)
"""

import sys
import time
import statistics
from collections import deque
import serial
import serial.tools.list_ports

RESET  = "\033[0m"
BOLD   = "\033[1m"
RED    = "\033[91m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
WHITE  = "\033[97m"
GRAY   = "\033[90m"
CLEAR_SCREEN = "\033[H\033[J"

MASK_CM   = 20.0   # 본체 마스킹 (20cm 이하 무시)
MAX_CM    = 800.0  # 최대 감지 거리 (8m = 800cm, 실내 환경 최적 유효 범위)
SMOOTH_N  = 5      # 중앙값 스무딩 프레임

# 8방향 정의 (방향 이름, 중심 각도, 기호)
DIRECTIONS = [
    ("N  (전방)",     0,   "⬆️ "),
    ("NE (전방우측)", 45,  "↗️ "),
    ("E  (우측)",     90,  "➡️ "),
    ("SE (후방우측)", 135, "↘️ "),
    ("S  (후방)",     180, "⬇️ "),
    ("SW (후방좌측)", 225, "↙️ "),
    ("W  (좌측)",     270, "⬅️ "),
    ("NW (전방좌측)", 315, "↖️ ")
]

def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "CP210" in (p.description or "") or "10C4:EA60" in (p.hwid or ""):
            return p.device
    for p in ports:
        if "ttyUSB" in p.device or "ttyACM" in p.device:
            return p.device
    return "/dev/ttyUSB0"

def make_bar(dist_cm, max_cm=MAX_CM, width=24):
    """실시간 거리 시각화 바 (최대 8m)"""
    if dist_cm is None:
        return f"{GRAY}[{'':{width}s}] --cm{RESET}"
    clamped = max(0, min(dist_cm, max_cm))
    filled = int(round((clamped / max_cm) * width))
    bar = "█" * filled + "░" * (width - filled)
    
    if dist_cm < 60:
        color = RED + BOLD
    elif dist_cm < 150:
        color = YELLOW
    elif dist_cm < 300:
        color = CYAN
    else:
        color = GREEN
    
    m_val = dist_cm / 100.0
    return f"{color}[{bar}] {m_val:4.2f}m ({dist_cm:4.0f}cm){RESET}"

def fmt_dir_badge(val_cm):
    """방향별 거리 배지 표시"""
    if val_cm is None or val_cm < MASK_CM:
        return f"{GRAY}--m (--cm){RESET}"
    m_val = val_cm / 100.0
    if val_cm < 60:
        return f"{RED}{BOLD}{m_val:4.2f}m ({val_cm:3.0f}cm) [위험]{RESET}"
    elif val_cm < 150:
        return f"{YELLOW}{BOLD}{m_val:4.2f}m ({val_cm:3.0f}cm) [주의]{RESET}"
    else:
        return f"{GREEN}{m_val:4.2f}m ({val_cm:3.0f}cm) [안전]{RESET}"

def run():
    port = find_port()
    print(f"{CYAN}[*] LiDAR 포트 연결 중: {port}...{RESET}")
    try:
        ser = serial.Serial(port, 230400, timeout=1.0)
    except Exception as e:
        print(f"{RED}[!] 연결 실패: {e}{RESET}")
        return

    ser.reset_input_buffer()

    scan_buffers = [deque(maxlen=SMOOTH_N) for _ in range(360)]
    scan_raw   = [None] * 360
    timestamps = [0.0]  * 360
    last_render = 0.0

    print(f"{GREEN}[✔] LiDAR 연결 완료! 8방향 실시간 주행 감지를 시작합니다.{RESET}")
    time.sleep(0.3)

    try:
        while True:
            h = ser.read(2)
            if len(h) < 2:
                continue
            if h[0] == 0x54 and h[1] == 0x2C:
                data = ser.read(45)
                if len(data) != 45:
                    continue

                now = time.time()
                sa = ((data[3] << 8) | data[2]) / 100.0
                ea = ((data[41]<< 8) | data[40]) / 100.0
                if ea < sa:
                    ea += 360.0
                step = (ea - sa) / 11.0

                for i in range(12):
                    dist_mm = (data[5 + i*3] << 8) | data[4 + i*3]
                    conf    =  data[6 + i*3]
                    if conf > 10 and dist_mm >= 30:
                        ang = int(round(sa + step * i)) % 360
                        dist_cm = dist_mm / 10.0
                        if dist_cm <= MAX_CM:  # 최대 유효 거리 필터
                            scan_buffers[ang].append(dist_cm)
                            scan_raw[ang]   = statistics.median(scan_buffers[ang])
                            timestamps[ang] = now

                # 0.1초 (10 FPS) 갱신 주기로 CPU 부하 0% 유지
                if now - last_render < 0.1:
                    continue
                last_render = now

                # 0.5초 이내 유효 데이터만 취급
                active = [
                    scan_raw[i] if (now - timestamps[i] <= 0.5) else None
                    for i in range(360)
                ]

                # 8방향 거리 산출 (각 방향 중심 ±20° 부채꼴 영역 최솟값)
                dir_dists = {}
                for name, center, icon in DIRECTIONS:
                    vals = [
                        active[(center + o) % 360]
                        for o in range(-20, 21)
                        if active[(center + o) % 360] is not None
                        and active[(center + o) % 360] >= MASK_CM
                    ]
                    dir_dists[center] = min(vals) if vals else None

                # 전방 집중 구역 (315° ~ 45°, 90도 부채꼴) 최단거리
                front_vals = [
                    active[d] for d in range(360)
                    if (d <= 45 or d >= 315) and active[d] is not None and active[d] >= MASK_CM
                ]
                min_front_cm = min(front_vals) if front_vals else None

                # 360도 외부 장애물 (후방 사용자 구역 120°~240° 제외)
                obs_pts = [
                    (active[d], d) for d in range(360)
                    if active[d] is not None and active[d] >= MASK_CM and not (120 <= d <= 240)
                ]
                if obs_pts:
                    closest_cm, closest_deg = min(obs_pts, key=lambda x: x[0])
                else:
                    closest_cm, closest_deg = None, 0

                # 후방 사용자(보행자) 거리 추적 (120°~240°)
                user_pts = [
                    (active[d], d) for d in range(360)
                    if active[d] is not None and active[d] >= MASK_CM and (120 <= d <= 240)
                ]
                if user_pts:
                    user_cm, user_deg = min(user_pts, key=lambda x: x[0])
                    user_badge = f"{CYAN}{user_cm/100:.2f}m ({user_cm:3.0f}cm) [보행자 정상 동행]{RESET}"
                else:
                    user_cm, user_deg = None, 180
                    user_badge = f"{GRAY}미감지 (손잡이 이격){RESET}"

                # 주행 안전 상태 판정
                if min_front_cm is not None and min_front_cm < 60:
                    status = f"{RED}{BOLD}🚨 비상 정지! [DANGER] 전방 {min_front_cm/100:.2f}m 장애물 감지{RESET}"
                elif min_front_cm is not None and min_front_cm < 150:
                    status = f"{YELLOW}{BOLD}⚠️  주의 감속! [CAUTION] 전방 {min_front_cm/100:.2f}m 장애물 접근{RESET}"
                else:
                    status = f"{GREEN}{BOLD}✅ 안전 주행! [SAFE] 전방 경로 확보{RESET}"

                # 8방향 텍스트 콤파스 생성
                c_n  = fmt_dir_badge(dir_dists[0])
                c_ne = fmt_dir_badge(dir_dists[45])
                c_e  = fmt_dir_badge(dir_dists[90])
                c_se = fmt_dir_badge(dir_dists[135])
                # S(후방)은 사용자가 있는 위치이므로 사용자 배지로 표시
                c_s  = f"{CYAN}{dir_dists[180]/100:.2f}m [👤사용자]{RESET}" if dir_dists[180] else f"{GRAY}--m (--cm){RESET}"
                c_sw = fmt_dir_badge(dir_dists[225])
                c_w  = fmt_dir_badge(dir_dists[270])
                c_nw = fmt_dir_badge(dir_dists[315])

                lines = [
                    CLEAR_SCREEN,
                    f"{BOLD}{CYAN}══════════════════════════════════════════════════════════════════════{RESET}",
                    f"{BOLD}{CYAN}      🛰️  D500 LiDAR 실시간 8방향 자율주행 모니터 (최대 8.0m)       {RESET}",
                    f"{BOLD}{CYAN}══════════════════════════════════════════════════════════════════════{RESET}",
                    f"  • 포트: {WHITE}{port}{RESET} | 필터: {WHITE}본체 20cm 마스킹{RESET} | 최대거리: {WHITE}8.0m (800cm){RESET}",
                    f"  • 센서 기준: 0° = 정면 (케이블 반대편)",
                    "",
                    f"  ▶ 종합 주행 상태 : {status}",
                    f"  ▶ 전방위 최근접 장애물: {fmt_dir_badge(closest_cm)} (방위각 {closest_deg}°, 후방 사용자 제외)",
                    f"  ▶ 후방 사용자(동행자) : {user_badge} (180° 후방 영역)",
                    "",
                    f"{BOLD}[🧭 8방향 나침반 레이아웃]{RESET}",
                    f"      [NW 315°]                [⬆️ N 0° 정면]               [NE 45°]",
                    f"      {c_nw}           {c_n}            {c_ne}",
                    f"              \\                      |                      /",
                    f"   [⬅️ W 270°]  {c_w}  --- 🛒 카트중심 ---  {c_e}  [➡️ E 90°]",
                    f"              /                      |                      \\",
                    f"      {c_sw}           {c_s}            {c_se}",
                    f"      [SW 225°]                [⬇️ S 180° 후방]              [SE 135°]",
                    "",
                    f"{BOLD}[📊 8방향 실시간 거리 바 (최대 8m / 0.6m위험 🔴, 1.5m주의 🟡, 3m안전 🟢)]{RESET}",
                ]

                for name, deg, icon in DIRECTIONS:
                    d_cm = dir_dists[deg]
                    bar = make_bar(d_cm, max_cm=MAX_CM)
                    lines.append(f"  {icon} {name:12s} : {bar}")

                lines.extend([
                    "",
                    f"{GRAY}※ 종료하려면 터미널에서 Ctrl+C 를 누르세요.{RESET}"
                ])

                sys.stdout.write("\n".join(lines) + "\n")
                sys.stdout.flush()

    except KeyboardInterrupt:
        print(f"\n{CYAN}[*] 8방향 모니터링 종료{RESET}")
    finally:
        ser.close()
        print(f"{GREEN}[*] 포트가 안전하게 해제되었습니다.{RESET}\n")

if __name__ == "__main__":
    run()
