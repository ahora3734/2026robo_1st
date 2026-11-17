#!/usr/bin/env python3
"""
가상 메카넘 모터 회전 테스트 (회로 연결 전 키보드 조종 + 4륜 속도/방향 시각화)
- 키보드(W/S/A/D/Q/E, SPACE=정지, Ctrl+C/ESC=종료) 입력으로 실제 /cmd_vel을 발행합니다.
- 연결된 ESP32(esp32_mecanum_node)가 /cmd_vel을 수신하며, 터미널 화면에 4바퀴(FL/FR/RL/RR)의
  예상 회전 방향, RPM, 가상 이동 좌표(x, y, yaw)를 실시간으로 표시합니다.
"""

import math
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# esp32_mecanum_controller.ino와 동일한 기구학 파라미터
WHEEL_RADIUS = 0.05  # m
L_SUM = 0.36         # LX(0.18) + LY(0.18) m

LINEAR_SPEED = 0.30   # m/s
ANGULAR_SPEED = 0.80  # rad/s

# 2D 가상 지도 설정 (터미널 상단 시각화 - 작은 터미널 창에도 한눈에 들어가도록 컴팩트하게)
MAP_W = 35            # 가로 칸 수 (홀수)
MAP_H = 7             # 세로 칸 수 (홀수)
CELL_SIZE = 0.10      # 한 칸당 미터 (m)


def heading_char(yaw_rad: float) -> str:
    """yaw(0=전진/+x=위쪽, +yaw=반시계/좌회전)에 맞는 화살표 반환"""
    deg = math.degrees(yaw_rad) % 360.0
    arrows = ["↑", "↖", "←", "↙", "↓", "↘", "→", "↗"]
    idx = int(((deg + 22.5) % 360.0) // 45.0)
    return arrows[idx]


def render_map(sim_x: float, sim_y: float, sim_yaw: float, trail: set) -> str:
    """로봇 위치(x=+위, y=+왼쪽)와 이동 궤적을 2D 격자로 렌더링"""
    cx, cy = MAP_W // 2, MAP_H // 2
    # ROS 좌표계: +x = 앞(화면 위쪽), +y = 왼쪽(화면 왼쪽)
    rx = cx - int(round(sim_y / CELL_SIZE))
    ry = cy - int(round(sim_x / CELL_SIZE))

    lines = ["+" + "-" * MAP_W + "+"]
    for row in range(MAP_H):
        chars = []
        for col in range(MAP_W):
            if col == rx and row == ry:
                chars.append(heading_char(sim_yaw))
            elif (col, row) in trail:
                chars.append("·")
            elif col == cx and row == cy:
                chars.append("+")
            else:
                chars.append(" ")
        lines.append("|" + "".join(chars) + "|")
    lines.append("+" + "-" * MAP_W + "+")
    return "\n".join(lines)


def mecanum_ik(vx: float, vy: float, wz: float):
    """(vx, vy, wz) -> 4바퀴 목표 각속도(rad/s): [FL, FR, RL, RR]"""
    fl = (vx - vy - L_SUM * wz) / WHEEL_RADIUS
    fr = (vx + vy + L_SUM * wz) / WHEEL_RADIUS
    rl = (vx + vy - L_SUM * wz) / WHEEL_RADIUS
    rr = (vx - vy + L_SUM * wz) / WHEEL_RADIUS
    return fl, fr, rl, rr


def wheel_bar(rad_s: float, max_rad_s: float = 15.0) -> str:
    rpm = rad_s * 60.0 / (2.0 * math.pi)
    if abs(rad_s) < 0.01:
        return "■ 정지    [          ]   0.0 RPM"
    arrow = "▲ 정회전" if rad_s > 0 else "▼ 역회전"
    filled = min(10, max(1, int(round(abs(rad_s) / max_rad_s * 10))))
    bar = ("#" * filled).ljust(10)
    return f"{arrow} [{bar}] {rpm:+6.1f} RPM"


class VirtualMotorTester(Node):
    def __init__(self):
        super().__init__("virtual_motor_tester")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/wheel/odom", self._odom_cb, 10
        )
        self.last_odom_time = 0.0

    def _odom_cb(self, _msg: Odometry):
        self.last_odom_time = time.time()


def get_key(timeout: float = 0.05) -> str:
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if not rlist:
        return ""
    ch = sys.stdin.read(1)
    if ch == "\x1b":
        # 화살표 키 이스케이프 시퀀스(\x1b[A~D) 판별 (SSH 지연 고려해 0.05초 대기)
        r2, _, _ = select.select([sys.stdin], [], [], 0.05)
        if r2:
            ch2 = sys.stdin.read(1)
            if ch2 in ("[", "O"):
                r3, _, _ = select.select([sys.stdin], [], [], 0.05)
                if r3:
                    ch3 = sys.stdin.read(1)
                    arrow_map = {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}
                    return arrow_map.get(ch3, "")
        return ""  # 단독 ESC 입력으로 프로그램이 꺼지지 않도록 무시
    return ch


def main():
    rclpy.init()
    node = VirtualMotorTester()

    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    # 키 입력 시 터미널 아래에 글자/이스케이프 문자가 찍히지 않도록 ECHO 비활성화
    no_echo = termios.tcgetattr(sys.stdin)
    no_echo[3] = no_echo[3] & ~termios.ECHO
    termios.tcsetattr(sys.stdin, termios.TCSANOW, no_echo)

    vx, vy, wz = 0.0, 0.0, 0.0
    sim_x, sim_y, sim_yaw = 0.0, 0.0, 0.0
    trail = set()
    last_t = time.time()
    last_move_time = 0.0
    last_rot_time = 0.0
    last_draw = 0.0
    auto_stop_sec = 0.34  # 한 번 톡 누르면 약 1칸(0.10m) 이동 후 자동 정지

    move_map = {
        "w": (LINEAR_SPEED, 0.0, "전진 (W)"),
        "s": (-LINEAR_SPEED, 0.0, "후진 (S)"),
        "a": (0.0, LINEAR_SPEED, "좌측 게걸음 (A)"),
        "d": (0.0, -LINEAR_SPEED, "우측 게걸음 (D)"),
    }
    move_label = "정지"
    rot_label = "회전 없음"

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key(0.05)
            key_lower = key.lower() if len(key) == 1 else key

            if key_lower in ("\x03", "\x1b", "x"):  # Ctrl+C, ESC, x
                break
            if key_lower == "r":
                sim_x, sim_y, sim_yaw = 0.0, 0.0, 0.0
                trail.clear()
            elif key_lower == " ":
                vx, vy, wz = 0.0, 0.0, 0.0
                move_label = "정지 (SPACE)"
                rot_label = "회전 없음"
            elif key_lower in move_map:
                vx, vy, move_label = move_map[key_lower]
                last_move_time = time.time()
            elif key_lower == "q":
                wz = ANGULAR_SPEED
                rot_label = "좌회전 (Q)"
                last_rot_time = time.time()
            elif key_lower == "e":
                wz = -ANGULAR_SPEED
                rot_label = "우회전 (E)"
                last_rot_time = time.time()

            now = time.time()
            if (vx != 0.0 or vy != 0.0) and (now - last_move_time > auto_stop_sec):
                vx, vy = 0.0, 0.0
                move_label = "정지 (키 뗌)"
            if wz != 0.0 and (now - last_rot_time > auto_stop_sec):
                wz = 0.0
                rot_label = "회전 없음"

            dt = now - last_t
            last_t = now

            # /cmd_vel 발행 (ESP32로 실제 전송)
            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.angular.z = wz
            node.cmd_pub.publish(msg)

            # 가상 좌표 적분 및 이동 궤적 기록
            sim_x += (vx * math.cos(sim_yaw) - vy * math.sin(sim_yaw)) * dt
            sim_y += (vx * math.sin(sim_yaw) + vy * math.cos(sim_yaw)) * dt
            sim_yaw += wz * dt

            tx = (MAP_W // 2) - int(round(sim_y / CELL_SIZE))
            ty = (MAP_H // 2) - int(round(sim_x / CELL_SIZE))
            if 0 <= tx < MAP_W and 0 <= ty < MAP_H:
                trail.add((tx, ty))

            if now - last_draw >= 0.1:
                last_draw = now
                fl, fr, rl, rr = mecanum_ik(vx, vy, wz)
                esp_ok = (now - node.last_odom_time) < 1.0
                esp_status = "연결됨 (수신 중)" if esp_ok else "미수신 (에이전트 확인)"
                map_view = render_map(sim_x, sim_y, sim_yaw, trail)

                sys.stdout.write("\033[2J\033[H")
                sys.stdout.write(
                    "=== 가상 메카넘 모터 회전 테스트 (/cmd_vel -> ESP32) ===\n"
                    "이동: [W]전진 [S]후진 [A]좌 [D]우 | 회전: [Q]좌회전 [E]우회전 (모두 떼면 자동정지)\n"
                    "기타: [SPACE]전체정지 [R]위치초기화 [X]종료\n"
                    f"ESP32 상태: {esp_status}\n"
                    f"현재 상태 : {move_label} / {rot_label} | vx={vx:+.2f}, vy={vy:+.2f}, wz={wz:+.2f}\n"
                    "-------------------------------------------------------------\n"
                    f"  앞-왼쪽(FL): {wheel_bar(fl)}   |   앞-오른쪽(FR): {wheel_bar(fr)}\n"
                    f"  뒤-왼쪽(RL): {wheel_bar(rl)}   |   뒤-오른쪽(RR): {wheel_bar(rr)}\n"
                    "-------------------------------------------------------------\n"
                    f"{map_view}\n"
                    f"가상 위치: x={sim_x:+.2f} m, y={sim_y:+.2f} m, yaw={math.degrees(sim_yaw):+6.1f} deg  (1칸 = {CELL_SIZE}m)\n"
                )
                sys.stdout.flush()
    finally:
        # 종료 시 안전 정지 명령 발행 (Ctrl+C로 이미 context가 닫힌 경우 무시)
        try:
            if rclpy.ok():
                node.cmd_pub.publish(Twist())
        except Exception:
            pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

