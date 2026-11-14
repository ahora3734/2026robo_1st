#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32 메카넘 모터 및 수동 제어 단품 테스트 도구
- W, A, S, D: 전진, 좌횡이동, 후진, 우횡이동
- Q, E: 좌회전, 우회전
- Space: 긴급 정지 (E-STOP)
- ESP32로부터 엔코더 및 배터리 전압 피드백 수신
"""

import sys
import time
import threading
import serial
import serial.tools.list_ports

try:
    import curses
except ImportError:
    curses = None

PORT_DEFAULT = "/dev/ttyCartESP32"
BAUDRATE_DEFAULT = 115200

class ESP32MotorTester:
    def __init__(self, port=PORT_DEFAULT, baudrate=BAUDRATE_DEFAULT):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = True
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.speed_step = 0.05
        self.max_speed = 0.6
        self.latest_odom = "대기 중..."

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"[*] ESP32 연결 성공: {self.port} ({self.baudrate} bps)")
            return True
        except Exception as e:
            print(f"[!] 포트 열기 실패 ({self.port}): {e}")
            # 시리얼 포트 목록 검색
            ports = [p.device for p in serial.tools.list_ports.comports()]
            print(f"[*] 현재 사용 가능한 포트 목록: {ports}")
            if ports:
                try:
                    self.ser = serial.Serial(ports[0], self.baudrate, timeout=0.1)
                    print(f"[*] 대체 포트 연결 성공: {ports[0]}")
                    return True
                except Exception as ex:
                    print(f"[!] 대체 포트 연결 실패: {ex}")
            return False

    def send_cmd(self, vx, vy, wz):
        """BLUEPRINT 4.1절 형식: $CMD,vx,vy,wz,checksum\\n"""
        if not self.ser or not self.ser.is_open:
            return
        payload = f"{vx:.2f},{vy:.2f},{wz:.2f}"
        checksum = 0
        for char in payload:
            checksum ^= ord(char)
        packet = f"$CMD,{payload},{checksum:02X}\n"
        try:
            self.ser.write(packet.encode('ascii'))
        except Exception as e:
            pass

    def emergency_stop(self):
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.send_cmd(0.0, 0.0, 0.0)

    def reader_thread(self):
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("$ODOM"):
                        self.latest_odom = line
                except Exception:
                    pass
            time.sleep(0.02)

    def start_heartbeat(self):
        """지속적으로 10Hz 속도 명령 전송 (ESP32 Watchdog 방지)"""
        while self.running:
            self.send_cmd(self.current_vx, self.current_vy, self.current_wz)
            time.sleep(0.1)

def run_cli_test():
    tester = ESP32MotorTester()
    if not tester.connect():
        print("[!] ESP32에 연결할 수 없습니다. 시뮬레이션 모드로 동작합니다.")

    t_read = threading.Thread(target=tester.reader_thread, daemon=True)
    t_beat = threading.Thread(target=tester.start_heartbeat, daemon=True)
    t_read.start()
    t_beat.start()

    print("\n" + "="*50)
    print("      ESP32 4륜 메카넘 키보드 수동 주행 테스트")
    print("="*50)
    print("  [W] : 전진 (+Vx)       [S] : 후진 (-Vx)")
    print("  [A] : 좌측 게걸음 (+Vy) [D] : 우측 게걸음 (-Vy)")
    print("  [Q] : 좌회전 (+Wz)     [E] : 우회전 (-Wz)")
    print("  [SPACE] : 긴급 정지 (E-STOP)")
    print("  [X] : 서서히 감속 / 정지")
    print("  [ESC / Ctrl+C] : 종료")
    print("="*50 + "\n")

    # Windows 또는 터미널 단독 실행을 위한 키 입력 처리
    import getch_compat as getch_module
    
    try:
        while True:
            key = getch_module.getkey()
            if key in ['w', 'W']:
                tester.current_vx = min(tester.max_speed, tester.current_vx + tester.speed_step)
            elif key in ['s', 'S']:
                tester.current_vx = max(-tester.max_speed, tester.current_vx - tester.speed_step)
            elif key in ['a', 'A']:
                tester.current_vy = min(tester.max_speed, tester.current_vy + tester.speed_step)
            elif key in ['d', 'D']:
                tester.current_vy = max(-tester.max_speed, tester.current_vy - tester.speed_step)
            elif key in ['q', 'Q']:
                tester.current_wz = min(1.0, tester.current_wz + 0.1)
            elif key in ['e', 'E']:
                tester.current_wz = max(-1.0, tester.current_wz - 0.1)
            elif key == ' ':
                tester.emergency_stop()
                print("\n🚨 [긴급 정지 (E-STOP) 발동!]")
            elif key in ['x', 'X']:
                tester.emergency_stop()
            elif key in ['\x1b', '\x03', 'q' if key == chr(27) else None]: # ESC or Ctrl+C
                break

            print(f"\r[제어 상태] Vx: {tester.current_vx:+.2f} m/s | Vy: {tester.current_vy:+.2f} m/s | Wz: {tester.current_wz:+.2f} rad/s | 피드백: {tester.latest_odom}", end="", flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        tester.emergency_stop()
        tester.running = False
        print("\n[*] 테스트를 안전하게 종료합니다.")

if __name__ == "__main__":
    # getch 호환 래퍼 인라인 생성
    class GetchCompat:
        @staticmethod
        def getkey():
            if sys.platform == "win32":
                import msvcrt
                ch = msvcrt.getch()
                if ch in [b'\x00', b'\xe0']:
                    ch = msvcrt.getch()
                return ch.decode('utf-8', errors='ignore')
            else:
                import tty, termios
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setraw(sys.stdin.fileno())
                    ch = sys.stdin.read(1)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                return ch

    import types
    sys.modules['getch_compat'] = types.ModuleType('getch_compat')
    sys.modules['getch_compat'].getkey = GetchCompat.getkey
    run_cli_test()
