#!/usr/bin/env python3
"""
D500 (STL-19P) LiDAR 센서 테스트 스크립트
==========================================
- 포트: /dev/ttyUSB0 (또는 /dev/ttyCartLiDAR)
- Baudrate: 230400
- 프로토콜: STL-19P (LDROBOT D500 계열)

패킷 구조 (STL-19P):
  Header(1B): 0x54
  VerLen(1B): 0x2C (상위 3비트=버전, 하위 5비트=포인트 수 = 12)
  Speed(2B): 회전 속도 (deg/s * 100)
  StartAngle(2B): 시작 각도 (deg * 100)
  Data(36B): 12개 포인트 x 3바이트 (거리 2B + 신뢰도 1B)
  EndAngle(2B): 끝 각도 (deg * 100)
  Timestamp(2B): 타임스탬프 (ms)
  CRC(1B): CRC8 체크섬
  총 패킷 길이: 47 바이트
"""

import serial
import struct
import sys
import time
import math
import signal

# ──────────────────── 설정 ────────────────────
PORT = "/dev/ttyUSB0"
BAUDRATE = 230400
HEADER = 0x54
VERLEN = 0x2C
PACKET_SIZE = 47
POINTS_PER_PACKET = 12

# ──────────────────── CRC8 테이블 (STL-19P) ────────────────────
CRC_TABLE = [
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE,
    0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
    0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07,
    0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
    0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1,
    0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18,
    0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
    0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90,
    0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
    0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39,
    0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F,
    0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
    0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26,
    0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
    0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2,
    0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B,
    0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
    0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD,
    0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
    0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64,
    0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC,
    0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
    0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
    0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3,
    0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A,
    0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8,
]


def calc_crc8(data: bytes) -> int:
    """STL-19P CRC8 체크섬 계산"""
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


def parse_packet(packet: bytes):
    """
    47바이트 패킷을 파싱하여 포인트 데이터를 반환합니다.
    
    Returns:
        dict: speed_deg_s, start_angle, end_angle, timestamp, points[(angle, distance_mm, confidence)]
        또는 None (CRC 오류 시)
    """
    if len(packet) != PACKET_SIZE:
        return None

    # CRC 검증
    crc_calc = calc_crc8(packet[:-1])
    crc_recv = packet[-1]
    if crc_calc != crc_recv:
        return None

    # 헤더 및 버전/길이 확인
    header = packet[0]
    verlen = packet[1]
    if header != HEADER or verlen != VERLEN:
        return None

    # 속도 (deg/s)
    speed_raw = struct.unpack_from('<H', packet, 2)[0]
    speed_deg_s = speed_raw / 100.0

    # 시작 각도 (deg)
    start_angle_raw = struct.unpack_from('<H', packet, 4)[0]
    start_angle = start_angle_raw / 100.0

    # 12개 포인트 데이터 파싱
    points = []
    for i in range(POINTS_PER_PACKET):
        offset = 6 + i * 3
        distance_mm = struct.unpack_from('<H', packet, offset)[0]
        confidence = packet[offset + 2]
        points.append((distance_mm, confidence))

    # 끝 각도 (deg)
    end_angle_raw = struct.unpack_from('<H', packet, 42)[0]
    end_angle = end_angle_raw / 100.0

    # 타임스탬프
    timestamp = struct.unpack_from('<H', packet, 44)[0]

    # 각 포인트의 각도 보간
    angle_diff = end_angle - start_angle
    if angle_diff < 0:
        angle_diff += 360.0

    step = angle_diff / (POINTS_PER_PACKET - 1) if POINTS_PER_PACKET > 1 else 0

    point_data = []
    for i, (dist, conf) in enumerate(points):
        angle = (start_angle + step * i) % 360.0
        point_data.append((angle, dist, conf))

    return {
        'speed_deg_s': speed_deg_s,
        'start_angle': start_angle,
        'end_angle': end_angle,
        'timestamp': timestamp,
        'points': point_data,
    }


def print_scan_summary(scan_data, scan_count):
    """360도 스캔 데이터 요약을 출력합니다."""
    if not scan_data:
        return

    # 각도별 정렬
    scan_data.sort(key=lambda x: x[0])
    
    valid_points = [(a, d, c) for a, d, c in scan_data if d > 0 and c > 0]
    
    if not valid_points:
        print(f"  [스캔 #{scan_count}] 유효 데이터 없음 (모든 포인트가 거리 0 또는 신뢰도 0)")
        return

    distances = [d for _, d, _ in valid_points]
    confidences = [c for _, _, c in valid_points]
    
    min_dist = min(distances)
    max_dist = max(distances)
    avg_dist = sum(distances) / len(distances)
    avg_conf = sum(confidences) / len(confidences)
    
    # 최소 거리 포인트의 각도 찾기
    min_point = min(valid_points, key=lambda x: x[1])
    
    print(f"\n{'='*70}")
    print(f"  📡 LiDAR 스캔 #{scan_count} 결과")
    print(f"{'='*70}")
    print(f"  총 포인트 수     : {len(scan_data)}")
    print(f"  유효 포인트 수   : {len(valid_points)}")
    print(f"  평균 거리        : {avg_dist:.0f} mm ({avg_dist/1000:.2f} m)")
    print(f"  최소 거리        : {min_dist} mm ({min_dist/1000:.2f} m) @ {min_point[0]:.1f}°")
    print(f"  최대 거리        : {max_dist} mm ({max_dist/1000:.2f} m)")
    print(f"  평균 신뢰도      : {avg_conf:.0f} / 255")
    print(f"{'='*70}")
    
    # 8방향 요약 (전방=0°, 시계방향)
    directions = [
        ("  ⬆️  전방  (350°~ 10°)", 350, 10),
        ("  ↗️  우전방 (35°~ 55°)",  35, 55),
        ("  ➡️  우측  (80°~100°)",  80, 100),
        ("  ↘️  우후방(125°~145°)", 125, 145),
        ("  ⬇️  후방 (170°~190°)", 170, 190),
        ("  ↙️  좌후방(215°~235°)", 215, 235),
        ("  ⬅️  좌측 (260°~280°)", 260, 280),
        ("  ↖️  좌전방(305°~325°)", 305, 325),
    ]
    
    print(f"\n  {'─'*40}")
    print(f"  📐 8방향 최근접 거리 요약:")
    print(f"  {'─'*40}")
    
    for name, a_start, a_end in directions:
        if a_start > a_end:  # 전방(350~10)처럼 0도를 넘는 경우
            sector_pts = [d for a, d, c in valid_points if (a >= a_start or a <= a_end) and c > 0]
        else:
            sector_pts = [d for a, d, c in valid_points if a_start <= a <= a_end and c > 0]
        
        if sector_pts:
            nearest = min(sector_pts)
            bar_len = min(int(nearest / 200), 30)  # 200mm당 1블록, 최대 30
            bar = '█' * bar_len + '░' * (30 - bar_len)
            print(f"  {name}: {nearest:5d}mm ({nearest/1000:.2f}m) |{bar}|")
        else:
            print(f"  {name}: -- 데이터 없음 --")

    print()


def main():
    print(f"""
╔══════════════════════════════════════════════════════════╗
║       D500 (STL-19P) LiDAR 센서 테스트 프로그램         ║
║                                                          ║
║  포트     : {PORT:<42s}  ║
║  Baudrate : {BAUDRATE:<42d}  ║
║  Ctrl+C 로 종료                                          ║
╚══════════════════════════════════════════════════════════╝
""")

    # 시리얼 포트 열기
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2.0,
        )
        print(f"✅ 시리얼 포트 {PORT} 열기 성공!")
    except serial.SerialException as e:
        print(f"❌ 시리얼 포트 열기 실패: {e}")
        print(f"   - 장치가 연결되었는지 확인하세요 (ls /dev/ttyUSB*)")
        print(f"   - 권한을 확인하세요 (sudo chmod 666 /dev/ttyUSB0)")
        sys.exit(1)

    # Ctrl+C 핸들러
    running = True
    def signal_handler(sig, frame):
        nonlocal running
        running = False
        print("\n\n🛑 종료 신호 수신, 정리 중...")
    signal.signal(signal.SIGINT, signal_handler)

    # 데이터 수신 시작
    print("📡 라이다 데이터 수신 대기 중...")
    print("   (데이터가 오지 않으면 TX/RX 배선 및 전원을 확인하세요)\n")

    buffer = bytearray()
    packet_count = 0
    error_count = 0
    scan_count = 0
    scan_data = []  # 한 바퀴(360°) 동안의 포인트 누적
    prev_start_angle = None
    start_time = time.time()
    last_raw_print_time = 0

    # 첫 5초간 원시 바이트 확인용
    raw_check_done = False
    raw_bytes_received = 0

    try:
        while running:
            # 데이터 읽기
            incoming = ser.read(ser.in_waiting or 1)
            if not incoming:
                elapsed = time.time() - start_time
                if elapsed > 5 and not raw_check_done and raw_bytes_received == 0:
                    print("⚠️  5초간 데이터 수신 없음!")
                    print("   확인사항:")
                    print("   1. 라이다 전원(5V)이 연결되어 있나요?")
                    print("   2. USB 케이블이 제대로 연결되어 있나요?")
                    print("   3. 다른 프로그램이 이 포트를 사용 중이 아닌가요?")
                    print(f"      (확인: sudo fuser {PORT})")
                    raw_check_done = True
                continue

            raw_bytes_received += len(incoming)
            buffer.extend(incoming)

            # 첫 수신 시 원시 데이터 출력
            if not raw_check_done and raw_bytes_received > 0:
                now = time.time()
                if now - last_raw_print_time > 1.0:
                    print(f"  📥 원시 데이터 수신 중... ({raw_bytes_received} bytes)")
                    # 처음 20바이트 헥스 출력
                    sample = incoming[:min(20, len(incoming))]
                    hex_str = ' '.join(f'{b:02X}' for b in sample)
                    print(f"      HEX: {hex_str}")
                    last_raw_print_time = now
                    
                    if raw_bytes_received > 200:
                        raw_check_done = True
                        print(f"\n✅ 데이터 스트림 확인 완료! 패킷 파싱 시작...\n")

            # 패킷 동기화 및 파싱
            while len(buffer) >= PACKET_SIZE:
                # 헤더 바이트(0x54) 찾기
                try:
                    idx = buffer.index(HEADER)
                except ValueError:
                    buffer.clear()
                    break
                
                if idx > 0:
                    # 헤더 이전의 쓰레기 바이트 제거
                    buffer = buffer[idx:]

                if len(buffer) < PACKET_SIZE:
                    break

                # 두 번째 바이트(VerLen) 확인
                if buffer[1] != VERLEN:
                    buffer = buffer[1:]
                    continue

                # 패킷 추출 및 파싱
                raw_packet = bytes(buffer[:PACKET_SIZE])
                buffer = buffer[PACKET_SIZE:]

                result = parse_packet(raw_packet)
                if result is None:
                    error_count += 1
                    continue

                packet_count += 1

                # 각도 wrap-around 감지 (한 바퀴 완료)
                if prev_start_angle is not None:
                    if result['start_angle'] < prev_start_angle and \
                       (prev_start_angle - result['start_angle']) > 180:
                        # 360도 한 바퀴 완료
                        scan_count += 1
                        print_scan_summary(scan_data, scan_count)
                        
                        elapsed = time.time() - start_time
                        print(f"  📊 통계: 패킷 {packet_count}개 수신, "
                              f"CRC 에러 {error_count}개, "
                              f"회전 속도 {result['speed_deg_s']:.1f} deg/s, "
                              f"경과 시간 {elapsed:.1f}초")
                        
                        scan_data = []
                        
                        # 5번 스캔 후 종료
                        if scan_count >= 5:
                            print(f"\n✅ {scan_count}회 스캔 완료! 라이다 정상 동작 확인됨.")
                            running = False
                            break

                prev_start_angle = result['start_angle']
                scan_data.extend(result['points'])

    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        ser.close()
        print(f"\n{'='*70}")
        print(f"  📋 최종 테스트 결과 요약")
        print(f"{'='*70}")
        print(f"  총 수신 패킷       : {packet_count}")
        print(f"  CRC 에러 패킷      : {error_count}")
        print(f"  완료 스캔 수        : {scan_count}")
        print(f"  총 수신 바이트      : {raw_bytes_received}")
        elapsed = time.time() - start_time
        print(f"  총 경과 시간        : {elapsed:.1f}초")
        
        if packet_count > 0:
            print(f"\n  ✅ 결론: D500 LiDAR 센서 정상 동작!")
            print(f"     → ROS 2 드라이버 노드 개발로 진행하세요.")
        elif raw_bytes_received > 0:
            print(f"\n  ⚠️  결론: 데이터는 수신되나 패킷 파싱 실패")
            print(f"     → Baudrate({BAUDRATE}) 또는 프로토콜 확인 필요")
        else:
            print(f"\n  ❌ 결론: 데이터 수신 없음")
            print(f"     → 배선, 전원, 포트 번호 확인 필요")
        print(f"{'='*70}\n")


if __name__ == '__main__':
    main()
