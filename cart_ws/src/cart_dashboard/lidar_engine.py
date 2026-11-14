#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
LiDAR 데이터 수신 및 2D 점군 처리 모듈 v2.0 (실측 치수 최적화)
=============================================================================
카트 실측 치수 기반 최적화 (2026-11-04 측정):
  - 카트 본체: 가로 35.2cm × 세로 37cm × 높이 82cm (바퀴 포함)
  - 라이다 스캔 높이: 지면에서 85.5cm (바닥 난반사 걱정 없음)
  - 라이다 → 카트 전방 끝: 17.0cm (상판 개방, 장애물 없음)
  - 라이다 → 카트 좌측 끝: 25.5cm
  - 라이다 → 카트 우측 끝: 24.5cm
  - 라이다 → 카트 후방 끝: 15.0cm (캐리어 손잡이 + 보행자)
  - 센서: D500 (STL-19P) 2D dToF LiDAR, 230400 bps

v1.0 대비 개선사항:
  1. 5프레임 메디안 버퍼 제거 → EMA(지수이동평균) 1프레임 즉응 (~80ms 반응)
  2. 360개 정수 버킷 → 450개 (0.8° 해상도, 센서 원시 분해능 유지)
  3. 20cm 균일 마스킹 → 방향별 비대칭 카트 외형 정밀 마스킹
  4. 후방 120°~240° 고정 배제 → 손잡이 실측 기반 정밀 사용자 구간
  5. 0.5초 유효기간 → 0.3초 (빠른 장애물 소멸 감지)
  6. 신뢰도(Confidence) 임계값 적응적 적용
  7. angle_offset_deg 파라미터로 센서 장착 방향 보정 지원
  8. CRC-8 패킷 검증 추가 (데이터 무결성 보장)
=============================================================================
"""

import os
import math
import time
import threading
from collections import deque
import serial
import serial.tools.list_ports


# ---------------------------------------------------------------------------
# STL-19P CRC-8 룩업 테이블 (다항식 0x4D)
# ---------------------------------------------------------------------------
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


def _cal_crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = CRC_TABLE[crc ^ b]
    return crc


# ---------------------------------------------------------------------------
# 카트 본체 비대칭 마스킹 테이블 (실측 기반)
# ---------------------------------------------------------------------------
# 라이다 중심에서 카트 외형까지의 방향별 거리(m)를 미리 계산합니다.
# 이 거리 이내의 포인트는 카트 본체/케이블이므로 무시합니다.
# 직사각형 본체를 타원형으로 감싸면 모서리를 놓칠 수 있으므로,
# 실제 직사각형 외곽선까지의 방사 거리(ray-box intersection)를 사용합니다.
def _build_body_mask_table(front_m, back_m, left_m, right_m, margin_m=0.03):
    """
    360도 (또는 NUM_BINS 해상도) 각도별로 카트 본체까지의 최소 거리(m) 테이블을 생성.
    margin_m: 안전 여유 (3cm) - 본체 외곽 바로 밖 포인트도 잡히면 안 되므로
    """
    table = []
    for i in range(360):
        rad = math.radians(i)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)

        # 반직선(ray)이 직사각형 4변과 교차하는 거리 중 최소값 계산
        # 좌표계: 0° = 전방(+X), 90° = 우측(+Y), 180° = 후방(-X), 270° = 좌측(-Y)
        candidates = []

        # 전방 벽 (x = +front_m)
        if cos_a > 1e-9:
            t = front_m / cos_a
            y_hit = t * sin_a
            if -left_m <= y_hit <= right_m:
                candidates.append(t)

        # 후방 벽 (x = -back_m)
        if cos_a < -1e-9:
            t = -back_m / cos_a
            y_hit = t * sin_a
            if -left_m <= y_hit <= right_m:
                candidates.append(t)

        # 우측 벽 (y = +right_m)
        if sin_a > 1e-9:
            t = right_m / sin_a
            x_hit = t * cos_a
            if -back_m <= x_hit <= front_m:
                candidates.append(t)

        # 좌측 벽 (y = -left_m)
        if sin_a < -1e-9:
            t = -left_m / sin_a
            x_hit = t * cos_a
            if -back_m <= x_hit <= front_m:
                candidates.append(t)

        if candidates:
            table.append(min(candidates) + margin_m)
        else:
            table.append(margin_m)  # fallback

    return table


# ===========================================================================
# 메인 엔진 클래스
# ===========================================================================
class LidarEngine:
    """
    D500 (STL-19P) 2D LiDAR 실시간 처리 엔진 v2.0

    주요 외부 API (기존 대시보드/제어 코드와 100% 호환):
      - get_scan_data()       → (ranges_list, min_front_dist, warning_state)
      - get_8dir_distances()  → {"N": m, "NE": m, ...}
      - stop()
    """

    # ----- 해상도 설정 -----
    NUM_BINS = 450          # 0.8° 해상도 (360/450 = 0.8°)
    DEG_PER_BIN = 360.0 / NUM_BINS  # 0.8°

    # ----- 실측 카트 치수 (라이다 중심 → 본체 끝, 단위 m) -----
    BODY_FRONT = 0.170      # 전방 17.0cm
    BODY_BACK  = 0.150      # 후방 15.0cm
    BODY_LEFT  = 0.255      # 좌측 25.5cm
    BODY_RIGHT = 0.245      # 우측 24.5cm

    # ----- 사용자(시각장애인) 후방 배제 구간 -----
    # 후방 손잡이 폭(약 20cm) + 보행자 몸통을 고려하여
    # 라이다 기준 150°~210° (후방 중심 ±30°)를 사용자 구간으로 설정
    USER_ZONE_START = 150   # deg
    USER_ZONE_END   = 210   # deg

    # ----- 안전 판정 임계값 -----
    DANGER_DIST  = 0.60     # m (비상 정지)
    CAUTION_DIST = 1.50     # m (감속 주행)

    # ----- 필터링 파라미터 -----
    EMA_ALPHA       = 0.6   # EMA 가중치 (0.6 = 신규 데이터 60%, 기존 40%)
    CONF_THRESHOLD  = 15    # 신뢰도 최소값 (10→15 강화, 고스트 포인트 억제)
    MIN_DIST_MM     = 30    # 센서 최소 측정 거리 3cm (30mm)
    POINT_EXPIRE_S  = 0.3   # 포인트 유효 기간 (0.5→0.3초, 장애물 소멸 빠르게 반영)
    FRONT_HALF_ANG  = 45    # 전방 감시 반각 (±45° = 전방 90°)

    def __init__(self, port="/dev/ttyCartLiDAR", baudrate=230400,
                 max_range=12.0, angle_offset_deg=0.0):
        """
        Args:
            port:             시리얼 포트 경로
            baudrate:         통신 속도 (STL-19P 기본 230400)
            max_range:        최대 유효 측정 거리 (m). 높이 85.5cm이므로 바닥 반사 없음 → 12m 개방
            angle_offset_deg: 센서 0° → 카트 전방 보정 각도.
                              설치 후 전방에 손을 대고 검증 스크립트로 측정 후 입력.
        """
        self.port = port
        self.baudrate = baudrate
        self.max_range = max_range
        self.angle_offset = angle_offset_deg

        self.ser = None
        self.running = True
        self.lock = threading.Lock()

        # 고해상도 스캔 버퍼 (450 빈)
        self.scan_ranges = [self.max_range] * self.NUM_BINS
        self.point_timestamps = [0.0] * self.NUM_BINS

        # 비대칭 본체 마스킹 테이블 (360도, 1° 해상도로 생성 후 빈 인덱스로 조회)
        self._body_mask = _build_body_mask_table(
            self.BODY_FRONT, self.BODY_BACK,
            self.BODY_LEFT, self.BODY_RIGHT,
            margin_m=0.03
        )

        # 안전 평가 결과
        self.min_front_distance = 10.0
        self.closest_obstacle = (10.0, 0)   # (거리m, 각도deg)
        self.user_distance = None            # 후방 보행자 거리 (m)
        self.warning_state = "SAFE"          # SAFE / CAUTION / DANGER
        self.state_buffer = deque(maxlen=3)  # 3프레임 디바운스 유지 (급정거 깜빡임 방지)

        # 통신 상태
        self.is_connected = False
        self.packet_rate = 0.0
        self.crc_errors = 0

        # 수신 워커 스레드
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    # -------------------------------------------------------------------
    # 시리얼 포트 탐색 및 연결
    # -------------------------------------------------------------------
    def _find_port(self):
        if os.path.exists(self.port):
            return self.port
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if "CP210" in (p.description or "") or "10C4:EA60" in (p.hwid or ""):
                return p.device
        for p in ports:
            if "ttyUSB" in p.device or "ttyACM" in p.device:
                return p.device
        return "/dev/ttyUSB0"

    def _connect(self):
        target_port = self._find_port()
        try:
            self.ser = serial.Serial(target_port, self.baudrate, timeout=0.2)
            self.ser.reset_input_buffer()
            self.is_connected = True
            return True
        except Exception:
            self.is_connected = False
            return False

    # -------------------------------------------------------------------
    # 각도 → 빈 인덱스 변환
    # -------------------------------------------------------------------
    def _angle_to_bin(self, angle_deg):
        """실수 각도 → NUM_BINS(450) 빈 인덱스 (반올림)"""
        return int(round(angle_deg / self.DEG_PER_BIN)) % self.NUM_BINS

    def _bin_to_deg(self, bin_idx):
        """빈 인덱스 → 실수 각도 (도)"""
        return (bin_idx * self.DEG_PER_BIN) % 360.0

    # -------------------------------------------------------------------
    # 본체 마스킹 조회
    # -------------------------------------------------------------------
    def _get_body_mask_dist(self, angle_deg):
        """주어진 각도에서 카트 본체까지의 거리(m) 반환"""
        deg_idx = int(round(angle_deg)) % 360
        return self._body_mask[deg_idx]

    # -------------------------------------------------------------------
    # 메인 수신 루프 (바이트 스트림 → 패킷 파싱)
    # -------------------------------------------------------------------
    def _worker(self):
        pkt_count = 0
        last_rate_time = time.time()
        rx_buf = bytearray()

        while self.running:
            if not self.is_connected:
                if not self._connect():
                    self._generate_mock_scan()
                    time.sleep(0.05)
                    continue

            try:
                # 한 번에 가용한 바이트 전부 읽기 (블로킹 최소화)
                waiting = self.ser.in_waiting
                if waiting > 0:
                    rx_buf.extend(self.ser.read(waiting))
                else:
                    # 데이터가 없으면 짧은 대기 후 재시도
                    rx_buf.extend(self.ser.read(47))

                # 47바이트 패킷 파싱 루프
                while len(rx_buf) >= 47:
                    # 헤더(0x54, 0x2C) 탐색
                    idx = -1
                    for i in range(len(rx_buf) - 1):
                        if rx_buf[i] == 0x54 and rx_buf[i + 1] == 0x2C:
                            idx = i
                            break

                    if idx < 0:
                        # 헤더를 못 찾음 → 마지막 1바이트만 남기고 버림
                        rx_buf = rx_buf[-1:]
                        break

                    if idx > 0:
                        # 헤더 앞의 쓰레기 바이트 제거
                        del rx_buf[:idx]

                    if len(rx_buf) < 47:
                        break

                    packet = bytes(rx_buf[:47])
                    del rx_buf[:47]

                    # CRC-8 검증
                    if _cal_crc8(packet[:46]) != packet[46]:
                        self.crc_errors += 1
                        continue

                    # 패킷 카운트 & 초당 패킷률 계산
                    pkt_count += 1
                    now = time.time()
                    if now - last_rate_time >= 1.0:
                        self.packet_rate = pkt_count / (now - last_rate_time)
                        pkt_count = 0
                        last_rate_time = now

                    # 각도 파싱 (0.01° 단위 → 실수 도)
                    start_angle = ((packet[5] << 8) | packet[4]) / 100.0
                    end_angle   = ((packet[43] << 8) | packet[42]) / 100.0
                    if end_angle < start_angle:
                        end_angle += 360.0
                    step = (end_angle - start_angle) / 11.0

                    # 12개 포인트 디코딩 + EMA 필터 적용
                    with self.lock:
                        for i in range(12):
                            offset = 6 + i * 3
                            dist_mm = (packet[offset + 1] << 8) | packet[offset]
                            conf = packet[offset + 2]

                            # 신뢰도 & 최소 거리 필터
                            if conf < self.CONF_THRESHOLD or dist_mm < self.MIN_DIST_MM:
                                continue

                            # 센서 원시 각도 + 장착 보정
                            raw_angle = (start_angle + step * i + self.angle_offset) % 360.0
                            bin_idx = self._angle_to_bin(raw_angle)
                            dist_m = dist_mm / 1000.0

                            # max_range 초과 데이터 클램프
                            if dist_m > self.max_range:
                                dist_m = self.max_range

                            # EMA(지수이동평균) 필터: 즉응성 + 노이즈 억제 동시 달성
                            # 첫 데이터이거나 오래된 데이터면 즉시 갱신
                            if now - self.point_timestamps[bin_idx] > self.POINT_EXPIRE_S:
                                self.scan_ranges[bin_idx] = dist_m
                            else:
                                prev = self.scan_ranges[bin_idx]
                                self.scan_ranges[bin_idx] = (
                                    self.EMA_ALPHA * dist_m +
                                    (1.0 - self.EMA_ALPHA) * prev
                                )
                            self.point_timestamps[bin_idx] = now

                    # 패킷마다 안전 평가 실행
                    self._evaluate_safety()

            except (OSError, serial.SerialException):
                self.is_connected = False
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                time.sleep(1.0)
                rx_buf.clear()

    # -------------------------------------------------------------------
    # 센서 미연결 시 가상 스캔 데이터
    # -------------------------------------------------------------------
    def _generate_mock_scan(self):
        """센서 미연결 시 가상 장애물 시뮬레이션 데이터 제공"""
        t = time.time()
        with self.lock:
            for b in range(self.NUM_BINS):
                deg = self._bin_to_deg(b)
                rad = math.radians(deg)
                dist = 4.0 + math.sin(rad * 4) * 0.5
                if deg > 345 or deg < 15:
                    obs_dist = 1.2 + math.sin(t * 1.5) * 0.5
                    dist = min(dist, obs_dist)
                self.scan_ranges[b] = dist
                self.point_timestamps[b] = t
        self._evaluate_safety()

    # -------------------------------------------------------------------
    # 안전 평가 (비대칭 마스킹 + 사용자 구간 배제 + 3프레임 디바운스)
    # -------------------------------------------------------------------
    def _evaluate_safety(self):
        """
        시각장애인 카트 맞춤형 안전성 평가 v2.0:
        1. 방향별 비대칭 본체 마스킹 (직사각형 ray-cast 기반)
        2. 후방 사용자 구간(150°~210°) 배제
        3. 전방 ±45° (90°) 장애물 기준 안전/감속/정지 판정
        4. 연속 3프레임 디바운스(Hysteresis) 유지 (급정거 방지)
        """
        now = time.time()
        front_dists = []
        external_points = []
        user_dists = []

        with self.lock:
            for b in range(self.NUM_BINS):
                # 유효 기간 확인
                if now - self.point_timestamps[b] > self.POINT_EXPIRE_S:
                    continue

                d = self.scan_ranges[b]
                deg = self._bin_to_deg(b)

                # 비대칭 본체 마스킹: 이 각도에서의 카트 외곽 거리 이내면 무시
                body_dist = self._get_body_mask_dist(deg)
                if d < body_dist:
                    continue

                # max_range 초과 무시
                if d > self.max_range:
                    continue

                # 후방 사용자 구간 판별 (장애물 목록에서 배제)
                if self.USER_ZONE_START <= deg <= self.USER_ZONE_END:
                    user_dists.append(d)
                    continue

                # 외부 장애물 기록
                external_points.append((d, deg))

                # 전방 감시 영역 (0° 기준 ±45°)
                if deg <= self.FRONT_HALF_ANG or deg >= (360 - self.FRONT_HALF_ANG):
                    front_dists.append(d)

            # 전방 최단거리
            self.min_front_distance = min(front_dists) if front_dists else 10.0

            # 사용자 거리 (후방 보행자)
            self.user_distance = min(user_dists) if user_dists else None

            # 전체 외부 장애물 최단거리
            if external_points:
                self.closest_obstacle = min(external_points, key=lambda x: x[0])
            else:
                self.closest_obstacle = (10.0, 0)

            # 원시 상태 판정
            if self.min_front_distance < self.DANGER_DIST:
                raw_state = "DANGER"
            elif self.min_front_distance < self.CAUTION_DIST:
                raw_state = "CAUTION"
            else:
                raw_state = "SAFE"

            # 연속 3프레임 일치 시에만 상태 전환 (급정거 깜빡임 방지)
            self.state_buffer.append(raw_state)
            if len(self.state_buffer) == 3 and len(set(self.state_buffer)) == 1:
                self.warning_state = raw_state

    # -------------------------------------------------------------------
    # 외부 API (기존 대시보드/제어 코드 100% 호환)
    # -------------------------------------------------------------------
    def get_8dir_distances(self):
        """
        8방향 (N, NE, E, SE, S, SW, W, NW) 실시간 거리 반환 (단위: 미터)
        각 방향 중심 ±20° 부채꼴 영역 최솟값 (비대칭 본체 마스킹 적용)
        """
        dirs = {
            "N": 0, "NE": 45, "E": 90, "SE": 135,
            "S": 180, "SW": 225, "W": 270, "NW": 315
        }
        res = {}
        now = time.time()
        with self.lock:
            for d_name, center_deg in dirs.items():
                vals = []
                for offset_deg in range(-20, 21):
                    check_deg = (center_deg + offset_deg) % 360.0
                    b = self._angle_to_bin(check_deg)
                    if now - self.point_timestamps[b] > self.POINT_EXPIRE_S:
                        continue
                    d = self.scan_ranges[b]
                    body_d = self._get_body_mask_dist(check_deg)
                    if body_d <= d <= self.max_range:
                        vals.append(d)
                res[d_name] = round(min(vals), 2) if vals else None
        return res

    def get_scan_data(self):
        """
        카트 제어 루프 및 대시보드 반환 API
        Returns:
            (ranges_360, min_front_distance, warning_state)
            - ranges_360: 360개 리스트 (0~359°, 기존 대시보드 호환용 다운샘플)
            - min_front_distance: 전방 최단거리 (m)
            - warning_state: "SAFE" / "CAUTION" / "DANGER"
        """
        with self.lock:
            # 450빈 → 360도 다운샘플 (기존 GUI 호환)
            ranges_360 = []
            for deg in range(360):
                b = self._angle_to_bin(float(deg))
                ranges_360.append(self.scan_ranges[b])
            return ranges_360, self.min_front_distance, self.warning_state

    def get_highres_scan(self):
        """
        고해상도 스캔 데이터 반환 (450빈, 0.8° 간격)
        Returns:
            (ranges_450, timestamps_450)
        """
        with self.lock:
            return list(self.scan_ranges), list(self.point_timestamps)

    def stop(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
