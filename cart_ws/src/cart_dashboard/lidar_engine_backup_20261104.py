#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR 데이터 수신 및 2D 점군 처리 모듈
- D500 (STL-19P) 시리얼 데이터 수신 및 실시간 360도 스캔 버퍼 유지
- 20cm 차체 마스킹 필터 (센서 주변 20cm 이내 카트 차체/케이블 간섭 무시)
- 후방 120° 블라인드 존 처리 (손잡이를 잡고 걷는 시각장애인 본인 간섭 배제)
- 최근 5개 프레임 중앙값(Median) 스무딩 & 3프레임 연속 상태 히스테리시스
- 0.6m 비상정지 / 1.5m 감속 주행 구역 판정
"""

import os
import math
import time
import threading
import statistics
from collections import deque
import serial
import serial.tools.list_ports

class LidarEngine:
    def __init__(self, port="/dev/ttyCartLiDAR", baudrate=230400, mask_radius=0.20, max_range=8.0):
        self.port = port
        self.baudrate = baudrate
        self.mask_radius = mask_radius  # 20cm 본체 마스킹 반경
        self.max_range = max_range      # 8.0m 최대 유효 탐지 거리 (성능 저하 없는 최적값)
        self.ser = None
        self.running = True
        self.lock = threading.Lock()
        
        # 360도 각도별 거리 배열 (0~359도, 단위: 미터)
        self.scan_ranges = [self.max_range] * 360
        self.scan_buffers = [deque(maxlen=5) for _ in range(360)]
        self.point_timestamps = [0.0] * 360
        
        self.min_front_distance = 10.0
        self.closest_obstacle = (10.0, 0) # (거리m, 각도deg)
        self.user_distance = None          # 후방 사용자(보행자) 거리 (m)
        self.warning_state = "SAFE"        # SAFE, CAUTION, DANGER
        self.state_buffer = deque(maxlen=3)
        self.is_connected = False
        self.packet_rate = 0.0

        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _find_port(self):
        if os.path.exists(self.port):
            return self.port
        # CP2102 USB 시리얼 포트 우선 탐색
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

    def _worker(self):
        pkt_count = 0
        last_rate_time = time.time()

        while self.running:
            if not self.is_connected:
                if not self._connect():
                    self._generate_mock_scan()
                    time.sleep(0.05)
                    continue

            try:
                header = self.ser.read(2)
                if len(header) < 2:
                    continue
                if header[0] == 0x54 and header[1] == 0x2C:
                    data = self.ser.read(45)
                    if len(data) != 45:
                        continue

                    pkt_count += 1
                    now = time.time()
                    if now - last_rate_time >= 1.0:
                        self.packet_rate = pkt_count / (now - last_rate_time)
                        pkt_count = 0
                        last_rate_time = now

                    start_angle = ((data[3] << 8) | data[2]) / 100.0
                    end_angle = ((data[41] << 8) | data[40]) / 100.0
                    if end_angle < start_angle:
                        end_angle += 360.0
                    
                    step = (end_angle - start_angle) / 11.0

                    with self.lock:
                        for i in range(12):
                            dist_mm = (data[5 + i*3] << 8) | data[4 + i*3]
                            conf = data[6 + i*3]
                            
                            # 신뢰도 필터 & 센서 스펙 최소 3cm(30mm) 이상
                            if conf > 10 and dist_mm >= 30:
                                ang = int(round(start_angle + step * i)) % 360
                                dist_m = dist_mm / 1000.0
                                self.scan_buffers[ang].append(dist_m)
                                self.scan_ranges[ang] = statistics.median(self.scan_buffers[ang])
                                self.point_timestamps[ang] = now

                    self._evaluate_safety()

            except Exception:
                self.is_connected = False
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                time.sleep(1.0)

    def _generate_mock_scan(self):
        """센서 미연결 시 가상 장애물 시뮬레이션 데이터 제공"""
        t = time.time()
        with self.lock:
            for deg in range(360):
                rad = math.radians(deg)
                dist = 4.0 + math.sin(rad * 4) * 0.5
                if deg > 345 or deg < 15:
                    obs_dist = 1.2 + math.sin(t * 1.5) * 0.5
                    dist = min(dist, obs_dist)
                self.scan_ranges[deg] = dist
        self._evaluate_safety()

    def _evaluate_safety(self):
        """
        시각장애인 카트 맞춤형 안전성 평가:
        1. 20cm 본체 마스킹 (카트 프레임 간섭 무시)
        2. 후방(120°~240°) 사용자 블라인드 존 배제
        3. 전방 90° (315°~45°) 장애물 기준 안전/감속/정지 판정
        4. 연속 3프레임 디바운스(Hysteresis) 필터링
        """
        now = time.time()
        user_dists = []

        with self.lock:
            for deg in range(360):
                # 0.5초 이내의 유효 포인트만 처리
                if now - self.point_timestamps[deg] <= 0.5:
                    d = self.scan_ranges[deg]
                    # 본체 마스킹 거리(20cm) 이상인 외부 물체만 취급
                    if self.mask_radius <= d <= self.max_range:
                        # 후방 사용자 보행 구역 (120° ~ 240°) 판별
                        if 120 <= deg <= 240:
                            user_dists.append(d)  # 사용자 보행자 추적용 (장애물 목록에는 넣지 않음)
                        else:
                            external_points.append((d, deg))

                        # 전방 90도 부채꼴 집중 감시 (315° ~ 45°)
                        if deg >= 315 or deg <= 45:
                            front_dists.append(d)

            # 전방 최단거리 계산
            self.min_front_distance = min(front_dists) if front_dists else 10.0
            
            # 사용자 거리 (뒤에서 손잡이를 잡고 따라오는 보행자 거리)
            self.user_distance = min(user_dists) if user_dists else None
            
            # 외부 장애물 최단거리 (후방 사용자 제외)
            if external_points:
                self.closest_obstacle = min(external_points, key=lambda x: x[0])
            else:
                self.closest_obstacle = (10.0, 0)

            # 원시 상태 판정
            if self.min_front_distance < 0.60:
                raw_state = "DANGER"   # 비상 정지 대상
            elif self.min_front_distance < 1.50:
                raw_state = "CAUTION"  # 감속 대상
            else:
                raw_state = "SAFE"     # 정상 주행

            # 연속 3프레임 일치 시에만 상태 전환 (급정거 깜빡임 방지)
            self.state_buffer.append(raw_state)
            if len(self.state_buffer) == 3 and len(set(self.state_buffer)) == 1:
                self.warning_state = raw_state

    def get_8dir_distances(self):
        """
        8방향 (N, NE, E, SE, S, SW, W, NW) 실시간 거리 반환 (단위: 미터)
        각 방향 중심 ±20° 부채꼴 영역 최솟값 (본체 20cm 마스킹 필터 적용)
        """
        dirs = {
            "N": 0, "NE": 45, "E": 90, "SE": 135,
            "S": 180, "SW": 225, "W": 270, "NW": 315
        }
        res = {}
        now = time.time()
        with self.lock:
            for d_name, center in dirs.items():
                vals = [
                    self.scan_ranges[(center + o) % 360]
                    for o in range(-20, 21)
                    if (now - self.point_timestamps[(center + o) % 360] <= 0.5)
                    and self.mask_radius <= self.scan_ranges[(center + o) % 360] <= self.max_range
                ]
                res[d_name] = round(min(vals), 2) if vals else None
        return res

    def get_scan_data(self):
        """카트 제어 루프 및 대시보드 반환 API"""
        with self.lock:
            return list(self.scan_ranges), self.min_front_distance, self.warning_state

    def stop(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

