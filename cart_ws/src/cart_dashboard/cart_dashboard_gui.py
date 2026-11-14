#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2026 자율주행 스마트 카트 4분할 통합 GUI 대시보드 & 제어 시스템
- [1분할/좌상단] TMAP 지도 & 실시간 GPS 보행자 내비게이션
- [2분할/우상단] LiDAR 360도 스캔 레이더 GUI (위험구역 0.6m/1.5m)
- [3분할/좌하단] 속도계, 차량 텔레메트리, 수동(WASD)/자율 모드 통합 제어기
- [4분할/우하단] 실시간 카메라 스트림 및 YOLO AI 객체 인식 결과
"""

import os
import sys
import math
import time
import cv2
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QGridLayout, QVBoxLayout,
    QHBoxLayout, QLabel, QPushButton, QFrame, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QRectF, QPointF
from PyQt5.QtGui import (
    QImage, QPixmap, QPainter, QColor, QPen, QBrush, QFont, QRadialGradient
)

# QWebEngineView (가능한 경우 사용, 없으면 고성능 2D 캔버스 지도 폴백)
try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    HAS_WEBENGINE = True
except ImportError:
    HAS_WEBENGINE = False

# 서브모듈 임포트
from tmap_engine import TmapEngine
from lidar_engine import LidarEngine
from vision_engine import VisionEngine

# ESP32 시리얼 통신 제어
import serial
import serial.tools.list_ports


# ==========================================
# 1. LiDAR 레이더 뷰어 커스텀 위젯 (2분할)
# ==========================================
class LidarRadarWidget(QWidget):
    def __init__(self, lidar_engine, parent=None):
        super().__init__(parent)
        self.lidar_engine = lidar_engine
        self.sweep_angle = 0.0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_radar)
        self.timer.start(50) # 20Hz

    def update_radar(self):
        self.sweep_angle = (self.sweep_angle + 6.0) % 360.0
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        radius = min(cx, cy) - 25.0
        if radius <= 10:
            return

        # 1. 배경 어두운 레이더 그라디언트
        bg_grad = QRadialGradient(cx, cy, radius)
        bg_grad.setColorAt(0.0, QColor(10, 20, 35))
        bg_grad.setColorAt(1.0, QColor(5, 10, 20))
        painter.fillRect(self.rect(), QColor(15, 23, 42))
        painter.setBrush(QBrush(bg_grad))
        painter.setPen(QPen(QColor(30, 58, 138), 1))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        # 2. 동심원 가이드 (1m, 2m, 3m, 4m 기준)
        max_dist = 4.0 # 최대 4m 반경으로 줌
        for d in [1.0, 2.0, 3.0, 4.0]:
            r = (d / max_dist) * radius
            pen_color = QColor(56, 189, 248, 60)
            if d == 1.0: # 위험 구역
                pen_color = QColor(239, 68, 68, 90)
            elif d == 2.0: # 경고 구역
                pen_color = QColor(245, 158, 11, 70)
            painter.setPen(QPen(pen_color, 1, Qt.DashLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QPointF(cx, cy), r, r)
            painter.setPen(QPen(QColor(148, 163, 184), 1))
            painter.setFont(QFont("Segoe UI", 8))
            painter.drawText(int(cx + 4), int(cy - r + 12), f"{d:.0f}m")

        # 3. 십자선 및 부채꼴 전방 위험 영역 표시
        painter.setPen(QPen(QColor(56, 189, 248, 80), 1))
        painter.drawLine(int(cx), int(cy - radius), int(cx), int(cy + radius))
        painter.drawLine(int(cx - radius), int(cy), int(cx + radius), int(cy))

        # 전방 1.5m / 0.6m 안전존 하이라이트
        r_06 = (0.6 / max_dist) * radius
        r_15 = (1.5 / max_dist) * radius
        painter.setBrush(QBrush(QColor(239, 68, 68, 30))) # 0.6m DANGER
        painter.setPen(Qt.NoPen)
        painter.drawPie(QRectF(cx - r_06, cy - r_06, r_06 * 2, r_06 * 2), 45 * 16, 90 * 16)
        painter.setBrush(QBrush(QColor(245, 158, 11, 20))) # 1.5m CAUTION
        painter.drawPie(QRectF(cx - r_15, cy - r_15, r_15 * 2, r_15 * 2), 45 * 16, 90 * 16)

        # 4. 라이다 360도 점군(Point Cloud) 그리기
        ranges, min_front, state = self.lidar_engine.get_scan_data()
        
        for deg in range(360):
            dist = ranges[deg]
            if dist < 0.05 or dist > max_dist:
                continue
            # 카트 좌표계: 0도가 전방(위쪽, 90도 회전)
            ang_rad = math.radians(deg - 90)
            pt_r = (dist / max_dist) * radius
            px = cx + pt_r * math.cos(ang_rad)
            py = cy + pt_r * math.sin(ang_rad)

            # 점 색상 (거리별 색상)
            if dist < 0.20:
                pt_color = QColor(100, 116, 139, 80) # 20cm 본체 마스킹 영역 (연한 회색)
            elif dist < 0.6:
                pt_color = QColor(255, 60, 60, 240)
            elif dist < 1.5:
                pt_color = QColor(255, 200, 0, 220)
            else:
                pt_color = QColor(0, 240, 255, 200)

            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(pt_color))
            painter.drawEllipse(QPointF(px, py), 2.5, 2.5)

        # 5. 회전 스위프 빔 효과
        sweep_rad = math.radians(self.sweep_angle - 90)
        sx = cx + radius * math.cos(sweep_rad)
        sy = cy + radius * math.sin(sweep_rad)
        painter.setPen(QPen(QColor(56, 189, 248, 150), 2))
        painter.drawLine(int(cx), int(cy), int(sx), int(sy))

        # 6. 중앙 로봇 카트 본체 아이콘
        painter.setBrush(QBrush(QColor(16, 185, 129)))
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawRect(int(cx - 8), int(cy - 12), 16, 24)
        # 전방 헤딩 화살표
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        triangle = [QPointF(cx, cy - 18), QPointF(cx - 5, cy - 12), QPointF(cx + 5, cy - 12)]
        painter.drawPolygon(*triangle)

        # 7. 상태 텍스트 배너
        status_color = QColor(16, 185, 129) if state == "SAFE" else (QColor(245, 158, 11) if state == "CAUTION" else QColor(239, 68, 68))
        painter.setPen(QPen(status_color, 1))
        painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
        painter.drawText(15, 25, f"LiDAR 상태: {state}")
        painter.setFont(QFont("Segoe UI", 9))
        painter.drawText(15, 45, f"전방 최단거리: {min_front:.2f} m")


# ==========================================
# 2. 고성능 2D 지도 폴백 위젯 (1분할)
# ==========================================
class CanvasMapWidget(QWidget):
    def __init__(self, tmap_engine, parent=None):
        super().__init__(parent)
        self.tmap = tmap_engine
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(100)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        
        # 다크 배경
        painter.fillRect(self.rect(), QColor(15, 23, 42))

        # 모눈 격자
        painter.setPen(QPen(QColor(30, 41, 59), 1))
        for x in range(0, w, 40): painter.drawLine(x, 0, x, h)
        for y in range(0, h, 40): painter.drawLine(0, y, w, y)

        # 가상 도로망
        painter.setPen(QPen(QColor(51, 65, 85), 18))
        painter.drawLine(50, h - 80, w - 80, 80)
        painter.drawLine(80, 100, w - 50, h - 100)

        # TMAP 경로 Polyline
        if len(self.tmap.route_points) >= 2:
            pts = []
            min_lat = min(p[0] for p in self.tmap.route_points)
            max_lat = max(p[0] for p in self.tmap.route_points)
            min_lon = min(p[1] for p in self.tmap.route_points)
            max_lon = max(p[1] for p in self.tmap.route_points)

            lat_span = max(max_lat - min_lat, 0.0001)
            lon_span = max(max_lon - min_lon, 0.0001)

            for lat, lon in self.tmap.route_points:
                px = 60 + ((lon - min_lon) / lon_span) * (w - 120)
                py = (h - 60) - ((lat - min_lat) / lat_span) * (h - 120)
                pts.append(QPointF(px, py))

            # 발광 네온 경로선
            painter.setPen(QPen(QColor(0, 240, 255, 80), 8))
            for i in range(len(pts)-1): painter.drawLine(pts[i], pts[i+1])
            painter.setPen(QPen(QColor(0, 240, 255, 255), 3, Qt.DashLine))
            for i in range(len(pts)-1): painter.drawLine(pts[i], pts[i+1])

            # 시작점 및 끝점
            painter.setBrush(QBrush(QColor(16, 185, 129)))
            painter.setPen(QPen(Qt.white, 2))
            painter.drawEllipse(pts[0], 8, 8)
            painter.setBrush(QBrush(QColor(239, 68, 68)))
            painter.drawEllipse(pts[-1], 8, 8)

        # HUD 상단 오버레이 바
        painter.setBrush(QBrush(QColor(30, 41, 59, 220)))
        painter.setPen(QPen(QColor(56, 189, 248, 120), 1))
        painter.drawRoundedRect(QRectF(15, 15, w - 30, 50), 8, 8)

        guide = self.tmap.guide_descriptions[0] if self.tmap.guide_descriptions else "경로 안내 준비 완료"
        painter.setPen(QPen(QColor(248, 250, 252), 1))
        painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
        painter.drawText(25, 45, f"📍 {guide}")


# ==========================================
# 3. 메인 통합 4분할 윈도우
# ==========================================
class CartMainDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("2026 자율주행 스마트 카트 통합 관제 & 제어 대시보드")
        self.resize(1366, 768)
        self.setStyleSheet("background-color: #0b0f19; color: #f8fafc;")

        # 엔진 인스턴스 초기화
        self.tmap = TmapEngine()
        self.tmap.request_pedestrian_route()
        self.lidar = LidarEngine()
        self.vision = VisionEngine()

        # ESP32 시리얼 통신 초기화
        self.ser = None
        self._init_esp32_serial()

        # 주행 상태 변수
        self.mode = "MANUAL" # "MANUAL", "AUTO", "EMERGENCY_STOP"
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.current_speed = 0.0 # m/s
        self.battery_volt = 12.4
        self.pressed_keys = set()

        # UI 레이아웃 구축
        self._setup_ui()

        # 타이머 루프 (GUI 갱신 & 제어 명령 송신)
        self.main_timer = QTimer(self)
        self.main_timer.timeout.connect(self._main_loop)
        self.main_timer.start(50) # 20Hz

    def _init_esp32_serial(self):
        port = "/dev/ttyCartESP32"
        if not os.path.exists(port):
            ports = [p.device for p in serial.tools.list_ports.comports()]
            if ports: port = ports[0]
            else: port = None
        if port:
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.05)
                print(f"[*] 대시보드: ESP32 시리얼 연결 ({port})")
            except Exception:
                self.ser = None

    def _setup_ui(self):
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        grid = QGridLayout(central_widget)
        grid.setSpacing(10)
        grid.setContentsMargins(10, 10, 10, 10)

        # --------------------------------------------------
        # [1사분면 / 좌상단] TMAP 보행자 지도 패널
        # --------------------------------------------------
        self.map_frame = self._create_card_frame("1. TMAP 실시간 경로 지도 & 내비게이션")
        map_layout = QVBoxLayout(self.map_frame)
        if HAS_WEBENGINE:
            self.web_map = QWebEngineView()
            self.web_map.setHtml(self.tmap.generate_leaflet_html())
            map_layout.addWidget(self.web_map)
        else:
            self.canvas_map = CanvasMapWidget(self.tmap)
            map_layout.addWidget(self.canvas_map)
        grid.addWidget(self.map_frame, 0, 0)

        # --------------------------------------------------
        # [2사분면 / 우상단] LiDAR 센싱 레이더 GUI 패널
        # --------------------------------------------------
        self.lidar_frame = self._create_card_frame("2. LiDAR 360° 스캔 & 안전 영역 (D500)")
        lidar_layout = QVBoxLayout(self.lidar_frame)
        self.radar_widget = LidarRadarWidget(self.lidar)
        lidar_layout.addWidget(self.radar_widget)
        grid.addWidget(self.lidar_frame, 0, 1)

        # --------------------------------------------------
        # [3사분면 / 좌하단] 속도계, 텔레메트리 & 주행 모드 제어
        # --------------------------------------------------
        self.telemetry_frame = self._create_card_frame("3. 속도계 & 주행 제어기 (수동/자율)")
        telem_layout = QVBoxLayout(self.telemetry_frame)

        # 상태 및 모드 배너
        mode_box = QHBoxLayout()
        self.mode_label = QLabel("MODE: MANUAL (수동)")
        self.mode_label.setStyleSheet("""
            background-color: #0284c7; color: white; font-size: 14px; 
            font-weight: bold; padding: 6px 12px; border-radius: 6px;
        """)
        self.btn_mode_toggle = QPushButton("모드 전환 [M]")
        self.btn_mode_toggle.setStyleSheet("""
            background-color: #334155; color: #38bdf8; font-weight: bold;
            padding: 6px 14px; border: 1px solid #38bdf8; border-radius: 6px;
        """)
        self.btn_mode_toggle.clicked.connect(self.toggle_mode)

        self.btn_estop = QPushButton("🚨 긴급정지 [SPACE]")
        self.btn_estop.setStyleSheet("""
            background-color: #ef4444; color: white; font-weight: bold;
            padding: 6px 14px; border-radius: 6px;
        """)
        self.btn_estop.clicked.connect(self.trigger_estop)

        mode_box.addWidget(self.mode_label)
        mode_box.addWidget(self.btn_mode_toggle)
        mode_box.addWidget(self.btn_estop)
        telem_layout.addLayout(mode_box)

        # 속도 및 오도메트리 정보 레이블
        self.speed_display = QLabel("0.00 m/s (0.0 km/h)")
        self.speed_display.setAlignment(Qt.AlignCenter)
        self.speed_display.setStyleSheet("font-size: 32px; font-weight: bold; color: #10b981; margin: 8px 0;")
        telem_layout.addWidget(self.speed_display)

        # 조작 가이드 및 벡터 상태
        self.vector_info = QLabel("Vx: +0.00 m/s | Vy: +0.00 m/s | Wz: +0.00 rad/s | 배터리: 12.4V")
        self.vector_info.setAlignment(Qt.AlignCenter)
        self.vector_info.setStyleSheet("font-size: 12px; color: #94a3b8;")
        telem_layout.addWidget(self.vector_info)

        # 가상 방향키 HUD
        key_hud = QLabel("[W]:전진  [S]:후진  [A]:좌이동  [D]:우이동  [Q/E]:회전  [SPACE]:정지")
        key_hud.setAlignment(Qt.AlignCenter)
        key_hud.setStyleSheet("background-color: #1e293b; padding: 6px; border-radius: 4px; font-size: 11px; color: #38bdf8;")
        telem_layout.addWidget(key_hud)

        grid.addWidget(self.telemetry_frame, 1, 0)

        # --------------------------------------------------
        # [4사분면 / 우하단] AI 비전 인식 & 웹캠 스트림 패널
        # --------------------------------------------------
        self.vision_frame = self._create_card_frame("4. AI 비전 & 객체 인식 (Logitech C920)")
        vision_layout = QVBoxLayout(self.vision_frame)
        self.cam_label = QLabel("카메라 스트림 로딩 중...")
        self.cam_label.setAlignment(Qt.AlignCenter)
        self.cam_label.setStyleSheet("background-color: #020617; border-radius: 6px;")
        self.cam_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        vision_layout.addWidget(self.cam_label)
        grid.addWidget(self.vision_frame, 1, 1)

        # 4개 영역 크기 동일 분할 (50:50)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)

    def _create_card_frame(self, title):
        frame = QFrame()
        frame.setStyleSheet("""
            QFrame {
                background-color: #0f172a;
                border: 1px solid #1e293b;
                border-radius: 10px;
            }
        """)
        return frame

    # --------------------------------------------------
    # 키보드 입력 핸들러 (WASD / Space / Mode Switch)
    # --------------------------------------------------
    def keyPressEvent(self, event):
        key = event.key()
        self.pressed_keys.add(key)
        
        if key == Qt.Key_Space:
            self.trigger_estop()
        elif key == Qt.Key_M:
            self.toggle_mode()
        else:
            if self.mode == "MANUAL":
                self._update_manual_speed()

    def keyReleaseEvent(self, event):
        key = event.key()
        if key in self.pressed_keys:
            self.pressed_keys.remove(key)
        if self.mode == "MANUAL":
            self._update_manual_speed()

    def _update_manual_speed(self):
        vx, vy, wz = 0.0, 0.0, 0.0
        step = 0.35 # 수동 기본 주행 속도 (m/s)
        rot_step = 0.6 # 회전 각속도 (rad/s)

        if Qt.Key_W in self.pressed_keys: vx += step
        if Qt.Key_S in self.pressed_keys: vx -= step
        if Qt.Key_A in self.pressed_keys: vy += step
        if Qt.Key_D in self.pressed_keys: vy -= step
        if Qt.Key_Q in self.pressed_keys: wz += rot_step
        if Qt.Key_E in self.pressed_keys: wz -= rot_step

        self.target_vx = vx
        self.target_vy = vy
        self.target_wz = wz

    def toggle_mode(self):
        if self.mode == "MANUAL":
            self.mode = "AUTO"
            self.mode_label.setText("MODE: AUTONOMOUS (자율주행)")
            self.mode_label.setStyleSheet("background-color: #059669; color: white; font-size: 14px; font-weight: bold; padding: 6px 12px; border-radius: 6px;")
        elif self.mode == "AUTO":
            self.mode = "MANUAL"
            self.mode_label.setText("MODE: MANUAL (수동)")
            self.mode_label.setStyleSheet("background-color: #0284c7; color: white; font-size: 14px; font-weight: bold; padding: 6px 12px; border-radius: 6px;")
            self.target_vx, self.target_vy, self.target_wz = 0.0, 0.0, 0.0
        elif self.mode == "EMERGENCY_STOP":
            self.mode = "MANUAL"
            self.mode_label.setText("MODE: MANUAL (수동)")
            self.mode_label.setStyleSheet("background-color: #0284c7; color: white; font-size: 14px; font-weight: bold; padding: 6px 12px; border-radius: 6px;")

    def trigger_estop(self):
        self.mode = "EMERGENCY_STOP"
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.mode_label.setText("🚨 EMERGENCY STOP (긴급 정지)")
        self.mode_label.setStyleSheet("background-color: #dc2626; color: white; font-size: 14px; font-weight: bold; padding: 6px 12px; border-radius: 6px;")
        self._send_command(0.0, 0.0, 0.0)

    # --------------------------------------------------
    # 메인 제어 루프 (20Hz)
    # --------------------------------------------------
    def _main_loop(self):
        # 1. 자율주행 모드일 때 알고리즘 연산
        if self.mode == "AUTO":
            _, min_front, state = self.lidar.get_scan_data()
            if state == "DANGER":
                # 0.6m 이내 위험 장애물 감지 시 정지 및 회피 회전
                self.target_vx = 0.0
                self.target_vy = 0.0
                self.target_wz = 0.4 # 우측/좌측 회피 회전
            elif state == "CAUTION":
                # 1.5m 이내 감속 주행
                self.target_vx = 0.15
                self.target_vy = 0.0
                self.target_wz = 0.0
            else:
                # 정상 주행
                self.target_vx = 0.30
                self.target_vy = 0.0
                self.target_wz = 0.0

        # 2. ESP32로 명령 송신
        if self.mode != "EMERGENCY_STOP":
            self._send_command(self.target_vx, self.target_vy, self.target_wz)
        else:
            self._send_command(0.0, 0.0, 0.0)

        # 3. 텔레메트리 UI 갱신
        linear_spd = math.sqrt(self.target_vx**2 + self.target_vy**2)
        kmh = linear_spd * 3.6
        self.speed_display.setText(f"{linear_spd:.2f} m/s ({kmh:.1f} km/h)")
        self.vector_info.setText(
            f"Vx: {self.target_vx:+.2f} m/s | Vy: {self.target_vy:+.2f} m/s | Wz: {self.target_wz:+.2f} rad/s | 배터리: {self.battery_volt:.1f}V"
        )

        # 4. 카메라 영상 프레임 갱신 (4사분면)
        frame, detections, fps = self.vision.get_latest_frame()
        if frame is not None:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            q_img = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_img).scaled(
                self.cam_label.width(), self.cam_label.height(),
                Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self.cam_label.setPixmap(pixmap)

    def _send_command(self, vx, vy, wz):
        """ESP32로 ASCII 시리얼 패킷 전송: $CMD,vx,vy,wz,chk\\n"""
        if self.ser and self.ser.is_open:
            payload = f"{vx:.2f},{vy:.2f},{wz:.2f}"
            chk = 0
            for c in payload: chk ^= ord(c)
            packet = f"$CMD,{payload},{chk:02X}\n"
            try:
                self.ser.write(packet.encode('ascii'))
            except Exception:
                pass

    def closeEvent(self, event):
        self.lidar.stop()
        self.vision.stop()
        self._send_command(0.0, 0.0, 0.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        event.accept()

def main():
    app = QApplication(sys.argv)
    dashboard = CartMainDashboard()
    dashboard.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
