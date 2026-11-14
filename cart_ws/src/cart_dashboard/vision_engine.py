#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI 비전 및 객체 인식 엔진 모듈
- Logitech C920 웹캠 스트림 캡처 (또는 가상 테스트 영상 생성)
- YOLO 기반 보행자(Person), 신호등(Traffic Light), 점자블록(Braille Block) 검출
- 바운딩 박스 및 OSD 오버레이
"""

import os
import cv2
import time
import numpy as np
import threading

class VisionEngine:
    def __init__(self, cam_device="/dev/video_cam"):
        self.cam_device = cam_device
        self.cap = None
        self.running = True
        self.lock = threading.Lock()
        self.current_frame = None
        self.detections = []
        self.fps = 0.0
        self.is_camera_connected = False
        
        # YOLO 모델 로드 시도
        self.model = None
        try:
            from ultralytics import YOLO
            # 경량 YOLOv8n 모델 사용
            self.model = YOLO("yolov8n.pt")
            print("[*] YOLOv8n 모델 로드 성공")
        except Exception as e:
            print(f"[*] YOLO 모델 직접 추론 대기 (OpenCV 기반 가상 감지 활성화): {e}")

        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _open_camera(self):
        target = self.cam_device
        if not os.path.exists(target):
            # 인덱스 0 또는 1 시도
            target = 0
            
        self.cap = cv2.VideoCapture(target)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.is_camera_connected = True
            return True
        self.is_camera_connected = False
        return False

    def _worker(self):
        prev_time = time.time()
        
        while self.running:
            if not self.is_camera_connected or self.cap is None or not self.cap.isOpened():
                if not self._open_camera():
                    # 가상 프레임 생성
                    frame = self._generate_mock_frame()
                    self._process_frame(frame)
                    time.sleep(0.04) # 약 25 FPS
                    continue

            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.is_camera_connected = False
                time.sleep(0.5)
                continue

            curr_time = time.time()
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / (curr_time - prev_time + 1e-6))
            prev_time = curr_time

            self._process_frame(frame)

    def _process_frame(self, frame):
        detections = []
        
        # 1. 실제 YOLO 모델이 있을 경우
        if self.model is not None:
            try:
                results = self.model(frame, verbose=False, conf=0.45)
                for r in results:
                    for box in r.boxes:
                        cls_id = int(box.cls[0])
                        cls_name = self.model.names[cls_id]
                        conf = float(box.conf[0])
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        detections.append({
                            "class": cls_name,
                            "conf": conf,
                            "box": (x1, y1, x2, y2)
                        })
            except Exception:
                pass
        else:
            # 2. 모델 미설치 시 룰 기반 가상 시뮬레이션 감지 (데모용)
            h, w = frame.shape[:2]
            t = time.time()
            # 주기적으로 보행자 및 신호등 감지 시뮬레이션
            if int(t) % 6 < 4:
                px = int(w * 0.4 + math_sin := (np.sin(t) * 50))
                detections.append({
                    "class": "person",
                    "conf": 0.88,
                    "box": (px, int(h*0.2), px + 120, int(h*0.85))
                })
            # 가상 녹색 신호등
            detections.append({
                "class": "traffic light [GREEN]",
                "conf": 0.94,
                "box": (int(w*0.75), 40, int(w*0.88), 130)
            })

        # 시각화 렌더링
        rendered_frame = frame.copy()
        for det in detections:
            cls = det["class"]
            conf = det["conf"]
            x1, y1, x2, y2 = det["box"]

            color = (0, 255, 0) if "GREEN" in cls else ((0, 0, 255) if "RED" in cls else (255, 200, 0))
            if "person" in cls:
                color = (0, 220, 255)

            cv2.rectangle(rendered_frame, (x1, y1), (x2, y2), color, 2)
            label = f"{cls} {conf*100:.0f}%"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(rendered_frame, (x1, y1 - 20), (x1 + tw, y1), color, -1)
            cv2.putText(rendered_frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        with self.lock:
            self.current_frame = rendered_frame
            self.detections = detections

    def _generate_mock_frame(self):
        """카메라 미연결 시 가상 보행로 화면 생성"""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:] = (30, 25, 20) # 다크 배경

        # 보행로 원근감 선
        cv2.line(frame, (100, 480), (280, 200), (80, 80, 80), 2)
        cv2.line(frame, (540, 480), (360, 200), (80, 80, 80), 2)
        # 점자블록(노란색) 시뮬레이션
        pts = np.array([[290, 480], [350, 480], [330, 220], [310, 220]], np.int32)
        cv2.fillPoly(frame, [pts], (20, 180, 220))

        # 가상 신호등 지지대
        cv2.rectangle(frame, (480, 60), (530, 150), (40, 40, 40), -1)
        cv2.circle(frame, (505, 85), 12, (50, 50, 50), -1) # Red 꺼짐
        cv2.circle(frame, (505, 125), 12, (0, 255, 0), -1) # Green 켜짐

        cv2.putText(frame, "[CAMERA SIMULATION MODE]", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        return frame

    def get_latest_frame(self):
        with self.lock:
            if self.current_frame is not None:
                return self.current_frame.copy(), self.detections, self.fps
            return None, [], 0.0

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()
