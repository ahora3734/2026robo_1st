#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Logitech C920 웹캠 및 OpenCV 영상 테스트 도구
- 실시간 영상 스트림 표시 (해상도 및 FPS 측정)
- 키보드 'q' 누르면 종료
"""

import os
import sys
import time
import cv2

def test_camera():
    dev_path = "/dev/video_cam"
    cam_index = 0
    
    if os.path.exists(dev_path):
        cap = cv2.VideoCapture(dev_path)
        print(f"[*] /dev/video_cam 심볼릭 링크로 카메라 열기 시도")
    else:
        # 인덱스로 순차 검색
        print("[*] 심볼릭 링크 없음 -> 디폴트 카메라 인덱스 탐색")
        cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("[!] 카메라를 열 수 없습니다. USB 연결 및 권한을 확인하세요.")
        return

    # 권장 해상도 및 FPS 설정
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    print("[*] 카메라 영상 스트림 시작 (종료하려면 영상 창에서 'q' 또는 ESC를 누르세요)")
    
    prev_time = time.time()
    fps = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[!] 프레임을 읽어오지 못했습니다.")
            break

        curr_time = time.time()
        fps = 0.9 * fps + 0.1 * (1.0 / (curr_time - prev_time + 1e-6))
        prev_time = curr_time

        # OSD 정보 표시
        h, w = frame.shape[:2]
        cv2.putText(frame, f"Logitech C920 | Res: {w}x{h} | FPS: {fps:.1f}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, "Press 'q' to exit", (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)

        cv2.imshow("C920 Camera Test", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    print("[*] 카메라 테스트 완료.")

if __name__ == "__main__":
    test_camera()
