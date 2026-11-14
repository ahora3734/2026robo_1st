#!/bin/bash
# =============================================================================
# [100% 무충돌] 라이다 + SLAM + 실시간 웹 뷰어 통합 실행기
# =============================================================================
set -e

# ROS 2 환경 로드
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "❌ /opt/ros/humble/setup.bash를 찾을 수 없습니다."
    exit 1
fi

PROJECT_DIR="/home/pogo/2026_ROBO_1"
LIDAR_NODE="${PROJECT_DIR}/tools/cart_lidar_node.py"
SLAM_CONFIG="${PROJECT_DIR}/cart_ws/src/cart_slam_nav/config/slam_toolbox.yaml"
WEB_NODE="${PROJECT_DIR}/scripts/jetson_lidar_web.py"

# 시리얼 포트 권한 체크
sudo chmod 666 /dev/ttyCartLiDAR /dev/ttyCartIMU /dev/ttyCartESP32 /dev/ttyUSB* 2>/dev/null || true

# 기존 실행 중인 웹 서버/노드 정리
pkill -f "jetson_lidar_web.py" 2>/dev/null || true
pkill -f "cart_lidar_node.py" 2>/dev/null || true
pkill -f "async_slam_toolbox_node" 2>/dev/null || true
sleep 0.5

JETSON_IP=$(hostname -I | awk '{print $1}')

echo "============================================================"
echo "  🚀 [웹 전용 SLAM 모니터] 라이다 & SLAM 웹 뷰어 기동 중..."
echo "============================================================"

# 1. 라이다 노드 백그라운드 실행
python3 "${LIDAR_NODE}" &
PID_LIDAR=$!

# 2. SLAM Toolbox 백그라운드 실행
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file "${SLAM_CONFIG}" &
PID_SLAM=$!

# 3. 실시간 웹 뷰어 백그라운드 실행
python3 "${WEB_NODE}" &
PID_WEB=$!

cleanup() {
    echo ""
    echo "🛑 종료 신호 수신. 모든 프로세스를 안전하게 종료합니다..."
    kill -SIGINT "${PID_WEB}" "${PID_SLAM}" "${PID_LIDAR}" 2>/dev/null || true
    wait "${PID_WEB}" "${PID_SLAM}" "${PID_LIDAR}" 2>/dev/null || true
    echo "✅ 종료 완료!"
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

sleep 2

echo "============================================================"
echo "  ✅ 웹 서버 가동 성공! (8765 포트 열림)"
echo "------------------------------------------------------------"
echo "  👉 지금 크롬 브라우저에서 '새로고침'을 누르세요!"
echo "     주소: http://${JETSON_IP}:8765"
echo "============================================================"
echo "  (종료하려면 터미널에서 Ctrl + C 를 누르세요)"
echo ""

wait "${PID_WEB}"
