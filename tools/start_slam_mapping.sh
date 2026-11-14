#!/bin/bash
# =============================================================================
# [매핑 모드] 실시간 SLAM 복도 지도 그리기 원클릭 실행 스크립트
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
LAUNCH_FILE="${PROJECT_DIR}/cart_ws/src/cart_slam_nav/launch/cart_slam.launch.py"

# 시리얼 포트 권한 체크
sudo chmod 666 /dev/ttyCartLiDAR /dev/ttyCartIMU /dev/ttyCartESP32 /dev/ttyUSB* 2>/dev/null || true

# 기존 실행 중인 웹 서버/노드 정리 (충돌 방지)
pkill -f "jetson_lidar_web.py" 2>/dev/null || true
pkill -f "cart_lidar_node.py" 2>/dev/null || true
pkill -f "async_slam_toolbox_node" 2>/dev/null || true
sleep 0.5

JETSON_IP=$(hostname -I | awk '{print $1}')

echo "============================================================"
echo "  🗺️  [SLAM 실시간 지도 작성 시작]"
echo "============================================================"
echo "  1. D500 LiDAR 드라이버 가동 (10Hz, 12m 범위)"
echo "  2. slam_toolbox 2D SLAM 매핑 엔진 가동"
echo "  3. 실시간 웹 뷰어 웹서버 가동 (포트 8765)"
echo "------------------------------------------------------------"
echo "  🌐 [실시간 2D 지도 확인 방법]"
echo "     👉 노트북 크롬 브라우저에서 아래 주소로 접속하세요:"
echo "     http://${JETSON_IP}:8765"
echo "------------------------------------------------------------"
echo "  💾 지도 완성 후 저장 명령어 (새 터미널에서 실행):"
echo "     bash ~/2026_ROBO_1/tools/save_map.sh school_map"
echo "============================================================"
echo ""

ros2 launch "${LAUNCH_FILE}"
