#!/bin/bash
# =============================================================================
# [자율주행 모드] 저장된 지도 기반 Nav2 자율주행 & 웨이포인트 이동 스크립트
# =============================================================================

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

MAP_NAME="${1:-school_map}"
MAP_PATH="/home/pogo/2026_ROBO_1/maps/${MAP_NAME}.yaml"
PROJECT_DIR="/home/pogo/2026_ROBO_1"
LAUNCH_FILE="${PROJECT_DIR}/cart_ws/src/cart_slam_nav/launch/cart_nav2.launch.py"

if [ ! -f "${MAP_PATH}" ]; then
    echo "❌ 지도 파일을 찾을 수 없습니다: ${MAP_PATH}"
    echo "👉 먼저 SLAM 매핑을 실행하여 지도를 저장하세요:"
    echo "   1) bash ~/2026_ROBO_1/tools/start_slam_mapping.sh"
    echo "   2) bash ~/2026_ROBO_1/tools/save_map.sh ${MAP_NAME}"
    exit 1
fi

echo "============================================================"
echo "  🚀 [Nav2 자율주행 모드] 지도 기반 목표 지점 이동 시작"
echo "============================================================"
echo "  🗺️  로드할 지도: ${MAP_PATH}"
echo "============================================================"

# 시리얼 포트 권한 체크
sudo chmod 666 /dev/ttyCartLiDAR /dev/ttyCartIMU /dev/ttyCartESP32 /dev/ttyUSB* 2>/dev/null || true

# 젯슨 Orin 전용 그래픽 크래시 방지 환경변수
export DISPLAY="${DISPLAY:-:0}"
export QT_QPA_PLATFORM=xcb
export OGRE_RTT_MODE=Copy
export LIBGL_ALWAYS_SOFTWARE=1
xhost +local: 2>/dev/null || true

ros2 launch "${LAUNCH_FILE}" map:="${MAP_PATH}"
