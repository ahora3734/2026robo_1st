#!/bin/bash
# =============================================================================
# D500 LiDAR 실시간 RViz2 3D 모니터 원클릭 실행 스크립트
# =============================================================================

# ROS 2 환경 로드
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "❌ /opt/ros/humble/setup.bash를 찾을 수 없습니다."
    exit 1
fi

PROJECT_DIR="/home/pogo/2026_ROBO_1"
RVIZ_CONFIG="${PROJECT_DIR}/config/cart_lidar.rviz"
LIDAR_NODE="${PROJECT_DIR}/tools/cart_lidar_node.py"

echo "============================================================"
echo "  🛰️  D500 LiDAR ↔ RViz2 실시간 3D 시각화 실행기"
echo "============================================================"
echo "  1. 실측 치수 반영 카트 LiDAR 드라이버 노드 가동 (/scan)"
echo "  2. 실측 좌표계 TF 발행 (base_footprint -> base_link -> laser_frame)"
echo "  3. RViz2 3D 뷰어 화면 실행"
echo "============================================================"

# 시리얼 포트 권한 체크 (필요시)
if [ -e "/dev/ttyCartLiDAR" ]; then
    sudo chmod 666 /dev/ttyCartLiDAR 2>/dev/null || true
fi

# 백그라운드로 LiDAR 노드 실행
python3 "${LIDAR_NODE}" &
LIDAR_PID=$!

# 종료 시 자식 프로세스 정리 함수
cleanup() {
    echo ""
    echo "🛑 RViz2가 종료되었습니다. LiDAR 노드를 안전하게 정리합니다..."
    kill -SIGINT "${LIDAR_PID}" 2>/dev/null || true
    wait "${LIDAR_PID}" 2>/dev/null || true
    echo "✅ 완료!"
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# 1초 대기 후 RViz2 실행
sleep 1.5

echo "🖥️  RViz2 화면을 실행합니다..."
rviz2 -d "${RVIZ_CONFIG}"

