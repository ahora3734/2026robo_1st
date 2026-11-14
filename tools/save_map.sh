#!/bin/bash
# =============================================================================
# 완성된 SLAM 지도 영구 저장 스크립트
# 사용법: bash save_map.sh [지도이름] (기본값: school_map)
# =============================================================================

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

MAP_NAME="${1:-school_map}"
SAVE_DIR="/home/pogo/2026_ROBO_1/maps"
mkdir -p "${SAVE_DIR}"

echo "============================================================"
echo "  💾 지도를 저장합니다: ${SAVE_DIR}/${MAP_NAME}"
echo "============================================================"

ros2 run nav2_map_server map_saver_cli -f "${SAVE_DIR}/${MAP_NAME}"

echo ""
echo "✅ 지도 저장 완료!"
echo "   - 이미지 파일 : ${SAVE_DIR}/${MAP_NAME}.pgm"
echo "   - 설정 파일   : ${SAVE_DIR}/${MAP_NAME}.yaml"
echo "============================================================"

