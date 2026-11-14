#!/usr/bin/env bash
# =============================================================================
# 파일: ~/2026_ROBO_1/scripts/run_slam_view.sh
# 실행: bash ~/2026_ROBO_1/scripts/run_slam_view.sh auto
# =============================================================================

set -euo pipefail

PROJECT="$HOME/2026_ROBO_1"
MODE="${1:-auto}"
RVIZ_CFG="$PROJECT/config/slam_view.rviz"
LOG="$PROJECT/rviz2_last.log"

source /opt/ros/humble/setup.bash

if [[ -f "$PROJECT/install/setup.bash" ]]; then
  source "$PROJECT/install/setup.bash"
fi

# GNOME 세션의 실제 DISPLAY 사용
if [[ -r "$PROJECT/.jetson_gui_env" ]]; then
  source "$PROJECT/.jetson_gui_env"
else
  export DISPLAY="${DISPLAY:-:0}"
fi

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

if [[ -S "$XDG_RUNTIME_DIR/bus" ]]; then
  export DBUS_SESSION_BUS_ADDRESS="unix:path=$XDG_RUNTIME_DIR/bus"
fi

# Qt / X11 호환 모드
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1

unset WAYLAND_DISPLAY || true
unset QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH || true

mkdir -p "$PROJECT/config"

# RViz 설정 파일 최초 생성
if [[ ! -f "$RVIZ_CFG" ]]; then
cat > "$RVIZ_CFG" <<'RVIZ'
Panels:
  - Class: rviz_common/Displays
    Name: Displays

Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Map
      Name: SLAM Map
      Enabled: true
      Alpha: 0.7
      Color Scheme: map
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic: /map_updates
      Value: true

    - Class: rviz_default_plugins/LaserScan
      Name: LiDAR Scan
      Enabled: true
      Style: Points
      Size (Pixels): 3
      Size (m): 0.04
      Color Transformer: FlatColor
      Color: 53; 211; 153
      Decay Time: 0
      Topic:
        Depth: 10
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Value: true

  Enabled: true

  Global Options:
    Background Color: 30; 30; 30
    Fixed Frame: base_footprint
    Frame Rate: 15

  Name: root
  Tools: []

  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Name: Current View
      Scale: 80
      Angle: 0
      Target Frame: <Fixed Frame>
      Value: TopDownOrtho (rviz_default_plugins)
RVIZ
fi

run_web() {
  echo "[WEB] 브라우저 접속: http://127.0.0.1:8765 또는 http://$(hostname -I | awk '{print $1}'):8765"
  exec python3 "$PROJECT/scripts/jetson_lidar_web.py"
}

run_rviz() {
  local software="$1"

  if [[ "$software" == yes ]]; then
    export LIBGL_ALWAYS_SOFTWARE=1
    echo "[RViz2] CPU 렌더링 모드"
  else
    unset LIBGL_ALWAYS_SOFTWARE || true
    echo "[RViz2] GPU 렌더링 모드"
  fi

  echo "DISPLAY=$DISPLAY"
  echo "XAUTHORITY=${XAUTHORITY:-<unset>}"

  rviz2 -d "$RVIZ_CFG" 2>&1 | tee "$LOG"
}

case "$MODE" in
  web)
    run_web
    ;;

  rviz|software|auto)
    if [[ "$MODE" == rviz ]]; then
      run_rviz no

    elif [[ "$MODE" == software ]]; then
      run_rviz yes

    elif run_rviz no; then
      exit 0

    else
      echo "[경고] GPU RViz 실패, CPU 모드 재시도"

      if run_rviz yes; then
        exit 0
      fi

      echo "[경고] RViz 실패, 웹 뷰어로 자동 전환"
      run_web
    fi
    ;;

  *)
    echo "사용법: $0 [auto|rviz|software|web]" >&2
    exit 2
    ;;
esac

