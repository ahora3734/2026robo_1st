#!/usr/bin/env bash
set -euo pipefail

: "${DISPLAY:=:0}"
PROJECT="$HOME/2026_ROBO_1"

# 전체 사용자 허용(xhost +) 및 현재 사용자 허용
xhost "+SI:localuser:$(id -un)" 2>/dev/null || xhost + 2>/dev/null || true

{
  printf 'export DISPLAY=%q\n' "$DISPLAY"

  if [[ -n "${XAUTHORITY:-}" && -r "$XAUTHORITY" ]]; then
    printf 'export XAUTHORITY=%q\n' "$XAUTHORITY"
  elif [[ -r "$HOME/.Xauthority" ]]; then
    printf 'export XAUTHORITY=%q\n' "$HOME/.Xauthority"
  else
    printf 'unset XAUTHORITY\n'
  fi

  printf 'export XDG_RUNTIME_DIR=%q\n' \
    "${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
} > "$PROJECT/.jetson_gui_env"

chmod 600 "$PROJECT/.jetson_gui_env"
echo "✅ GUI 인증 환경 저장 완료: $PROJECT/.jetson_gui_env"

