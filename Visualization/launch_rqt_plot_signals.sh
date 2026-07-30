#!/usr/bin/env bash

set -euo pipefail

DOMAIN_ID="${REPLAY_ROS_DOMAIN_ID:-66}"

if [[ $# -gt 0 ]]; then
  if [[ $# -eq 2 && "$1" == "--domain-id" ]]; then
    DOMAIN_ID="$2"
  else
    echo "用法: bash launch_rqt_plot_signals.sh [--domain-id N]" >&2
    exit 1
  fi
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[ERROR] 没找到 /opt/ros/humble/setup.bash" >&2
  exit 1
fi

safe_source() {
  local setup_file="$1"
  set +u
  source "$setup_file"
  set -u
}

safe_source /opt/ros/humble/setup.bash

if [[ -n "${REPLAY_EXTRA_SETUP_FILES:-}" ]]; then
  OLD_IFS="$IFS"
  IFS=':'
  read -ra SETUP_FILES <<<"$REPLAY_EXTRA_SETUP_FILES"
  IFS="$OLD_IFS"
  for setup_file in "${SETUP_FILES[@]}"; do
    if [[ -n "$setup_file" && -f "$setup_file" ]]; then
      safe_source "$setup_file"
    fi
  done
fi

export ROS_DOMAIN_ID="$DOMAIN_ID"

if ! command -v rqt_plot >/dev/null 2>&1; then
  echo "[ERROR] 没找到 rqt_plot。请先安装对应系统依赖。" >&2
  exit 1
fi

echo "[INFO] 启动 rqt_plot, ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
rqt_plot \
  /handle/control/linear/x \
  /cmd_vel/linear/x \
  /cmd_vel/angular/z \
  /nav/control/linear/x \
  /nav/control/angular/z \
  /odom/fused/twist/twist/linear/x \
  /odom/fused/twist/twist/angular/z
