#!/usr/bin/env bash

set -euo pipefail

DOMAIN_ID="${REPLAY_ROS_DOMAIN_ID:-66}"

if [[ $# -gt 0 ]]; then
  if [[ $# -eq 2 && "$1" == "--domain-id" ]]; then
    DOMAIN_ID="$2"
  else
    echo "用法: bash launch_plotjuggler.sh [--domain-id N]" >&2
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

PLOTJUGGLER_BIN=""

if command -v plotjuggler >/dev/null 2>&1; then
  PLOTJUGGLER_BIN="$(command -v plotjuggler)"
elif [[ -x /opt/ros/humble/lib/plotjuggler/plotjuggler ]]; then
  PLOTJUGGLER_BIN="/opt/ros/humble/lib/plotjuggler/plotjuggler"
fi

if [[ -z "$PLOTJUGGLER_BIN" ]]; then
  cat <<'EOF' >&2
[ERROR] 没找到 plotjuggler。
请先安装：
  sudo apt install ros-humble-plotjuggler ros-humble-plotjuggler-ros
EOF
  exit 1
fi

echo "[INFO] 启动 PlotJuggler, ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[INFO] 在 PlotJuggler 里选择 ROS 2 data source，然后订阅这些话题："
echo "       /handle/control/linear/x"
echo "       /cmd_vel/linear/x"
echo "       /cmd_vel/angular/z"
echo "       /nav/control/linear/x"
echo "       /nav/control/angular/z"
echo "       /odom/fused/twist/twist/linear/x"
echo "       /odom/fused/twist/twist/angular/z"

"$PLOTJUGGLER_BIN"
