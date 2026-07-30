#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "用法: bash launch_rviz_clean.sh <rviz_config>" >&2
  exit 1
fi

RVIZ_CONFIG="$1"

if [[ ! -f "$RVIZ_CONFIG" ]]; then
  echo "[ERROR] RViz 配置不存在: $RVIZ_CONFIG" >&2
  exit 1
fi

exec env -i \
  HOME="$HOME" \
  USER="${USER:-wzq}" \
  LOGNAME="${LOGNAME:-${USER:-wzq}}" \
  SHELL="${SHELL:-/bin/bash}" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  DISPLAY="${DISPLAY:-:0}" \
  XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}" \
  LANG="${LANG:-C.UTF-8}" \
  LC_ALL="${LC_ALL:-C.UTF-8}" \
  TERM="${TERM:-xterm-256color}" \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-}" \
  ROS_LOG_DIR="${ROS_LOG_DIR:-}" \
  bash --noprofile --norc -lc '
    set -e
    source /opt/ros/humble/setup.bash
    exec /opt/ros/humble/bin/rviz2 -d "'"$RVIZ_CONFIG"'" \
      --ros-args -p use_sim_time:=true \
      > >(grep -vE "TF_OLD_DATA ignoring data from the past|according to authority Authority undetectable|Possible reasons are listed at http://wiki.ros.org/tf/Errors%20explained|at line [0-9]+ in ./src/buffer_core.cpp") \
      2> >(grep -vE "TF_OLD_DATA ignoring data from the past|according to authority Authority undetectable|Possible reasons are listed at http://wiki.ros.org/tf/Errors%20explained|at line [0-9]+ in ./src/buffer_core.cpp" >&2)
  '
