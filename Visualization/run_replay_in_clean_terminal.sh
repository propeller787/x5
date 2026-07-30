#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -lt 1 || $# -gt 9 ]]; then
  cat <<'EOF'
用法:
  bash run_replay_in_clean_terminal.sh <bag_dir> [rate] [--start-offset 秒] [--rewind-seconds 秒] [--domain-id N] [--auto-start] [--plotjuggler]

示例:
  bash run_replay_in_clean_terminal.sh bags/bag_20260730_095927 0.8
  bash run_replay_in_clean_terminal.sh /home/howard/文档/tools/Visualization/bags/bag_20260729_161818 0.8

  bash run_replay_in_clean_terminal.sh /home/wzq/wzq/rosbag/0729mppi_stack_cyclic1/mppi_stack_cyclic/bag_20260729_101624 0.8
  bash run_replay_in_clean_terminal.sh /home/wzq/wzq/rosbag/0729mppi_stack_cyclic1/mppi_stack_cyclic/bag_20260729_101824 0.8 --start-offset 35
  bash run_replay_in_clean_terminal.sh /home/wzq/wzq/rosbag/0729mppi_stack_cyclic1/mppi_stack_cyclic/bag_20260729_101824 0.8 --start-offset 35 --rewind-seconds 8 --plotjuggler
  bash run_replay_in_clean_terminal.sh /home/wzq/wzq/rosbag/0729mppi_stack_cyclic1/mppi_stack_cyclic/bag_20260729_101824 0.8 --start-offset 35 --rewind-seconds 8 --domain-id 66 --auto-start
EOF
  exit 1
fi

BAG_DIR="$1"
# 兼容相对路径和绝对路径：统一规范成绝对路径，避免新终端从 $HOME 启动时相对路径找不到
if [[ -d "$BAG_DIR" ]]; then
  BAG_DIR="$(cd "$BAG_DIR" && pwd)"
fi
RATE="0.5"
START_OFFSET_SEC="0"
REWIND_SECONDS="5"
AUTO_START_FLAG=""
DOMAIN_ID="${REPLAY_ROS_DOMAIN_ID:-66}"
REPLAY_SCRIPT="$SCRIPT_DIR/replay_bag_with_rviz.sh"
PLOTJUGGLER_SCRIPT="$SCRIPT_DIR/launch_plotjuggler.sh"
ENABLE_PLOTJUGGLER="0"

shift
if [[ $# -gt 0 && "$1" != --* ]]; then
  RATE="$1"
  shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --start-offset)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --start-offset 后面需要跟秒数" >&2
        exit 1
      fi
      START_OFFSET_SEC="$2"
      shift 2
      ;;
    --auto-start)
      AUTO_START_FLAG="--auto-start"
      shift
      ;;
    --rewind-seconds)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --rewind-seconds 后面需要跟秒数" >&2
        exit 1
      fi
      REWIND_SECONDS="$2"
      shift 2
      ;;
    --domain-id)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --domain-id 后面需要跟整数" >&2
        exit 1
      fi
      DOMAIN_ID="$2"
      shift 2
      ;;
    --plotjuggler)
      ENABLE_PLOTJUGGLER="1"
      shift
      ;;
    *)
      echo "[ERROR] 不支持的参数: $1" >&2
      exit 1
      ;;
  esac
done

if [[ ! -x "$REPLAY_SCRIPT" ]]; then
  echo "[ERROR] 回放脚本不存在或不可执行: $REPLAY_SCRIPT" >&2
  exit 1
fi

if [[ "$ENABLE_PLOTJUGGLER" == "1" && ! -x "$PLOTJUGGLER_SCRIPT" ]]; then
  echo "[ERROR] PlotJuggler 启动脚本不存在或不可执行: $PLOTJUGGLER_SCRIPT" >&2
  exit 1
fi

TERMINAL_CMD=""
if command -v gnome-terminal >/dev/null 2>&1; then
  TERMINAL_CMD="/usr/bin/gnome-terminal"
elif command -v x-terminal-emulator >/dev/null 2>&1; then
  TERMINAL_CMD="$(command -v x-terminal-emulator)"
fi

if [[ -z "$TERMINAL_CMD" ]]; then
  echo "[ERROR] 没找到 gnome-terminal / x-terminal-emulator，请改用系统终端手动执行回放脚本。" >&2
  exit 1
fi

if [[ "$ENABLE_PLOTJUGGLER" == "1" ]]; then
  env -i \
    HOME="$HOME" \
    USER="${USER:-wzq}" \
    LOGNAME="${LOGNAME:-${USER:-wzq}}" \
    SHELL="/bin/bash" \
    PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
    DISPLAY="${DISPLAY:-:0}" \
    XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}" \
    LANG="${LANG:-C.UTF-8}" \
    LC_ALL="${LC_ALL:-C.UTF-8}" \
    TERM="${TERM:-xterm-256color}" \
    REPLAY_ROS_DOMAIN_ID="$DOMAIN_ID" \
    "$TERMINAL_CMD" -- /bin/bash -lc \
    "cd \"$HOME\"; \
     echo '[INFO] 新终端已打开，准备启动 PlotJuggler'; \
     echo '[INFO] domain_id: $DOMAIN_ID'; \
     bash \"$PLOTJUGGLER_SCRIPT\" --domain-id \"$DOMAIN_ID\"; \
     rc=\$?; \
     echo \"[INFO] PlotJuggler 已退出，exit code=\$rc\"; \
     exec /bin/bash" &
fi

env -i \
  HOME="$HOME" \
  USER="${USER:-wzq}" \
  LOGNAME="${LOGNAME:-${USER:-wzq}}" \
  SHELL="/bin/bash" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  DISPLAY="${DISPLAY:-:0}" \
  XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}" \
  LANG="${LANG:-C.UTF-8}" \
  LC_ALL="${LC_ALL:-C.UTF-8}" \
  TERM="${TERM:-xterm-256color}" \
  REPLAY_ROS_DOMAIN_ID="$DOMAIN_ID" \
  "$TERMINAL_CMD" -- /bin/bash -lc \
  "cd \"$HOME\"; \
     echo '[INFO] 新终端已打开，准备启动回放'; \
     echo '[INFO] bag: $BAG_DIR'; \
     echo '[INFO] rate: $RATE'; \
     echo '[INFO] start_offset: $START_OFFSET_SEC'; \
     echo '[INFO] rewind_seconds: $REWIND_SECONDS'; \
     echo '[INFO] domain_id: $DOMAIN_ID'; \
     echo '[INFO] plotjuggler: $([[ \"$ENABLE_PLOTJUGGLER\" == \"1\" ]] && echo on || echo off)'; \
     echo '[INFO] keyboard: SPACE 暂停/继续, Backspace 回退 ${REWIND_SECONDS}s, r 手动刷新 RViz'; \
     bash \"$REPLAY_SCRIPT\" \"$BAG_DIR\" \"$RATE\" --start-offset \"$START_OFFSET_SEC\" --rewind-seconds \"$REWIND_SECONDS\" --domain-id \"$DOMAIN_ID\" $AUTO_START_FLAG; \
     rc=\$?; \
     echo \"[INFO] 回放脚本已退出，exit code=\$rc\"; \
     exec /bin/bash"
