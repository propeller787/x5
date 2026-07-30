#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_RATE="0.5"
DEFAULT_RVIZ_CONFIG="$SCRIPT_DIR/wheelchair_obstacle_debug.rviz"
KEYBOARD_CONTROLLER_SCRIPT="$SCRIPT_DIR/playback_keyboard_controller.py"
RVIZ_LAUNCHER_SCRIPT="$SCRIPT_DIR/launch_rviz_clean.sh"
ROS_LOG_DIR_DEFAULT="$SCRIPT_DIR/.ros_logs"
DEFAULT_ROBOT_URDF="$SCRIPT_DIR/yisbot_o1.urdf"
RSP_LOG_FILE="$SCRIPT_DIR/.ros_logs/robot_state_publisher.log"

usage() {
  cat <<'EOF'
用法:
  bash replay_bag_with_rviz.sh <bag_dir> [rate] [--start-offset 秒] [--rewind-seconds 秒] [--domain-id N] [--baseline-bag 目录] [--robot-urdf 文件] [--auto-start]

示例:
  bash replay_bag_with_rviz.sh /home/wzq/wzq/rosbag/0729mppi_stack_cyclic2/mppi_stack_cyclic/bag_20260728_210133
  bash replay_bag_with_rviz.sh /home/wzq/wzq/rosbag/0729mppi_stack_cyclic2/mppi_stack_cyclic/bag_20260728_210133 1.0
  bash replay_bag_with_rviz.sh /home/wzq/wzq/rosbag/0729mppi_stack_cyclic1/mppi_stack_cyclic/bag_20260729_101824 0.7 --start-offset 35
  bash replay_bag_with_rviz.sh /home/wzq/wzq/rosbag/0729mppi_stack_cyclic1/mppi_stack_cyclic/bag_20260729_101824 0.7 --start-offset 35 --rewind-seconds 8 --robot-urdf /home/wzq/wzq/rosbag/yisbot_o1.urdf --domain-id 66 --auto-start
EOF
}

if [[ $# -lt 1 || $# -gt 13 ]]; then
  usage
  exit 1
fi

BAG_DIR="$1"
# 兼容相对路径和绝对路径：把 BAG_DIR 统一规范成绝对路径
if [[ -d "$BAG_DIR" ]]; then
  BAG_DIR="$(cd "$BAG_DIR" && pwd)"
fi
RATE="$DEFAULT_RATE"
START_OFFSET_SEC="0"
REWIND_SECONDS="5"
WAIT_FOR_SPACE="1"
REPLAY_DOMAIN_ID="${REPLAY_ROS_DOMAIN_ID:-66}"
ROBOT_URDF_PATH="$DEFAULT_ROBOT_URDF"
BASELINE_BAG=""

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
        usage
        exit 1
      fi
      START_OFFSET_SEC="$2"
      shift 2
      ;;
    --auto-start)
      WAIT_FOR_SPACE="0"
      shift
      ;;
    --rewind-seconds)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --rewind-seconds 后面需要跟秒数" >&2
        usage
        exit 1
      fi
      REWIND_SECONDS="$2"
      shift 2
      ;;
    --domain-id)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --domain-id 后面需要跟整数" >&2
        usage
        exit 1
      fi
      REPLAY_DOMAIN_ID="$2"
      shift 2
      ;;
    --robot-urdf)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --robot-urdf 后面需要跟文件路径" >&2
        usage
        exit 1
      fi
      ROBOT_URDF_PATH="$2"
      shift 2
      ;;
    --baseline-bag)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --baseline-bag 后面需要跟 bag 目录" >&2
        usage
        exit 1
      fi
      BASELINE_BAG="$2"
      shift 2
      ;;
    *)
      echo "[ERROR] 不支持的参数: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ ! -d "$BAG_DIR" ]]; then
  echo "[ERROR] bag 目录不存在: $BAG_DIR" >&2
  exit 1
fi

resolve_baseline_bag() {
  local replay_dir="$1"
  local base
  base="$(basename "$replay_dir")"
  if [[ ! "$base" =~ ^replay_(bag_[0-9]+_[0-9]+)_[0-9]+_[0-9]+$ ]]; then
    return 1
  fi
  local baseline_name="${BASH_REMATCH[1]}"
  local repo_root
  repo_root="$(cd "${SCRIPT_DIR}/.." && pwd)"
  local candidate
  for candidate in \
    "${SCRIPT_DIR}/bags/${baseline_name}" \
    "${repo_root}/Visualization/bags/${baseline_name}" \
    "${repo_root}/src/perception/wheelchair_obstacle/bags/${baseline_name}"; do
    if [[ -d "$candidate" ]]; then
      echo "$(cd "$candidate" && pwd)"
      return 0
    fi
  done
  return 1
}

list_replay_topics_excluding_pose() {
  python3 - "$1" <<'PY'
import sys
import yaml
from pathlib import Path

bag_dir = Path(sys.argv[1])
meta = yaml.safe_load((bag_dir / "metadata.yaml").read_text())
skip = {"/odom/fused", "/tf", "/tf_static"}
for item in meta["rosbag2_bagfile_information"]["topics_with_message_count"]:
    name = item["topic_metadata"]["name"]
    if name not in skip:
        print(name)
PY
}

if [[ -n "$BASELINE_BAG" && -d "$BASELINE_BAG" ]]; then
  BASELINE_BAG="$(cd "$BASELINE_BAG" && pwd)"
elif resolved="$(resolve_baseline_bag "$BAG_DIR")"; then
  BASELINE_BAG="$resolved"
fi

USE_POSE_BASELINE="0"
REPLAY_TOPICS=()
if [[ "$(basename "$BAG_DIR")" =~ ^replay_ && -n "$BASELINE_BAG" ]]; then
  USE_POSE_BASELINE="1"
  mapfile -t REPLAY_TOPICS < <(list_replay_topics_excluding_pose "$BAG_DIR")
  if [[ ${#REPLAY_TOPICS[@]} -eq 0 ]]; then
    echo "[ERROR] replay bag 没有可回放的非 pose topic: $BAG_DIR" >&2
    exit 1
  fi
fi

if [[ ! -f "$DEFAULT_RVIZ_CONFIG" ]]; then
  echo "[ERROR] RViz 配置不存在: $DEFAULT_RVIZ_CONFIG" >&2
  exit 1
fi

if [[ ! -f "$KEYBOARD_CONTROLLER_SCRIPT" ]]; then
  echo "[ERROR] 键盘控制脚本不存在: $KEYBOARD_CONTROLLER_SCRIPT" >&2
  exit 1
fi

if [[ ! -f "$RVIZ_LAUNCHER_SCRIPT" ]]; then
  echo "[ERROR] RViz 启动脚本不存在: $RVIZ_LAUNCHER_SCRIPT" >&2
  exit 1
fi

if [[ ! -f "$ROBOT_URDF_PATH" ]]; then
  echo "[ERROR] 车体 URDF 不存在: $ROBOT_URDF_PATH" >&2
  exit 1
fi

mkdir -p "$ROS_LOG_DIR_DEFAULT"
export ROS_LOG_DIR="$ROS_LOG_DIR_DEFAULT"
export ROS_DOMAIN_ID="$REPLAY_DOMAIN_ID"

safe_source() {
  local setup_file="$1"
  set +u
  # ROS setup 脚本会读取一些未定义环境变量，这里临时关闭 nounset。
  source "$setup_file"
  set -u
}

source_extra_setups() {
  local setups_env="${REPLAY_EXTRA_SETUP_FILES:-}"
  if [[ -z "$setups_env" ]]; then
    return 0
  fi

  local old_ifs="$IFS"
  IFS=':'
  read -ra setup_files <<<"$setups_env"
  IFS="$old_ifs"

  for setup_file in "${setup_files[@]}"; do
    if [[ -z "$setup_file" ]]; then
      continue
    fi
    source_overlay_if_ok "$setup_file" "extra overlay"
  done
}

source_overlay_if_ok() {
  local setup_file="$1"
  local label="$2"

  if [[ ! -f "$setup_file" ]]; then
    echo "[WARN] 跳过 ${label}: 不存在 ${setup_file}"
    return 0
  fi

  echo "[INFO] 尝试 source ${label}: ${setup_file}"
  if safe_source "$setup_file"; then
    echo "[INFO] 已加载 ${label}"
  else
    echo "[WARN] ${label} 加载失败，继续仅用当前环境回放"
  fi
}

launch_rviz_clean() {
  env -i \
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
    ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    ROS_LOG_DIR="$ROS_LOG_DIR" \
    bash --noprofile --norc -lc '
      set -e
      source /opt/ros/humble/setup.bash
      exec /opt/ros/humble/bin/rviz2 -d "'"$DEFAULT_RVIZ_CONFIG"'" \
        --ros-args -p use_sim_time:=true \
        > >(grep -vE "TF_OLD_DATA ignoring data from the past|according to authority Authority undetectable|Possible reasons are listed at http://wiki.ros.org/tf/Errors%20explained|at line [0-9]+ in ./src/buffer_core.cpp") \
        2> >(grep -vE "TF_OLD_DATA ignoring data from the past|according to authority Authority undetectable|Possible reasons are listed at http://wiki.ros.org/tf/Errors%20explained|at line [0-9]+ in ./src/buffer_core.cpp" >&2)
    '
}

echo "[INFO] source /opt/ros/humble/setup.bash"
safe_source /opt/ros/humble/setup.bash

source_extra_setups

if ros2 pkg prefix interface >/dev/null 2>&1; then
  echo "[INFO] 已找到自定义消息包 interface"
else
  cat <<'EOF'
[WARN] 当前环境里没有找到 interface 包。
       回放时 /sensor/joystick、/vlago/car/info 这类自定义消息 topic 可能会被忽略。
       但如果你主要看 RViz 里的 TF / 点云 / obstacle_points / 路径 / 速度曲线，通常仍可继续分析。
EOF
fi

RVIZ_PID=""
PLAYER_PID=""
POSE_PLAYER_PID=""
RSP_PID=""
RVIZ_PID_FILE="$(mktemp /tmp/rosbag_rviz_pid.XXXXXX)"
cleanup() {
  if [[ -f "$RVIZ_PID_FILE" ]]; then
    local current_rviz_pid=""
    current_rviz_pid="$(cat "$RVIZ_PID_FILE" 2>/dev/null || true)"
    if [[ -n "$current_rviz_pid" ]] && kill -0 "$current_rviz_pid" >/dev/null 2>&1; then
      kill "$current_rviz_pid" >/dev/null 2>&1 || true
      wait "$current_rviz_pid" 2>/dev/null || true
    fi
    rm -f "$RVIZ_PID_FILE"
  fi
  if [[ -n "$RSP_PID" ]] && kill -0 "$RSP_PID" >/dev/null 2>&1; then
    kill "$RSP_PID" >/dev/null 2>&1 || true
    wait "$RSP_PID" 2>/dev/null || true
  fi
  if [[ -n "$POSE_PLAYER_PID" ]] && kill -0 "$POSE_PLAYER_PID" >/dev/null 2>&1; then
    kill "$POSE_PLAYER_PID" >/dev/null 2>&1 || true
    wait "$POSE_PLAYER_PID" 2>/dev/null || true
  fi
  if [[ -n "$PLAYER_PID" ]] && kill -0 "$PLAYER_PID" >/dev/null 2>&1; then
    kill "$PLAYER_PID" >/dev/null 2>&1 || true
    wait "$PLAYER_PID" 2>/dev/null || true
  fi
  if [[ -n "$RVIZ_PID" ]] && kill -0 "$RVIZ_PID" >/dev/null 2>&1; then
    kill "$RVIZ_PID" >/dev/null 2>&1 || true
    wait "$RVIZ_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[INFO] 启动 robot_state_publisher: $ROBOT_URDF_PATH"
ros2 run robot_state_publisher robot_state_publisher "$ROBOT_URDF_PATH" \
  --ros-args -p use_sim_time:=true >"$RSP_LOG_FILE" 2>&1 &
RSP_PID=$!

echo "[INFO] 启动 RViz: $DEFAULT_RVIZ_CONFIG"
/bin/bash "$RVIZ_LAUNCHER_SCRIPT" "$DEFAULT_RVIZ_CONFIG" &
RVIZ_PID=$!
echo "$RVIZ_PID" > "$RVIZ_PID_FILE"

sleep 2

if ! kill -0 "$RVIZ_PID" >/dev/null 2>&1; then
  cat <<'EOF'
[WARN] RViz 没有成功拉起。
       如果终端里看到的是 snap / glibc / libpthread 相关报错，通常是 shell 继承了 VS Code snap 环境。
       这个脚本已经在尽量隔离 snap 变量；如果仍失败，请把新报错贴给我，我继续帮你收。
EOF
fi

cat <<EOF
[INFO] 开始回放:
       bag  = $BAG_DIR
       rate = $RATE
       start_offset = ${START_OFFSET_SEC}s
       rewind_seconds = ${REWIND_SECONDS}s
       ros_domain_id = $ROS_DOMAIN_ID
       ros_log_dir = $ROS_LOG_DIR
       robot_urdf = $ROBOT_URDF_PATH
       pose_baseline = $([[ "$USE_POSE_BASELINE" == "1" ]] && echo "$BASELINE_BAG (/odom/fused+/tf)" || echo "off")
       wait_for_space = $([[ "$WAIT_FOR_SPACE" == "1" ]] && echo "on" || echo "off")
       keyboard = SPACE 开始/暂停/继续, Backspace 回退 ${REWIND_SECONDS}s, r 手动刷新 RViz, q 退出控制, Ctrl+C 停止
EOF

if [[ "$USE_POSE_BASELINE" == "1" ]]; then
  if [[ "$WAIT_FOR_SPACE" == "1" ]]; then
    echo "[WARN] replay bag 使用 baseline pose 时强制 --auto-start，避免双 bag 进度不同步"
    WAIT_FOR_SPACE="0"
  fi
  echo "[INFO] replay bag 不含可靠 pose；从 baseline 播放 /odom/fused /tf /tf_static"
  ros2 bag play \
    "$BASELINE_BAG" \
    --rate "$RATE" \
    --start-offset "$START_OFFSET_SEC" \
    --topics /odom/fused /tf /tf_static \
    --disable-keyboard-controls &
  POSE_PLAYER_PID=$!

  ros2 bag play \
    "$BAG_DIR" \
    --rate "$RATE" \
    --start-offset "$START_OFFSET_SEC" \
    --clock 20 \
    --topics "${REPLAY_TOPICS[@]}" \
    --disable-keyboard-controls &
  PLAYER_PID=$!
else
  ros2 bag play \
    "$BAG_DIR" \
    --rate "$RATE" \
    --start-offset "$START_OFFSET_SEC" \
    --clock 20 \
    --start-paused \
    --disable-keyboard-controls &
  PLAYER_PID=$!
fi

CONTROLLER_ARGS=(
  /usr/bin/python3 -u "$KEYBOARD_CONTROLLER_SCRIPT"
  --start-offset "$START_OFFSET_SEC"
  --rewind-seconds "$REWIND_SECONDS"
  --player-pid "$PLAYER_PID"
  --rviz-config "$DEFAULT_RVIZ_CONFIG"
  --rviz-pid-file "$RVIZ_PID_FILE"
  --rviz-launcher "$RVIZ_LAUNCHER_SCRIPT"
)
if [[ "$WAIT_FOR_SPACE" == "0" ]]; then
  CONTROLLER_ARGS+=(--auto-start)
fi

"${CONTROLLER_ARGS[@]}"
wait "$PLAYER_PID"
if [[ -n "$POSE_PLAYER_PID" ]]; then
  wait "$POSE_PLAYER_PID" 2>/dev/null || true
fi
