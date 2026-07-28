#!/usr/bin/env bash
# Replay a recorded bag through the MPPI stack, record new outputs, and diff metrics.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${PKG_ROOT}/../../.." && pwd)}"

source_ros_setup() {
  local restore_nounset=0
  if [[ "$-" == *u* ]]; then
    restore_nounset=1
    set +u
  fi
  # shellcheck disable=SC1091
  source "$1"
  if [[ "${restore_nounset}" -eq 1 ]]; then
    set -u
  fi
}

BASELINE_BAG=""
REPLAY_BAG=""
REPORT=""
PLAY_TOPICS_FILE=""
RECORD_TOPICS_FILE=""
QOS_FILE=""
PLAY_RATE="1.0"
USE_SIM_TIME=1
START_RVIZ=0
DRY_RUN=0
SKIP_BUILD=0
STACK_WARMUP_SEC=3
LAUNCH_SETTLE_SEC=3

usage()
{
  cat <<'USAGE'
Replay a baseline rosbag through the MPPI navigation stack and compare outputs.

Usage:
  scripts/replay_mppi_bag.sh --baseline PATH [options]

Options:
  --baseline PATH     Input bag directory (required)
  --replay PATH       Output bag path. Default: bags/replay_<basename>_TIMESTAMP
  --report PATH       Markdown report path. Default: reports/<basename>_compare.md
  --play-topics PATH  Newline-separated input topics to play
  --record-topics PATH Newline-separated output topics to record
  --qos PATH          QoS overrides for ros2 bag play (scan best_effort)
  --rate RATE         ros2 bag play rate. Default: 1.0
  --stack-warmup SEC  Seconds to wait after launch before record/play. Default: 3
  --start-rviz        Open RViz2 with nav_debug.rviz
  --wall-clock        Use wall clock instead of sim time (not recommended)
  --skip-build        Do not run colcon build for MPPI stack packages
  --dry-run           Print planned commands without executing
  -h, --help          Show this help

Examples:
  scripts/replay_mppi_bag.sh \\
    --baseline ../../../rosbag/bag_20260728_114751

  scripts/replay_mppi_bag.sh \\
    --baseline ../../../rosbag/bag_20260728_114751 \\
    --replay bags/replay_mppi_114751 \\
    --report reports/mppi_114751_compare.md
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --baseline)
      BASELINE_BAG="$2"
      shift 2
      ;;
    --replay)
      REPLAY_BAG="$2"
      shift 2
      ;;
    --report)
      REPORT="$2"
      shift 2
      ;;
    --play-topics)
      PLAY_TOPICS_FILE="$2"
      shift 2
      ;;
    --record-topics)
      RECORD_TOPICS_FILE="$2"
      shift 2
      ;;
    --qos)
      QOS_FILE="$2"
      shift 2
      ;;
    --rate)
      PLAY_RATE="$2"
      shift 2
      ;;
    --stack-warmup)
      STACK_WARMUP_SEC="$2"
      shift 2
      ;;
    --start-rviz)
      START_RVIZ=1
      shift
      ;;
    --wall-clock)
      USE_SIM_TIME=0
      shift
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${BASELINE_BAG}" ]]; then
  echo "--baseline is required." >&2
  usage >&2
  exit 2
fi

if [[ ! "${BASELINE_BAG}" = /* ]]; then
  BASELINE_BAG="${WORKSPACE_ROOT}/${BASELINE_BAG#./}"
fi
if [[ ! -d "${BASELINE_BAG}" ]]; then
  echo "Baseline bag not found: ${BASELINE_BAG}" >&2
  exit 2
fi

PLAY_TOPICS_FILE="${PLAY_TOPICS_FILE:-${PKG_ROOT}/config/mppi_stack_bag_replay_play_topics.txt}"
RECORD_TOPICS_FILE="${RECORD_TOPICS_FILE:-${PKG_ROOT}/config/mppi_stack_bag_replay_record_topics.txt}"
QOS_FILE="${QOS_FILE:-${PKG_ROOT}/config/mppi_stack_bag_qos.yaml}"

if [[ ! -f "${PLAY_TOPICS_FILE}" ]]; then
  echo "Play topics file not found: ${PLAY_TOPICS_FILE}" >&2
  exit 2
fi
if [[ ! -f "${RECORD_TOPICS_FILE}" ]]; then
  echo "Record topics file not found: ${RECORD_TOPICS_FILE}" >&2
  exit 2
fi
if [[ ! -f "${QOS_FILE}" ]]; then
  echo "QoS file not found: ${QOS_FILE}" >&2
  exit 2
fi

mapfile -t PLAY_TOPICS < <(grep -v '^[[:space:]]*#' "${PLAY_TOPICS_FILE}" | grep -v '^[[:space:]]*$' || true)
mapfile -t RECORD_TOPICS < <(grep -v '^[[:space:]]*#' "${RECORD_TOPICS_FILE}" | grep -v '^[[:space:]]*$' || true)
if [[ ${#PLAY_TOPICS[@]} -eq 0 ]]; then
  echo "No play topics in ${PLAY_TOPICS_FILE}" >&2
  exit 2
fi
if [[ ${#RECORD_TOPICS[@]} -eq 0 ]]; then
  echo "No record topics in ${RECORD_TOPICS_FILE}" >&2
  exit 2
fi

BASELINE_NAME="$(basename "${BASELINE_BAG}")"
stamp="$(date +%Y%m%d_%H%M%S)"
if [[ -z "${REPLAY_BAG}" ]]; then
  REPLAY_BAG="${PKG_ROOT}/bags/replay_${BASELINE_NAME}_${stamp}"
fi
if [[ ! "${REPLAY_BAG}" = /* ]]; then
  REPLAY_BAG="${PKG_ROOT}/${REPLAY_BAG}"
fi
if [[ -z "${REPORT}" ]]; then
  REPORT="${PKG_ROOT}/reports/${BASELINE_NAME}_compare_${stamp}.md"
fi
if [[ ! "${REPORT}" = /* ]]; then
  REPORT="${PKG_ROOT}/${REPORT}"
fi

source_ros_setup /opt/ros/humble/setup.bash
if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  source_ros_setup "${WORKSPACE_ROOT}/install/setup.bash"
fi
set -u

DURATION_S="$(python3 - <<PY
import yaml
from pathlib import Path
meta = Path("${BASELINE_BAG}") / "metadata.yaml"
data = yaml.safe_load(meta.read_text())
print(data["rosbag2_bagfile_information"]["duration"]["nanoseconds"] / 1e9 + 4.0)
PY
)"

SIM_TIME_FLAG="false"
PLAY_CLOCK_FLAG=()
RECORD_SIM_ARGS=()
if [[ "${USE_SIM_TIME}" -eq 1 ]]; then
  SIM_TIME_FLAG="true"
  PLAY_CLOCK_FLAG=(--clock 100)
  RECORD_SIM_ARGS=(--use-sim-time)
fi

RVIZ_ARG="start_rviz:=false"
if [[ "${START_RVIZ}" -eq 1 ]]; then
  RVIZ_ARG="start_rviz:=true"
fi

echo "Workspace:     ${WORKSPACE_ROOT}"
echo "Baseline:      ${BASELINE_BAG}"
echo "Replay out:    ${REPLAY_BAG}"
echo "Report:        ${REPORT}"
echo "Duration:      ${DURATION_S}s (+4s margin)"
echo "Sim time:      ${USE_SIM_TIME}"
echo "Play topics:   ${#PLAY_TOPICS[@]} from ${PLAY_TOPICS_FILE}"
echo "Record topics: ${#RECORD_TOPICS[@]} from ${RECORD_TOPICS_FILE}"
echo "QoS overrides: ${QOS_FILE}"

if [[ "${DRY_RUN}" -eq 1 ]]; then
  echo "[dry-run] colcon build --packages-up-to wheelchair_obstacle lidar_fusion localization (unless --skip-build)"
  echo "[dry-run] ros2 launch wheelchair_obstacle mppi_stack_no_sensors.launch.py use_sim_time:=${SIM_TIME_FLAG} ${RVIZ_ARG} start_bag_record:=false"
  echo "[dry-run] ros2 bag record ${RECORD_SIM_ARGS[*]} -o ${REPLAY_BAG} ${RECORD_TOPICS[*]}"
  echo "[dry-run] ros2 bag play ${BASELINE_BAG} --rate ${PLAY_RATE} ${PLAY_CLOCK_FLAG[*]} --qos-profile-overrides-path ${QOS_FILE} --topics ${PLAY_TOPICS[*]}"
  echo "[dry-run] python3 ${SCRIPT_DIR}/compare_mppi_bag_replay.py --baseline ${BASELINE_BAG} --replay ${REPLAY_BAG} --output ${REPORT}"
  exit 0
fi

if [[ "${SKIP_BUILD}" -eq 0 ]]; then
  cd "${WORKSPACE_ROOT}"
  colcon build --symlink-install --packages-up-to wheelchair_obstacle lidar_fusion localization
  source_ros_setup "${WORKSPACE_ROOT}/install/setup.bash"
fi

mkdir -p "$(dirname "${REPLAY_BAG}")" "$(dirname "${REPORT}")"

STACK_PID=""
REC_PID=""
PLAY_PID=""
WAIT_PID=""
INTERRUPTED=0

stop_stack()
{
  if [[ -z "${STACK_PID}" ]] || ! kill -0 "${STACK_PID}" 2>/dev/null; then
    return 0
  fi

  echo "Stopping MPPI stack (pid ${STACK_PID})..." >&2
  kill -INT "${STACK_PID}" 2>/dev/null || true
  pkill -INT -P "${STACK_PID}" 2>/dev/null || true

  local i
  for i in $(seq 1 20); do
    if ! kill -0 "${STACK_PID}" 2>/dev/null; then
      STACK_PID=""
      return 0
    fi
    sleep 0.25
  done

  kill -KILL "${STACK_PID}" 2>/dev/null || true
  pkill -KILL -P "${STACK_PID}" 2>/dev/null || true
  wait "${STACK_PID}" 2>/dev/null || true
  STACK_PID=""
}

cleanup()
{
  if [[ -n "${WAIT_PID}" ]] && kill -0 "${WAIT_PID}" 2>/dev/null; then
    kill "${WAIT_PID}" 2>/dev/null || true
    wait "${WAIT_PID}" 2>/dev/null || true
  fi
  if [[ -n "${PLAY_PID}" ]] && kill -0 "${PLAY_PID}" 2>/dev/null; then
    kill -INT "${PLAY_PID}" 2>/dev/null || true
    wait "${PLAY_PID}" 2>/dev/null || true
  fi
  if [[ -n "${REC_PID}" ]] && kill -0 "${REC_PID}" 2>/dev/null; then
    kill -INT "${REC_PID}" 2>/dev/null || true
    wait "${REC_PID}" 2>/dev/null || true
  fi
  stop_stack
}

on_interrupt()
{
  INTERRUPTED=1
  echo "" >&2
  echo "Interrupted." >&2
  cleanup
  exit 130
}

trap on_interrupt INT TERM
trap cleanup EXIT

wait_for_controller_active()
{
  # With use_sim_time, ros2 CLI calls can block until /clock is published.
  # Only call this after bag play has started.
  local timeout="${1:-30}"
  local elapsed=0
  local state=""
  while [[ "${elapsed}" -lt "${timeout}" ]]; do
    state="$(timeout 2 ros2 lifecycle get /controller_server 2>/dev/null || true)"
    if [[ "${state}" =~ ^active ]]; then
      echo "controller_server is active."
      return 0
    fi
    if (( elapsed % 5 == 0 )); then
      echo "Waiting for controller_server... ${elapsed}s (state: ${state:-not found})"
    fi
    if ! sleep 1; then
      on_interrupt
    fi
    elapsed=$((elapsed + 1))
  done
  echo "Warning: controller_server not active after ${timeout}s (state: ${state:-not found}); continuing replay." >&2
  return 0
}

ros2 launch wheelchair_obstacle mppi_stack_no_sensors.launch.py \
  use_sim_time:="${SIM_TIME_FLAG}" \
  "${RVIZ_ARG}" \
  start_bag_record:=false &
STACK_PID=$!

echo "Launch settle: ${LAUNCH_SETTLE_SEC}s for nodes to register..."
if ! sleep "${LAUNCH_SETTLE_SEC}"; then
  on_interrupt
fi

echo "Stack warmup: ${STACK_WARMUP_SEC}s before bag record/play..."
if ! sleep "${STACK_WARMUP_SEC}"; then
  on_interrupt
fi

ros2 bag record "${RECORD_SIM_ARGS[@]}" -o "${REPLAY_BAG}" "${RECORD_TOPICS[@]}" &
REC_PID=$!
sleep 1

echo "Playing baseline bag (~${DURATION_S}s). /clock and sensor inputs come from the bag."
set +e
timeout --signal INT "${DURATION_S}" \
  ros2 bag play "${BASELINE_BAG}" \
    --rate "${PLAY_RATE}" \
    "${PLAY_CLOCK_FLAG[@]}" \
    --qos-profile-overrides-path "${QOS_FILE}" \
    --topics "${PLAY_TOPICS[@]}" &
PLAY_PID=$!

# Best-effort check after /clock is available; never block completion on this.
wait_for_controller_active 20 &
WAIT_PID=$!

wait "${PLAY_PID}"
play_status=$?
kill "${WAIT_PID}" 2>/dev/null || true
wait "${WAIT_PID}" 2>/dev/null || true
set -e

sleep 1
cleanup
trap - EXIT INT TERM
REC_PID=""
PLAY_PID=""
WAIT_PID=""
STACK_PID=""

if [[ "${INTERRUPTED}" -eq 1 ]]; then
  exit 130
fi

if [[ "${play_status}" -ne 0 && "${play_status}" -ne 124 && "${play_status}" -ne 130 ]]; then
  echo "ros2 bag play failed with status ${play_status}" >&2
  exit "${play_status}"
fi

python3 "${SCRIPT_DIR}/compare_mppi_bag_replay.py" \
  --baseline "${BASELINE_BAG}" \
  --replay "${REPLAY_BAG}" \
  --output "${REPORT}"

echo ""
echo "Done."
echo "Replay bag: ${REPLAY_BAG}"
echo "Report:     ${REPORT}"
