#!/usr/bin/env bash
# Replay a recorded bag through safety_node, record new outputs, and diff metrics.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${PKG_ROOT}/../../.." && pwd)}"

BASELINE_BAG=""
REPLAY_BAG=""
REPORT=""
PARAMS_FILE=""
PLAY_RATE="1.0"
USE_SIM_TIME=1
DRY_RUN=0
SKIP_BUILD=0

usage()
{
  cat <<'USAGE'
Replay a baseline rosbag through safety_node and compare outputs offline.

Usage:
  tools/replay_and_compare_bag.sh --baseline PATH [options]

Options:
  --baseline PATH     Input bag directory (required)
  --replay PATH       Output bag path. Default: bags/replay_<basename>_TIMESTAMP
  --report PATH       Markdown report path. Default: reports/<basename>_compare.md
  --params PATH       safety_node params yaml
  --rate RATE         ros2 bag play rate. Default: 1.0
  --wall-clock        Use wall clock instead of sim time (not recommended)
  --skip-build        Do not run colcon build for safety_node
  --dry-run           Print planned commands without executing
  -h, --help          Show this help

Examples:
  tools/replay_and_compare_bag.sh \\
    --baseline bags/20260610_101927_T3_SIM-031_car01

  tools/replay_and_compare_bag.sh \\
    --baseline bags/20260610_101927_T3_SIM-031_car01 \\
    --replay bags/replay_SIM-031_test \\
    --report reports/SIM-031_compare.md
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
    --params)
      PARAMS_FILE="$2"
      shift 2
      ;;
    --rate)
      PLAY_RATE="$2"
      shift 2
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
  BASELINE_BAG="${PKG_ROOT}/${BASELINE_BAG}"
fi
if [[ ! -d "${BASELINE_BAG}" ]]; then
  echo "Baseline bag not found: ${BASELINE_BAG}" >&2
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

source /opt/ros/humble/setup.bash
if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  source "${WORKSPACE_ROOT}/install/setup.bash"
fi
set -u

if [[ -z "${PARAMS_FILE}" ]]; then
  PARAMS_FILE="$(ros2 pkg prefix safety_node)/share/safety_node/config/safety_node_params.yaml"
fi

PLAY_TOPICS=(
  /handle/control
  /wheel/twist
  /scan/left
  /scan/right
  /scan/back
  /perception/left_tilt/pointcloud
  /perception/right_tilt/pointcloud
  /tf_static
)

RECORD_TOPICS=(
  /handle/control
  /wheel/twist
  /nav/control
  /safety/intervention_log
)

DURATION_S="$(python3 - <<PY
import yaml
from pathlib import Path
meta = Path("${BASELINE_BAG}") / "metadata.yaml"
data = yaml.safe_load(meta.read_text())
print(data["rosbag2_bagfile_information"]["duration"]["nanoseconds"] / 1e9 + 2.0)
PY
)"

echo "Workspace:  ${WORKSPACE_ROOT}"
echo "Baseline:   ${BASELINE_BAG}"
echo "Replay out: ${REPLAY_BAG}"
echo "Report:     ${REPORT}"
echo "Params:     ${PARAMS_FILE}"
echo "Duration:   ${DURATION_S}s (+2s margin)"
echo "Sim time:   ${USE_SIM_TIME}"

SIM_TIME_ARGS=()
PLAY_CLOCK_FLAG=()
RECORD_SIM_ARGS=()
if [[ "${USE_SIM_TIME}" -eq 1 ]]; then
  SIM_TIME_ARGS=(--ros-args -p use_sim_time:=true)
  PLAY_CLOCK_FLAG=(--clock 100)
  RECORD_SIM_ARGS=(--use-sim-time)
fi

if [[ "${DRY_RUN}" -eq 1 ]]; then
  echo "[dry-run] colcon build --packages-select safety_node (unless --skip-build)"
  echo "[dry-run] ros2 run safety_node safety_node_node --params-file ${PARAMS_FILE} ${SIM_TIME_ARGS[*]}"
  echo "[dry-run] ros2 bag record ${RECORD_SIM_ARGS[*]} -o ${REPLAY_BAG} ${RECORD_TOPICS[*]}"
  echo "[dry-run] ros2 bag play ${BASELINE_BAG} --rate ${PLAY_RATE} ${PLAY_CLOCK_FLAG[*]} --topics ${PLAY_TOPICS[*]}"
  echo "[dry-run] python3 ${SCRIPT_DIR}/compare_bag_replay.py --baseline ${BASELINE_BAG} --replay ${REPLAY_BAG} --output ${REPORT}"
  exit 0
fi

if [[ "${SKIP_BUILD}" -eq 0 ]]; then
  cd "${WORKSPACE_ROOT}"
  colcon build --symlink-install --packages-select safety_node
  source "${WORKSPACE_ROOT}/install/setup.bash"
fi

mkdir -p "$(dirname "${REPLAY_BAG}")" "$(dirname "${REPORT}")"

SAFETY_PID=""
REC_PID=""
cleanup()
{
  if [[ -n "${REC_PID}" ]] && kill -0 "${REC_PID}" 2>/dev/null; then
    kill -INT "${REC_PID}" 2>/dev/null || true
    wait "${REC_PID}" 2>/dev/null || true
  fi
  if [[ -n "${SAFETY_PID}" ]] && kill -0 "${SAFETY_PID}" 2>/dev/null; then
    kill "${SAFETY_PID}" 2>/dev/null || true
    wait "${SAFETY_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ros2 run safety_node safety_node_node --ros-args --params-file "${PARAMS_FILE}" "${SIM_TIME_ARGS[@]}" &
SAFETY_PID=$!
sleep 2

ros2 bag record "${RECORD_SIM_ARGS[@]}" -o "${REPLAY_BAG}" "${RECORD_TOPICS[@]}" &
REC_PID=$!
sleep 1

set +e
timeout --signal INT "${DURATION_S}" \
  ros2 bag play "${BASELINE_BAG}" --rate "${PLAY_RATE}" "${PLAY_CLOCK_FLAG[@]}" --topics "${PLAY_TOPICS[@]}"
play_status=$?
set -e

sleep 1
cleanup
trap - EXIT
REC_PID=""
SAFETY_PID=""

if [[ "${play_status}" -ne 0 && "${play_status}" -ne 124 && "${play_status}" -ne 130 ]]; then
  echo "ros2 bag play failed with status ${play_status}" >&2
  exit "${play_status}"
fi

python3 "${SCRIPT_DIR}/compare_bag_replay.py" \
  --baseline "${BASELINE_BAG}" \
  --replay "${REPLAY_BAG}" \
  --output "${REPORT}"

echo ""
echo "Done."
echo "Replay bag: ${REPLAY_BAG}"
echo "Report:     ${REPORT}"
