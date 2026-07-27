#!/usr/bin/env bash
# Cyclic rosbag2 recorder for the MPPI navigation stack.
# Records one bag every INTERVAL seconds, keeps at most MAX_COUNT bags (FIFO).
#
# Usage (standalone):
#   cyclic_rosbag_record.sh --interval 60 --max-count 10 --output-dir ~/bags/mppi_stack
#
# Usually started from mppi_stack_no_sensors.launch.py (start_bag_record:=true).

set -eo pipefail

INTERVAL_SEC=60
MAX_COUNT=10
OUTPUT_DIR="${HOME}/bags/mppi_stack_cyclic"
STORAGE_ID="sqlite3"
QOS_FILE=""
TOPICS_FILE=""
DRY_RUN=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_QOS="${SCRIPT_DIR}/../config/mppi_stack_bag_qos.yaml"
DEFAULT_TOPICS="${SCRIPT_DIR}/../config/mppi_stack_bag_topics.txt"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi
if [[ -n "${ROS_WORKSPACE:-}" && -f "${ROS_WORKSPACE}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${ROS_WORKSPACE}/install/setup.bash"
  set -u
elif [[ -f "${SCRIPT_DIR}/../../../../install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/../../../../install/setup.bash"
  set -u
fi

set -u

usage() {
  cat <<'USAGE'
Cyclic rosbag2 recorder: one bag per interval, keep newest N bags.

Options:
  --interval SEC       Seconds per bag (default: 60)
  --max-count N        Max bags to retain (default: 10)
  --output-dir DIR     Output directory (default: ~/bags/mppi_stack_cyclic)
  --storage ID         rosbag2 storage backend (default: sqlite3)
  --qos-file PATH      QoS overrides YAML (default: package config)
  --topics-file PATH   Newline-separated topic list (default: package config)
  --dry-run            Print actions without recording
  -h, --help           Show help
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --interval) INTERVAL_SEC="$2"; shift 2 ;;
    --max-count) MAX_COUNT="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --storage) STORAGE_ID="$2"; shift 2 ;;
    --qos-file) QOS_FILE="$2"; shift 2 ;;
    --topics-file) TOPICS_FILE="$2"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if ! [[ "${INTERVAL_SEC}" =~ ^[1-9][0-9]*$ ]]; then
  echo "[ERROR] --interval must be a positive integer (seconds)" >&2
  exit 2
fi
if ! [[ "${MAX_COUNT}" =~ ^[1-9][0-9]*$ ]]; then
  echo "[ERROR] --max-count must be a positive integer" >&2
  exit 2
fi

QOS_FILE="${QOS_FILE:-${DEFAULT_QOS}}"
TOPICS_FILE="${TOPICS_FILE:-${DEFAULT_TOPICS}}"

if [[ ! -f "${QOS_FILE}" ]]; then
  echo "[ERROR] QoS file not found: ${QOS_FILE}" >&2
  exit 1
fi
if [[ ! -f "${TOPICS_FILE}" ]]; then
  echo "[ERROR] Topics file not found: ${TOPICS_FILE}" >&2
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] ros2 not found in PATH" >&2
  exit 1
fi

mapfile -t TOPICS < <(grep -v '^[[:space:]]*#' "${TOPICS_FILE}" | grep -v '^[[:space:]]*$' || true)
if [[ ${#TOPICS[@]} -eq 0 ]]; then
  echo "[ERROR] No topics in ${TOPICS_FILE}" >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"

CURRENT_RECORD_PID=""
STOP_REQUESTED=0

stop_current_record() {
  if [[ -n "${CURRENT_RECORD_PID}" ]] && kill -0 "${CURRENT_RECORD_PID}" 2>/dev/null; then
    kill -INT "${CURRENT_RECORD_PID}" 2>/dev/null || true
    wait "${CURRENT_RECORD_PID}" 2>/dev/null || true
  fi
  CURRENT_RECORD_PID=""
}

on_signal() {
  STOP_REQUESTED=1
  stop_current_record
}

trap on_signal INT TERM

prune_old_bags() {
  local -a bags=()
  local bag
  shopt -s nullglob
  for bag in "${OUTPUT_DIR}"/bag_*; do
    [[ -d "${bag}" ]] && bags+=("${bag}")
  done
  shopt -u nullglob

  if [[ ${#bags[@]} -le ${MAX_COUNT} ]]; then
    return 0
  fi

  # Delete oldest bags first (by directory mtime).
  mapfile -t bags < <(printf '%s\n' "${bags[@]}" | xargs -I{} stat -c '%Y %n' {} 2>/dev/null | sort -n | awk '{print $2}')
  local to_delete=$(( ${#bags[@]} - MAX_COUNT ))
  local i
  for (( i=0; i<to_delete; i++ )); do
    echo "[INFO] Removing old bag: ${bags[i]}"
    rm -rf "${bags[i]}"
  done
}

record_one_bag() {
  local stamp bag_path
  stamp="$(date +%Y%m%d_%H%M%S)"
  bag_path="${OUTPUT_DIR}/bag_${stamp}"

  local -a cmd=(
    ros2 bag record
    --storage "${STORAGE_ID}"
    --qos-profile-overrides-path "${QOS_FILE}"
    -o "${bag_path}"
  )
  cmd+=("${TOPICS[@]}")

  echo "[INFO] Recording ${INTERVAL_SEC}s -> ${bag_path}"
  if [[ ${DRY_RUN} -eq 1 ]]; then
    printf '  '
    printf '%q ' "${cmd[@]}"
    printf '\n'
    return 0
  fi

  set +e
  timeout --signal=INT "${INTERVAL_SEC}" "${cmd[@]}" &
  CURRENT_RECORD_PID=$!
  wait "${CURRENT_RECORD_PID}"
  local status=$?
  set -e
  CURRENT_RECORD_PID=""

  if [[ ${status} -eq 124 || ${status} -eq 130 || ${status} -eq 0 ]]; then
    echo "[INFO] Bag segment finished (exit ${status}): ${bag_path}"
    prune_old_bags
    return 0
  fi
  echo "[WARN] ros2 bag record exited with status ${status}: ${bag_path}" >&2
  prune_old_bags
  return 0
}

echo "[INFO] Cyclic rosbag recorder"
echo "[INFO]   interval : ${INTERVAL_SEC}s"
echo "[INFO]   max count: ${MAX_COUNT}"
echo "[INFO]   output   : ${OUTPUT_DIR}"
echo "[INFO]   topics   : ${#TOPICS[@]} from ${TOPICS_FILE}"
echo "[INFO]   qos      : ${QOS_FILE}"

if [[ ${DRY_RUN} -eq 1 ]]; then
  record_one_bag
  exit 0
fi

while [[ ${STOP_REQUESTED} -eq 0 ]]; do
  record_one_bag || true
done

echo "[INFO] Cyclic rosbag recorder stopped"
exit 0
