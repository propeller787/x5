#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"

STAGE="${STAGE:-T3}"
CASE_ID="${CASE_ID:-manual}"
VEHICLE="${VEHICLE:-vehicle}"
STORAGE_ID="${STORAGE_ID:-sqlite3}"
DURATION=""
OUTPUT=""
DRY_RUN=0

usage()
{
  cat <<'USAGE'
Record safety_node validation topics with rosbag2.

Usage:
  tools/record_safety_topics.sh [options]

Options:
  -o, --output PATH      Bag output path. Defaults to bags/YYYYMMDD_HHMMSS_<stage>_<case>_<vehicle>
      --stage NAME      Stage label, for example T3 or T4. Default: T3
      --case-id ID      Case ID, for example SIM-002 or HIL-ESTOP. Default: manual
      --vehicle ID      Vehicle or bench ID. Default: vehicle
      --duration SEC    Stop recording after SEC seconds. Default: record until Ctrl+C
      --storage ID      rosbag2 storage ID. Default: sqlite3
      --dry-run         Print the command without recording
  -h, --help            Show this help

Examples:
  tools/record_safety_topics.sh --stage T3 --case-id SIM-002 --vehicle car01
  tools/record_safety_topics.sh --stage T4 --case-id HIL-ESTOP --duration 120
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -o|--output)
      OUTPUT="$2"
      shift 2
      ;;
    --stage)
      STAGE="$2"
      shift 2
      ;;
    --case-id)
      CASE_ID="$2"
      shift 2
      ;;
    --vehicle)
      VEHICLE="$2"
      shift 2
      ;;
    --duration)
      DURATION="$2"
      shift 2
      ;;
    --storage)
      STORAGE_ID="$2"
      shift 2
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

source /opt/ros/humble/setup.bash
if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  source "${WORKSPACE_ROOT}/install/setup.bash"
fi
set -u

if [[ -z "${OUTPUT}" ]]; then
  stamp="$(date +%Y%m%d_%H%M%S)"
  OUTPUT="${WORKSPACE_ROOT}/bags/${stamp}_${STAGE}_${CASE_ID}_${VEHICLE}"
fi
mkdir -p "$(dirname "${OUTPUT}")"

QOS_FILE="$(mktemp /tmp/safety_bag_qos.XXXXXX.yaml)"
trap 'rm -f "${QOS_FILE}"' EXIT

cat > "${QOS_FILE}" <<'YAML'
/scan/left:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10
/scan/right:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10
/scan/back:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10
/safety/debug/perception_points:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10
/tf_static:
  reliability: reliable
  durability: transient_local
  history: keep_last
  depth: 100
YAML

TOPICS=(
  /handle/control
  /sensor/joystick
  /wheel/twist
  /scan/left
  /scan/right
  /scan/back
  /perception/left_tilt/pointcloud
  /perception/right_tilt/pointcloud
  /nav/control
  /safety/debug/perception_points
  /safety/debug/perception_markers
  /safety/local_map
  /safety/intervention_log
  /tf
  /tf_static
  /rosout
)

CMD=(
  ros2 bag record
  --storage "${STORAGE_ID}"
  --qos-profile-overrides-path "${QOS_FILE}"
  -o "${OUTPUT}"
  "${TOPICS[@]}"
)

echo "Workspace: ${WORKSPACE_ROOT}"
echo "Output:    ${OUTPUT}"
echo "QoS file:  ${QOS_FILE}"
printf 'Command:'
printf ' %q' "${CMD[@]}"
printf '\n'

if [[ "${DRY_RUN}" -eq 1 ]]; then
  exit 0
fi

if [[ -n "${DURATION}" ]]; then
  set +e
  timeout --signal INT "${DURATION}" "${CMD[@]}"
  status=$?
  set -e
  if [[ "${status}" -eq 124 || "${status}" -eq 130 ]]; then
    exit 0
  fi
  exit "${status}"
fi

exec "${CMD[@]}"
