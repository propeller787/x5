#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-autoware_chaire}"
WORKSPACE="/workspace/chaire/localmap"
BAG_DIR="${BAG_DIR:-/workspace/chaire/bags/case1}"
TOPIC="${TOPIC:-/localmap/occupymap_raw}"
OUTPUT="${OUTPUT:-${WORKSPACE}/output/occupymap_raw/case1}"
IDLE_TIMEOUT="${IDLE_TIMEOUT:-3.0}"
HOST_OUTPUT="/Users/zhengqi/workspace/chaire/localmap/output/occupymap_raw/case1"
HOST_COSTMAP_OUTPUT="/Users/zhengqi/workspace/chaire/localmap/output/costmap_raw/case1"
CLEAN="${CLEAN:-1}"
EXPORT_COSTMAP="${EXPORT_COSTMAP:-1}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cleanup() {
  docker exec "${CONTAINER_NAME}" bash -lc "
    pkill -f 'localmap_occupymap/occupymap_node' 2>/dev/null || true
    pkill -f 'export_costmap_live.py' 2>/dev/null || true
  " || true
}
trap cleanup EXIT

if [[ "${EXPORT_COSTMAP}" == "1" ]]; then
  echo "=== Export costmap_raw from bag -> output/costmap_raw/case1/ ==="
  CLEAN="${CLEAN}" "${ROOT}/script/export-costmap-raw-docker.sh"
  echo ""
fi

if [[ "${CLEAN}" == "1" ]]; then
  echo "Cleaning ${HOST_OUTPUT} ..."
  rm -rf "${HOST_OUTPUT}"
fi
mkdir -p "${HOST_OUTPUT}"

echo "Building occupymap..."
docker exec "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  cd ${WORKSPACE}
  colcon build --packages-select localmap_occupymap --cmake-args -DCMAKE_BUILD_TYPE=Release
"

echo "Starting occupymap_node (use_sim_time)..."
docker exec -d "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  source ${WORKSPACE}/install/setup.bash
  ros2 launch localmap_occupymap occupymap.launch.py use_sim_time:=true
"

sleep 2

echo "Starting live exporter (${TOPIC}) -> ${OUTPUT}"
docker exec -d "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  cd ${WORKSPACE}/script
  python3 export_costmap_live.py \
    -t ${TOPIC} \
    -o ${OUTPUT} \
    --prefix occupymap_raw \
    --use-sim-time \
    --bag-dir ${BAG_DIR} \
    --idle-timeout ${IDLE_TIMEOUT}
"

sleep 1

echo "Playing bag: ${BAG_DIR}"
docker exec "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  ros2 bag play ${BAG_DIR} --clock
"

WAIT_SEC=$(( ${IDLE_TIMEOUT%.*} + 2 ))
echo "Waiting ${WAIT_SEC}s for exporter to finish..."
sleep "${WAIT_SEC}"

FRAME_COUNT="$(find "${HOST_OUTPUT}" -name 'occupymap_raw_*.png' 2>/dev/null | wc -l | tr -d ' ')"
COSTMAP_COUNT="$(find "${HOST_COSTMAP_OUTPUT}" -name 'costmap_raw_*.png' 2>/dev/null | wc -l | tr -d ' ')"
echo "Done."
echo "  costmap:   ${COSTMAP_COUNT} PNGs in output/costmap_raw/case1/"
echo "  occupymap: ${FRAME_COUNT} PNGs in output/occupymap_raw/case1/"
echo "Pair with: ./script/pair-exports-docker.sh"
