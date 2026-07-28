#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-autoware_chaire}"
WORKSPACE="/workspace/chaire/localmap"
BAG_DIR="${BAG_DIR:-/workspace/chaire/bags/case1}"
TOPIC="${TOPIC:-/local_costmap/costmap_raw}"
OUTPUT="${OUTPUT:-${WORKSPACE}/output/costmap_raw}"
HOST_OUTPUT="/Users/zhengqi/workspace/chaire/localmap/output/costmap_raw/case1"
CLEAN="${CLEAN:-1}"

if [[ "${CLEAN}" == "1" ]]; then
  echo "Cleaning ${HOST_OUTPUT} ..."
  rm -rf "${HOST_OUTPUT}"
fi

docker exec "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  cd ${WORKSPACE}/script
  python3 export_costmap_raw.py ${BAG_DIR} -o ${OUTPUT} -t ${TOPIC}
"

FRAME_COUNT="$(find "${HOST_OUTPUT}" -name 'costmap_raw_*.png' 2>/dev/null | wc -l | tr -d ' ')"
echo "Done. ${FRAME_COUNT} costmap PNGs in output/costmap_raw/case1/ (occupymap 在 output/occupymap_raw/case1/)"
