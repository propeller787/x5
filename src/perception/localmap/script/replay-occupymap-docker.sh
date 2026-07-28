#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-autoware_chaire}"
BAG_DIR="${BAG_DIR:-/workspace/chaire/bags/case1}"
WORKSPACE="/workspace/chaire/localmap"

docker exec -d "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  source ${WORKSPACE}/install/setup.bash
  ros2 launch localmap_occupymap occupymap.launch.py
"

sleep 2

docker exec "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  ros2 bag play ${BAG_DIR} --clock
"
