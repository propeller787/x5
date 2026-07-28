#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-autoware_chaire}"
WORKSPACE="/workspace/chaire/localmap"
BAG_DIR="${BAG_DIR:-/workspace/chaire/bags/case1}"
RVIZ_CONFIG="${WORKSPACE}/config/costmap_raw_view.rviz"

docker exec -d "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  cd ${WORKSPACE}/script
  python3 costmap_raw_viz.py --ros-args \
    -p input_topic:=/local_costmap/costmap_raw \
    -p output_topic:=/local_costmap/costmap_raw/viz
"

docker exec -d "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  cd ${WORKSPACE}/script
  python3 costmap_raw_viz.py --ros-args \
    -p input_topic:=/localmap/occupymap_raw \
    -p output_topic:=/localmap/occupymap_raw/viz
"

sleep 1

echo "Starting RViz (costmap_raw_view)..."
docker exec -d "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  rviz2 -d ${RVIZ_CONFIG}
"

sleep 2

echo "Playing bag: ${BAG_DIR}"
docker exec "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  ros2 bag play ${BAG_DIR} --clock
"
