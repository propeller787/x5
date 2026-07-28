#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-autoware_chaire}"
WORKSPACE="/workspace/chaire/localmap"

docker exec -it "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  cd ${WORKSPACE}
  colcon build --packages-select localmap_occupymap --cmake-args -DCMAKE_BUILD_TYPE=Release
  source install/setup.bash
  echo 'Build complete. Run with:'
  echo '  ros2 launch localmap_occupymap occupymap.launch.py'
"
