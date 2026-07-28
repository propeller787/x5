#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS2 Humble not found. Source /opt/ros/humble/setup.bash on the vehicle first." >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
cd "${ROOT}"

colcon build \
  --packages-select localmap_occupymap \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# shellcheck disable=SC1091
source install/setup.bash
echo "Build complete: localmap_occupymap v$(cat "${ROOT}/VERSION" | tr -d '[:space:]')"
