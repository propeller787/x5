#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f "${ROOT}/install/setup.bash" ]]; then
  echo "Workspace not built. Run ./script/build-occupymap.sh first." >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROOT}/install/setup.bash"

exec ros2 launch localmap_occupymap occupymap.launch.py "$@"
