#!/usr/bin/env bash
# Export occupymap_raw (live replay) and costmap_raw (offline bag) to separate dirs.
# Both use script/costmap_colormap.py for identical rendering.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=== 1/2 costmap_raw from bag -> output/costmap_raw/case1/ ==="
"${ROOT}/script/export-costmap-raw-docker.sh"

echo ""
echo "=== 2/2 occupymap_raw live replay -> output/occupymap_raw/case1/ ==="
"${ROOT}/script/export-occupymap-docker.sh"

echo ""
echo "Compare:"
echo "  costmap:   ${ROOT}/output/costmap_raw/case1/costmap_raw_*.png"
echo "  occupymap: ${ROOT}/output/occupymap_raw/case1/occupymap_raw_*.png"
