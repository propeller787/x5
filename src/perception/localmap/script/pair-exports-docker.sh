#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-autoware_chaire}"
WORKSPACE="/workspace/chaire/localmap"
MAX_DT_MS="${MAX_DT_MS:-100}"

docker exec "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  cd ${WORKSPACE}/script
  python3 pair_costmap_exports.py \
    --costmap-dir ${WORKSPACE}/output/costmap_raw/case1 \
    --occupymap-dir ${WORKSPACE}/output/occupymap_raw/case1 \
    --output ${WORKSPACE}/output/pairs/case1 \
    --max-dt-ms ${MAX_DT_MS} \
    --side-by-side
"

echo "Pairs: localmap/output/pairs/case1/pairs.csv"
echo "Side-by-side: localmap/output/pairs/case1/side_by_side/"
