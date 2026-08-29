#!/bin/bash

# 同步 perception（及可选 localization / nav2_mppi_controller）到车端并编译
#
# 用法: ./sync_perception.sh [模式]
#   1 — 仅同步 perception，编译 wheelchair_obstacle
#   2 — 同步 perception + localization + nav2_mppi_controller，编译三者（默认）

MODE="${1:-2}"

REMOTE_USER="ztl"
REMOTE_IP="192.168.100.101"
LOCAL_SRC_DIR="$HOME/code/freego_2025_tft/src"
REMOTE_DEST_DIR="/home/zq/freego_2025/src"
REMOTE_WS_DIR="/home/zq/freego_2025"
ZIP_NAME="perception_sync.zip"

# 车端内存有限：nav2_mppi_controller 单独编译且限制并行，避免 cc1plus OOM Killed
MPPI_BUILD_JOBS="${MPPI_BUILD_JOBS:-1}"

case "$MODE" in
    1)
        SYNC_FOLDERS=(perception)
        BUILD_PACKAGES=(wheelchair_obstacle)
        OVERRIDE_PACKAGES=()
        ;;
    2)
        SYNC_FOLDERS=(perception localization nav2_mppi_controller)
        BUILD_PACKAGES=(wheelchair_obstacle localization nav2_mppi_controller)
        OVERRIDE_PACKAGES=(nav2_mppi_controller)
        ;;
    *)
        echo "用法: $0 [1|2]"
        echo "  1 — 仅同步 perception，编译 wheelchair_obstacle"
        echo "  2 — 同步 perception + localization + nav2_mppi_controller，编译三者（默认）"
        exit 1
        ;;
esac

echo ">>> 模式 ${MODE}: 同步 [${SYNC_FOLDERS[*]}]，编译 [${BUILD_PACKAGES[*]}]"

echo ">>> 步骤 1: 在本地打包 ${SYNC_FOLDERS[*]} ..."
cd "$LOCAL_SRC_DIR" || exit 1

for folder in "${SYNC_FOLDERS[@]}"; do
    if [ ! -d "$folder" ]; then
        echo "错误: 本地目录 $LOCAL_SRC_DIR/$folder 不存在"
        exit 1
    fi
done

rm -f "$ZIP_NAME"
zip -r "$ZIP_NAME" "${SYNC_FOLDERS[@]}"

if [ $? -ne 0 ]; then
    echo "本地打包失败，退出。"
    exit 1
fi
echo "本地打包完成: $ZIP_NAME"

echo ">>> 步骤 2: 上传压缩包到远程服务器..."
scp "$ZIP_NAME" "${REMOTE_USER}@${REMOTE_IP}:${REMOTE_DEST_DIR}/"

if [ $? -ne 0 ]; then
    echo "SCP 上传失败，退出。"
    exit 1
fi
echo "上传完成。"

FOLDERS_SPACE="${SYNC_FOLDERS[*]}"
BUILD_SPACE="${BUILD_PACKAGES[*]}"
OVERRIDE_SPACE="${OVERRIDE_PACKAGES[*]}"

echo ">>> 步骤 3: 远程清理旧文件夹、解压并编译..."
ssh "${REMOTE_USER}@${REMOTE_IP}" bash -s <<EOF
set -e
cd "${REMOTE_DEST_DIR}"

for folder in ${FOLDERS_SPACE}; do
    if [ -d "\$folder" ]; then
        rm -rf "\$folder"
        echo "已删除远程旧文件夹: \$folder"
    fi
done

unzip -o "${ZIP_NAME}"
echo "远程解压成功。"
rm -f "${ZIP_NAME}"
echo "已清理远程压缩包。"

echo ">>> 开始编译: ${BUILD_SPACE}"
cd "${REMOTE_WS_DIR}"
source /opt/ros/humble/setup.bash
source install/setup.bash

if [ "${MODE}" = "1" ]; then
    colcon build --packages-select wheelchair_obstacle --parallel-workers 2
else
    # 先编轻量包（可并行）
    echo ">>> [1/2] localization + wheelchair_obstacle ..."
    colcon build --packages-select localization wheelchair_obstacle --parallel-workers 2

    # nav2_mppi_controller 模板/xtensor 编译极耗内存，必须单独编且 -j1
    echo ">>> [2/2] nav2_mppi_controller (MAKE -j${MPPI_BUILD_JOBS}, 防 OOM) ..."
    export MAKEFLAGS="-j${MPPI_BUILD_JOBS}"
    colcon build --packages-select nav2_mppi_controller \
      --allow-overriding ${OVERRIDE_SPACE} \
      --parallel-workers 1 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_PARALLEL_LEVEL=${MPPI_BUILD_JOBS}
    unset MAKEFLAGS
fi

echo "编译成功。"
EOF

if [ $? -ne 0 ]; then
    echo "远程操作失败，请检查网络连接、权限或编译日志。"
    exit 1
fi

echo ">>> 步骤 4: 清理本地临时压缩包..."
rm -f "$ZIP_NAME"
echo "本地压缩包已删除。"

echo ">>> 全部流程完成（模式 ${MODE}）。已同步 ${SYNC_FOLDERS[*]} 至远程并编译 ${BUILD_PACKAGES[*]}。"
