#!/bin/bash
# deploy_exec.sh
set -e

TARGET_IP=$1
TARGET_PATH="/opt/freego"
TARGET_CAN_PATH="/opt/can"
TARGET_USR_LOCAL_PATH="/usr/local"
TARGET_SYSTEMD_PATH="/etc/systemd/system"

if [ -z "$TARGET_IP" ]; then
    echo "Usage: $0 <target_ip>"
    exit 1
fi

unzip ./etc/rknpu2_install2.1.zip -d ./etc/rknpu2_install2.1

echo "==== step 1: 推送文件到目标机器 ===="
ssh $TARGET_IP "sudo mkdir -p $TARGET_PATH"
ssh $TARGET_IP "sudo mkdir -p $TARGET_CAN_PATH"
rsync -av --delete ./install $TARGET_IP:$TARGET_PATH/install
rsync -av --delete ./etc/can_init.sh $TARGET_IP:$TARGET_CAN_PATH/can_init.sh
rsync -av --delete ./etc/ch9344.ko $TARGET_IP:$TARGET_CAN_PATH/ch9344.ko
rsync -av --delete ./etc/rknpu2_install2.1 $TARGET_IP:$TARGET_USR_LOCAL_PATH
rsync -av --delete ./etc/freego.service $TARGET_IP:$TARGET_SYSTEMD_PATH/freego.service
rsync -av --delete ./etc/caninit.service $TARGET_IP:$TARGET_SYSTEMD_PATH/caninit.service
echo "==== step 2: 重启目标机器 ===="
ssh $TARGET_IP "sudo systemctl daemon-reload"
ssh $TARGET_IP "sudo systemctl enable freego.service"
ssh $TARGET_IP "sudo systemctl enable caninit.service"
ssh $TARGET_IP "sudo systemctl start freego.service"
ssh $TARGET_IP "sudo systemctl start caninit.service"