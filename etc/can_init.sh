#!/bin/bash
ip link set can0 down
ip link set can0 type can bitrate 1000000
ip link set can0 up

ip link set can1 down
ip link set can1 type can bitrate 1000000
ip link set can1 up

stty -F /dev/ttyS5 115200 cs8 -parenb -cstopb
chmod 777 /dev/mpu6050
#insmod /opt/can/ch9344.ko