# freego

#### 软件架构

+-------------------------------------------------------------+
| 云端/运维平台 |
| - OTA / 镜像库 / 节点配置 / 远程回放 / 数据仓库 / 告警 |
+-------------------------------------------------------------+
^ ^
| 4G/5G / Wi-Fi / RS232 | 车载与 Pad 通信（WebSocket/MQTT/DDS)
+-------------------------------------------------------------+
| 车载 PAD（人机交互） 远程诊断 / 运维界面 |
+-------------------------------------------------------------+
| 车载主控（ROS2 Domain） - 容器/进程隔离（Composition） |
| +----------------+ +------------------+ +---------------+ |
| | 接入层 (Drivers)| | 算法层 (Perception)| | 决策/控制层 | |
| | - GPS/IMU/CAN | | - Detector/Fusion | | - Mission FSM | |
| | - RGB-D / Lidar | | - Localization | | - Planner | |
| +----------------+ +------------------+ +---------------+ |
| | | | |
| v v v |
| 车辆执行层 (CAN / Actuators / Brakes / E-Stop) |
+-------------------------------------------------------------+
| 硬件（SoC, GPU, NIC, CAN-IF, Modem, IMU, Cameras, Lidar, GPS） |
+-------------------------------------------------------------+
common 工具
control_executor 控制执行
decision_manager 决策层
freego_resources 配置和启动服务
interface msg/svr 文件 完成
ota_agent OTA
pad_interface pad 通行模块 完成
perception 算法模块
postback 数据回传
scene 模式
sensor 接入模块
system_monitor 系统监控与诊断
colud_manager 云通信 上传设备 ID
tft_manager tft 服务
transducer 激光雷达 RGB-D

落盘数据/opt/recode (can imu+camera(h264) log lidar)

1. 安装 ros2 humble 版本
   wget http://fishros.com/install -O fishros && . fishros
2. 安装三方库
   sudo apt install ros-humble-robot-state-publisher
   sudo apt install ros-humble-rosbridge-suite
   sudo apt install ros-humble-ffmpeg-image-transport-msgs
   sudo apt install -y protobuf-compiler libprotobuf-dev  
   sudo apt install ros-humble-can-msgs
   sudo apt install -y libzmq3-dev zmqpp-dev 默认关闭中 如果想使用打开 common 的中注释
3. 安装 muduo 第三方库（只适用于 3588 (22.04 版本)） https://alidocs.dingtalk.com/i/nodes/9E05BDRVQ2nlQoXPCGjyawd3J63zgkYA?doc_type=wiki_doc
   cd release-install-cpp11
   sudo cp include/muduo /usr/include/ -rf sudo cp lib/\* /usr/lib -rf
4. 安装 geographiclib 第三方库 https://alidocs.dingtalk.com/i/nodes/9E05BDRVQ2nlQoXPCGjyawd3J63zgkYA?doc_type=wiki_doc
   mkdir build
   cmake ..
   make -j4
   sudo make install
5. 安装模型使用 cd etc unzip rknpu2_install2.1.zip sudo cp rknpu2_install2.1 /usr/local
6. wsl 版本 暂不编译 sensor transducer

# 数据回传配置

# config.yaml

handle:
ros\_\_parameters:
can_record:
interval_minutes: 10 #间隔时长
directory: "/opt/freego/recode/can" #默认存放位置
topic_name: "/can/frames" #默认回放 topic
其他节点修改方法

# 修改记录间隔为 10 分钟

ros2 param set /handle can_record.interval_minutes 10

# 修改记录目录

ros2 param set /handle can_record.directory "/tmp/can_data"

# 修改 topic 名称

ros2 param set /handle can_record.topic_name "/can/raw_frames"
