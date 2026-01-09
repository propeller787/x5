# Scene Auxiliary Test - 雷达数据处理和车辆控制干预系统

## 项目概述

Scene Auxiliary Test 是一个基于雷达数据的车辆安全辅助系统，能够实时监测车辆周围环境，并在检测到障碍物时自动进行速度干预，确保行车安全。该系统采用模块化设计，支持多雷达数据融合、目标跟踪、智能决策等功能。

## 系统特性

- **多雷达支持**: 支持左、右、左倾斜、右倾斜、后雷达数据
- **实时处理**: 10Hz 雷达数据处理，70Hz 控制输出
- **智能决策**: 基于距离的速度限制和紧急制动
- **目标跟踪**: 多目标跟踪和位置预测
- **平滑控制**: 指数平滑滤波避免急停急起
- **故障安全**: 雷达数据超时时的预测处理

## 系统架构

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   左雷达数据    │    │   右雷达数据    │    │   摇杆控制      │
│ /scan/left      │    │ /scan/right     │    │ /handle/control │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          └──────────────────────┼──────────────────────┘
                                 │
                    ┌─────────────▼──────────────┐
                    │      LidarDriver          │
                    │  (雷达数据接收与ROI过滤)    │
                    └─────────────┬──────────────┘
                                 │
                    ┌─────────────▼──────────────┐
                    │      Clustering           │
                    │    (点云聚类处理)          │
                    └─────────────┬──────────────┘
                                 │
                    ┌─────────────▼──────────────┐
                    │      Tracking             │
                    │    (目标跟踪与预测)        │
                    └─────────────┬──────────────┘
                                 │
                    ┌─────────────▼──────────────┐
                    │    DecisionMaker          │
                    │  (速度干预决策与平滑滤波)   │
                    └─────────────┬──────────────┘
                                 │
                    ┌─────────────▼──────────────┐
                    │    控制命令输出             │
                    │  /nav/control             │
                    └────────────────────────────┘
```

## 核心组件

### 1. LidarDriver (雷达驱动器)

- 接收多个雷达数据源
- 坐标系转换和 ROI 过滤
- 点云数据发布

### 2. Clustering (聚类处理器)

- 欧几里得聚类算法
- 聚类信息计算
- 噪声过滤

### 3. Tracking (目标跟踪器)

- 多目标跟踪
- 位置预测
- 数据关联

### 4. DecisionMaker (决策器)

- 距离检测
- 速度限制计算
- 碰撞检测

### 5. VehicleInterface (车辆接口)

- 摇杆控制接收
- 轮速反馈处理
- 车辆状态管理

## 快速开始

### 环境要求

- Ubuntu 20.04 LTS 或更高版本
- ROS2 Humble 或更高版本
- PCL 1.12 或更高版本
- Eigen3

### 安装依赖

```bash
# 安装ROS2依赖包
sudo apt install -y \
    ros-humble-rclcpp \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-std-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-laser-geometry \
    ros-humble-pcl-conversions

# 安装PCL库
sudo apt install -y libpcl-dev pcl-tools

# 安装Eigen3
sudo apt install -y libeigen3-dev
```

### 编译安装

```bash
# 创建工作空间
mkdir -p ~/vlago_ws/src
cd ~/vlago_ws

# 克隆项目
git clone https://github.com/your-org/vlago.git src/

# 安装依赖
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 编译项目
colcon build --packages-select scene_auxiliary_test

# 设置环境
source install/setup.bash
```

### 运行系统

```bash
# 启动系统
ros2 run scene_auxiliary_test scene_auxiliary_test_node

# 或者使用配置文件
ros2 run scene_auxiliary_test scene_auxiliary_test_node \
    --ros-args \
    --params-file config/scene_auxiliary_test/config.yaml
```

## 配置说明

### 基本配置

```yaml
system:
  radar_frequency: 10.0 # 雷达频率 (Hz)
  control_frequency: 70.0 # 控制频率 (Hz)
  radar_timeout: 0.15 # 雷达超时时间 (s)

topics:
  radar:
    left: "/scan/left"
    right: "/scan/right"
  control:
    handle: "/handle/control"
    nav: "/nav/control"
```

### ROI 区域配置

```yaml
radar:
  roi:
    left:
      min_x: 0.0 # 前方最小距离 (m)
      max_x: 1.1 # 前方最大距离 (m)
      min_y: -0.3 # 左侧最小距离 (m)
      max_y: 0.08 # 右侧最大距离 (m)
```

### 决策参数配置

```yaml
decision:
  min_distance_threshold: 0.3 # 最小距离阈值 (m)
  max_distance_threshold: 1.1 # 最大距离阈值 (m)
  min_speed_limit: 0.056 # 最小速度限制 (m/s)
  max_speed_limit: 1.67 # 最大速度限制 (m/s)
  emergency_brake_distance: 0.3 # 急刹距离 (m)
```

## 话题接口

### 订阅话题

- `/scan/left` - 左雷达数据 (sensor_msgs::msg::LaserScan)
- `/scan/right` - 右雷达数据 (sensor_msgs::msg::LaserScan)
- `/scan/left_tilt` - 左倾斜雷达数据 (sensor_msgs::msg::LaserScan)
- `/scan/right_tilt` - 右倾斜雷达数据 (sensor_msgs::msg::LaserScan)
- `/scan/back` - 后雷达数据 (sensor_msgs::msg::LaserScan)
- `/handle/control` - 摇杆控制数据 (geometry_msgs::msg::Twist)
- `/wheel/twist` - 轮速反馈数据 (geometry_msgs::msg::TwistStamped)

### 发布话题

- `/nav/control` - 干预后的控制命令 (geometry_msgs::msg::Twist)
- `/laser/left_roi` - 左雷达 ROI 过滤点云 (sensor_msgs::msg::PointCloud2)
- `/laser/right_roi` - 右雷达 ROI 过滤点云 (sensor_msgs::msg::PointCloud2)
- `/laser/left_base_link` - 左雷达 base_link 点云 (sensor_msgs::msg::PointCloud2)
- `/laser/right_base_link` - 右雷达 base_link 点云 (sensor_msgs::msg::PointCloud2)

## 速度干预策略

### 距离-速度映射

- **0-30cm**: 急刹 (速度限制: 0 m/s)
- **30-110cm**: 线性插值 (0.2-6 km/h)
- **>110cm**: 无限制

### 速度计算公式

```cpp
speed_limit = min_speed + (distance - min_distance) / (max_distance - min_distance) * (max_speed - min_speed)
```

其中：

- min_speed = 0.056 m/s (0.2 km/h)
- max_speed = 1.67 m/s (6 km/h)
- min_distance = 0.3 m
- max_distance = 1.1 m

## 测试

### 运行测试

```bash
# 运行单元测试
colcon test --packages-select scene_auxiliary_test

# 运行系统测试
python3 test_system.py

# 运行性能测试
ros2 run scene_auxiliary_test performance_test
```

### 调试模式

```bash
# 启用调试日志
ros2 run scene_auxiliary_test scene_auxiliary_test_node \
    --ros-args --log-level DEBUG

# 查看话题信息
ros2 topic list
ros2 topic echo /scan/left
ros2 topic echo /nav/control
```

## 监控和运维

### 系统监控

```bash
# 检查系统状态
ros2 node list
ros2 topic list
ros2 service list

# 查看节点信息
ros2 node info /scene_auxiliary_test_node

# 监控话题频率
ros2 topic hz /scan/left
ros2 topic hz /nav/control
```

### 日志管理

```bash
# 查看系统日志
tail -f /var/log/scene_auxiliary_test.log

# 分析日志
ros2 run scene_auxiliary_test log_analyzer

# 清理日志
find /var/log -name "scene_auxiliary_test*.log" -mtime +7 -delete
```

## 故障排除

### 常见问题

1. **雷达数据未接收**

   - 检查话题名称和数据类型
   - 确认雷达设备正常工作
   - 检查网络连接

2. **聚类结果为空**

   - 调整聚类参数
   - 检查 ROI 设置
   - 确认点云数据质量

3. **控制命令无输出**

   - 确认回调函数连接正确
   - 检查摇杆控制输入
   - 验证决策器状态

4. **系统响应延迟**
   - 检查系统负载
   - 优化算法性能
   - 调整系统频率

### 调试工具

```bash
# 使用调试脚本
./scripts/debug_scene_auxiliary_test.sh

# 使用性能监控
./scripts/performance_monitor.sh

# 使用健康检查
./scripts/health_check.sh
```

## 文档结构

本项目包含以下详细文档：

- **[开发文档](DEVELOPMENT_DOCUMENTATION.md)** - 系统架构、组件设计、数据流等
- **[API 文档](API_DOCUMENTATION.md)** - 详细的 API 接口说明
- **[配置指南](CONFIGURATION_GUIDE.md)** - 参数配置、性能调优等
- **[测试指南](TESTING_GUIDE.md)** - 单元测试、集成测试、性能测试等
- **[部署指南](DEPLOYMENT_GUIDE.md)** - 环境准备、编译安装、系统部署等

## 版本信息

- **版本**: 0.1.0
- **日期**: 2025-10-16
- **作者**: likang
- **公司**: Freego Technology Co., Ltd.

## 许可证

Copyright (c) 2025 Freego Technology Co., Ltd.

## 联系方式

- 作者: likang
- 邮箱: m15137328778@163.com
- 公司: Freego Technology Co., Ltd.

## 贡献指南

欢迎提交 Issue 和 Pull Request 来改进项目。在提交代码前，请确保：

1. 代码符合项目规范
2. 通过所有测试
3. 更新相关文档
4. 添加适当的注释

## 更新日志

### v0.1.0 (2025-10-16)

- 初始版本发布
- 基本雷达数据处理功能
- 简单聚类算法
- 基础决策逻辑
- 车辆接口支持
- 目标跟踪功能
- 平滑滤波支持

## 致谢

感谢所有为项目做出贡献的开发者和测试人员。
