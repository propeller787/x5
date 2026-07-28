# 车端部署指南

本文档描述 `localmap_occupymap` **v0.1.0** 在车端（Ubuntu + ROS2 Humble）的编译与运行，不含可视化与离线渲染。

## 环境要求

| 项 | 要求 |
|----|------|
| OS | Ubuntu 22.04 |
| ROS2 | Humble |
| 依赖包 | `nav2_msgs`, `map_msgs`, `tf2_ros`, `tf2_sensor_msgs` |

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav2-msgs \
  ros-humble-map-msgs \
  ros-humble-tf2-ros \
  ros-humble-tf2-sensor-msgs
```

## 工作区路径

将车端工作区部署为：

```text
/opt/chaire/localmap/          # 或自定义路径
├── src/localmap_occupymap/
├── install/
├── build/
└── log/
```

以下命令以 `/opt/chaire/localmap` 为例，请按实际路径替换。

## 编译

```bash
source /opt/ros/humble/setup.bash
cd /opt/chaire/localmap

colcon build \
  --packages-select localmap_occupymap \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

或使用仓库脚本：

```bash
./script/build-occupymap.sh
```

## 运行

确保上游话题已发布：

- `/odom/fused`（`nav_msgs/Odometry`）
- `/perception/obstacle_points`（`sensor_msgs/PointCloud2`）
- `/tf`（点云到 `odom` 变换，失败时节点回退用 odom 位姿）

启动节点：

```bash
source /opt/ros/humble/setup.bash
cd /opt/chaire/localmap
source install/setup.bash

ros2 launch localmap_occupymap occupymap.launch.py
```

或使用脚本：

```bash
./script/run-occupymap.sh
```

### 开机自启（systemd 示例）

```ini
# /etc/systemd/system/localmap-occupymap.service
[Unit]
Description=localmap occupymap node
After=network.target ros2.service

[Service]
Type=simple
User=nvidia
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source /opt/chaire/localmap/install/setup.bash && ros2 launch localmap_occupymap occupymap.launch.py'
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable localmap-occupymap
sudo systemctl start localmap-occupymap
sudo systemctl status localmap-occupymap
```

## 验收

```bash
ros2 topic list | grep localmap
ros2 topic hz /localmap/occupymap_raw
ros2 topic echo /localmap/occupymap_raw --once
```

预期：

- `/localmap/occupymap_raw` 频率与 `/perception/obstacle_points` 一致（约 10 Hz）
- `frame_id=odom`，`metadata.size_x=160`，`metadata.size_y=160`（8 m × 8 m @ 0.05 m）

## 参数

车端默认配置：`src/localmap_occupymap/config/occupymap.param.yaml`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_extent_*` | 4 m | odom 滚动窗口 8 m × 8 m |
| `observe_extent_forward/backward_m` | 4 m | 点云观测前后范围 |
| `observe_extent_left/right_m` | 3 m | 点云观测左右范围 |
| `inflation_radius_m` | 0.55 | Nav2 膨胀半径 |
| `inscribed_radius_m` | 0.35 | 内切半径 |
| `cost_scaling_factor` | 10.0 | Nav2 指数膨胀系数 |
| `track_unknown_space` | false | 未观测区域输出 free |

覆盖参数：

```bash
ros2 launch localmap_occupymap occupymap.launch.py \
  --params-file /path/to/custom.param.yaml
```

## 开发机 Docker（可选）

macOS 开发机请使用 Docker 编译/回放，见 [README.md](../README.md) 开发环境章节。
