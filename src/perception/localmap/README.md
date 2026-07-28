# localmap

局部占据地图模块（`localmap_occupymap` v**0.1.0**）。根据里程计与障碍物点云在 `odom` 下维护滚动代价地图，发布与 Nav2 `local_costmap` 兼容的话题，供 MPPI 等规划模块接入。

详细架构见 [docs/architecture.md](docs/architecture.md)。  
车端部署见 [docs/deployment.md](docs/deployment.md)。

## 功能概览

| 方向 | 话题 | 类型 |
|------|------|------|
| 输入 | `/odom/fused` | `nav_msgs/Odometry` |
| 输入 | `/perception/obstacle_points` | `sensor_msgs/PointCloud2` |
| 输出 | `/localmap/occupymap` | `nav_msgs/OccupancyGrid` |
| 输出 | `/localmap/occupymap_raw` | `nav2_msgs/Costmap` |
| 输出 | `/localmap/occupymap_updates` | `map_msgs/OccupancyGridUpdate` |

**v0.1.0 行为摘要：**

- odom 滚动地图：**8 m × 8 m**（前后/左右各 4 m），栅格 **160 × 160**，分辨率 0.05 m
- 点云观测范围（base_link）：前后 4 m、左右 3 m
- 发布频率：随点云（约 10 Hz），无独立定时器
- 膨胀：Nav2 指数模型（`inflation_radius=0.55`, `inscribed_radius=0.35`, `cost_scaling_factor=10`）
- 未观测区域：`track_unknown_space=false`，输出 free

## 目录结构

```text
localmap/
├── VERSION
├── src/localmap_occupymap/     # ROS2 包
├── script/                     # 构建/运行脚本
├── docs/
│   ├── architecture.md
│   └── deployment.md           # 车端部署
└── config/                     # 开发用 RViz 配置（可选）
```

## 车端编译与运行

环境：Ubuntu 22.04 + ROS2 Humble。完整说明见 [docs/deployment.md](docs/deployment.md)。

```bash
# 编译
cd /opt/chaire/localmap
./script/build-occupymap.sh

# 运行
./script/run-occupymap.sh
```

手动命令：

```bash
source /opt/ros/humble/setup.bash
cd /opt/chaire/localmap
colcon build --packages-select localmap_occupymap --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch localmap_occupymap occupymap.launch.py
```

验收：

```bash
ros2 topic hz /localmap/occupymap_raw
ros2 topic echo /localmap/occupymap_raw --once
```

## 开发环境（Docker，可选）

macOS 开发机使用 Docker 容器 `autoware_chaire` 编译与 bag 回放。

```bash
# 宿主机
cd /path/to/chaire/localmap
./script/build-occupymap-docker.sh
./script/replay-occupymap-docker.sh
```

容器内路径：`/workspace/chaire/localmap`

> macOS 宿主机不可直接运行 ROS2；车端部署不使用 Docker。

## 配置

默认参数：`src/localmap_occupymap/config/occupymap.param.yaml`

```yaml
map_extent_forward/backward/left/right_m: 4.0   # odom 地图 8x8m
observe_extent_forward/backward_m: 4.0          # 观测前后 4m
observe_extent_left/right_m: 3.0                # 观测左右 3m
resolution_m: 0.05
map_frame: odom
inflation_radius_m: 0.55
inscribed_radius_m: 0.35
cost_scaling_factor: 10.0
track_unknown_space: false
```

## 版本

当前固定版本：**0.1.0**（见 [VERSION](VERSION)）。

变更要点：

- Nav2 兼容 raw costmap 与增量更新
- 分离地图窗口与点云观测范围
- 点云触发发布、指数膨胀、unknown 视为 free

离线 PNG 导出与并排对比脚本位于 `script/export-*`、`script/pair-*`，仅用于开发验证，**车端部署不需要**。
