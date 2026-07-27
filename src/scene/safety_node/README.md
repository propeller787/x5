# safety_node

**技术框架与当前实现状态**：[TECHNICAL_FRAMEWORK.md](./TECHNICAL_FRAMEWORK.md)（阶段基线：2026-07-08，`d086e80` P0–P2）

**输入话题清单**：[INPUT_TOPICS.md](./INPUT_TOPICS.md)

**整体测试计划**：[SAFETY_NODE_TEST_PLAN.md](./SAFETY_NODE_TEST_PLAN.md)

**测试执行报告**：[TEST_EXECUTION_REPORT.md](./TEST_EXECUTION_REPORT.md)

Footprint + Rolling LocalMap 混合避障**独立节点**，与 `scene_node` / `scene_node_node` **无任何代码或运行时依赖**。

**代码阅读**：见 [CODE_READING.md](./CODE_READING.md)

架构设计参考：`../scene_node/HYBRID_FOOTPRINT_LOCALMAP.md`（仅文档，不链接该包）。

## 当前阶段能力（摘要）

| 层级 | 能力 | 默认 |
|---|---|---|
| 基础 | 水平+斜雷达统一障碍场、Footprint 扫掠、LocalMap veto、MDB、紧急制动、脱困 | 开 |
| 模式 | 安全/标准/巡航服务与 scene_node 对齐（`/scene/auxiliary`、`/scene/standard`、`/scene/cruise`；自动退巡航回调 `/manage/cruise`）；安全开关启动默认关，并从 `/opt/freego/user_config.yaml` 的 `safety_warning_enabled` 恢复（无文件/字段则保持关；与 scene_node 缺省相反） | 安全关（可被 user_config 覆盖为开）、巡航关 |
| P0 | 近距指数速度律 + 停短脉冲补偿（`PulseCompensator`） | 开 |
| P1 | 人机共享控制连续 α 接管（`shared_control`） | 开 |
| P2 | 窄道走廊提取 + Bézier 居中导引（`NarrowPassagePlanner`） | 开 |
| 限速 | 斜雷达前向近障软限速：前向走廊（±30cm）探到障碍即钳前向速度上限 4.5km/h（`FrontTiltSpeedCapGate`，只压正向 v，含巡航） | 关 |

## 运行

```bash
colcon build --packages-select safety_node
source install/setup.bash

ros2 run safety_node safety_node_node --ros-args \
  --params-file install/safety_node/share/safety_node/config/safety_node_params.yaml
```

> 与 `scene_node_node` 二选一运行，勿同时发布 `/nav/control`。

## 模块

```text
SafetyNode
  ├── LidarPerception          自研雷达订阅 + TF + 体素 ObstacleField（水平+斜雷达统一）
  ├── RollingLocalMap          10 Hz 占据栅格 + 膨胀
  ├── AvoidanceController      Footprint 选速 → Map veto → EmergencyStop
  │     ├── PulseCompensator   P0 停短脉冲补偿
  │     ├── NarrowPassagePlanner  P2 窄道 Bézier 导引
  │     └── SharedControl α    P1 连续接管融合
  ├── ModeSupervisor           模式剖面 + 安全/巡航服务
  ├── VehicleIo                手柄 / 轮速 / CarInfo 订阅
  └── OutputShaper             斜率限制 + 换向快停
```

## 话题

| 话题 | 方向 |
|------|------|
| `/handle/control` | 订阅 |
| `/sensor/joystick` | 订阅 |
| `/wheel/twist` | 订阅 |
| `/vlago/car/info` | 订阅（巡航限速/抱闸退出） |
| `/scan/left`, `/scan/right`, `/scan/back` | 订阅 |
| `/perception/left_tilt/pointcloud`, `/perception/right_tilt/pointcloud` | 订阅（默认并入水平流） |
| `/nav/control` | 发布 |
| `/safety/local_map` | 发布（调试） |
| `/safety/debug/perception_points` | 发布（RViz2 过滤后融合点云） |
| `/safety/debug/perception_markers` | 发布（ROI、Footprint、障碍物、输出轨迹） |
| `/safety/intervention_log` | 发布 |

## 服务

| 服务 | 用途 |
|---|---|
| `/scene/auxiliary` | 安全（避障）开关（与 scene_node 同名） |
| `/scene/standard` | 标准模式开关（与 scene_node 同名） |
| `/scene/cruise` | 巡航（定速保持）开关（与 scene_node 同名） |
| `/manage/cruise`（客户端） | 巡航自动退出时通知 scene_services |

## 回滚

```yaml
local_map:
  enabled: false
map_constraint:
  enabled: false
```

## RViz2 Perception Debug

```bash
rviz2 -d "$(ros2 pkg prefix safety_node)/share/safety_node/config/safety_node_debug.rviz"
```

The fixed frame is `base_link`. The point cloud colors identify the source:
left scan cyan, right scan green, back scan orange, left tilt blue, and right
tilt magenta. Disable the debug stream with:

```yaml
debug:
  perception:
    enabled: false
```

即退化为纯 Footprint 主路径。

启动：
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run safety_node safety_node_node --ros-args \
  --params-file src/scene/safety_node/config/safety_node_params.yaml \
  -p local_map.enabled:=true

查看rviz2
source /opt/ros/humble/setup.bash
source install/setup.bash

rviz2 -d "$(ros2 pkg prefix safety_node)/share/safety_node/config/safety_node_debug.rviz"

录制
src/scene/safety_node/tools/record_safety_topics.sh \
  --stage T3 \
  --case-id SIM-002 \
  --vehicle car01