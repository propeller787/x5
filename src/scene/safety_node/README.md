# safety_node

**整体测试计划**：[SAFETY_NODE_TEST_PLAN.md](./SAFETY_NODE_TEST_PLAN.md)

**测试执行报告**：[TEST_EXECUTION_REPORT.md](./TEST_EXECUTION_REPORT.md)

Footprint + Rolling LocalMap 混合避障**独立节点**，与 `scene_node` / `scene_node_node` **无任何代码或运行时依赖**。

**代码阅读**：见 [CODE_READING.md](./CODE_READING.md)

架构设计参考：`../scene_node/HYBRID_FOOTPRINT_LOCALMAP.md`（仅文档，不链接该包）。

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
  ├── LidarPerception     自研雷达订阅 + TF + 体素 ObstacleField
  ├── RollingLocalMap     10 Hz 占据栅格 + 膨胀
  ├── AvoidanceController Footprint 选速 → Map veto → EmergencyStop
  ├── ModeSupervisor      模式剖面
  ├── VehicleIo           手柄 / 轮速订阅
  └── OutputShaper        斜率限制
```

## 话题

| 话题 | 方向 |
|------|------|
| `/handle/control` | 订阅 |
| `/wheel/twist` | 订阅 |
| `/scan/left`, `/scan/right`, `/scan/back` | 订阅 |
| `/perception/left_tilt/pointcloud`, `/perception/right_tilt/pointcloud` | 订阅 |
| `/nav/control` | 发布 |
| `/safety/local_map` | 发布（调试） |
| `/safety/debug/perception_points` | 发布（RViz2 过滤后融合点云） |
| `/safety/debug/perception_markers` | 发布（ROI、Footprint、障碍物、输出轨迹） |
| `/safety/intervention_log` | 发布 |

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