# safety_node 代码阅读指南

这份指南按当前实现给出阅读顺序。想快速理解模块，先沿数据流读；想改某个行为，再跳到对应小节。

## 1. 推荐阅读顺序

```text
src/main.cpp
  -> SafetyNode::attachComponents()
  -> SafetyNode::setupTimers()
  -> SafetyNode::onControlTick()
       -> VehicleIo::desiredControl()
       -> VehicleIo::wheelFeedback()
       -> LidarPerception::health()
       -> LidarPerception::buildObstacleField()
       -> AvoidanceController::evaluate()
       -> OutputShaper::shape()
       -> /nav/control

  -> SafetyNode::onLocalMapUpdate()
       -> LidarPerception::takePendingPoints()
       -> RollingLocalMap::integratePoints()
       -> RollingLocalMap::finalizeFrame()
```

先读类型定义：`include/safety_node/core/avoidance_types.h`。这里定义了模式、参数、候选速度、预测位姿、碰撞结果、风险和最终输出。

## 2. 文件入口

| 文件 | 先看什么 | 读完应该知道什么 |
|---|---|---|
| `src/safety_node.cpp` | `onControlTick()` | 控制循环如何串联输入、感知、地图、控制器和输出 |
| `src/vehicle_io.cpp` | `desiredControl()`、`wheelFeedback()` | 手柄和轮速如何判断 fresh，轮速字段如何解释 |
| `src/lidar_perception.cpp` | 构造函数、`appendBaseLinkPoints()`、`health()` | 哪些雷达真正参与、缓存如何分流、健康状态如何计算 |
| `src/core/motion_rollout.cpp` | `sampleCandidates()`、`rolloutAll()` | 手柄只生成候选目标，位置预测从轮速反馈起步 |
| `src/avoidance_controller.cpp` | `evaluate()` | Footprint、Map veto、recovery、SafetyFilter、EmergencyBrake 的顺序 |
| `src/output_shaper.cpp` | `shape()` | 急停保持、加速限制和换向归零 |
| `src/rolling_local_map.cpp` | `integratePoints()`、`finalizeFrame()` | LocalMap 如何落格、衰减、膨胀 |
| `src/mode_supervisor.cpp` | `profile()` | 各安全模式打开或关闭哪些子系统 |
| `src/safety_params.cpp` | `loadFromNode()` | YAML 参数如何映射到运行时结构 |

## 3. 控制循环要点

`SafetyNode::onControlTick()` 是主线：

1. 计算 `dt_s`，异常时用控制频率回退。
2. 读取 `ControlCommand` 和 `WheelFeedback`。
3. 检查 finite、fresh、轮速必需性和感知 fresh 源数量。
4. 输入故障直接生成零 Twist 和 `INPUT_FAULT`。
5. 正常时调用 `AvoidanceController::evaluate()`。
6. 输出经过 `OutputShaper`，并清零 `linear.y/z`、`angular.x/y`。
7. 更新 `prev_output_`，发布 `/nav/control` 和干预日志。

默认配置下，即使是 `pass_through`，手柄和轮速健康检查仍在 SafetyNode 层执行；感知检查只在模式启用 obstacle field 时执行。

## 4. 轮速反馈和轨迹预测

当前 `/wheel/twist` 是车体速度反馈，不是左右轮速：

- `twist.linear.x` -> 车体线速度，m/s
- `twist.angular.z` -> 车体角速度，rad/s

`VehicleIo::onWheel()` 优先使用 `header.stamp`。重复或倒序 stamp 不会覆盖最新反馈。超时后 `wheelFeedback()` 返回零速度，`fresh=false`。

`MotionRollout::sampleCandidates()` 仍然用手柄期望生成候选速度；`MotionRollout::rolloutAll()` 对每个候选都从轮速反馈起步：

```text
rolloutCandidate(candidate, fb_linear, fb_angular)
```

因此“车辆未来位置”不再由手柄速度瞬时决定。手柄速度只表达目标，真实运动起点来自轮速反馈。

## 5. 感知通道

水平三路 LaserScan 当前完整参与：

- `/scan/left`
- `/scan/right`
- `/scan/back`

处理步骤是：时间戳检查 -> TF 到 `base_link` -> range 过滤 -> 自车过滤 -> ROI 过滤 -> 写入缓存。

缓存分三类：

| 缓存 | 写入 | 读取 | 用途 |
|---|---|---|---|
| `latest_source_points_` | 每个来源最新点集 | `appendBaseLinkPoints()` 内部合并 | 避免单路更新时丢失其它来源 |
| `obstacle_points_` | 所有最新来源合并 | `buildObstacleField()` | 20/40 Hz Footprint 检查 |
| `pending_points_` | local map enabled 时累积 | `takePendingPoints()` | 10 Hz RollingLocalMap |
| `debug_points_` | debug enabled 时累积 | `takeDebugPoints()` | 5 Hz RViz 点云 |

左右倾斜 PointCloud2 当前订阅存在，但构造函数里 `processCloud()` 调用被注释掉。它们不会产生 obstacle、local map 点或 debug 点，也不会通过 transform fresh 检查。

## 6. AvoidanceController 阅读路径

读 `evaluate()` 时按这些分支看：

1. `ESTOP`：最先处理，直接输出零 Twist。
2. `!footprint_selector_enabled`：pass-through 分支，只做绝对限速。
3. `rollout_.rolloutAll(eval_cmd, fb_linear, fb_angular)`：生成全部候选轨迹。
4. `checker_.checkTrajectory()`：对每条轨迹做 Footprint 碰撞。
5. `std::sort(scored...)`：安全优先，再按手柄偏差排序。
6. `map_checker_.trajectoryLethal()`：LocalMap veto。
7. `analyzeFootprintIntrusion()` 和 `tryBuildRecoveryCommand()`：当前 footprint 内有障碍时尝试低速脱困。
8. `safety_filter_.selectSafeVelocity()`：无可行候选且未到急停条件时降级。
9. `applyCandidateSpeedUpHold()` 和周期加减速限制：避免刚清障就突然加速。
10. `final_rollout`：对最终输出命令重新做 Footprint 和 Map 检查。
11. `emergency_brake_.update()`：最终硬闸和保持逻辑。

特别注意：recovery 和最终复检也都从 `fb_linear/fb_angular` 起步预测。

## 7. LocalMap 阅读路径

`RollingLocalMap` 的当前默认是 `rebuild_each_frame=true`：

1. `integratePoints()` 每个地图周期先清空概率层。
2. 点落到以 `base_link` 为中心的 6m x 6m 栅格。
3. `hitCell()` 对命中格做饱和累加。
4. `finalizeFrame()` 处理衰减、过期清理和膨胀。
5. `MapConstraintCheck` 取 `max(probability, inflated)` 判断轨迹是否进入 lethal 区。

`local_map.integrate_primitives=false` 时，只写入雷达点；为 true 时还会把 ObstacleField primitive 中心作为弱观测写入地图。

## 8. 模式速查

| 模式 | 快速理解 |
|---|---|
| `safe_assist` | 标准 Footprint + LocalMap 混合避障 |
| `pass_through` | 控制器只限速，但 SafetyNode 输入健康检查仍有效 |
| `cruise` | 与 safe_assist 类似，可叠加巡航线速度 cap |
| `narrow` | 与 safe_assist 类似，Map lethal 阈值更敏感 |
| `estop` | 控制器优先输出零 Twist |

当前模式只在启动时由 `mode.default` 设置，包内没有运行时切换接口。

## 9. 测试入口

修改轨迹预测或控制器时，优先跑：

```bash
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then source install/setup.bash; fi

colcon build --symlink-install --packages-select safety_node
build/safety_node/test_motion_rollout
build/safety_node/test_avoidance_controller
```

修改输入/ROS wiring 时再跑：

```bash
build/safety_node/test_vehicle_io
build/safety_node/test_safety_node_io
```

截至 2026-06-08，`test_motion_rollout` 和 `test_avoidance_controller` 通过；完整 `colcon test --packages-select safety_node` 仍有 `SafetyNodeIoTest.FiveSensorSourcesAppearWithExpectedDebugColors` 失败，和当前倾斜点云处理未启用有关。
