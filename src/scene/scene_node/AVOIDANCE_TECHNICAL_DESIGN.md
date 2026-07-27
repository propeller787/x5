# 新避障架构技术设计

## 1. 设计目标

新避障架构采用“轨迹采样 + 车体 footprint 扫掠 + 安全过滤器”的模型，替代当前“聚类最近点 + 前后左右 TTC”的主模型。

目标：

- 人多场景下不被最近 cluster 绑架。
- 障碍物不在未来轨迹上时不触发强制刹停。
- 前进、倒车、转弯、原地旋转统一使用同一套碰撞检测。
- 控制输出连续、可解释、易调参。
- `DecisionMaker` 从算法大杂烩变成 ROS 编排层。

非目标：

- 不做全局路径规划。
- 不主动替用户绕路。
- 不依赖稳定目标 ID。
- 第一版不引入复杂人群预测。

## 2. 坐标与运动约定

坐标系：`base_link`

- `x > 0`：车体前方。
- `x < 0`：车体后方。
- `y > 0`：车体左侧。
- `y < 0`：车体右侧。
- `linear.x > 0`：前进。
- `linear.x < 0`：后退。
- `angular.z > 0`：左转。
- `angular.z < 0`：右转。

车体运动模型采用平面 unicycle 模型：

```text
v = linear.x
w = angular.z

若 |w| < eps:
  x(t) = v * t
  y(t) = 0
  yaw(t) = 0

若 |w| >= eps:
  R = v / w
  x(t) = R * sin(w * t)
  y(t) = R * (1 - cos(w * t))
  yaw(t) = w * t
```

原地旋转场景：

```text
v ~= 0, |w| > eps
  x(t) = 0
  y(t) = 0
  yaw(t) = w * t
```

## 3. 核心数据结构

### 3.1 控制输入

```cpp
struct MotionCommand {
    double linear_x = 0.0;
    double angular_z = 0.0;
};

struct VehicleFeedback {
    double linear_x = 0.0;
    double angular_z = 0.0;
    rclcpp::Time stamp;
};
```

`MotionCommand` 来自手柄或上层控制指令，表示用户意图。`VehicleFeedback` 来自轮速反馈，主要用于急停判断和恢复斜率限制。

### 3.2 预测轨迹

```cpp
struct PredictedPose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double t = 0.0;
};

struct CandidateTrajectory {
    MotionCommand command;
    std::vector<PredictedPose> poses;
};
```

### 3.3 车体 footprint

```cpp
struct FootprintBox {
    double front = 0.0;
    double back = 0.0;
    double half_width = 0.0;
};

struct FootprintMargins {
    double front = 0.0;
    double back = 0.0;
    double side = 0.0;
};
```

第一版使用膨胀矩形：

```text
inflated_front = front + margin.front + obstacle.radius
inflated_back = back + margin.back + obstacle.radius
inflated_half_width = half_width + margin.side + obstacle.radius
```

### 3.4 障碍物 primitive

```cpp
struct ObstaclePrimitive {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double radius = 0.08;
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    std::string source;
    rclcpp::Time stamp;
    bool valid = false;
};

struct ObstacleField {
    std::vector<ObstaclePrimitive> obstacles;
    rclcpp::Time stamp;
};
```

第一版从 `ClusterInfo` 生成：

- `x/y/z`：使用离车最近代表点或包围盒最近点。
- `radius`：根据包围盒尺寸估算，限制在 `[min_radius, max_radius]`。
- `source`：雷达来源。
- `stamp`：该雷达最后更新时间。

后续可从点云或占据栅格直接生成 `ObstacleField`。

### 3.5 碰撞风险

```cpp
enum class RiskLevel {
    CLEAR,
    WARNING,
    LIMITING,
    BRAKING
};

struct CollisionRisk {
    RiskLevel level = RiskLevel::CLEAR;
    double min_distance = std::numeric_limits<double>::infinity();
    double time_to_collision = std::numeric_limits<double>::infinity();
    double collision_x = 0.0;
    double collision_y = 0.0;
    std::string source;
    bool has_collision = false;
};

struct SafetyFilterResult {
    MotionCommand selected_command;
    CollisionRisk risk;
    bool emergency_brake = false;
};
```

## 4. 模块接口设计

### 4.1 ObstacleFieldBuilder

输入：

- `radar_cluster_map_`
- `radar_last_update_time_map_`
- 当前时间
- 雷达超时参数

输出：

- `ObstacleField`

接口：

```cpp
class ObstacleFieldBuilder {
public:
    ObstacleField buildFromClusters(
        const std::map<std::string, std::vector<ClusterInfo>>& radar_clusters,
        const std::map<std::string, rclcpp::Time>& radar_times,
        const rclcpp::Time& now,
        const ObstacleFieldParams& params) const;
};
```

处理规则：

- 过滤严重超时的雷达。
- 过滤 NaN/Inf 点。
- 对过多 primitive 按距离排序裁剪。
- 不使用 cluster ID，不跟踪目标。

### 4.2 MotionRollout

输入：

- 当前手柄指令 `MotionCommand`
- 参数 `horizon_s`、`dt_s`、采样数量

输出：

- 候选轨迹列表

接口：

```cpp
class MotionRollout {
public:
    std::vector<CandidateTrajectory> generate(
        const MotionCommand& command,
        const RolloutParams& params) const;
};
```

采样规则：

1. 原始指令必须作为第一个候选。
2. 前进/后退优先降低 `linear_x`，保持 `angular_z`。
3. 原地旋转优先降低 `angular_z`。
4. 不主动改变转向方向。
5. 不主动生成与手柄相反的速度。

示例：

```text
输入 v=1.0, w=0.4
候选:
  (1.0, 0.4)
  (0.8, 0.4)
  (0.6, 0.4)
  (0.4, 0.4)
  (0.2, 0.4)
  (0.0, 0.4)
  (0.0, 0.2)
  (0.0, 0.0)
```

### 4.3 SweptFootprintChecker

输入：

- 候选轨迹
- footprint
- obstacle field

输出：

- 每条轨迹对应的 `CollisionRisk`

接口：

```cpp
class SweptFootprintChecker {
public:
    CollisionRisk evaluate(
        const CandidateTrajectory& trajectory,
        const ObstacleField& field,
        const FootprintBox& footprint,
        const FootprintMargins& margins,
        const RiskParams& params) const;
};
```

核心检测：

```text
对每个 predicted pose:
  对每个 obstacle:
    将 obstacle 从当前 base_link 坐标变换到该 pose 的车体局部坐标
    判断是否落入膨胀 footprint box
    若进入:
       记录首次碰撞时间 t
       记录碰撞点和来源
```

坐标变换：

```text
dx = obs.x - pose.x
dy = obs.y - pose.y

local_x =  cos(yaw) * dx + sin(yaw) * dy
local_y = -sin(yaw) * dx + cos(yaw) * dy
```

矩形碰撞判断：

```text
-back <= local_x <= front
abs(local_y) <= half_width
```

### 4.4 SafetyFilter

输入：

- 原始手柄指令
- 候选轨迹及其风险

输出：

- 最终选择的安全指令

接口：

```cpp
class SafetyFilter {
public:
    SafetyFilterResult select(
        const MotionCommand& desired,
        const std::vector<CandidateTrajectory>& trajectories,
        const std::vector<CollisionRisk>& risks,
        const SafetyFilterParams& params) const;
};
```

选择规则第一版：

1. 若原始指令无碰撞，直接通过。
2. 否则从无碰撞候选中选择与原始指令最接近的。
3. 若所有候选都有碰撞，选择 TTC 最大的候选。
4. 若最佳候选 TTC 仍小于急停阈值，输出急停。

距离原始指令的代价：

```text
command_error =
    abs(v_candidate - v_desired)
  + angular_weight * abs(w_candidate - w_desired)
```

### 4.5 EmergencyBrake

输入：

- `SafetyFilterResult`
- 当前反馈速度
- 硬停车参数

输出：

- 是否强制置零

规则：

- `time_to_collision <= emergency_ttc_s`。
- 障碍物已进入 `hard_stop_distance_m` 对应的硬停车区。
- 所有候选都碰撞，且最佳 TTC 低于安全阈值。

急停输出：

```text
linear.x = 0
angular.z = 0
```

急停后恢复由 `ControlLimiter` 的加速度限制处理。

### 4.6 ControlLimiter

输入：

- 原始 `geometry_msgs::msg::Twist`
- `SafetyFilterResult`
- 上周期输出
- 平滑参数

输出：

- 最终 `geometry_msgs::msg::Twist`

职责：

- 保留非避障相关轴向字段。
- 应用最大减速度。
- 应用恢复加速度上限。
- 应用角速度变化率限制。
- 急停时快速衰减。

第一版可以复用现有 `SmoothFilter`，但入口应从 `DecisionMaker` 移到 `ControlLimiter`。

## 5. 风险等级设计

风险等级由轨迹碰撞结果计算：

```text
无碰撞:
  CLEAR

有碰撞且 TTC > warning_ttc:
  WARNING

limiting_ttc < TTC <= warning_ttc:
  LIMITING

TTC <= emergency_ttc 或 hard_stop_distance 内:
  BRAKING
```

建议默认：

```text
warning_ttc_s = 1.5
limiting_ttc_s = 1.0
emergency_ttc_s = 0.45
```

人多低速场景中，`SafetyFilter` 会优先选择低一点但仍可动的速度，而不是只要风险存在就刹停。

## 6. 人多场景处理策略

当前 cluster + TTC 模型在人多场景不好用的原因是：它关心“谁最近”，而不是“谁挡路”。

新模型处理方式：

- 不依赖 cluster 稳定 ID。
- 不以最近 cluster 作为主决策。
- 多个行人靠边时，如果不进入未来 footprint，不限制速度。
- 行人横穿轨迹时，由最早碰撞时间触发降速。
- 人群中慢行时，选择仍安全的较低速度，而不是长时间急停。

后续增强：

- 使用局部 occupancy grid 替代 cluster primitive。
- 使用 distance field 快速评估轨迹 clearance。
- 对动态行人引入短时速度估计，但不作为第一版必需项。

## 7. 与现有代码的集成方式

### 7.1 DecisionMaker 新职责

保留：

- ROS 参数声明和热更新。
- 订阅手柄、TFT、CarInfo、Joystick。
- 接收聚类数据缓存。
- 定时触发避障 pipeline。
- 发布 `/nav/control`。
- 发布报警消息。

移除：

- 直接计算前/后/左/右距离。
- 直接维护复杂方向状态机。
- 直接在限速函数中发布报警。
- 直接在多个函数中反复修改 `Twist`。

### 7.2 影子模式

参数：

```yaml
avoidance_engine: "shadow"
```

行为：

- legacy 逻辑仍输出控制。
- swept footprint 逻辑只计算，不接管。
- 每周期或节流日志输出差异。

日志字段：

```text
desired_v, desired_w
legacy_v, legacy_w
swept_v, swept_w
swept_level
swept_ttc
swept_collision_x
swept_collision_y
swept_source
tick_cost_ms
```

### 7.3 接管模式

参数：

```yaml
avoidance_engine: "swept_footprint"
```

行为：

- 新 pipeline 输出 `/nav/control`。
- legacy 逻辑不再参与主输出。
- 可保留 legacy 急停兜底一个阶段。

### 7.4 回滚模式

参数：

```yaml
avoidance_engine: "legacy"
```

行为：

- 完全使用旧避障。
- 新 pipeline 不运行或只低频诊断。

## 8. 文件规划

建议新增：

```text
include/avoidance_types.h
include/obstacle_field.h
include/motion_rollout.h
include/vehicle_footprint.h
include/swept_footprint_checker.h
include/safety_filter.h
include/control_limiter.h

src/obstacle_field.cpp
src/motion_rollout.cpp
src/vehicle_footprint.cpp
src/swept_footprint_checker.cpp
src/safety_filter.cpp
src/control_limiter.cpp
```

建议重写：

```text
include/risk_assessment.h
src/risk_assessment.cpp
```

建议逐步删减：

```text
include/decision_maker.h
src/decision_maker.cpp
```

测试：

```text
test/test_motion_rollout.cpp
test/test_swept_footprint_checker.cpp
test/test_safety_filter.cpp
test/test_risk_assessment.cpp
```

## 9. 算法伪代码

```cpp
void DecisionMaker::runDecisionTick()
{
    auto desired = readDesiredCommand();
    auto feedback = readVehicleFeedback();
    auto clusters = copyRadarClusterCacheLightly();

    if (avoidance_engine == "legacy") {
        runLegacyDecisionTick();
        return;
    }

    ObstacleField field = obstacle_field_builder_.buildFromClusters(
        clusters, radar_times, now, obstacle_params);

    auto trajectories = motion_rollout_.generate(desired, rollout_params);

    std::vector<CollisionRisk> risks;
    risks.reserve(trajectories.size());
    for (const auto& traj : trajectories) {
        risks.push_back(swept_checker_.evaluate(
            traj, field, footprint, margins, risk_params));
    }

    SafetyFilterResult result = safety_filter_.select(
        desired, trajectories, risks, filter_params);

    if (emergency_brake_.shouldBrake(result, feedback, brake_params)) {
        result.emergency_brake = true;
        result.selected_command = MotionCommand{0.0, 0.0};
    }

    auto output = control_limiter_.apply(
        original_twist, result, limiter_params);

    if (avoidance_engine == "shadow") {
        auto legacy_output = runLegacyDecisionTickAndReturn();
        logShadowDiff(legacy_output, output, result);
        publish(legacy_output);
        return;
    }

    publish(output);
    publishAlarmFromRisk(result.risk);
}
```

## 10. 性能设计

控制周期目标：20 Hz。

推荐默认：

```text
horizon = 1.2 s
dt = 0.1 s
poses_per_trajectory = 13
trajectory_count <= 20
obstacle_count <= 128
```

最坏检测量：

```text
20 trajectories * 13 poses * 128 obstacles = 33280 checks/tick
```

矩形点碰撞判断计算量很低，C++ 中可接受。

优化措施：

- `std::vector` 预分配。
- 只保留近距离障碍物 primitive。
- 先做粗略距离过滤，再做 footprint 判断。
- 对远离轨迹包围盒的 obstacle 早跳过。
- debug 日志必须节流。

## 11. 参数调试建议

调试顺序：

1. 先确认 footprint 几何准确。
2. 再调 `hard_stop_distance_m` 和 `emergency_ttc_s`，保证安全。
3. 再调 `warning_ttc_s` 和 `limiting_ttc_s`，改善舒适性。
4. 最后调 `comfort_clearance` 和采样数量，改善人多场景操作感。

不要一开始就打开复杂预测或过大的安全边界，否则会重新变得保守。

## 12. 旧逻辑替换映射

| 旧逻辑 | 新逻辑 |
| --- | --- |
| `calculateMinDirectionalDistance` | `ObstacleFieldBuilder + SweptFootprintChecker` |
| `calculateSpeedLimit` | `SafetyFilter + RiskLevel` |
| `calculateSpeedLimitForBackward` | 同一套轨迹检测，候选 `v < 0` |
| `calculateSpeedLimitForLateral` | 原地旋转/圆弧 footprint 检测 |
| `DirectionalState` | `CollisionRisk` + 简化恢复状态 |
| `forward_brake_hold_` | `EmergencyBrake` + 控制恢复斜率 |
| `backward_brake_hold_` | `EmergencyBrake` + 控制恢复斜率 |
| 避障函数内发布报警 | `publishAlarmFromRisk` |

## 13. 安全兜底

新模型上线阶段保留三层兜底：

1. `legacy` 回滚开关。
2. `shadow` 对比模式。
3. `EmergencyBrake` 独立硬刹。

接管前向、侧向、后向时分阶段打开，不一次性替换全部方向。

## 14. 最终形态

最终避障主链路应变成：

```text
ClusterInfo / 点云
  -> ObstacleField

Twist
  -> MotionRollout
  -> CandidateTrajectory[]

ObstacleField + CandidateTrajectory[] + Footprint
  -> CollisionRisk[]
  -> SafetyFilterResult
  -> ControlLimiter
  -> /nav/control
```

最终 `DecisionMaker` 应只回答：

- 当前用哪个避障引擎？
- 当前参数是什么？
- 当前输入和缓存是什么？
- 该发布哪个控制和报警？

避障算法本身全部由纯 C++ 模块完成，便于单测、回放、调参和后续替换。
