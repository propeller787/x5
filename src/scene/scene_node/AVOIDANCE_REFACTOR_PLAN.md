# 避障重构详细方案

## 1. 背景与目标

当前避障主链路以“雷达点云聚类 -> 最近簇距离 -> 按前/后/左/右方向计算 TTC -> 限速/刹停”为核心。这个模型在稀疏障碍场景中可用，但在人多、贴边、转弯、窄道等场景中会带来明显的操作感问题：

- 人群导致 cluster 频繁合并、分裂，最近簇不稳定。
- 最近障碍物不一定在车辆未来轨迹上，却会触发减速或刹停。
- 前进 + 转弯被拆成前向和侧向两个独立判断，无法准确表达圆弧轨迹。
- 状态保持、制动保持、TTC、速度恢复、报警发布混在 `DecisionMaker` 中，逻辑重且难解释。
- 每个决策周期反复合并/拷贝 `ClusterInfo`，性能和内存行为不稳定。

本次重构目标是推翻现有避障主模型，改为：

```text
手柄控制 Twist
  -> 预测未来车辆运动轨迹
  -> 计算车体 footprint 扫掠区域
  -> 判断障碍物是否进入未来扫掠区域
  -> 选择最接近手柄意图的安全 Twist
  -> 最后一层硬急停兜底
```

重构后的避障不再以“前方最近 cluster”作为主判断依据，而以“车辆未来真实占据空间是否会碰撞”为主。

## 2. 重构边界

### 2.1 纳入重构

- `DecisionMaker` 中的避障主流程：
  - `runDecisionTick`
  - `calculateMinDirectionalDistance`
  - `applySpeedForwardIntervention`
  - `applySpeedBackwardIntervention`
  - `applySpeedLeftIntervention`
  - `applySpeedRightIntervention`
  - `applyDirectionalInterventionInternal`
  - 各方向制动保持/恢复逻辑
- `risk_assessment.h/cpp`：当前基本为空壳，作为新避障核心入口重写。
- 避障参数结构：保留兼容字段，新增轨迹预测和安全过滤参数。
- 避障相关单元测试。

### 2.2 不纳入重构

- `LidarDriver` 的雷达订阅、TF、ROI 过滤接口。
- `Clustering` 的 ROS 接口和输出回调。
- `VehicleInterface` 的手柄和轮速接入。
- `EnvironmentDetector` 的室内/室外识别接口。
- 巡航、标准、安全模式服务入口。
- 虚拟导轨/过门居中模块。

## 3. 新架构

```text
SceneNode / VehicleInterface / LidarDriver / Clustering
        |
        v
DecisionMaker
  - ROS 参数
  - topic/service/timer
  - 新旧避障切换开关
  - 报警和控制发布
        |
        v
AvoidancePipeline
        |
        +--> ObstacleField
        |      ClusterInfo / 点云 -> 轻量障碍物场
        |
        +--> MotionRollout
        |      根据 Twist 采样候选轨迹
        |
        +--> SweptFootprintChecker
        |      车体 footprint 扫掠碰撞检测
        |
        +--> SafetyFilter
        |      选择最接近手柄意图的安全速度
        |
        +--> EmergencyBrake
               最短 TTC / 贴脸距离硬兜底
```

`DecisionMaker` 不再承载避障算法细节，只作为 ROS 编排层。

## 4. 模块拆分

### 4.1 ObstacleField

职责：

- 接收每路雷达的 `ClusterInfo`。
- 转换成轻量障碍物 primitive。
- 不依赖 cluster ID，不做复杂目标跟踪。
- 对人多场景避免因聚类合并/分裂导致决策抖动。

建议数据结构：

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

第一版仍可从 `ClusterInfo` 生成 obstacle，后续再升级为直接使用点云/局部栅格。

### 4.2 MotionRollout

职责：

- 根据当前手柄 `Twist` 和车辆反馈速度预测未来轨迹。
- 为安全过滤器生成候选控制序列。
- 只做短时局部预测，不承担导航规划。

候选速度策略：

- 保留原始手柄指令 `(v_cmd, w_cmd)`。
- 优先降低线速度：`1.0, 0.8, 0.6, 0.4, 0.2, 0.0`。
- 转弯时优先保持角速度，避免用户想绕人时被强行拉直。
- 原地旋转时采样角速度比例：`1.0, 0.75, 0.5, 0.25, 0.0`。

### 4.3 VehicleFootprint

职责：

- 定义车体 footprint。
- 支持安全膨胀。
- 支持前后/侧向不同安全余量。

第一版使用矩形 footprint：

```text
base_link 原点位于后轴中心附近

x_forward = robot_length - rear_offset
x_back    = rear_offset
y_half    = robot_width / 2
```

后续如果 URDF 或碰撞形状更复杂，可替换为多边形 footprint。

### 4.4 SweptFootprintChecker

职责：

- 将未来每个预测 pose 下的 footprint 与障碍物场做碰撞检测。
- 计算最早碰撞时间 `ttc`。
- 计算碰撞点、碰撞来源和最小距离。

检测方式：

- 将障碍物点从 `base_link` 当前坐标系变换到预测 pose 的车体局部坐标。
- 判断点/圆是否落入膨胀矩形。
- 对带 radius 的障碍物，footprint 边界额外膨胀 `obstacle.radius`。

### 4.5 SafetyFilter

职责：

- 在候选速度中选择“最接近手柄指令且安全”的速度。
- 输出连续的限速结果，而不是只靠硬阈值。
- 人贴边但不挡轨迹时不减速。
- 轨迹确实被挡时平滑降速。

评分建议：

```text
score = command_error_cost
      + collision_cost
      + clearance_cost
      + smoothness_cost
```

第一版可以简化为：

- 无碰撞候选中选择 `|v - v_cmd| + angular_weight * |w - w_cmd|` 最小的。
- 如果所有候选都碰撞，选择 TTC 最大的候选。
- 如果 TTC 小于急停阈值，交给 `EmergencyBrake` 置零。

### 4.6 EmergencyBrake

职责：

- 作为最后一层安全兜底。
- 不追求舒适，只保证碰撞风险极高时立即刹停。

触发条件：

- 最短 TTC 小于 `emergency_ttc`。
- 障碍物已经进入硬停车距离。
- 当前所有候选轨迹都不安全且最近碰撞时间过短。

## 5. 参数方案

新增参数建议：

```yaml
decision_maker:
  ros__parameters:
    avoidance_engine: "swept_footprint"   # legacy / shadow / swept_footprint
    avoidance_shadow_log: true

    rollout_horizon_s: 1.2
    rollout_dt_s: 0.1
    rollout_min_speed_ratio: 0.0
    rollout_speed_samples: 6
    rollout_angular_samples: 5

    footprint_front_margin: 0.10
    footprint_back_margin: 0.12
    footprint_side_margin: 0.05
    footprint_obstacle_radius_default: 0.08

    comfort_clearance: 0.30
    warning_ttc_s: 1.5
    limiting_ttc_s: 1.0
    emergency_ttc_s: 0.45
    hard_stop_distance_m: 0.12

    safety_filter_angular_weight: 0.30
    safety_filter_smoothness_weight: 0.15
    max_decel_m_s2: 1.2
    max_recover_accel_m_s2: 0.5
```

兼容原则：

- 旧参数不立即删除。
- 新引擎默认先以 `shadow` 方式运行。
- `legacy` 模式可一键回滚旧避障。

## 6. 实施阶段

### 阶段 0：准备与基线

目标：

- 梳理当前避障行为。
- 加入性能和行为日志。
- 不改变控制输出。

工作项：

- 记录旧逻辑输出的风险方向、限速值、刹停原因。
- 记录 `runDecisionTick()` 耗时。
- 准备典型 bag 或实车场景：
  - 单人横穿
  - 多人贴边
  - 人群中低速穿行
  - 前方障碍绕行
  - 原地旋转
  - 倒车避障

### 阶段 1：新链路影子模式

目标：

- 新避障算法跑通，但不接管控制。

工作项：

- 新增 `ObstacleField`。
- 新增 `MotionRollout`。
- 新增 `VehicleFootprint`。
- 新增 `SweptFootprintChecker`。
- 新增 `SafetyFilter`。
- `DecisionMaker` 每周期同时运行 legacy 和新链路。
- 日志输出新旧差异：
  - legacy 是否刹停
  - swept 是否认为碰撞
  - swept 最短 TTC
  - swept 推荐速度
  - 差异原因

验收：

- 新链路不影响 `/nav/control`。
- 新链路单周期耗时稳定。
- 人贴边但不挡轨迹时，新链路不触发刹停。

### 阶段 2：前向避障接管

目标：

- 用新模型替代前向 `cluster + TTC` 主逻辑。

工作项：

- 当前进 `linear.x > 0` 时，新链路输出接管线速度限制。
- 保留 legacy 前向急停作为隐藏兜底，观察一段时间。
- 保留报警发布接口，但报警来源改为新 `CollisionRisk`。

验收：

- 绕人时不再因前方最近 cluster 误刹。
- 正前方有人站定时仍可靠减速/刹停。
- 障碍移开后恢复时间可控。

### 阶段 3：转弯与侧向接管

目标：

- 用 footprint 扫掠替代当前 `abs(angular.z) * half_width` 的侧向近似。

工作项：

- 原地旋转检测四角扫掠。
- 小半径转弯检测内外侧边界。
- 取消左右侧向独立 TTC 主逻辑。

验收：

- 原地旋转不会刮到近距离侧边障碍。
- 转弯绕人时不会被不在弧线上的最近点压停。

### 阶段 4：后向接管

目标：

- 倒车避障也统一纳入轨迹扫掠。

工作项：

- 倒车轨迹使用同一 `MotionRollout`。
- 后向报警由碰撞点所在区域派生。
- 删除旧后向 brake hold 特殊逻辑。

验收：

- 倒车贴边不过度抖动。
- 后方近障碍保持硬兜底能力。

### 阶段 5：删除旧避障主逻辑

目标：

- 移除 legacy 避障主路径，保留必要兼容参数。

删除或废弃：

- `applySpeedForwardIntervention`
- `applySpeedBackwardIntervention`
- `applySpeedLeftIntervention`
- `applySpeedRightIntervention`
- `applyDirectionalInterventionInternal`
- 旧 `DirectionalState` 保持/恢复逻辑
- 前后向 brake hold 特殊分支
- 基于最近 cluster 的主 TTC 判断

保留：

- 硬急停兜底。
- 参数热更新。
- 报警发布接口。
- 室内/室外 profile。

## 7. 测试计划

### 7.1 单元测试

- 直线轨迹 footprint 碰撞。
- 圆弧轨迹 footprint 碰撞。
- 原地旋转 footprint 碰撞。
- 障碍物在轨迹外时不触发。
- 所有候选都碰撞时选择 TTC 最大候选。
- 急停阈值触发。
- 速度恢复斜率限制。

### 7.2 回放测试

每个 bag 输出：

- legacy 输出速度。
- swept 输出速度。
- 最短 TTC。
- 碰撞点坐标。
- 是否急停。
- tick 耗时。

### 7.3 实车验收

重点场景：

- 人贴车侧走过，车保持可控低速。
- 前方人横穿，车减速而非突兀急停。
- 人群中慢速穿行，不被侧边最近点长期压速。
- 操作者绕开前方人，新模型允许保留转向。
- 原地旋转时角点不刮碰。
- 倒车接近障碍可稳定刹停。

## 8. 性能目标

- `DecisionMaker` 控制 tick 平均耗时小于 3 ms。
- p95 耗时小于 8 ms。
- 避免每 tick 大量拷贝完整 `ClusterInfo::points`。
- 轨迹采样数量第一版控制在 20 条以内。
- 每条轨迹预测点数量控制在 12 到 15 个。
- 障碍物 primitive 数量超过上限时按距离裁剪，例如最多保留最近 128 个。

## 9. 风险与回滚

### 9.1 风险

- footprint 参数不准会导致过保守或漏判。
- 轨迹离散步长太大可能漏掉短时间碰撞。
- 候选速度过少会导致输出不够细腻。
- 人多场景 primitive 数量过多可能造成耗时上升。

### 9.2 缓解

- 第一阶段必须使用 shadow 模式。
- 新旧结果同时记录，禁止直接全量替换。
- 保留 `avoidance_engine=legacy` 回滚开关。
- 急停层独立于舒适层，避免舒适策略误放行。
- footprint 安全边界先保守，再根据实测逐步缩小。

## 10. 验收标准

重构完成后应满足：

- 避障主链路不再依赖“最近 cluster + 分方向 TTC”作为主判断。
- `DecisionMaker` 中避障算法逻辑显著减少，只保留编排、参数和发布。
- 人多场景误刹和拖拽感明显下降。
- 转弯绕障时能够保留操作者意图。
- 原地旋转和倒车场景统一由 footprint 扫掠检测覆盖。
- 每次限速/刹停都能解释为“哪条候选轨迹、哪个时间点、哪个障碍物产生风险”。
