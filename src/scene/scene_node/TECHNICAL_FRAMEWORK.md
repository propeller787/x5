# scene_node 技术框架文档

## 1. 节点定位

`scene_node` 是一个 ROS2 场景安全辅助节点包，核心目标是把多路雷达感知、车辆输入、轮速反馈、环境识别和安全决策串成实时闭环，最终输出经过安全干预后的 `/nav/control` 控制命令。

它不是单一逻辑类，而是一个由主控节点编排的多节点组件系统：

- `SceneNode`：系统主控，负责组件创建、回调连接、模式服务、状态持久化和最终控制发布。
- `LidarDriver`：订阅多路雷达数据，执行 `base_link` 坐标转换、ROI/Z 过滤和调试点云发布。
- `Clustering`：对过滤后的点云做欧式聚类，生成障碍物簇特征。
- `DecisionMaker`：融合障碍物簇、车辆状态、轮速反馈、环境 profile，计算前/后/侧向限速和报警。
- `VehicleInterface`：订阅手柄控制和轮速反馈，维护车辆状态并向上游回调。
- `EnvironmentDetector`：根据水平/倾斜雷达开放度和侧向墙体对称性识别室内/室外，触发决策参数预设切换。
- `Tracking`：阶段 A 中已瘦身为轮速反馈缓存，保留历史命名以减少重命名影响。
- `ObstaclePredictor`：基于轮速反馈对缓存障碍物簇做运动补偿。
- `EvasionDetector`：根据实际转向/前进反馈判断避让意图，放松非贴脸场景下的前向限速。
- `SmoothFilter` / `SpeedReductionKalmanFilter`：用于输出控制命令平滑。
- `WallDetector`、`CenterLineCalculator`、`PathTracker`、`VirtualGuideRailController`：虚拟导轨/过门居中能力，当前通过 `SCENE_NODE_BUILD_GUIDE_RAIL` 选项控制是否编译。

## 2. 构建与运行形态

包名：`scene_node`

可执行文件：`scene_node_node`

主要构建产物：

- 库目标 `${PROJECT_NAME}`：包含雷达、聚类、决策、轮速缓存、滤波、环境检测、预测和避让等核心实现。
- 可执行目标 `scene_node_node`：由 `src/main.cpp` 和 `src/scene_node.cpp` 组成，并链接 `scene_node` 库。

主要依赖：

- ROS2：`rclcpp`、`geometry_msgs`、`sensor_msgs`、`std_msgs`、`nav_msgs`
- TF：`tf2`、`tf2_ros`、`tf2_geometry_msgs`
- 点云：`PCL`、`pcl_conversions`、`laser_geometry`
- 项目内部：`common`、`interface`
- 第三方：`Eigen3`、`utils`、`muduo`

运行模型：

```text
main()
  -> rclcpp::init()
  -> create SceneNode
  -> register SIGINT/SIGTERM
  -> MultiThreadedExecutor
       add_node(SceneNode)
       add_node(VehicleInterface)
       add_node(LidarDriver)
       add_node(DecisionMaker)
       add_node(Clustering)
       add_node(EnvironmentDetector)
  -> executor.spin()
```

多节点被加入同一个 `MultiThreadedExecutor`，因此雷达、手柄、轮速、决策、环境识别等回调可能并发执行。代码中主要通过 `std::mutex`、`std::atomic` 和模块内部缓存隔离共享状态。

## 3. 总体架构

```text
                             +------------------+
                             |    SceneNode     |
                             | 编排/服务/发布    |
                             +----+--------+----+
                                  |        |
              +-------------------+        +-------------------+
              |                                        |
              v                                        v
       +-------------+                          +------------------+
       | LidarDriver |                          | VehicleInterface |
       | 雷达/TF/ROI  |                          | 手柄/轮速反馈     |
       +------+------+                          +--------+---------+
              |                                          |
              v                                          |
       +-------------+                                   |
       | Clustering  |                                   |
       | 点云聚类     |                                   |
       +------+------+                                   |
              |                                          |
              v                                          v
       +-------------+       inject/cache        +------------------+
       | DecisionMaker|<-------------------------| Tracking          |
       | 限速/报警/平滑|<-------------------------| ObstaclePredictor |
       +------+------+                          | EvasionDetector   |
              ^                                 +------------------+
              |
       +------+------------+
       | EnvironmentDetector|
       | 室内/室外 profile  |
       +-------------------+
              |
              v
          /nav/control
```

核心设计方式：

- 回调驱动：组件之间通过 `std::function` 解耦，`SceneNode` 负责接线。
- 依赖注入：`Tracking`、`ObstaclePredictor`、`EvasionDetector` 由 `SceneNode` 创建后注入 `DecisionMaker`。
- 周期决策：`DecisionMaker` 缓存期望控制量，由 `decision_rate_hz` 定时执行避障决策，避免手柄回调频率直接驱动复杂决策。
- Profile 切换：`EnvironmentDetector` 只判断环境，参数应用由 `DecisionMaker::applyProfile()` 完成。
- 分方向干预：前向、侧向、后向分别计算距离/TTC/限速，再合成最终 `Twist`。

## 4. 核心数据流

### 4.1 雷达到控制输出

```text
/scan/left, /scan/right, /scan/back
/perception/left_tilt/pointcloud, /perception/right_tilt/pointcloud
        |
        v
LidarDriver
  - LaserScan/PointCloud2 -> PCL 点云
  - TF 转到 base_link
  - ROI / Z 阈值过滤
  - 可选发布 base_link 调试点云
        |
        v
SceneNode::onPointCloudReceived()
        |
        v
Clustering::clusterPointCloud()
  - 欧式聚类
  - 计算中心、包围盒、尺寸、点数、来源
  - 可选发布聚类点云
        |
        v
SceneNode::onClustersDetected()
  - DecisionMaker::processClusters()
  - ObstaclePredictor::updateRadarClusters()
        |
        v
DecisionMaker::runDecisionTick()
  - 读取期望手柄控制
  - 融合原始簇/预测簇
  - 计算前/后/左/右最小安全距离
  - 距离阈值 + TTC 分层判断
  - 应用方向性限速、避让衰减和平滑滤波
        |
        v
SceneNode::onControlCommandGenerated()
        |
        v
/nav/control
```

### 4.2 车辆输入与轮速反馈

```text
/handle/control
        |
        v
VehicleInterface::handleControlCallback()
        |
        v
SceneNode::onVehicleStateChanged()
        |
        +--> DecisionMaker::updateVehicleState()
        +--> DecisionMaker::updateDesiredTwist()
```

```text
/wheel/twist
        |
        v
VehicleInterface::wheelSpeedCallback()
        |
        v
SceneNode wheel speed callback
        |
        +--> Tracking::updateWheelSpeed()
        +--> ObstaclePredictor::onWheelSpeedUpdate()
```

轮速反馈有两个用途：

- 给 `Tracking` 缓存实际线速度和角速度，供 `DecisionMaker` 计算接近速度、避让状态和低速清零条件。
- 给 `ObstaclePredictor` 做高频运动补偿，降低雷达低频刷新造成的障碍物位置滞后。

### 4.3 环境识别流

```text
/scan/left, /scan/right, /scan/left_tilt, /scan/right_tilt
        |
        v
EnvironmentDetector
  - 水平雷达开放度
  - 倾斜雷达开放度/天花板特征
  - DecisionMaker 回传的左右侧距离对称性
        |
        v
滞回状态机
        |
        v
DecisionMaker::applyProfile(INDOOR / OUTDOOR)
```

`EnvironmentDetector` 默认从室外状态开始。它使用 30 帧滑动窗口，室内确认时间为 6 秒，室外确认时间为 12 秒，体现“更快进入保守配置、更慢退出保守配置”的安全策略。

## 5. ROS 接口

### 5.1 订阅

| 话题 | 消息类型 | 使用方 | 作用 |
| --- | --- | --- | --- |
| `/scan/left` | `sensor_msgs/msg/LaserScan` | `LidarDriver`, `EnvironmentDetector` | 左侧水平雷达 |
| `/scan/right` | `sensor_msgs/msg/LaserScan` | `LidarDriver`, `EnvironmentDetector` | 右侧水平雷达 |
| `/scan/back` | `sensor_msgs/msg/LaserScan` | `LidarDriver` | 后向雷达 |
| `/perception/left_tilt/pointcloud` | `sensor_msgs/msg/PointCloud2` | `LidarDriver` | 左倾斜雷达点云 |
| `/perception/right_tilt/pointcloud` | `sensor_msgs/msg/PointCloud2` | `LidarDriver` | 右倾斜雷达点云 |
| `/scan/left_tilt` | `sensor_msgs/msg/LaserScan` | `EnvironmentDetector` | 左倾斜雷达开放度检测 |
| `/scan/right_tilt` | `sensor_msgs/msg/LaserScan` | `EnvironmentDetector` | 右倾斜雷达开放度检测 |
| `/handle/control` | `geometry_msgs/msg/Twist` | `VehicleInterface` | 手柄/人工控制输入 |
| `/wheel/twist` | `geometry_msgs/msg/TwistStamped` | `VehicleInterface` | 底盘轮速反馈 |
| `/vlago/car/info` | `interface/msg/CarInfo` | `SceneNode`, `DecisionMaker` | 档位、驾驶模式、抱闸等车辆状态 |
| `/scene_services/mode` | `std_msgs/msg/String` | `SceneNode` | 与 scene services 同步当前模式 |
| `/sensor/joystick` | `interface/msg/Joystick` | `DecisionMaker` | 摇杆原始位置，用于制动死区等判断 |
| `/vlago/tft/car/mode` | `std_msgs/msg/Int32` | `DecisionMaker` | TFT 舒适/运动模式，影响平滑限幅 |

### 5.2 发布

| 话题 | 消息类型 | 发布方 | 作用 |
| --- | --- | --- | --- |
| `/nav/control` | `geometry_msgs/msg/Twist` | `SceneNode` | 最终安全干预后的控制命令 |
| `/scene_auxiliary/status` | `std_msgs/msg/Bool` | `SceneNode` | 安全辅助功能开关状态，1 秒周期发布 |
| `/vlago/cruise/status` | `std_msgs/msg/Int32` | `SceneNode` | 巡航过程状态/结束原因 |
| `/vlago/auxiliary/alarm` | `interface/msg/AuxiliaryAlarm` | `DecisionMaker` | 前向/侧向安全报警 |
| `/vlago/backcar/alarm` | `interface/msg/AuxiliaryAlarm` | `DecisionMaker` | 后向安全报警 |
| `/vlago/collision/warning` | `interface/msg/CollisionWarning` | `DecisionMaker` | 碰撞预警结构化消息 |
| `/scene_auxiliary/*/base_link` | `sensor_msgs/msg/PointCloud2` | `LidarDriver` | 各雷达转换到 `base_link` 的调试点云 |
| `/scene_auxiliary/*/clusters` | `sensor_msgs/msg/PointCloud2` | `Clustering` | 各雷达聚类调试点云 |

### 5.3 服务

| 服务 | 类型 | 提供方 | 作用 |
| --- | --- | --- | --- |
| `/scene/auxiliary` | `interface/srv/SceneControls` | `SceneNode` | 开关安全辅助功能 |
| `/scene/standard` | `interface/srv/SceneControls` | `SceneNode` | 开关标准模式 |
| `/scene/cruise` | `interface/srv/SceneControls` | `SceneNode` | 开关巡航模式 |
| `/manage/cruise` | `interface/srv/SceneControls` | 外部服务，`SceneNode` 为客户端 | 巡航异常时请求外部管理服务关闭巡航 |

## 6. 模块职责

### 6.1 SceneNode

`SceneNode` 是全系统编排层，职责包括：

- 创建所有组件并检查创建结果。
- 建立 `LidarDriver -> Clustering -> DecisionMaker -> SceneNode` 的主数据链路。
- 将手柄控制、轮速反馈、车辆信息、场景服务模式接入决策。
- 管理安全、标准、巡航等模式开关。
- 周期发布 `/scene_auxiliary/status`。
- 在巡航模式中处理无前进速度超时、障碍导致退出等状态。
- 对 `DecisionMaker` 输出做最后发布到 `/nav/control`。

### 6.2 LidarDriver

输入包括左右水平雷达、后向雷达、左右倾斜点云。主要处理步骤：

- `LaserScan` 转 PCL 点云，或 `PointCloud2` 转 PCL 点云。
- 通过 TF 转换到 `base_link`。
- 根据 ROI 区域过滤前、后、左、右相关点云。
- 对倾斜雷达执行 Z 阈值过滤。
- 在 `debug_publish` 开启时发布各路 `base_link` 调试点云。

### 6.3 Clustering

`Clustering` 使用 PCL 欧式聚类提取障碍物簇。输出 `ClusterInfo`，包含：

- 聚类点云和点索引。
- 中心点、包围盒、宽高深。
- 点数、零 Z 点数、来源雷达。

聚类中心不是简单几何中心，而是结合车体位置选择与车体最近的代表点，用于后续距离判断。

### 6.4 DecisionMaker

`DecisionMaker` 是安全策略核心。它维护每路雷达簇缓存、车辆状态、期望控制、方向状态、平滑滤波器和参数集。

关键能力：

- 分方向计算最小安全距离：前向、后向、左侧、右侧。
- 距离阈值与 TTC 联合判断：预警区、减速区、制动区。
- 前向线速度限制、后向倒车速度限制、侧向角速度限制。
- 雷达超时和历史状态保持，降低感知闪烁造成的输出抖动。
- 使用轮速缓存辅助实际接近速度、低速清零和避让衰减判断。
- 发布辅助报警、后向报警和结构化碰撞预警。
- 支持 `ros2 param set` 热更新，并通过 `applyProfile()` 切换室内/室外预设。

### 6.5 VehicleInterface

`VehicleInterface` 聚合车辆输入：

- `/handle/control`：写入当前期望线速度/角速度，并触发状态变化回调。
- `/wheel/twist`：写入轮速反馈，并触发轮速回调。

状态访问通过互斥锁保护，保证在多线程 executor 下可安全读取。

### 6.6 EnvironmentDetector

`EnvironmentDetector` 融合三类信号：

- 水平扫描开放度：室内墙体多时开放度低，室外开阔时开放度高。
- 倾斜雷达开放度：室内天花板导致开放度低，室外无顶导致开放度高。
- 双侧墙体对称性：两侧距离均较近且对称时更像室内走廊。

融合得分通过滞回状态机确认后才触发环境切换，避免频繁抖动。

### 6.7 ObstaclePredictor

`ObstaclePredictor` 用轮速反馈对雷达簇做增量运动补偿。假设障碍物在世界坐标中静止，车辆以 `(v, omega)` 运动 `dt` 后，障碍物在车体系中的坐标应按反向车体运动更新。

关键保护：

- `max_prediction_age` 限制单帧雷达数据可补偿的最大时间。
- `max_distance_drift_ratio` 限制补偿后距离漂移。
- `min_valid_x` 丢弃已明显落到无效后方的目标。

### 6.8 EvasionDetector

`EvasionDetector` 用轮速反馈判断操作者是否正在转向避让。满足明显转向、仍在前进、且障碍物不在固定停车距离内时，会输出 `[min_factor, 1.0]` 的衰减因子，将前向限速向 `max_speed_limit` 放松。

它是无状态计算模块，不缓存历史，适合在决策线程中直接调用。

## 7. 参数体系

主要配置文件：

- `config/decision_maker_params.yaml`：完整决策参数配置，包含注释、约束和热更新说明。
- `config/outdoor_default.yaml`：阶段 A 室外现场默认 profile，默认关闭复杂补偿能力，建立稳定基线。

关键参数类别：

| 类别 | 代表参数 | 作用 |
| --- | --- | --- |
| 周期 | `decision_rate_hz`, `radar_frequency`, `radar_timeout` | 决策频率、雷达预期频率、数据超时 |
| 车体几何 | `robot_length`, `robot_width`, `robot_safety_margin_*` | 计算碰撞区域和最小距离 |
| 前向距离 | `warning_zone`, `deceleration_zone`, `emergency_brake_distance`, `fixed_stopping_distance_forward` | 前向预警、减速、制动、硬停车 |
| 前向 TTC | `warning_ttc`, `deceleration_ttc`, `emergency_brake_ttc` | 按接近速度自适应干预 |
| 侧向 | `warning_zone_lateral`, `deceleration_zone_lateral`, `emergency_brake_distance_lateral`, `lateral_dominance_ratio` | 侧向角速度干预 |
| 后向 | `warning_zone_backward`, `deceleration_zone_backward`, `emergency_brake_distance_backward` | 倒车限速和报警 |
| 输出速度 | `min_speed_limit`, `max_speed_limit`, `absolute_max_linear_speed`, `absolute_max_angular_speed` | 输出限幅 |
| 状态平滑 | `state_hold_time`, `speed_recovery_time`, `smooth_*` | 保持、恢复和平滑 |
| 功能开关 | `enable_forward_collision`, `enable_lateral_collision`, `enable_backward_collision` | 分方向启停 |
| 预测/避让 | `enable_obstacle_prediction`, `enable_evasion_detection`, `evasion_*` | 轮速补偿和避让衰减 |

参数约束应满足：

- `emergency_brake_distance < deceleration_zone < warning_zone`
- `emergency_brake_distance_lateral < deceleration_zone_lateral < warning_zone_lateral`
- `emergency_brake_distance_backward < deceleration_zone_backward < warning_zone_backward`
- `fixed_stopping_distance_forward < emergency_brake_distance`
- `min_speed_limit < max_speed_limit <= absolute_max_linear_speed`
- 各方向 `deceleration_ttc > emergency_brake_ttc`
- `state_hold_time > 0`、`speed_recovery_time > 0`、`radar_timeout > 0`

## 8. 决策策略摘要

### 8.1 前向

前向干预只在前进控制意图下生效。决策依据包括：

- 最近前向障碍物距离。
- 实际接近速度。
- TTC 是否进入预警/减速/制动层。
- 固定停车距离兜底。
- 巡航模式、摇杆死区、车速低速清零等状态。
- 避让检测是否允许放松前向限速。

输出是对 `linear.x` 的上限约束，必要时将前进速度置零。

### 8.2 侧向

侧向干预主要约束 `angular.z`。当左转或右转方向上存在近距离侧向障碍，或斜向几何被判定为侧向主导时，按距离和 TTC 限制对应转向角速度。

### 8.3 后向

后向干预只在倒车控制意图下生效。策略包括：

- 后向距离和 TTC 分层限速。
- 固定停车距离兜底。
- 后向报警区间发布。
- 倒车急停确认窗口、爬行速度和恢复斜率，避免倒车末端突兀。

## 9. 并发与状态边界

- `MultiThreadedExecutor` 允许多个节点回调并发执行。
- `VehicleInterface` 使用 `state_mutex_` 保护车辆状态。
- `EnvironmentDetector` 使用 `mutex_` 保护滑动窗口和状态机，并在锁外执行切换回调。
- `ObstaclePredictor` 使用 `mutex_` 保护每路雷达缓存。
- `DecisionMaker` 内部维护多个方向状态、参数、雷达缓存和输出缓存，涉及热更新与周期决策时需要保持锁粒度清晰。
- `SceneNode` 的模式开关使用互斥锁和原子变量组合，避免服务回调、车辆回调和控制发布之间状态不一致。

## 10. 扩展点与注意事项

- 新增雷达源时，需要同步扩展 `common/utils/topic.h`、`LidarDriver` 订阅与发布、`Clustering` 发布器、`DecisionMaker` 的来源融合逻辑。
- 修改车体尺寸或安全边界时，优先通过 `decision_maker_params.yaml` 调整，并检查前/侧/后距离计算是否仍符合 `base_link` 约定。
- 开启 `enable_obstacle_prediction` 前，应先确认轮速时间戳、雷达频率和 `obstacle_prediction_max_age` 配置合理。
- 开启 `enable_evasion_detection` 前，应确认固定停车距离足够保守，因为该模块会在非贴脸场景放松前向限速。
- 虚拟导轨相关代码当前可通过 `-DSCENE_NODE_BUILD_GUIDE_RAIL=ON` 编译，但默认不进入核心构建。
- 现有 `Tracking` 名称与职责不完全一致，当前实际是轮速缓存；后续若大改接口，可考虑重命名为 `WheelSpeedCache`。

## 11. 测试覆盖

当前 CMake 中注册了：

- `test_decision_maker`：覆盖决策器核心业务。
- `test_smooth_filter`：覆盖平滑滤波器。

建议补充的测试方向：

- `ObstaclePredictor` 的运动补偿边界和超时丢弃。
- `EnvironmentDetector` 的滞回切换逻辑。
- `SceneNode` 模式切换与巡航关闭路径。
- 分方向参数校验失败时的启动/热更新行为。
