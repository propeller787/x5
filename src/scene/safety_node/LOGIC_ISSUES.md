# safety_node 现有逻辑问题

## 2026-06-07 T1/T2/T3 完成状态

本文件保留问题首次发现时的原因、影响和修复建议。以下状态表为当前结论；
后续各问题段落中的“修复方向”属于历史审查记录。

### 已关闭

- P0-1 至 P0-6：ESTOP、急停旁路、最终命令复检、Map veto、输入超时和
  反馈速度 rollout。
- P1-1 至 P1-8：感知缓存、pending 缓存、空帧清图、聚合误急停、
  逐点 TF、frame_id、Footprint 内部占用和地图越界。
- P2-2：非平面 Twist 分量透传。
- P2-3：干预日志中的非法 JSON `inf`。
- P2-5：非 rebuild 地图无连续衰减。
- P2-8：完整测试因格式检查失败。
- P1-11：感知冻结帧、旧帧和乱序帧曾通过回调到达时间刷新 fresh 状态。
- P1-12：`TwistStamped` 轮速冻结时间戳和乱序帧曾通过回调到达时间刷新 fresh 状态。
- P1-13：NaN/Inf 轮速反馈曾未作为输入故障显式处理。
- P2-9：PointCloud2 曾只过滤 NaN，没有显式过滤 Inf。

### 部分关闭

- P2-1：`local_map.update_rate_hz` 已正式声明并可覆盖；模式阈值和模式化
  inflation 参数仍未全部接入运行对象。
- P2-6：零膨胀半径除零路径已修复，并增加主要边界测试；完整参数范围和
  跨参数一致性校验仍未实现。

### 仍开放

- P1-9：默认配置仍为 `pass_through` 且 LocalMap 默认关闭。
- P1-10：模式和巡航仍无运行时服务/话题入口。
- P2-4：ObstacleField primitive 仍未保留具体传感器来源。
- P2-7：源码仍通过相对路径引用 monorepo 的 `common/utils/topic.h`，
  独立包边界尚未完全收敛。

### 新发现并已关闭

- 从未成功 TF 的来源曾在健康检查中执行不同时钟类型相减并抛异常。
- 空 PointCloud2 调试帧曾构造 iterator，UBSan 检出空 vector `front()`。
- 负坐标体素键曾对负整数执行左移，UBSan 检出未定义行为。
- 零膨胀半径曾进入 `dist / radius` 除零路径。
- 感知健康度曾使用回调到达时间而非消息头时间戳，冻结/重复帧可被误判为新鲜。
- 轮速反馈曾使用回调到达时间而非 `TwistStamped.header.stamp`，冻结反馈可被误判为新鲜。
- 感知健康检查已增加 50 ms 跨节点时钟微小偏差容忍，避免正常抖动误报。

验证结果：核心行覆盖率 `95.21%`，分支覆盖率 `85.83%`；ASan/UBSan
11/11 通过；完整回归 `205 tests, 0 failures`；五路高频长稳 30 分钟通过；T3 SIL 与故障注入通过。
详见 `TEST_EXECUTION_REPORT.md`。

> 审查日期：2026-06-06  
> 范围：`safety_node` 当前全部源码、默认参数、构建脚本和现有测试。  
> 分级：P0 = 可能直接破坏停车/避障安全语义；P1 = 高概率造成错误决策、失效保护缺失或长期运行故障；P2 = 配置、可维护性、性能或可观测性问题。

## 结论

原审查发现的 P0 问题已全部关闭，T1、T2 和 T3 退出条件已达到。当前剩余问题
主要集中在生产默认模式、运行时模式入口、来源可追溯性、参数完整接线和
独立包边界，不应在进入实车前忽略。

## P0 问题

### P0-1：ESTOP 模式实际执行为限速透传

**位置**

- `src/mode_supervisor.cpp:33-39`
- `src/avoidance_controller.cpp:80-92`

**原因**

ESTOP 将 `footprint_selector_enabled` 设置为 `false`。控制器看到该标志后直接进入“透传模式”早退，只限制 `linear.x` 和 `angular.z`，没有检查 `emergency_stop_enabled`，也没有输出零速度。

**影响**

配置 `mode.default=estop` 时，非零手柄命令仍可发布到 `/nav/control`。模式名称与实际安全行为相反。

**修复方向**

在所有透传判断之前单独处理 ESTOP；ESTOP 应无条件生成全零 Twist，并携带明确的急停状态。

### P0-2：急停零速度会被 OutputShaper 延迟

**位置**

- `src/avoidance_controller.cpp:264-268`
- `src/safety_node.cpp:173-176`
- `src/output_shaper.cpp:21-35`

**原因**

控制器即使已经返回全零急停命令，节点仍无条件调用 `OutputShaper::shape()`。以 20 Hz 运行时，线速度每周期最多下降约 `0.06 m/s`，角速度每周期最多下降 `0.05 rad/s`。

**影响**

“急停”被变成渐进减速。例如上周期输出 `1.0 m/s` 时，可能需要约 0.8 秒以上才降到零。

**修复方向**

急停结果直接发布零速度并重置 shaper；安全减速与舒适性加速限制必须使用不同通道。

### P0-3：安全候选经过限幅后变成未检查轨迹

**位置**

- `src/avoidance_controller.cpp:208-249`
- `src/core/safety_filter.cpp:114-155`
- `src/output_shaper.cpp:21-35`

**原因**

碰撞检查针对候选速度执行，但候选选中后又根据上一周期输出限制减速度，随后节点再次执行 OutputShaper。最终发布速度可能高于已验证的安全候选，且没有对该实际速度重新展开轨迹和碰撞检查。

**典型触发**

上一周期为 `1.0 m/s`，当前只有 `0.2 m/s` 候选安全。减速度限制可能仍输出接近 `0.94 m/s`，而这条轨迹没有通过本周期检查。

**影响**

决策层判定安全的命令与实际发布命令不一致，可能继续驶入障碍物。

**修复方向**

安全方向的降速不应被舒适性限幅阻止；若必须限幅，应对限幅后的最终命令重新做完整碰撞检查。

### P0-4：地图否决全部候选后，降级逻辑绕过地图

**位置**

- `src/avoidance_controller.cpp:157-187`
- `src/avoidance_controller.cpp:191-205`

**原因**

当 Footprint 安全候选全部被 LocalMap veto 时，`selected_index` 保持 `-1`。随后调用 `SafetyFilter::selectSafeVelocity()`，该过滤器只检查 ObstacleField，不检查 LocalMap，因此可能重新选择刚被地图否决的命令。

**影响**

文档中的“地图硬约束”不是硬约束。地图用于保留已经离开当前点云的障碍物时，该问题可能直接恢复危险速度。

**修复方向**

区分“Footprint 全碰撞”和“Map 全 veto”。Map 全 veto 必须停车或只从同时满足地图约束的集合中降级，不能调用忽略地图的过滤器。

### P0-5：控制输入丢失后会永久保持最后命令

**位置**

- `src/vehicle_io.cpp:21-25`
- `src/vehicle_io.cpp:33-40`
- `src/safety_node.cpp:139-142`

**原因**

手柄 Twist 没有接收时间戳、超时阈值或有效标志。发布端退出、网络中断或话题停止后，`desired_` 会一直保留最后一个非零命令。

**影响**

上游控制失联不能触发停车，车辆可能持续运动。

**修复方向**

记录控制接收时间；超过短超时立即输出零或进入故障模式。轮速和传感器也应采用同类健康检查。

### P0-6：轨迹预测忽略真实车速，无法覆盖车辆惯性运动

**位置**

- `src/core/motion_rollout.cpp:38-41`
- `src/core/motion_rollout.cpp:122-131`
- `src/safety_node.cpp:161-170`

**原因**

候选采样函数接收 `fb_linear/fb_angular`，但两个参数在实现中被匿名并完全忽略。轨迹从期望候选速度立即开始，没有从当前实测速度过渡。

**典型触发**

车辆仍以较高速度运动时，驾驶员松开手柄令期望速度变为零。轨迹预测认为车辆立即静止，但 OutputShaper 又会继续输出非零速度。

**影响**

预测模型和实际输出动态不一致，制动阶段的 TTC 可能被明显高估。

**修复方向**

轨迹状态至少应从当前反馈速度起步，并使用与最终输出一致的加减速度模型进行 rollout。

## P1 问题

### P1-1：ObstacleField 缓存永不清理且无容量上限

**位置**

- `src/lidar_perception.cpp:99-107`
- `src/lidar_perception.cpp:198-204`

**原因**

每帧雷达点持续追加到 `obstacle_points_`，只有模式切换预留的 `clear()` 可以清空，但当前没有运行时模式切换入口。正常运行中没有帧替换、时间过期或容量限制。

**影响**

- 内存持续增长。
- 每个控制周期的体素构建成本持续增长。
- 已消失的障碍物永久保留，形成长期假障碍。
- 节点长期运行后可能错过控制周期甚至耗尽内存。

**修复方向**

使用带时间戳的有限窗口、每传感器最新帧或固定容量环形缓存，并在构建障碍物场时剔除过期数据。

### P1-2：LocalMap 关闭或模式禁用时 pending_points_ 持续增长

**位置**

- `src/safety_node.cpp:74-84`
- `src/safety_node.cpp:97-108`
- `src/lidar_perception.cpp:103-106`

**原因**

地图总开关关闭时不会创建地图定时器；PASS_THROUGH/ESTOP 模式下地图回调直接返回。两种情况下 `pending_points_` 都不会被取走，但雷达回调仍持续追加。

**影响**

默认配置正是 `local_map.enabled=false`，因此默认运行即存在无界内存增长。

**修复方向**

地图禁用时不写入地图缓存，或仍周期性丢弃 pending 数据；所有感知缓存必须有硬容量上限。

### P1-3：rebuild 模式在空帧时不会清除旧地图

**位置**

- `src/safety_node.cpp:106-110`
- `src/rolling_local_map.cpp:98-106`
- `test/test_rolling_local_map.cpp:49-52`

**原因**

`rebuild_each_frame=true` 的清图动作位于 `integratePoints()` 内，但节点只在 points 非空时调用该函数。雷达本周期没有有效点时，旧概率层不会被清除。现有测试显式调用 `integratePoints({})`，没有覆盖节点真实调用方式。

**影响**

障碍物消失或雷达短时无数据后，旧地图可能无限保留。

**修复方向**

每个地图周期都调用 `integratePoints(points, stamp)`，包括空集合；或将帧清理移动到明确的 `beginFrame()`。

### P1-4：急停风险取所有候选的最坏值，安全候选也可能被误停

**位置**

- `src/avoidance_controller.cpp:113-137`
- `src/avoidance_controller.cpp:207-213`
- `src/avoidance_controller.cpp:255-268`

**原因**

`min_ttc` 和 `min_clearance` 聚合了所有候选轨迹，包括已经淘汰的高速危险候选。选中低速安全轨迹后，最终急停仍根据全候选最坏值判断。

**影响**

只要候选集中存在一条近碰轨迹，即使最终选择完全安全，也可能触发急停，导致避障退化为频繁停车。

**修复方向**

区分场景风险、当前运动风险和最终候选风险。最终急停应基于当前实际运动及最终将发布的轨迹。

### P1-5：雷达逐点等待 TF，可能阻塞整个控制执行器

**位置**

- `src/lidar_perception.cpp:74-95`
- `src/lidar_perception.cpp:122-142`
- `src/lidar_perception.cpp:166-183`
- `src/main.cpp:17-19`

**原因**

每个点都调用一次带 `0.05 s` 超时的 TF transform。节点使用单线程 spin；TF 缺失时，大量点可能逐个等待，控制定时器也无法执行。

**影响**

雷达帧较大或 TF 异常时，20 Hz 控制环可能长时间停顿，`/nav/control` 输出失去实时性。

**修复方向**

每条消息只查询一次变换矩阵，再批量变换所有点；传感器处理与控制定时器使用独立 callback group/执行线程，并监控周期超时。

### P1-6：LaserScan 忽略消息 frame_id

**位置**

- `src/lidar_perception.cpp:35-50`
- `src/lidar_perception.cpp:109-118`

**原因**

三个水平雷达使用硬编码的 `laser_frame_left/right/back`，没有使用 `msg->header.frame_id`。

**影响**

设备驱动或 URDF 的 frame 名称不一致时，全部点可能转换失败或使用错误变换。

**修复方向**

优先使用消息 frame_id，仅在 frame_id 为空时使用可配置的后备名称。

### P1-7：地图只检查四角，可能漏掉 Footprint 内部占用

**位置**

- `src/map_constraint_check.cpp:83-92`

**原因**

`check_footprint_corners=true` 时只查询矩形四个角，没有查询边线和内部。障碍格位于车体中心区域、但四角均为空时，地图会判定轨迹安全。

**影响**

LocalMap 作为历史空间记忆时，可能漏掉已经不在 ObstacleField 中、但位于车体覆盖区域内的障碍物。

**修复方向**

对 footprint 覆盖栅格做区域查询，或使用距离场/多边形栅格化，而不是仅采样四角。

### P1-8：地图边界外按自由空间处理

**位置**

- `src/map_constraint_check.cpp:42-48`
- `src/rolling_local_map.cpp:208-216`

**原因**

轨迹或 footprint 采样点超出局部地图范围时返回 cost `0.0`。

**影响**

高速长预测轨迹接近地图边缘时，未知区域被视为安全，Map veto 失效。

**修复方向**

安全模式下将越界视为 unknown/lethal，或保证地图范围覆盖完整预测轨迹和 footprint 外扩。

### P1-9：默认配置启动时关闭核心安全功能

**位置**

- `config/safety_node_params.yaml:12-14`
- `config/safety_node_params.yaml:40-43`

**原因**

默认模式为 `pass_through`，LocalMap 默认关闭。

**影响**

按 README 命令直接加载默认参数启动时，节点名称虽为 safety_node，但不会执行 Footprint、地图或急停逻辑。

**修复方向**

生产默认值应 fail-safe；至少启动日志必须明确打印“安全功能关闭”，并要求显式授权才能进入透传。

### P1-10：模式和巡航功能没有运行时入口

**位置**

- `src/safety_node.cpp:38-45`
- `src/safety_node.cpp:145-148`
- `src/safety_node.cpp:214-220`
- `src/mode_supervisor.cpp:49-65`

**原因**

模式只在启动时设置。`setModeFromString()`、`resetPipelineState()`、`cruise_active_` 和 `cruise_linear_cap_` 没有被订阅、服务或参数回调连接。

**影响**

运行中无法进入 ESTOP、NARROW 或 CRUISE；设计中的模式切换清理逻辑从不执行。

**修复方向**

接入明确的模式服务/话题，使用互斥和状态转换校验，并在切换时调用 pipeline reset。

## P2 问题

### P2-1：多个配置参数实际不生效

**位置**

- `src/safety_params.cpp:67-89`
- `src/mode_supervisor.cpp:24-44`
- `src/safety_node.cpp:74-83`

**问题**

- `local_map.update_rate_hz` 未在参数加载器声明，通常保持硬编码 10 Hz。
- `local_map.lethal_threshold` 被模式中的硬编码 `0.65` 替代。
- `map_constraint.narrow.lethal_threshold` 已加载，但 NARROW 仍硬编码 `0.55`。
- `map_constraint.narrow.inflation_radius_m` 已加载但未应用。
- `ModeProfile::inflation_radius_m` 仅保存数值，没有更新 RollingLocalMap。

**影响**

修改 YAML 后运行行为不变，调参结果不可预测。

### P2-2：控制消息只处理两个轴，其他轴原样透传

**位置**

- `src/avoidance_controller.cpp:76-89`
- `src/avoidance_controller.cpp:208-211`

**原因**

结果以完整输入 Twist 初始化，只修改 `linear.x` 和 `angular.z`。

**影响**

若上游误写 `linear.y/z` 或 `angular.x/y`，这些分量不会被清零或限幅。

**修复方向**

明确节点只输出平面差速控制，并构造新的零初始化 Twist。

### P2-3：干预日志在包含 infinity 时不是合法 JSON

**位置**

- `src/safety_node.cpp:198-207`

**原因**

未发生 map veto 时 `map_ttc` 默认为无穷大，流输出通常生成 `inf`，JSON 标准不接受该字面量。

**影响**

下游 JSON 解析器可能拒绝急停日志。

**修复方向**

无有限值时输出 `null`，或改用结构化 ROS 消息。

### P2-4：雷达来源信息被丢弃

**位置**

- `src/lidar_perception.cpp:99-106`
- `src/core/obstacle_field.cpp:62-69`

**原因**

`source_name` 被显式忽略，所有 primitive 的 source 都固定为 `"lidar"`。

**影响**

无法在干预日志中定位是哪一路传感器触发碰撞。

### P2-5：非 rebuild 地图没有连续衰减

**位置**

- `src/rolling_local_map.cpp:80-95`
- `src/rolling_local_map.cpp:141-155`

**原因**

概率衰减只在同一格再次命中时计算；未再次命中的格子保持原概率，直到超过 `max_hit_age_s` 后直接清零。

**影响**

实际行为是“保持后突变清除”，与文档描述的持续时间衰减不一致。

### P2-6：参数缺少范围和一致性校验

**位置**

- `src/safety_params.cpp:10-91`
- `src/rolling_local_map.cpp:34-49`
- `src/rolling_local_map.cpp:157-195`

**问题**

未检查分辨率、地图尺寸、膨胀半径、阈值、预测时域和采样数量的合法范围。零或负分辨率、零膨胀半径等配置可导致除零、异常尺寸或 NaN。

### P2-7：包声明和“独立包”边界不完整

**位置**

- `src/safety_node.cpp:7`
- `src/vehicle_io.cpp:7`
- `src/lidar_perception.cpp:9`
- `package.xml:12-21`

**问题**

源码通过 `../../../common/utils/topic.h` 依赖固定的 monorepo 目录结构，而不是使用 `common` 安装导出的头文件接口。`CMakeLists.txt` 使用 PCL，但 `package.xml` 没有声明对应系统依赖。

**影响**

包复制到其他工作区或执行 rosdep 时，可移植性和依赖完整性不足。

### P2-8：完整测试因格式检查失败

**验证结果**

```text
colcon build --packages-select safety_node     -> 成功
功能 GTest                                    -> 5/5 通过
colcon test --packages-select safety_node      -> 失败
ament_uncrustify                               -> 35 个文件失败
```

功能测试通过不代表上述控制逻辑安全；当前测试没有覆盖任何 P0 场景。

## 建议新增的回归测试

| 优先级 | 用例 |
|---|---|
| P0 | ESTOP 输入任意非零 Twist，最终发布必须立即为全零 |
| P0 | 急停从高速触发，OutputShaper 不得产生非零残余输出 |
| P0 | 地图否决全部候选时，控制器不得调用忽略地图的候选 |
| P0 | 最终限幅命令必须仍通过轨迹碰撞检查 |
| P0 | 手柄超时、雷达超时、TF 超时触发明确失效策略 |
| P0 | 车辆有反馈速度但期望速度为零时，rollout 必须覆盖制动距离 |
| P1 | obstacle/pending 缓存长时间运行保持有界 |
| P1 | rebuild 模式空帧清除上一帧地图 |
| P1 | 安全候选存在时，其他危险候选不得触发最终误急停 |
| P1 | footprint 内部 occupied、四角 free 时仍判定 lethal |
| P1 | 地图越界策略符合配置 |
| P2 | 所有 YAML 参数修改后能反映到运行对象 |
