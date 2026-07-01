# lidar_imu_fusion 技术文档

> 包路径：`src/perception/lidar_imu_fusion`
> 维护者：likang (m15137328778@163.com)
> 版本：0.1.0
> 适用平台：ROS 2 Humble (ament_cmake)，C++17
> 文档与代码同步状态：反映 2026-06-30 修复后的实现

## 1. 概述

`lidar_imu_fusion` 是一个独立的 ROS 2 节点，把左、右、后三个 2D 激光雷达（10 Hz）的 `sensor_msgs/msg/LaserScan` 数据同步、去畸变（deskew）后融合为一帧统一坐标系的 `LaserScan`。去畸变使用 100 Hz IMU 角速度对每条激光束做运动补偿。

设计原则：
- **只做旋转去畸变**：节点不消费轮速里程计，故不做平移补偿，避免引入未经核对的平移假设。
- **可独立运行**：不依赖其他感知模块，输入/输出均为标准消息。
- **TF 优先 + 静态外参兜底**：TF 不可用时退化到 YAML 中的静态外参。默认值与 YAML 对齐，裸跑与 launch 行为一致。
- **单传感器容错**：任一雷达 TF 失败只跳过该传感器，其余两个继续融合；只有 IMU 覆盖不足（且 `require_imu_for_publish=true`）或三雷达全失败才丢整帧。
- **刚性安装假设**：lidar/IMU 的 TF 查询使用最新可用变换（`TimePointZero`），依赖传感器刚性安装在 `target_frame` 上。

## 2. 包结构

```
lidar_imu_fusion/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── lidar_imu_fusion.yaml         # 节点参数与静态外参兜底
├── launch/
│   └── lidar_imu_fusion.launch.py    # 单节点启动文件
├── include/lidar_imu_fusion/
│   └── lidar_imu_fusion_node.hpp     # 公共类/结构/枚举声明
├── src/
│   ├── lidar_imu_fusion_node.cpp     # 节点与同步器/IMU 缓冲实现
│   └── main.cpp                      # 节点入口
├── test/
│   └── test_lidar_imu_fusion.cpp     # gtest 单元测试
└── docs/
    └── TECHNICAL_NOTES.md            # 本文档
```

## 3. 接口

### 3.1 订阅

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/scan/left`  | `sensor_msgs/msg/LaserScan` | 10 Hz | 左侧 2D 雷达 |
| `/scan/right` | `sensor_msgs/msg/LaserScan` | 10 Hz | 右侧 2D 雷达 |
| `/scan/back`  | `sensor_msgs/msg/LaserScan` | 10 Hz | 后侧 2D 雷达 |
| `/sensor/imu/data` | `sensor_msgs/msg/Imu` | 100 Hz | 提供角速度（rad/s）用于去畸变 |

所有订阅使用 `rclcpp::SensorDataQoS()`。

### 3.2 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/perception/lidar_fusion/scan` | `sensor_msgs/msg/LaserScan` | 融合后统一坐标系扫描帧，frame_id = `target_frame`（默认 `base_link`） |

### 3.3 TF 依赖

- `target_frame ← laser_frame_*`：每个雷达到融合坐标系的位姿变换。
- `target_frame ← imu_link`：将 IMU 角速度旋到融合坐标系。
- TF 不可用时，回退到 `config/lidar_imu_fusion.yaml` 中的静态外参（`*_lidar.x/y/z/roll/pitch/yaw` 与 `imu.*`）。
- **IMU→target 旋转带缓存**：约每 100 条 IMU（~1 s）刷新一次 TF 查询结果，TF 瞬时失败时沿用缓存，避免 100 Hz 高频查询开销。

## 4. 数据流与处理流程

```
  /scan/left ──┐
  /scan/right ─┤── ScanSynchronizer ──> TimedScanSet ──> pending_scan_sets_
  /scan/back ──┘     (tryPop)                                │
                                                            │
  /sensor/imu/data ──> ImuBuffer ──┐                         │
                       (角速度,       │                         │
                        target frame)│                         ▼
                                  ┌─┴──────┐    hasCoverage    ┌──────────────────┐
                                  │ 10ms   │ ─── gate ───────▶ │ drainPendingScans│
                                  │ timer  │                   │ (1 group/call)   │
                                  │ + IMU  │                   └────────┬─────────┘
                                  │ cb     │                            ▼
                                  └────────┘                   processSynchronizedScans
                                                                       │
                                                  appendDeskewedScanPoints (per lidar)
                                                  │  kSensorUnavailable → 跳过该雷达
                                                  │  kImuCoverageLost    → 丢整组
                                                  │  kOk                 → 累加点
                                                  ▼
                                            buildOutputScan (栅格化为 LaserScan)
                                                  │
                                                  ▼
                                  /perception/lidar_fusion/scan
```

详细步骤（与源码对应）：

1. **接收与同步入组**：`handleScan` 把每帧 scan 入队 `ScanSynchronizer`，调用 `tryPop()`。若得到匹配三元组，调用 `enqueueSynchronizedScans` 压入 `pending_scan_sets_` 队列（上限 `max_pending_scan_sets_`，默认 5，超出丢最旧并告警）。`enqueueSynchronizedScans` **不**主动触发处理——新入组不改变 IMU 覆盖状态。
2. **IMU 缓存**：`handleImu` 把 IMU 角速度（经 `imuAngularVelocityInTarget` 旋到 `target_frame`）按时间有序插入 `ImuBuffer`，超出 `imu_queue_duration_sec` 的旧样本被裁剪。随后调用 `drainPendingScans` 推进处理。
3. **触发 draining**：`drainPendingScans` 由两路驱动——IMU 回调（覆盖状态变化时）与 10 ms wall timer（兜底）。**每次调用最多处理 1 组**，避免单线程执行器被长时间占用。
4. **IMU 覆盖门控**（`imuCoversScans`）：仅当 `enable_imu_deskew_ && require_imu_for_publish_` 时生效。检查 `ImuBuffer::hasCoverage(earliestScanStartTime, fusionReferenceTime)`；不满足则该组留在队首等待，不弹出。`require_imu_for_publish_=false`（默认）时门控直接放行。
5. **融合参考时间**：`fusionReferenceTime` 取三个 scan 的 `scanEndTime` 中最晚者，作为把所有激光束回旋到的统一时刻。`scanDuration` 对 `scan_time` 字段做上限裁剪（`max(default_scan_time_*2, 0.2)`），防止异常 `scan_time` 把参考时间拉到未来。
6. **逐传感器去畸变**（`appendDeskewedScanPoints`，返回 `AppendResult`）：
   - `lidarTransform` 失败 → 返回 `kSensorUnavailable`，`processSynchronizedScans` 只跳过该雷达、保留其余。
   - 计算 beam 角度：`angle = angle_min + i * angle_increment`。
   - 在雷达坐标系生成点 `(range*cos a, range*sin a, 0)`，变换到 `target_frame`。
   - 该 beam 采样时刻：`point_time = scan_start + i * time_increment`。
   - `ImuBuffer::integrateRotation(point_time, fusion_time, &Δq)` 得到旋转增量，施加 `Δq⁻¹` 完成旋转去畸变。
   - IMU 覆盖不足时：`require_imu_for_publish_=true` → 返回 `kImuCoverageLost`（整组丢弃）；`=false` → 该 beam 保留但不去畸变，计入 `imu_miss_count`。
   - 三雷达全部 `kSensorUnavailable` → 丢整帧。
7. **栅格化输出**（`buildOutputScan`）：按 `output_angle_min/max/increment` 生成等距角度 bin，每个 bin 取最短距离，强度随之保存；范围外/角度外/无效点被丢弃。
8. **发布**：融合扫描的 `header.stamp = fusion_time`，`scan_time` 取三帧 `scanDuration` 的最大值，`angle_increment` 由 `outputAngleIncrement` 决定。

## 5. 关键模块详解

### 5.1 `ScanSynchronizer`

- 三条独立 `std::deque`，每条限制 `scan_queue_size`（默认 10）。
- `tryPop()` 流程：
  1. 三队列非空检查。
  2. **总是调用 `pruneStaleScans()`** 清理无法再匹配的旧帧（匹配成功与否都清）。
  3. O(L·R·B) 三重循环搜索 `max(早)-min(早)` 跨度最小且 ≤ `max_slop_ns_` 的三元组。
  4. 找到则**只删除匹配帧本身**（`erase(begin + best)`），保留更旧帧供后续匹配；未找到则返回 `nullopt`。
- `pruneStaleScans()`：以三队列最新时间为基准，删掉比该时间早 `max_slop_ns` 之外的旧帧（每队列至少保留 1 帧）。
- 仅使用 `header.stamp` 做同步判据，不依赖消息到达顺序。

### 5.2 `ImuBuffer`

- 线程安全（`std::mutex`）。
- `push` 按时间有序插入，裁剪超出 `retention_sec` 的旧样本。
- `angularVelocityAt(t)`：样本间线性插值；超出首尾时允许 `max_extrapolation_sec` 内使用边界值外推，超出返回 false。任意落在 `[first, last]` 区间内的目标必然插值成功（无内部空洞检测，跨空洞按线性插值）。
- `hasCoverage(from, to)`：仅检查端点 `angularVelocityAt(from)` 与 `angularVelocityAt(to)` 是否成功。由于内部点必然落在 `[first, last]` 内、插值必成功，端点检查等价于整区间覆盖检查。
- `integrateRotation(from, to, &Δq)`：
  - 复用成员 `knots_buffer_`（`mutable`，加锁访问），避免每 beam 堆分配。
  - 将 `[from, to]` 内所有 IMU 样本作为时间节点插入，得到中点法积分区间。
  - 每子区间用中点角速度 `ω_mid`，构造轴角 `ω_mid·dt` 转 `tf2::Quaternion` 累乘。
  - `to < from` 时最终取逆四元数。
  - 任意段 `angularVelocityAt` 失败则整体失败。
  - 等价于"常速度中点积分"，对小 dt 二阶精度，覆盖 100 Hz IMU + 10 Hz lidar 去畸变需求。

### 5.3 `ThreeLidarImuFusionNode`

- 构造时 `loadParameters()`：声明所有参数并做合法性校验（`output_angle_increment` 非 → 重置 π/720；角度区间；距离区间；scan_time），加载四个静态外参。
- **IMU basis 缓存**（`imuAngularVelocityInTarget`）：缓存 `tf2::Matrix3x3`，每 ~100 条 IMU 刷新一次 TF；TF 瞬时失败且缓存有效时沿用缓存不告警；缓存无效时回退 `static_imu_transform_.getBasis()` 并告警。
- `lidarTransform`：优先 `tf_buffer_->lookupTransform(target, source, TimePointZero)`（注释明确刚性安装假设）；失败且允许兜底时用 `static_lidar_transforms_[idx]`。
- `rangeIsValid`：拒绝 `NaN/Inf` 与 `range <= 0`（防止"无回波"被当有效点）；有效区间取 `output_range_*` 与 scan 自身 `range_*` 的交集。
- `scanStartTime`：`header.stamp=0` 时回退 `now()` 并 `WARN_ONCE` 提示检查驱动时钟。
- `outputAngleIncrement`：`output_angle_increment_` 有效（>0 且有限）时直接用；否则固定回退 `π/720`（~0.25°）。**不再从输入扫描推导**，避免三雷达分辨率不同时 bin 数被放大。
- 告警使用 `RCLCPP_WARN_THROTTLE`（3 s / 5 s）控制噪声。

### 5.4 `main.cpp`

`rclcpp::init → make_shared<ThreeLidarImuFusionNode> → spin → shutdown`。默认单线程执行器；IMU/scan/定时器回调串行处理。`drainPendingScans` 每次只处理 1 组，保证回调尽快让出执行器。

## 6. 参数详解

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `left_scan_topic` / `right_scan_topic` / `back_scan_topic` | `/scan/{left,right,back}` | 输入雷达话题 |
| `imu_topic` | `/sensor/imu/data` | IMU 话题 |
| `output_scan_topic` | `/perception/lidar_fusion/scan` | 输出话题 |
| `target_frame` | `base_link` | 融合输出坐标系 |
| `use_tf_transforms` | true | 是否使用 TF 做坐标变换 |
| `fallback_to_static_transforms` | true | TF 失败时是否使用 YAML 静态外参 |
| `enable_imu_deskew` | true | 是否启用 IMU 旋转去畸变 |
| `require_imu_for_publish` | false | true 时 IMU 覆盖不足则丢弃整组；false 时未覆盖 beam 保留但不去畸变 |
| `publish_empty_scan` | false | 融合后无有效点时是否仍发布空 scan |
| `output_angle_min` / `output_angle_max` | -π / π | 输出角度区间 |
| `output_angle_increment` | 0.004363323129985824 (~0.25°) | 输出角度分辨率；非正重置为 π/720 |
| `output_range_min` / `output_range_max` | 0.05 / 10.0 | 输出有效距离区间 |
| `default_scan_time` | 0.1 | scan_time 字段缺失时的兜底；同时作为 scan_time 上限裁剪基准 |
| `max_sync_slop_sec` | 0.03 | 三雷达时间戳最大允许跨度 |
| `scan_queue_size` | 10 | 每条雷达缓存队列长度 |
| `max_pending_scan_sets` | 5 | 已同步待处理组队列上限，超出丢最旧 |
| `imu_queue_duration_sec` | 2.0 | IMU 样本保留时长 |
| `max_imu_extrapolation_sec` | 0.03 | IMU 角速度外推上限 |
| `left_lidar/right_lidar/back_lidar/imu.{x,y,z,roll,pitch,yaw}` | 0 | 静态外参兜底（需标定后替换） |

> 头文件默认值与 `config/lidar_imu_fusion.yaml` 完全对齐，确保 `ros2 run` 与 `ros2 launch` 行为一致。

## 7. `AppendResult` 与容错语义

```44:55:include/lidar_imu_fusion/lidar_imu_fusion_node.hpp
// Result of appending one lidar's deskewed points to the fused cloud.
//   kOk                 - points appended (possibly with some beams un-deskewed)
//   kSensorUnavailable  - this lidar's transform could not be resolved; skip this
//                         sensor but keep the rest of the synchronized group.
//   kImuCoverageLost    - IMU coverage required by require_imu_for_publish_ was
//                         lost mid-scan; drop the whole synchronized group.
enum class AppendResult
{
  kOk = 0,
  kSensorUnavailable = 1,
  kImuCoverageLost = 2,
};
```

`processSynchronizedScans` 对三个雷达依次调用 `appendDeskewedScanPoints`：
- `kSensorUnavailable`：跳过该雷达，继续处理其余。
- `kImuCoverageLost`：立即返回，整组丢弃。
- `kOk`：累加该雷达的点。
- 三雷达全 `kSensorUnavailable`：丢整帧并告警。

## 8. 构建与运行

### 8.1 编译

```bash
cd <workspace>
colcon build --packages-select lidar_imu_fusion
source install/setup.bash
```

`CMakeLists.txt` 构建库 `lidar_imu_fusion_lib`、可执行 `three_lidar_imu_fusion_node` 与 gtest `test_lidar_imu_fusion`，安装 `config/` 与 `launch/` 到 share 目录。编译启用 `-Wall -Wextra -Wpedantic`。

### 8.2 启动

```bash
ros2 launch lidar_imu_fusion lidar_imu_fusion.launch.py
```

### 8.3 测试

```bash
colcon test --packages-select lidar_imu_fusion
```

含 gtest 单元测试与 ament lint（cppcheck/uncrustify/flake8/pep257/lint_cmake/xmllint）。`xmllint` 需联网拉取 ROS schema，离线/沙箱环境会因网络失败，与代码无关。

### 8.4 验证

```bash
ros2 topic echo /perception/lidar_fusion/scan
ros2 run tf2_ros tf2_echo base_link laser_frame_left
ros2 topic hz /sensor/imu/data
```

## 9. 数学说明

### 9.1 旋转去畸变模型

设某 beam 采样时刻 `t_i`，融合参考时刻 `t_f`。`target_frame` 在 `[t_i, t_f]` 内相对于惯性系旋转 `R(t_i→t_f)`。beam 在 `t_i` 时刻雷达系测得 `p_lidar`，经静态/TF 变换到 `t_i` 时刻 `target_frame`：

```
p_target(t_i) = T_lidar→target · p_lidar
```

要把点表达在"融合时刻的 `target_frame`"下，需施加反向旋转：

```
p_target(t_f) = R(t_i→t_f)^{-1} · p_target(t_i)
```

代码中 `delta_rotation = integrateRotation(point_time, fusion_time)` 即 `R(t_i→t_f)`，`deskew_transform(delta_rotation.inverse()) * target_point` 完成上述运算。该公式与"刚性安装下 `T_lidar→target` 为常量"的假设一致。

### 9.2 角速度积分

对子区间 `[t_a, t_b]` 取中点角速度 `ω_mid = ω(t_a + (t_b-t_a)/2)`：

```
Δq_{ab} = quat(axis = ω_mid/|ω_mid|, angle = |ω_mid|·(t_b-t_a))
```

整体 `Δq = Δq_{0,1} · Δq_{1,2} · … · Δq_{n-1,n}`（按时间左乘累加），最后归一化。`ω` 为 body-frame（即 target frame）角速度，body-frame 乘法语义与 `tf2::Quaternion` 一致。

### 9.3 输出栅格

输出 bin 数：

```
N = floor((angle_max - angle_min) / angle_increment) + 1
```

每个融合点 `p(x,y)`：

```
range = hypot(x, y)
angle = atan2(y, x)
idx   = round((angle - angle_min) / angle_increment)
```

`idx ∈ [0, N)` 且 `range ∈ [range_min, range_max]` 时，与该 bin 已存值比较，保留较近者（最近点覆盖）。`atan2` 返回 `(-π, π]`，与默认 `[-π, π]` 区间匹配；`-π` 与 `π` 物理同向但分属 bin 0 与 bin N-1，是 LaserScan 的标准表示。

## 10. 已知限制与改进方向

1. **仅旋转去畸变**：未集成轮速/里程计，平台平移较大时（高速行驶）仍有畸变残留。后续可接入 `nav_msgs/Odometry` 做 6-DoF 补偿。
2. **刚性安装假设**：`lidarTransform` 与 IMU basis 缓存均用 `TimePointZero`（最新变换）。若雷达/IMU 装在转动关节上，应改为按消息时间戳查询 TF（需时间感知 TF 源与非零超时）。代码注释已明确该前提。
3. **静态外参兜底默认全 0**：部署前必须根据真实车辆标定结果填写 `left_lidar/right_lidar/back_lidar/imu` 的 `x,y,z,roll,pitch,yaw`，否则 TF 缺失时三雷达会重叠在原点。
4. **单线程执行器**：默认 `spin` 为单线程。`drainPendingScans` 每次只处理 1 组以缓解阻塞；若仍需更高吞吐，可改 `MultiThreadedExecutor` + 回调组，但需重新审视 `scan_mutex_`/`pending_scan_mutex_`/`ImuBuffer::mutex_` 的锁粒度（当前锁序 `pending_mutex → imu_mutex` 一致，无死锁）。
5. **IMU 与 scan 时钟域**：假设共享同一时钟。`header.stamp=0` 时回退 `now()` 并 `WARN_ONCE`，应优先修复驱动时钟而非依赖回退。
6. **IMU 角速度单位**：标准 IMU `angular_velocity` 单位为 rad/s，代码直接使用。若驱动输出 deg/s 会错，需在驱动层确认。
7. **栅格化最近点策略**：仅保留最近点及其强度。若需多回波/第二强度聚合，可扩展 `buildOutputScan`。

## 11. 诊断告警速查

| 告警（节流） | 含义 | 处理 |
|------|------|------|
| `IMU header.stamp is 0`（once） | IMU 驱动未填时间戳，回退 `now()` | 检查 IMU 驱动时钟 |
| `LaserScan header.stamp is 0`（once） | 雷达驱动未填时间戳，回退 `now()` | 检查雷达驱动时钟 |
| `No transform available for <sensor> scan, skipping this sensor only` | 该雷达 TF/静态外参缺失 | 检查 TF 或填静态外参 |
| `All three lidar transforms unavailable, dropping synchronized group` | 三雷达 TF 全缺失 | 检查 TF 树/外参 |
| `IMU coverage is not enough for deskew, dropping synchronized group` | `require_imu_for_publish=true` 且 IMU 中段缺失 | 检查 IMU 频率/延迟，或设 `require_imu_for_publish=false` |
| `<sensor> scan: N/M valid points kept un-deskewed due to missing IMU coverage` | `require_imu_for_publish=false` 时部分 beam 未去畸变但仍保留 | 检查 IMU 覆盖区间 |
| `Dropping oldest synchronized lidar group because pending queue is full` | 待处理组超过 `max_pending_scan_sets` | 提高 IMU 频率或增大队列/降低雷达频率 |
| `TF lookup failed from <frame> to <frame>` | TF 查询失败 | 检查 TF 发布器/外参 |
| `Synchronized lidar scans produced no valid fused points` | 匹配成功但无有效点 | 检查雷达数据/范围配置 |

## 12. 引用源码位置

- 节点类、同步器、IMU 缓冲、`AppendResult` 声明：`include/lidar_imu_fusion/lidar_imu_fusion_node.hpp`
- 全部实现：`src/lidar_imu_fusion_node.cpp`
- 入口：`src/main.cpp`
- 单元测试：`test/test_lidar_imu_fusion.cpp`
- 默认参数：`config/lidar_imu_fusion.yaml`
- 启动：`launch/lidar_imu_fusion.launch.py`

## 13. 变更履历（相对初版）

| 变更 | 说明 |
|------|------|
| 单传感器容错 | 引入 `AppendResult`，TF 失败只跳过该雷达，不再丢整帧 |
| pending 队列 + IMU 覆盖门控 | 同步组先入 `pending_scan_sets_`，`require_imu_for_publish=true` 时等 IMU 覆盖再处理 |
| `drainPendingScans` 每次只处理 1 组 | 避免单线程执行器被长时间占用 |
| IMU basis 缓存 | TF 查询从 100 Hz 降至 ~1 Hz，瞬时失败沿用缓存 |
| `knots_buffer_` 复用 | `integrateRotation` 不再每 beam 堆分配 |
| `scanDuration` 上限裁剪 | 防止异常 `scan_time` 拖累 `fusionReferenceTime` |
| `tryPop` 总是清理陈旧帧 + 只删匹配帧 | 旧帧不堆积，更旧帧保留供后续匹配 |
| `rangeIsValid` 拒绝 `range<=0` | 防止"无回波"画成原点伪障碍 |
| `outputAngleIncrement` 固定回退 π/720 | 不再从输入推导，bin 数可预期 |
| stamp=0 `WARN_ONCE` | 暴露驱动时钟问题 |
| 默认值与 YAML 对齐 | `fallback=true, require_imu=false, increment=π/720`，裸跑不再"永不发布" |
