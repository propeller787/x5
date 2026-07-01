# lidar_imu_fusion 测试流程与操作步骤

> 适用模块：`src/perception/lidar_imu_fusion`
> 测试目标：验证融合输出的**稳定性**与**准确性**，覆盖输入、TF、输出、去畸变、长时运行全链路
> 测试环境：ROS 2 Humble，已 `colcon build --packages-select lidar_imu_fusion`

---

## 0. 测试流程总览

```
阶段0 环境准备
   ↓
阶段1 输入侧验证  (三雷达 + IMU 频率/时间戳/完整性)
   ↓
阶段2 TF/外参验证  (坐标变换正确性 + 目检)
   ↓
阶段3 输出侧基础验证  (频率/延迟/数据完整性)
   ↓
阶段4 去畸变效果验证  (静止基准 + 运动对照)
   ↓
阶段5 准确性验证  (量距法 + SLAM 对比)
   ↓
阶段6 稳定性/长时验证  (内存/频率漂移/丢帧率)
   ↓
阶段7 自动化验证  (fusion_validator 工具)
```

每个阶段给出：**操作命令 → 预期结果 → 验收标准 → 异常处理**。

---

## 阶段 0：环境准备

### 0.1 编译与 source

```bash
cd /home/m1513/sou/freego_tft/freego_2025_tft
source /opt/ros/humble/setup.bash
colcon build --packages-select lidar_imu_fusion
source install/setup.bash
```

**预期**：`Finished <<< lidar_imu_fusion` 无错误。

### 0.2 确认依赖话题已发布

本模块需要 4 个输入话题。确认它们的发布节点（雷达驱动、IMU 驱动）已启动：

```bash
ros2 topic list | rg -E "scan/(left|right|back)|sensor/imu/data"
```

**预期**：列出 4 个话题。若缺失，先启动对应驱动节点。

### 0.3 确认 TF 树（或准备静态外参）

```bash
ros2 run tf2_ros tf2_echo base_link laser_frame_left
```

**预期**：能解析出变换。若报 `ExtrapolationException`，需启动 `robot_state_publisher` 或在 `config/lidar_imu_fusion.yaml` 填好静态外参（`fallback_to_static_transforms: true` + `left_lidar/right_lidar/back_lidar/imu` 的 `x,y,z,roll,pitch,yaw` 标定值）。

### 0.4 准备测试场地

- **静止测试**：空旷平整场地，机器人锁定不动。
- **运动测试**：场地有清晰参照物（墙面、柱子），可原地旋转。
- **量距测试**：准备卷尺，可在机器人前方已知距离放平板。
- 在正前方（0°）放一块静止平板障碍物，距离 1.5~2m（用于 `--watch-angle 0` 稳定性监控）。

---

## 阶段 1：输入侧验证

> 目标：确认四个输入源头健康。融合质量天花板由输入决定。

### 1.1 频率验证

打开 4 个终端（或用 `tmux` 分屏），每个话题单独测频率，**持续观察 ≥ 30s**：

```bash
# 终端1
ros2 topic hz /scan/left
# 终端2
ros2 topic hz /scan/right
# 终端3
ros2 topic hz /scan/back
# 终端4
ros2 topic hz /sensor/imu/data
```

**预期输出**（以 left 为例）：
```
average rate: 10.02
min: 0.099s max: 0.101s std dev: 0.00120s window: 300
```

**验收标准**：
- 三雷达：**10 Hz ±5%**（9.5~10.5 Hz）
- IMU：**100 Hz ±5%**（95~105 Hz）
- `std dev` 小（雷达 < 3ms，IMU < 0.5ms）
- `min`/`max` 接近 `average` 周期

**异常处理**：
| 现象 | 原因 | 处理 |
|------|------|------|
| 频率明显偏低/波动大 | 驱动/总线问题 | 检查雷达串口/网络配置、CPU 占用 |
| 周期性掉帧 | 调度抢占 | 提高驱动进程优先级（`chrt`/`nice`） |
| 完全无输出 | 驱动未启动或话题名错 | `ros2 topic list` 确认话题名 |

### 1.2 时间戳验证

```bash
ros2 topic echo /sensor/imu/data --field header.stamp --once
ros2 topic echo /scan/left --field header.stamp --once
ros2 topic echo /scan/right --field header.stamp --once
ros2 topic echo /scan/back --field header.stamp --once
```

**预期**：返回非零的 `sec`/`nanosec`，且随时间递增。

**验收标准**：
- `sec` + `nanosec` ≠ 0（若为 0，本模块会 `WARN_ONCE` 回退 `now()`，破坏同步精度，**必须修驱动**）
- 时间戳应略早于 `date +%s`（差值即传感器延迟，通常 < 50ms）
- 连续 `echo` 多次，`sec` 单调递增

**异常处理**：
- 时间戳为 0 → 检查驱动是否填充 `header.stamp`，确认驱动时钟源（PTP/系统时钟）。
- 时间戳跳变 → 多时钟源混用，统一到一个时钟。

### 1.3 雷达数据完整性

```bash
ros2 topic echo /scan/left --once
```

**关注字段**：
- `ranges` 数组长度符合预期（如 360/720/1080）
- `angle_min`/`angle_max`/`angle_increment` 合理（如 `-3.14`/`3.14`/`0.0087`）
- `range_min`/`range_max` 覆盖你关心的距离（如 `0.05`/`10.0`）
- `time_increment > 0`（本模块用它算每 beam 时刻，缺失会回退估算，精度下降）
- `scan_time > 0`（缺失回退 `default_scan_time`）

**验收**：三雷达的 `angle_increment` 接近一致最佳；不一致也能工作但输出分辨率用固定回退（π/720）。

---

## 阶段 2：TF / 外参验证

> 目标：坐标变换正确。错误的 TF 会让三雷达点云错位，去畸变再准也没用。

### 2.1 TF 链检查

```bash
ros2 run tf2_ros tf2_echo base_link laser_frame_left
ros2 run tf2_ros tf2_echo base_link laser_frame_right
ros2 run tf2_ros tf2_echo base_link laser_frame_back
ros2 run tf2_ros tf2_echo base_link imu_link
```

**预期**：四条都能解析，输出 `Translation: [x, y, z]` 与 `Rotation: [r, p, y, w]`。

**验收标准**：
- 无 `ExtrapolationException` / `LookupException`
- 平移值与车辆标定一致（米）
- 旋转值与安装方向一致（弧度）

### 2.2 TF 树全貌

```bash
ros2 run tf2_tools view_frames
# 生成 frames.pdf，查看完整 TF 树
```

**验收**：`base_link` 下挂载三个 `laser_frame_*` 与 `imu_link`，链路完整。

### 2.3 静态外参兜底验证（TF 不可用时）

若 TF 不可用，确认 YAML 已填标定外参：

```bash
cat install/lidar_imu_fusion/share/lidar_imu_fusion/config/lidar_imu_fusion.yaml | rg -A7 "left_lidar|right_lidar|back_lidar|imu:"
```

**验收**：
- `fallback_to_static_transforms: true`
- 各 `x,y,z,roll,pitch,yaw` 为**标定值**（非全 0）。全 0 会导致三雷达重叠在 `base_link` 原点。

### 2.4 TF 正确性目检（关键）

启动融合节点（见阶段 3.1），在 rviz 里看融合点云，做以下目检：

**测试 A：距离一致性**
- 机器人正前方 1m 放平板
- 三雷达应都打到该平板 → 融合输出中同一角度 bin 的距离应 ≈ 1m
- 若三雷达打到不同位置 → 外参 `roll/yaw` 错
- 若距离整体偏移 → 外参 `x/y` 错

**测试 B：方位映射**
- 在机器人左前方挥手，融合点云应在左前方位出现
- 在右后方挥手，应在右后方位出现
- 若左右/前后颠倒 → 外参 `yaw` 偏差 ±π 或符号错

---

## 阶段 3：输出侧基础验证

> 目标：融合节点运行后，输出话题的基本指标正常。

### 3.1 启动融合节点

```bash
# 终端A：启动融合节点（加载 YAML）
ros2 launch lidar_imu_fusion lidar_imu_fusion.launch.py
```

**预期日志**：
```
[INFO] [three_lidar_imu_fusion]: three_lidar_imu_fusion started: left=/scan/left right=/scan/right back=/scan/back imu=/sensor/imu/data output=/perception/lidar_fusion/scan
```

**验收**：无 `WARN`/`ERROR` 刷屏。若出现以下告警，按提示排查：
- `No transform available for ... scan, skipping this sensor only` → 阶段2 TF
- `Dropping oldest synchronized lidar group...` → IMU 覆盖不足或处理慢
- `IMU header.stamp is 0` / `LaserScan header.stamp is 0` → 阶段1 时间戳

### 3.2 输出话题存在性

```bash
ros2 topic list | rg perception/lidar_fusion/scan
```

**预期**：列出 `/perception/lidar_fusion/scan`。

### 3.3 输出频率

新终端：

```bash
ros2 topic hz /perception/lidar_fusion/scan
```

**验收**：
- **≈ 10 Hz**（受三雷达中最慢者与同步 slop 限制）
- `std dev` < 10ms
- 若明显低于 10 Hz（如 7~8 Hz），查融合节点日志告警

### 3.4 端到端延迟

```bash
# 终端B：看输出 stamp
ros2 topic echo /perception/lidar_fusion/scan --field header.stamp
# 终端C：看当前时间
date +%S.%N
```

**验收**：`stamp` 应略早于 `date`，差值 < 100ms（传感器延迟 + 处理耗时）。若差值 > 200ms，说明处理积压或时钟域不一致。

### 3.5 输出数据完整性

```bash
ros2 topic echo /perception/lidar_fusion/scan --once
```

**关注字段**：
- `header.frame_id` = `base_link`（或配置的 `target_frame`）
- `angle_min`/`angle_max` = ±π（或配置值）
- `angle_increment` ≈ 0.00436（π/720）或配置值
- `ranges` 长度 = `floor((max-min)/inc)+1` ≈ 1441
- `range_min`/`range_max` = 0.05/10.0（或配置值）
- 空 bin 为 `inf`（非 0），有障碍 bin 为有限值

### 3.6 rviz 可视化

```bash
rviz2
```

添加 `LaserScan` 显示，话题选 `/perception/lidar_fusion/scan`，`Fixed Frame` 设 `base_link`。

**验收**：
- 点云覆盖三雷达的并集视野
- 障碍物轮廓清晰，无大规模断裂
- 旋转视角时点云位置符合物理方位

---

## 阶段 4：去畸变效果验证（核心）

> 目标：验证 IMU 旋转去畸变实际生效。这是本模块的核心价值，必须用对照实验。

### 4.1 静止基准测试

**前置**：机器人完全静止，正前方 1.5m 放平板（用于 `--watch-angle 0`）。

```bash
# 终端A：融合节点（默认开去畸变）
ros2 launch lidar_imu_fusion lidar_imu_fusion.launch.py

# 终端B：validator 录 30s
ros2 run lidar_imu_fusion fusion_validator.py --duration 30 --watch-angle 0.0 --output /tmp/static.json
```

**验收**（看报告 `监控角度 0.0° 稳定性`）：
- `距离标准差` < 5mm
- `距离峰峰值` < 2cm
- `帧间跳变 P99` < 5mm

静止时去畸变旋转量应≈0，融合帧应极稳定。若静止时指标就大，说明有噪声源（雷达噪声/TF 抖动/IMU 偏置），先解决再做运动测试。

### 4.2 运动对照测试（关键）

**前置**：场地有清晰墙面/柱子，机器人可原地旋转（产生明显角速度，如 30~60°/s）。

#### 步骤 1：开去畸变

```bash
# 终端A：融合节点（默认 enable_imu_deskew=true）
ros2 launch lidar_imu_fusion lidar_imu_fusion.launch.py

# 终端B：validator，让机器人在采集期间原地旋转
ros2 run lidar_imu_fusion fusion_validator.py --duration 30 --watch-angle 0.0 --output /tmp/deskew_on.json
```

采集期间：让机器人缓慢原地旋转 1~2 圈，确保正前方平板在旋转过程中被多次扫到。

#### 步骤 2：关去畸变（对照）

停掉融合节点，以 `enable_imu_deskew:=false` 重启：

```bash
# 终端A：关去畸变
ros2 run lidar_imu_fusion three_lidar_imu_fusion_node --ros-args \
  --params-file install/lidar_imu_fusion/share/lidar_imu_fusion/config/lidar_imu_fusion.yaml \
  -p enable_imu_deskew:=false

# 终端B：同样旋转条件下录 30s
ros2 run lidar_imu_fusion fusion_validator.py --duration 30 --watch-angle 0.0 --output /tmp/deskew_off.json
```

#### 步骤 3：对比两份报告

```bash
echo "=== 开去畸变 ===" && cat /tmp/deskew_on.json | python3 -c "import json,sys; d=json.load(sys.stdin); w=d['watch_angle_stability']; print(f'峰峰值={w[\"range_ptp_m\"]*1000:.1f}mm  P99跳变={w[\"range_p99_jitter_m\"]*1000:.1f}mm')"
echo "=== 关去畸变 ===" && cat /tmp/deskew_off.json | python3 -c "import json,sys; d=json.load(sys.stdin); w=d['watch_angle_stability']; print(f'峰峰值={w[\"range_ptp_m\"]*1000:.1f}mm  P99跳变={w[\"range_p99_jitter_m\"]*1000:.1f}mm')"
```

**验收标准**：
- 开去畸变的 `峰峰值` 与 `P99跳变` 应**明显小于**关去畸变（如开 2cm vs 关 15cm）
- 差值越大 → 去畸变效果越显著

**异常处理**（开/关几乎无差异）：
| 可能原因 | 排查 |
|---------|------|
| IMU 角速度为 0 | `ros2 topic echo /sensor/imu/data --field angular_velocity` 看旋转时是否有值 |
| IMU 单位错（deg/s 而非 rad/s） | 数值会小 57 倍，检查驱动 |
| IMU→target 旋转错 | 看融合日志 `TF lookup failed from IMU frame...` |
| `enable_imu_deskew` 没生效 | `ros2 param get /three_lidar_imu_fusion enable_imu_deskew` 确认 |
| 时间戳错导致 IMU 覆盖失败 | 日志 `kept un-deskewed due to missing IMU coverage` |

### 4.3 rviz 拖影视检（直观）

在 rviz 里观察原地旋转时的融合点云：

| 模式 | 预期表现 |
|------|---------|
| **关去畸变** | 墙面/柱子呈"扇形拖影"，同一面墙在不同角度被多次打到，呈弧形 smear |
| **开去畸变** | 墙面收敛成**清晰单线**，拖影明显减弱 |

若开去畸变后拖影仍明显，按 4.2 异常表排查。

### 4.4 IMU 覆盖率确认

```bash
# 看融合节点日志，长期观察
ros2 launch lidar_imu_fusion lidar_imu_fusion.launch.py 2>&1 | rg "un-deskewed|coverage"
```

**验收**：`kept un-deskewed due to missing IMU coverage` 计数长期为 0。若频繁出现：
- `ros2 topic hz /sensor/imu/data` 确认 IMU 频率
- 考虑增大 `max_imu_extrapolation_sec`（默认 0.03s）
- 确认 `imu_queue_duration_sec`（默认 2s）够覆盖融合区间

---

## 阶段 5：准确性验证

> 目标：融合输出的距离/方位与真实物理值一致。

### 5.1 量距法（最简单可靠）

**步骤**：
1. 机器人静止，正前方用卷尺量出 2.0m 处放平板
2. 启动融合节点
3. 查看输出对应角度 bin 的距离：

```bash
ros2 topic echo /perception/lidar_fusion/scan --field ranges
```

4. 找到平板对应 bin（正前方约第 720 个 bin，索引 = `round((0 - (-π))/inc)`）

**验收**：该 bin 距离 = 2.0m ± 雷达标称精度（通常 ±2cm）

**多方位重复**（验证三雷达外参都对）：
| 方位 | 角度 | 放平板距离 |
|------|------|-----------|
| 左前方 | 45° | 1.5m |
| 正前方 | 0° | 2.0m |
| 右前方 | -45° | 1.5m |
| 正后方 | 180° | 1.5m |
| 左侧 | 90° | 1.0m |
| 右侧 | -90° | 1.0m |

每个方位实测值与卷尺值误差应 < ±3cm。若某方位系统性偏移，对应雷达外参错。

### 5.2 SLAM 对比法

把融合输出喂给 SLAM，看生成地图质量：

```bash
# 安装 slam_toolbox（若未装）
sudo apt install ros-humble-slam-toolbox

# 启动 SLAM
ros2 launch slam_toolbox online_async_launch.py

# 启动融合节点
ros2 launch lidar_imu_fusion lidar_imu_fusion.launch.py

# 用键盘/手柄遥控机器人在场地建图
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**验收**：
- 墙体清晰、无重影 → 去畸变准确
- 墙体有重影/偏移 → 去畸变不足或 TF 错
- 与已知场地地图对比吻合度

### 5.3 多传感器交叉验证（可选）

若车上有其他感知（3D lidar/深度相机），把本模块 2D 融合投影到同一 rviz，对比同一障碍物位置是否一致。

---

## 阶段 6：稳定性 / 长时运行验证

> 目标：确认长时间运行不崩溃、不泄漏、不漂移。

### 6.1 长时录制

```bash
# 录 30min+ 数据
ros2 bag record -o /tmp/long_run \
  /perception/lidar_fusion/scan \
  /scan/left /scan/right /scan/back \
  /sensor/imu/data
```

### 6.2 内存监控

另开终端，记录节点 RSS：

```bash
# 找到节点 PID
pgrep -f three_lidar_imu_fusion
# 监控 RSS（每 60s 一次）
watch -n 60 'ps -o rss= -p $(pgrep -f three_lidar_imu_fusion)'
```

**验收**：
- RSS 不持续增长（轻微波动正常，但 30min 内不应增长 > 10MB）
- 持续增长 → 内存泄漏，需排查

### 6.3 频率漂移

长时运行期间持续 `ros2 topic hz /perception/lidar_fusion/scan`，确认频率不随时间下降。

**验收**：30min 后频率与起始一致（±5%）。

### 6.4 丢帧率统计

```bash
ros2 bag info /tmp/long_run
```

**验收**：
- `/perception/lidar_fusion/scan` 帧数 ≈ `min(三雷达帧数)` ≈ 10Hz × 时长
- 丢帧率 = 1 - (输出帧数 / 输入最小帧数) < 5%

若丢帧率高，查融合日志：
- `Dropping oldest synchronized lidar group...` → pending 队列满，提高 IMU 频率或增大 `max_pending_scan_sets`
- `All three lidar transforms unavailable` → TF 全缺失
- `Synchronized lidar scans produced no valid fused points` → 有效点全 0，检查 `range_min/max`

### 6.5 日志告警统计

```bash
ros2 launch lidar_imu_fusion lidar_imu_fusion.launch.py 2>&1 | tee /tmp/fusion.log
# 长时运行后统计
rg -c "WARN|ERROR" /tmp/fusion.log
```

**验收**：`WARN_THROTTLE` 已节流，若告警仍大量出现说明有持续性问题，按告警文案排查。

---

## 阶段 7：自动化验证（fusion_validator 工具）

> 目标：把上述阶段 3/4/6 的指标采集自动化，一键出量化报告。

### 7.1 一键启动（推荐）

融合节点 + validator 同时跑，30s 自动出报告：

```bash
ros2 launch lidar_imu_fusion validate.launch.py
```

**自定义**：

```bash
ros2 launch lidar_imu_fusion validate.launch.py \
  duration:=60 watch_angle:=0.0 output:=/tmp/report.json
```

参数说明：
| 参数 | 默认 | 含义 |
|------|------|------|
| `duration` | 30.0 | 采集时长(秒)，0=持续到 Ctrl+C |
| `watch_angle` | 0.0 | 监控稳定性的角度(度，0=正前方) |
| `output` | (空) | 报告写入 JSON 路径 |

### 7.2 独立运行（融合节点已自行启动）

```bash
ros2 run lidar_imu_fusion fusion_validator.py --duration 30 --watch-angle 0.0
```

### 7.3 报告解读

运行中每 2s 打印实时摘要：
```
[实时] frames=58 hz≈9.83 lat≈42.3ms empty≈87.2%
```

结束时输出完整报告，关键指标与验收标准对照：

| 指标 | 字段 | 验收 |
|------|------|------|
| 帧率 | `frame_rate_hz.mean` | 9.5~10.5 Hz |
| 帧率抖动 | `frame_rate_hz.interval_ms_p99` | < 150 ms |
| 延迟 | `latency_ms.p99` | < 100 ms |
| stamp 单调 | `stamp_monotonicity.violations` | 0 |
| 空 bin | `empty_bin_ratio.mean` | 与场景匹配 |
| 监控稳定性 | `watch_angle_stability.range_ptp_m` | 静止 < 2cm |
| 帧间跳变 | `watch_angle_stability.range_p99_jitter_m` | 静止 < 5mm |

### 7.4 持续模式

```bash
ros2 run lidar_imu_fusion fusion_validator.py --duration 0
# Ctrl+C 时自动生成最终报告
```

### 7.5 JSON 报告二次分析

```bash
# 提取关键指标
cat /tmp/report.json | python3 -c "
import json,sys
d=json.load(sys.stdin)
print(f'帧率={d[\"frame_rate_hz\"][\"mean\"]:.2f}Hz')
print(f'延迟P99={d[\"latency_ms\"][\"p99\"]:.1f}ms')
print(f'stamp违例={d[\"stamp_monotonicity\"][\"violations\"]}')
w=d['watch_angle_stability']
print(f'监控峰峰值={w[\"range_ptp_m\"]*1000:.1f}mm')
"
```

---

## 阶段 8：异常排查速查表

| 现象 | 阶段 | 可能原因 | 排查命令 |
|------|------|---------|---------|
| 融合无输出 | 3 | 输入缺失/TF 全缺 | `ros2 topic hz` 各输入；`tf2_echo` |
| 输出频率低 | 3 | pending 堆积/处理慢 | 看日志 `Dropping oldest...` |
| 延迟大 | 3 | 传感器延迟/时钟错 | `echo header.stamp` vs `date` |
| stamp 违例 | 7 | 驱动时间戳回跳 | 修驱动时钟 |
| 空 bin 100% | 7 | 三雷达无有效点/外参全0 | 检查 TF + `range_min/max` |
| 静止时稳定性差 | 4 | 雷达噪声/TF抖/IMU偏置 | 单雷达 `echo` 看噪声 |
| 去畸变无效果 | 4 | IMU未生效 | 见 4.2 异常表 |
| 内存增长 | 6 | 泄漏 | `ps -o rss` 长时监控 |
| 丢帧率高 | 6 | IMU覆盖不足/TF抖 | 看日志告警统计 |

---

## 阶段 9：验收 Checklist

测试完成后逐项确认：

### 输入侧
- [ ] 三雷达频率 10 Hz ±5%，IMU 100 Hz ±5%
- [ ] 四个输入 `header.stamp` 非零且单调递增
- [ ] 三雷达 `ranges` 长度/`angle_*`/`range_*` 合理

### TF/外参
- [ ] 四条 TF 链可解析（或静态外参已填标定值）
- [ ] 量距法多方位误差 < ±3cm
- [ ] rviz 目检方位映射正确

### 输出侧
- [ ] 输出频率 ≈ 10 Hz，`std dev` < 10ms
- [ ] 端到端延迟 < 100ms
- [ ] `header.frame_id`/`angle_*`/`range_*` 与配置一致
- [ ] rviz 点云覆盖三雷达并集视野

### 去畸变
- [ ] 静止基准：监控角度峰峰值 < 2cm
- [ ] 运动对照：开去畸变指标明显优于关去畸变
- [ ] rviz 旋转时墙面无拖影
- [ ] 日志无 `kept un-deskewed` 频繁告警

### 准确性
- [ ] 量距法 6 方位误差 < ±3cm
- [ ] SLAM 地图墙体清晰无重影（若做）

### 稳定性
- [ ] 30min 运行不崩溃
- [ ] RSS 不持续增长（< 10MB/30min）
- [ ] 频率不漂移
- [ ] 丢帧率 < 5%
- [ ] 日志告警不刷屏

### 自动化
- [ ] `fusion_validator` 报告各项指标达验收标准
- [ ] JSON 报告已存档（如 `/tmp/report.json`）

---

## 附录：一键回归测试脚本

把核心自动化步骤串起来（假设输入话题与 TF 已就绪）：

```bash
#!/bin/bash
# regression_test.sh —— lidar_imu_fusion 回归测试
set -e
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== 阶段3: 输出基础 (30s) ==="
ros2 launch lidar_imu_fusion validate.launch.py duration:=30 \
  watch_angle:=0.0 output:=/tmp/base.json
# Ctrl+C 或自动结束

echo "=== 阶段4.1: 静止基准 (30s, 机器人保持静止) ==="
ros2 run lidar_imu_fusion three_lidar_imu_fusion_node --ros-args \
  --params-file install/lidar_imu_fusion/share/lidar_imu_fusion/config/lidar_imu_fusion.yaml &
FUSION_PID=$!
sleep 2
ros2 run lidar_imu_fusion fusion_validator.py --duration 30 \
  --watch-angle 0.0 --output /tmp/static.json
kill $FUSION_PID

echo "=== 报告汇总 ==="
for f in /tmp/base.json /tmp/static.json; do
  echo "--- $f ---"
  python3 -c "
import json
d=json.load(open('$f'))
print(f'帧率={d[\"frame_rate_hz\"][\"mean\"]:.2f}Hz 延迟P99={d[\"latency_ms\"][\"p99\"]:.1f}ms')
w=d['watch_angle_stability']
print(f'监控峰峰值={w[\"range_ptp_m\"]*1000:.1f}mm P99跳变={w[\"range_p99_jitter_m\"]*1000:.1f}mm')
"
done
```

保存为 `regression_test.sh`，`chmod +x` 后执行即可一键跑阶段 3+4.1。

---

## 附录：常用命令速查

```bash
# 编译
colcon build --packages-select lidar_imu_fusion

# 启动融合节点
ros2 launch lidar_imu_fusion lidar_imu_fusion.launch.py

# 一键验证
ros2 launch lidar_imu_fusion validate.launch.py duration:=30 watch_angle:=0.0

# 独立 validator
ros2 run lidar_imu_fusion fusion_validator.py --duration 30 --watch-angle 0.0

# 输入检查
ros2 topic hz /scan/left
ros2 topic echo /scan/left --once
ros2 topic echo /sensor/imu/data --field header.stamp --once

# TF 检查
ros2 run tf2_ros tf2_echo base_link laser_frame_left
ros2 run tf2_tools view_frames

# 输出检查
ros2 topic hz /perception/lidar_fusion/scan
ros2 topic echo /perception/lidar_fusion/scan --once

# 参数检查
ros2 param get /three_lidar_imu_fusion enable_imu_deskew
ros2 param dump /three_lidar_imu_fusion

# 录包
ros2 bag record -o /tmp/run /perception/lidar_fusion/scan /scan/left /scan/right /scan/back /sensor/imu/data
ros2 bag info /tmp/run
ros2 bag play /tmp/run
```
