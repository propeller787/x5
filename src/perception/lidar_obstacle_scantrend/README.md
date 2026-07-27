# lidar_obstacle_scantrend

倾斜单线 2D 激光雷达（`LaserScan`）障碍物检测节点。将 scan 变换到地面坐标系后，沿 **scan 链顺序** 做分割：**标定 ground 种子 → 双向传播 → z/Δx 突变检测 → x 条带 pos_slab → 镂空过滤 → blind spot 虚拟障碍**；发布前在节点层做后处理：**z > 0.28 m 高障碍滤除**（tilt 坐标系）+ 与 **水平雷达** 在 `base_link` 下 xy 去重，避免与水平雷达重复报障。

相对 `lidar_obstacle_groundmodel`，本包去掉 DBSCAN 与多套分散规则，逻辑集中在一个 `ScanTrendSegmenter` 类中，适合低装斜射雷达的台阶、坑洞、凸台与侧向 blind spot 场景。

---

## 总体流程

每一帧 `LaserScan` 进入 `ScanTrendSegmenter::process()` 后，**严格按以下顺序**执行；前一步输出是后一步输入。

```
LaserScan
  │
  ├─ 1. transformScan              外参变换 + 角度/距离/噪声过滤 → ScanPoint[]
  ├─ 2. sortScanPoints             按 beam_index 排序，必要时 reverse 保证 x 大致递增
  ├─ 3. labelInitialSideRegionObstacles
  │       有 ground 种子 → 只标贴线种子
  │       无 ground 种子 → 侧向 x 带内投影在线段上的点全部标负障碍
  │
  ├─ 4. [条件] run_ground_pipeline
  │       ground_seed_count > 0 或 side_blind_spot 未加载时执行：
  │       ├─ propagateGroundAndDetectJumps   从种子沿 scan 双向传播 + jump/断链
  │       ├─ detectPositiveObstacles         x 条带 pos_slab（凸起/下沉）
  │       └─ filterGrateObstacles            镂空/格栅负障碍滤除
  │       无种子且 side 标定已加载 → 跳过上述三步（短路路径）
  │
  ├─ 5. buildOutput                  输出 obstacle / ground / transformed 点云
  └─ 6. injectBlindSpotSideObstacles 侧向 ground 不足时注入虚拟障碍点

节点层（`processScan`，tilt 每帧，`horiz_dedup_enable=true` 时）：
  ├─ 订阅 /scan/left、/scan/right，TF 到 base_link，缓存 x>0 的水平点云
  ├─ tilt 障碍：z > 0.28 m（tilt 系）直接剔除
  ├─ z ≤ 0.28 m 的点 TF 到 base_link，与合并水平点云做 xy 半径去重（默认 0.20 m）
  └─ 保留点仍以 left_tilt_rotate / right_tilt_rotate 发布
      xy 去重 fail-open：TF 失败或水平缓存为空时，仍做 z 过滤，仅跳过 xy 匹配
```

### 短路路径（方案 B）

当 `side_blind_spot` 已加载且 **标定线上没有任何 ground 种子**（`ground_seed_count == 0`）时：

- 跳过 `propagateGroundAndDetectJumps`、`detectPositiveObstacles`、`filterGrateObstacles`
- 保留第 3 步已将侧向区域标成的负障碍
- 直接 `buildOutput` + `injectBlindSpotSideObstacles`

无 side 标定时（`side_blind_spot.loaded == false`），仍走完整 pipeline。

### 标签语义

| 字段 | 含义 |
|------|------|
| `label == Ground` | 地面 |
| `label == Obstacle && negative_obstacle == false` | 正障碍（凸起 / cliff） |
| `label == Obstacle && negative_obstacle == true` | 负障碍（坑、台阶下沉、侧向 blind spot） |

初始所有点 `label = Ground`，`negative_obstacle = false`。

---

## 1. 坐标变换：transformScan

- 输入：`sensor_msgs/LaserScan`
- 外参：`left_tilt_lidar.txt` / `right_tilt_lidar.txt`（至少 6 行：tz, ty, tx, rz, ry, rx）
- 变换：`p_body = R * p_lidar + [0, 0, tz]`（仅使用 tz 作为平移 z）
- 坐标系：`frame_id` 为 `left_tilt_rotate` / `right_tilt_rotate`，**z = 0 为名义地面**

### 过滤条件

| 条件 | 默认 |
|------|------|
| range 有效 | `[range_min, range_max]` |
| 近距噪声 | 若 `< noise_check_range`（0.25 m）的有效点 `< noise_min_count`（10），则丢弃近距点 |
| 有效扇区 | `useScanAngle`：保留 ±90° 及对应补角扇区 |
| 高度上限 | `z < max_point_z`（1.5 m） |
| 后方截断 | `x >= -1.0` |

输出 `ScanPoint`：`x, y, z, angle, range, beam_index, label, negative_obstacle`。

---

## 2. Scan 排序：sortScanPoints

单线雷达必须保证后续 `i` / `i+1` 为 **物理相邻 beam 链**，而非任意 x 排序。

1. `transformScan` 为每个点记录 `beam_index`（LaserScan `ranges` 下标）
2. 按 `beam_index` 升序排序（整数比较，替代原 `sortAngle` 排序）
3. 若 `points.front().x > points.back().x`，整链 **reverse**，保证 x 大致递增

### 雷达兼容性

本实现假定配套 tilt 单线雷达为 **全周扫描（`angle_min ≈ -π`，`angle_max ≈ +π`）**。在此条件下，bag 验证 **`beam_index` 序与原 `sortAngle` 序一致**。

**不支持** 角域仅为 **0 ~ π**（或类似半周）的单线雷达：其 beam 序与 scan 链所需角序一般不等价，需改用 `sortAngle` 排序或重新验证。

外参 txt 若含第 7 行 `sort_angle_deg`（与 groundmodel 共用文件），scantrend **不读取**。

---

## 3. 侧向初始标注：labelInitialSideRegionObstacles

依赖 yaml 中 `ground_model.start_point_side` / `end_point_side` 定义的 **side_blind_spot 线段**。

### 几何判定

- **x 范围**：线段 x 范围 ± `blind_spot_side_x_margin`（0.05 m）
- **投影**：点投影必须落在线段 `[start, end]` 上（不含延长线）
- **垂距**：点到线段距离 `dist`

### 行为

| 情况 | 行为 |
|------|------|
| `dist ≤ blind_spot_line_dist_tol`（0.10 m） | 标 **Ground 种子** |
| 有 ≥1 个种子 | **结束**；`dist > tol` 的点保持默认 Ground，留给传播 |
| 无任何种子 | 侧向 x 带内、投影在线段上的 **全部点** 标 **负障碍** |

返回值：`ground_seed_count`（种子数量）。

---

## 4. Ground 传播与突变：propagateGroundAndDetectJumps

从所有 ground 种子段取 **全局 `min_seed_idx`** 与 **`max_seed_idx`**，各传播一次：

- `max_seed_idx` → `propagateGroundForward`（index 增大）
- `min_seed_idx` → `propagateGroundBackward`（index 减小）

### 传播状态机

在 `obstacle_mode == false` 时，对相邻边 `(i, i+1)`（或 backward 的 `prev, i`）：

| 事件 | 行为 |
|------|------|
| 无 Δx 断链、无 z jump | 下一标 **Ground**，`ground_z` 更新为当前点 z |
| **Δx 断链** `|Δx| > max_expand_dx`（1.5 m） | 进入 obstacle_mode；对侧及后续 sweep 标障碍 |
| **z jump** `isZJumpEdge` | 进入 obstacle_mode；`dz>0` → 正障碍，`dz<0` → 负障碍 |

进入 `obstacle_mode` 后，沿 scan 方向 **余段全部** 标为同类型障碍（sweep）。

### z 突变判定：isZJumpEdge

```
|dz| ≥ negative_dz_min（0.10 m）
且 非 isGradualRampEdge（缓坡）
```

**缓坡过滤** `isGradualRampEdge`（多基线斜率一致性）：

1. 锚点 `anchor = edge_i`（跳变前点，如 5），跳变斜率 `slope_jump = slope(anchor → edge_i+1)`（如 5→6）
2. 须为下坡（`slope_jump ≤ 0`）
3. 对 `k = 1 … z_ref_window-1`（默认 `slope(4→5)`, `slope(3→5)`, `slope(2→5)` …）
4. `|Δx| < ramp_min_dx` 跳过；**任一** `slope_k` 与 `slope_jump` 接近 → 缓坡延续
5. 接近判定：`|slope_k - slope_jump| ≤ ramp_slope_abs_tol` **或** 相对差 `≤ ramp_slope_ratio_tol`
6. 无一接近 → 真 jump

避免把连续下坡误当成台阶 jump；平路+末级大台阶时跳变斜率与回看基线不接近，不会被豁免。

### 负向突变：坑口/台阶唇边

当 `obstacle_negative == true`（下沉侧低于 ground）时，除跳变对侧及 sweep 段外，**突变前最后一个 ground 点（唇边）** 也标为负障碍：

```
向前:  ... G  G  |  O  O  O
              ↑ lip (i)   ↑ i+1...
```

正向凸台（`dz > 0`）不标唇边，传播侧 ground 保持 Ground。

### Δx 断链

低装斜射雷达在「换支」时相邻 beam 的 Δx 可能很大（> 1.5 m）。此时若对侧 z 低于当前 `ground_z`，同样标负向障碍并包含唇边。

---

## 5. x 条带 pos_slab：detectPositiveObstacles

检测条带内 **相对地面有明显 z 跨度** 的凸起或下沉（对已标 Obstacle 的点跳过）。

### x 条带

```
inXGrid(x_i, x_j) = |x_i - x_j| ≤ grid_x_max × 0.5   （默认 ±10 cm，总宽 20 cm）
```

不限制 y；条带内为当前 scan 线上的点。

### 前置门槛（条带内统计 min_z / max_z）

1. `Δz = max_z - min_z > positive_dz_min`（0.13 m）
2. `slope = Δz / max(|x_at_max - x_at_min|, grid_x_min) > positive_slope_min`（0.40）
3. 条带内至少有一个 `label == Ground` 的点（`ground_zs` 非空）

### 参考面 ref

```cpp
ref = (|min_z| < |max_z|) ? min_z : max_z;   // 取更贴近 z=0 的一侧
```

| 条件 | 标签 |
|------|------|
| `z_i ≥ ref + positive_dz_min` | Obstacle，正（凸起） |
| `z_i ≤ ref - positive_dz_min` | Obstacle，负（下沉） |
| 其余 | 不修改 |

条带内同时有坑底与格栅沿时，`|z|` 更小侧作 ref，避免格栅沿被误标。

---

## 6. 镂空过滤：filterGrateObstacles

仅作用于 **负障碍** 点。若 x 方向 **前后** 均有足够多的「地面型」邻居，则认为该点是镂空/格栅误检，改回 Ground。

### 邻居候选 `isGrateNeighborCandidate`

- `label == Ground`，或
- 正障碍且 `|z| < positive_dz_min`（低 z 正障碍视作地面型）

### 计数

- 后方：`x_j < x_i` 且 `|Δx| ≤ grate_x_radius`（0.20 m）
- 前方：`x_j > x_i` 且 `Δx ≤ grate_x_radius`
- 若 `back ≥ min_grate_back_neighbors`（10）且 `fwd ≥ min_grate_fwd_neighbors`（10）→ 改回 Ground

`neg_count_in_region` 在 **filterGrate 之前** 统计，供虚拟障碍注入使用。

---

## 6.5 水平雷达去重与高障碍滤除（节点层）

水平雷达已能稳定检出的障碍，斜雷达不再重复发布。原 **28cm 正障碍簇滤除**（`filterTallObstacleClusters`）已移除，改为此处后处理。

### 水平点云缓存

| 步骤 | 说明 |
|------|------|
| 订阅 | `/scan/left`、`/scan/right` |
| TF | 各 scan 的 `frame_id` → `dedup_target_frame`（默认 `base_link`） |
| 过滤 | 仅保留 **`x > horiz_min_x`**（默认 0）的点 |
| 合并 | 左右合并为 **一份** `horiz_all_base_`，tilt 两侧共用 |
| TF 失败 | 保留该侧上一帧缓存；本帧不去更新 |

### tilt 后处理（`dedupTiltObstaclesWithHorizontal`）

tilt scantrend 完成后、发布 obstacle 前（仅 `horiz_dedup_enable=true`）：

1. **高障碍滤除**：tilt 坐标系下 **`z > 0.28 m`** 的点直接丢弃（由水平雷达负责更高障碍；阈值 `kTiltMaxObstacleZ`，编译期常量）
2. **xy 去重**：`z ≤ 0.28 m` 的点 TF 到 `base_link`，与合并水平点云在 xy 平面做半径匹配
3. 若距离 **≤ dedup_radius_xy**（默认 **0.20 m**）→ 剔除
4. 未匹配点 **保持原 tilt 坐标**（`left_tilt_rotate` / `right_tilt_rotate`），`frame_id` 不变

### fail-open（仅 xy 去重）

| 情况 | z > 0.28 滤除 | xy 去重 |
|------|---------------|---------|
| 水平缓存为空 | 仍执行 | 跳过，保留 z≤0.28 点 |
| tilt→base TF 失败 | 仍执行 | 跳过，保留 z≤0.28 点 |
| 两者均就绪 | 执行 | 执行 |

`horiz_dedup_enable=false` 时，上述后处理**全部跳过**，直接发布 scantrend 原始障碍点云。

---

## 7. 输出：buildOutput

| 输出 | 内容 |
|------|------|
| `obstacle` | `label == Obstacle` |
| `ground` | `label == Ground` |
| `transformed` | 全量变换点（调试用，含 label 着色前的几何） |

---

## 8. Blind spot 虚拟障碍：injectBlindSpotSideObstacles

当侧向区域 **负障碍足够多** 但 **标定线附近 ground 不足** 时，在 `x ≈ 0` 注入一排虚拟障碍点，补偿 blind spot。

### 触发条件

1. `side_blind_spot` 已加载
2. `neg_count_in_region_before_grate ≥ blind_spot_min_neg_pts`（2）
3. 标定线走廊内（垂距 ≤ 0.10 m）且 `|z| ≤ blind_spot_ground_z_tol`（0.10 m）的 Ground 点数 **< blind_spot_min_ground_pts`（80）

### 虚拟点

- `x`：`blind_spot_virtual_x_min` ~ `max`，步长 `virtual_x_step`（默认 -0.10 ~ 0.10，步 0.05）
- `y`：走廊内 ground 的 y 中位数 × `blind_spot_y_scale`，或 `start_y * 0.5`；再经 `clampBlindSpotVirtualY` 限幅
- `z = 0`

---

## ROS 接口

### Topic

| 类型 | Topic | 说明 |
|------|-------|------|
| 订阅 | `/scan/left_tilt` | 左 tilt LaserScan |
| 订阅 | `/scan/right_tilt` | 右 tilt LaserScan |
| 订阅 | `/scan/left` | 左水平 LaserScan（去重用，`horiz_dedup_enable=true`） |
| 订阅 | `/scan/right` | 右水平 LaserScan（去重用） |
| 发布 | `/perception/scantrend/left_tilt/pointcloud` | 左障碍物点云 |
| 发布 | `/perception/scantrend/right_tilt/pointcloud` | 右障碍物点云 |
| 发布 | `/perception/scantrend/left_tilt/transform` | 左全量变换点云（debug） |
| 发布 | `/perception/scantrend/right_tilt/transform` | 右全量变换点云（debug） |

### 节点参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `calibration_dir` | `.../test_0608/calibration_files` | 标定目录 |
| `left_extrinsic` | `left_tilt_lidar.txt` | 左雷达外参 |
| `left_ground_yaml` | `left_lidar_groung_model.yaml` | 左地面模型 yaml |
| `right_extrinsic` | `right_tilt_lidar.txt` | 右雷达外参 |
| `right_ground_yaml` | `right_lidar_ground_model.yaml` | 右地面模型 yaml |
| `timing_log_enable` | `false` | 是否打印耗时日志（见下文「性能统计」） |
| `timing_log_interval_sec` | `5.0` | 耗时日志节流间隔（秒） |
| `horiz_dedup_enable` | `true` | 是否启用水平雷达去重 |
| `dedup_target_frame` | `base_link` | 水平缓存与去重使用的目标坐标系 |
| `dedup_radius_xy` | `0.20` | xy 平面去重半径（米） |
| `horiz_min_x` | `0.0` | 水平点保留的最小 x（base_link 下，米） |

算法阈值在 `ScanTrendParams`（头文件）中定义，当前未暴露为 ROS 参数。

### 性能统计

节点内置每帧耗时统计（`std::chrono::steady_clock`），**默认只计时、不打日志**（`timing_log_enable=false`）。计时本身每帧额外开销约 **0.002~0.005 ms**，可忽略。

开启后，每隔 `timing_log_interval_sec` 秒**成对**输出两行日志（同一帧、同一节流周期，不会只打一行）：

1. **callback timing**：ROS 回调总耗时，含 segmenter、PCL 转换、publish
2. **scantrend timing**：算法各步骤耗时

#### 算法步骤字段（scantrend timing）

| 字段 | 对应函数 |
|------|----------|
| `transform` | `transformScan` |
| `sort` | `sortScanPoints` |
| `label` | `labelInitialSideRegionObstacles` |
| `propagate` | `propagateGroundAndDetectJumps` |
| `pos_slab` | `detectPositiveObstacles` |
| `count_neg` | `countNegInSideRegion` |
| `grate` | `filterGrateObstacles` |
| `build` | `buildOutput` |
| `inject` | `injectBlindSpotSideObstacles` |

另含 `n`（有效点数）、`pipeline`（1=完整流程，0=短路路径）。

#### 日志示例

```
[left_tilt_rotate] callback timing total=1.85ms | segmenter=1.42 pcl_debug=0.12 pcl_obstacle=0.08 publish_debug=0.01 publish_obstacle=0.01
[left_tilt_rotate] scantrend timing n=412 pipeline=1 total=1.42ms | transform=0.08 sort=0.15 label=0.02 propagate=0.12 pos_slab=0.71 count_neg=0.01 grate=0.09 build=0.04 inject=0.01
```

#### 用法

```bash
# 默认：计时但不打日志
ros2 run lidar_obstacle_scantrend lidar_obstacle_scantrend_node

# 开启耗时日志，每 1 秒最多打一次
ros2 run lidar_obstacle_scantrend lidar_obstacle_scantrend_node \
  --ros-args -p timing_log_enable:=true -p timing_log_interval_sec:=1.0

# launch / yaml 中同样可设
# timing_log_enable: true
# timing_log_interval_sec: 5.0
```

耗时结果同时写入 `ScanTrendResult::timing`（微秒），可供后续离线分析或接 topic 扩展；当前仅通过日志输出。

### 标定文件

**外参 txt（至少 6 行；与 groundmodel 共用的 7 行文件也兼容，第 7 行忽略）**

```
tz
ty
tx
rz
ry
rx
```

**地面 yaml** 使用字段：

- `ground_model.ranges`：极坐标地面模型（当前 pipeline 主流程未直接使用，保留加载）
- `ground_model.start_point_side[0]` / `end_point_side[0]`：侧向 blind spot 线段 `{x, y, z}`

---

## 编译与运行

```bash
cd /path/to/vlago
colcon build --packages-select lidar_obstacle_scantrend
source install/setup.bash
ros2 run lidar_obstacle_scantrend lidar_obstacle_scantrend_node
```

回放 bag 时需同时提供：

- `/scan/left_tilt`、`/scan/right_tilt`
- `/scan/left`、`/scan/right`（`horiz_dedup_enable=true` 时）
- `/tf_static`（`base_link` → 各 `laser_frame_*` / `*_tilt_rotate`）

```bash
ros2 bag play <bag_dir>
```

节点使用默认 `rclcpp::spin`（单线程 executor），4 路 scan 订阅回调**串行**执行，同一时刻仅一个回调在跑。

---

## 参数一览（ScanTrendParams）

### 正向 pos_slab

| 参数 | 默认 | 说明 |
|------|------|------|
| `grid_x_min` | 0.10 m | 坡度分母下限 |
| `grid_x_max` | 0.20 m | x 条带总宽 |
| `positive_dz_min` | 0.13 m | 判障 \|Δz\| 下限 |
| `positive_slope_min` | 0.40 | 条带内 Δz/Δx 下限 |

### Ground 传播 / jump

| 参数 | 默认 | 说明 |
|------|------|------|
| `negative_dz_min` | 0.10 m | z jump 最小 \|Δz\| |
| `max_expand_dx` | 1.50 m | Δx 断链阈值 |
| `ramp_slope_max` | 0.45 | legacy，当前缓坡判定未使用 |
| `ramp_min_dx` | 0.10 m | 算斜率时 \|Δx\| 下限 |
| `ramp_slope_abs_tol` | 0.08 | 斜率绝对差容差 |
| `ramp_slope_ratio_tol` | 0.35 | 斜率相对差容差 |
| `ramp_min_valid_slopes` | 2 | legacy，当前为「任一相似即可」 |
| `z_ref_window` | 6 | 回看 `k=1…5`（anchor 到 anchor-k） |
| `ramp_step_dz` | 0.06 m | legacy，当前未使用 |

### 镂空过滤

| 参数 | 默认 | 说明 |
|------|------|------|
| `grate_x_radius` | 0.20 m | x 邻域半径 |
| `min_grate_back_neighbors` | 10 | 后方最少地面型邻居 |
| `min_grate_fwd_neighbors` | 10 | 前方最少地面型邻居 |

### 水平雷达去重（ROS 参数）

| 参数 | 默认 | 说明 |
|------|------|------|
| `horiz_dedup_enable` | `true` | 是否订阅水平 scan 并在发布前去重 |
| `dedup_target_frame` | `base_link` | 缓存与去重坐标系 |
| `dedup_radius_xy` | 0.20 m | xy 平面匹配半径 |
| `horiz_min_x` | 0.0 m | 水平点 x 下限（base_link） |
| `kTiltMaxObstacleZ` | 0.28 m | tilt 坐标系 z 上限，超过不发布（**cpp 编译期常量**，非 ROS 参数） |

### Blind spot

| 参数 | 默认 | 说明 |
|------|------|------|
| `blind_spot_line_dist_tol` | 0.10 m | 标定线垂距容差 |
| `blind_spot_side_x_margin` | 0.05 m | 侧向 x 带扩展 |
| `blind_spot_ground_z_tol` | 0.10 m | 虚拟障碍触发时 ground \|z\| 上限 |
| `blind_spot_min_ground_pts` | 80 | ground 不足阈值 |
| `blind_spot_min_neg_pts` | 2 | 触发虚拟障碍的最少 neg 数 |
| `blind_spot_virtual_x_min/max/step` | -0.10/0.10/0.05 | 虚拟点 x 范围 |

### 预处理

| 参数 | 默认 | 说明 |
|------|------|------|
| `noise_check_range` | 0.25 m | 近距噪声检测范围 |
| `noise_min_count` | 10 | 近距有效点计数 |
| `max_point_z` | 1.5 m | 点 z 上限 |

### Legacy（头文件中保留，当前 cpp 未使用）

`negative_dx_min`、`depression_low_z_thresh`、`platform_high_z_thresh`、`sink_z_tol`、`max_sink_x`、`max_sink_points`、`max_cliff_points`。

---

## 设计要点与已知限制

1. **scan 链顺序是核心**：传播、jump、sweep 都依赖 `sortScanPoints`；序错了后续全错。
2. **ground 种子驱动**：无种子时走短路，侧向保守输出；有种子时才能向带外传播 ground。
3. **pos_slab 缓坡误检**：x 条带内若前方有更低点，`min_z` 被拉低，可能产生 Δz 虚高（README 历史问题，x≈2.17 附近）；ref 取 \|z\| 更小侧可部分缓解坑/格栅场景。
4. **filterGrate 可能洗掉真坑**：若坑口前后 ground 邻居碰巧够多；负向唇边已标 neg，grate 对其同样生效。
5. **`revertNearGroundObstacles`** 已实现但未接入 pipeline（注释状态）。
6. **排序与雷达型号绑定**：仅验证过 **-π~π 全周** tilt 单线雷达；换用 0~π 雷达或更改外参后须重新验证 scan 链顺序。
7. **水平雷达互补**：`z > 0.28 m` 由 tilt 侧滤除 + xy 去重避免重复；二者均在 `horiz_dedup_enable=true` 时生效。
8. **回调串行**：单线程 executor，水平缓存与 tilt 去重不会并行执行；水平回调会占用队列时间。

---

## 与 groundmodel 的关系

| 项目 | groundmodel | scantrend |
|------|-------------|-----------|
| 输入 | LaserScan | LaserScan |
| 排序 | `Rz(sort)*R⁻¹` 角排序 | `beam_index` 排序 |
| 分割 | 极坐标粗分 + 多次 DBSCAN + 补丁 | scan 链传播 + jump + pos_slab |
| 侧向 blind spot | 分段计数 + 补点 | 标定线种子 + 虚拟障碍 |
| 镂空 | `filter_negative_z_by_x_ground` 等 | `filterGrateObstacles` |
| 高障碍 / 水平互补 | 水平雷达主检 | z>0.28 滤除 + xy 去重（节点层） |

本包适合作为 groundmodel 的轻量替代或并行 A/B 验证。
