# Intent Avoidance — 意图引导避障模块

## 架构

```
                    ┌─────────────────────────────────────────┐
                    │           IntentAvoidanceNode            │
                    │                                          │
/scan/left ──────► │  ┌──────────────┐    ┌───────────────┐  │
(LaserScan)        │  │ ScanProcessor │───►│ FreeSpaceAnalyzer │
/scan/right ─────► │  │ (range→阻塞扇区)│    │ (可通行扇区+窄道)│
(LaserScan)        │  └──────────────┘    └───────┬───────┘  │
                    │                              │          │
/sensor/joystick ─► │  ┌──────────────┐    ┌───────▼───────┐  │
(Joystick)          │  │ IntentDetector│───►│ AvoidanceEngine│  │
                    │  │ (摇杆→意图角) │    │ (ABC策略+防抖) │  │
                    │  └──────────────┘    └───────┬───────┘  │
                    │                              │          │
/wheel/twist ─────► │  ┌──────────────┐    ┌───────▼───────┐  │
(TwistStamped)      │  │ SpeedMonitor  │───►│ ControlGenerator│──► /nav/control
                    │  │ (实际车速)    │    │ (Twist+平滑)   │  │
                    │  └──────────────┘    └───────────────┘  │
                    └─────────────────────────────────────────┘
```

## 组件

### ScanProcessor — 激光预处理
- 输入: `/scan/left`, `/scan/right` (sensor_msgs::LaserScan)
- 输出: `ObstacleSectorList`（base_link坐标系阻塞扇区）
- 不做TF2，用标定安装参数(URDF)做刚性几何变换
- 每条射线: 雷达帧 → base_link 三角函数变换 → 角度+距离 → 聚合连续阻塞角度

### FreeSpaceAnalyzer — 可通行空间
- 输入: `ObstacleSectorList` (左右合并)
- 输出: `FreeSectorList` + `NarrowPassageInfo`
- 可通行扇区 = 阻塞扇区在[-90°,90°]的补集
- 过滤宽度 < 车体通过所需最小角度的扇区
- 窄道: 左右边界15帧稳定 + 通道∈[车宽×1.2, 车宽×2.5]

### IntentDetector — 意图检测
- 输入: `/sensor/joystick` (rocke_x, rocker_y ∈ [-100,100])
- 输出: `UserIntent`
- 意图角 = atan2(rocker_y, rocker_x)
- 死区判定: |y|>15 → 有转向意图, x>15 → 有前推意图

### AvoidanceEngine — 核心决策
- 输入: `FreeSectorList`, `UserIntent`, 实际车速
- 输出: `AvoidanceDecision`

```
情况A: 用户方向 ∈ FreeSector → DIRECT_PASS (直接执行, 适度降速)
情况B1: 不可行+有转向意图 → PROJECT_NEAREST (投影到角度最近可通行扇区中心)
情况B2: 不可行+无转向意图 → SELECT_BEST (选最宽可通行扇区)
情况C: 无FreeSector → FORCE_BRAKE
窄道: 左右稳定通道 → NARROW_GUIDE (居中控制, 优先级最高)
防抖: 方向锁定8帧, 仅意图变化>30°或方向不可行时才解锁
```

### ControlGenerator — 控制输出
- 输入: `AvoidanceDecision`, 实际车速
- 输出: `geometry_msgs::Twist` → `/nav/control`
- 限幅: |angular.z|≤0.5rad/s, |accel|≤0.8m/s²
- 平滑: 一阶低通 α=0.3
- 渐停: vx<0.05时直接置零

---

## 坐标变换（不做TF2）

### 为什么不做TF2

scene_node 的 LidarDriver 依赖 tf2 + PCL 做 `laser_frame_* → base_link` 的点云变换，增加了大量依赖和计算开销。本模块直接使用 URDF 中的固定安装参数，用三角函数做刚性变换，零外部依赖。

### 参数来源与含义

参数直接来自 CAD 机械设计导出的 URDF 文件 `scripts/yisbot_o1.urdf`：

```xml
<!-- 左雷达 -->
<joint name="laser_left_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame_left"/>
    <origin xyz="0.69 0.255 0.3" rpy="0 0 0.00"/>
</joint>

<!-- 右雷达 -->
<joint name="laser_right_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame_right"/>
    <origin xyz="0.69 -0.255 0.3" rpy="0 0 -0.03"/>
</joint>
```

### 各参数含义图解

```
                        车辆前进方向 (+X)
                             ▲

         左雷达              │              右雷达
    mount=(0.69,0.255)      │        mount=(0.69,-0.255)
      yaw=0.00              │           yaw=-0.03
           ●                │                ●
           │←── 0.255m ──→ ● ←── 0.255m ──→│
           │              base_link          │
           │             (0,0,0)            │
           │                                │
    ───────┼──────────── 0.69m ─────────────┼──────► 右 (+Y)
           │                                │
```

### 参数说明

| 参数 | 含义 | 左雷达值 | 右雷达值 |
|------|------|---------|---------|
| **x** | 雷达在车辆前后方向的位置。base_link 原点 → 雷达中心的前向距离(m) | **0.690** | **0.690** |
| **y** | 雷达在车辆左右方向的位置。base_link 原点 → 雷达中心的侧向距离(m)，左正右负 | **+0.255** | **-0.255** |
| **z** | 雷达离地高度。URDF中有此值，但2D雷达计算忽略Z轴 | 0.300 | 0.300 |
| **yaw** | 绕Z轴旋转角(rad)。0=雷达扫描正前方与车辆正前方平行，正值=左偏，负值=右偏 | **0.00** | **-0.03** |
| **roll** | 绕X轴旋转角(rad)，水平安装=0 | 0 | 0 |
| **pitch** | 绕Y轴旋转角(rad)，水平安装=0 | 0 | 0 |

### x (0.690m) 的含义

雷达安装孔位到 base_link 原点的前后距离。base_link 通常取后轮轴中心或车体几何中心。690mm = 雷达安装在车辆前半部分靠前位置。

### y (±0.255m) 的含义

左右雷达各距车辆中轴线 255mm，两台雷达横向间距 = 510mm。通过 CAD 设计图直接获取，安装完成后也可用卷尺测量：从车辆中轴线到雷达外壳中心的水平距离。

### yaw 的含义与来源

- **左雷达 yaw=0**：雷达扫描零度角方向与车辆正前方完全平行。设计目标如此。
- **右雷达 yaw=-0.03 rad (≈ -1.72°)**：轻微向右偏转。来源有两种可能：
  1. CAD 设计时有意为之，使右侧雷达略微扫向车辆前方中心区域，弥补左右雷达盲区
  2. 安装公差产生的角度偏差，在调试时实测后写入 URDF

yaw 的正负号含义：正值 = 雷达前方指向车辆左侧，负值 = 指向车辆右侧。

### 变换公式

```cpp
// 雷达帧下的点坐标
double x_r = range * cos(angle_radar);  // 雷达正前方
double y_r = range * sin(angle_radar);  // 雷达正左方

// 刚性变换到 base_link（考虑雷达安装偏移 + yaw 旋转）
double cos_yaw = cos(mount.yaw);
double sin_yaw = sin(mount.yaw);
double x_v = mount.x + x_r * cos_yaw - y_r * sin_yaw;  // base_link X
double y_v = mount.y + x_r * sin_yaw + y_r * cos_yaw;  // base_link Y

// 车辆坐标系下的角度和距离
double angle_base = atan2(y_v, x_v);
double range_base = hypot(x_v, y_v);
```

### 验证示例

左雷达雷达帧正前方 1m 处有障碍物 (angle_radar=0, range=1.0)：

```
x_v = 0.69 + 1.0×1 - 0×0 = 1.690
y_v = 0.255 + 1.0×0 + 0×1 = 0.255
range_base = sqrt(1.690² + 0.255²) = 1.709 m    ← base_link 下的实际距离
angle_base = atan2(0.255, 1.690) = 0.150 rad    ← base_link 下偏左 8.6°
```

**关键理解**：雷达报的 1m 不等于 base_link 下的 1m。因为雷达装在车辆前方 0.69m 处，base_link 下的实际距离是 1.709m。这在计算 TTC 和制动距离时至关重要。

### 如果安装位置变动怎么调整

修改 `config/intent_avoidance_params.yaml`：

```yaml
left_radar_x: 0.690
left_radar_y: 0.255
left_radar_yaw: 0.0
right_radar_x: 0.690
right_radar_y: -0.255
right_radar_yaw: -0.03
```

也可以用 `ros2 param set` 运行时动态调整，观察效果后固化到 YAML。

### 如何实际测量这些参数（当CAD数据不可用时）

**x (前后位置)**：用卷尺测量 base_link 原点到雷达扫描中心的水平前后距离。

**y (侧向位置)**：用卷尺测量车辆中轴线到雷达扫描中心的水平左右距离。左正右负。

**yaw (偏航角)**：在车辆正前方放置一个标定板，距离约 1-2m。读取雷达在该角度上的 range 值。转动标定板找到雷达检测到最小距离时对应的激光束角度——该角度即为 yaw（如果激光束标称 0° 对应实际正前方，则实测偏角就是 yaw）。更精确的方法：在车辆对称位置放置两个标定物，通过雷达返回的左右角度差推算偏航角。

---

## 参数

```yaml
safe_distance_threshold: 1.2       # 障碍判定距离(m)
min_free_sector_width: 0.349       # 最小可通行扇区宽度(rad, ≈20°)
joystick_deadzone: 15              # 摇杆死区
direction_lock_frames: 8           # 防抖锁定帧数
direction_unlock_threshold: 0.524  # 意图变化解锁阈值(rad, ≈30°)
speed_reduction_factor: 0.6        # 避障时速度因子
max_angular_guidance: 0.5          # 引导最大角速度(rad/s)
max_linear_accel: 0.8              # 最大线加速度(m/s²)
output_smooth_alpha: 0.3           # 输出低通系数
emergency_brake_ttc: 0.5           # 紧急制动TTC(s)
emergency_brake_distance: 0.3      # 紧急制动距离(m)
robot_width: 0.70                  # 车宽(m)
narrow_passage_ratio_min: 1.2      # 窄道最小比
narrow_passage_ratio_max: 2.5      # 窄道最大比
```

## 依赖

rclcpp + sensor_msgs + geometry_msgs + std_msgs + interface (Joystick)
无 PCL / TF2 / Eigen 依赖

---

## 开发记录

### 编译问题

**问题1：`common/utils/topic.h` 找不到路径**
- 原因：话题常量定义在 `common` 包中，但 intent_avoidance 不依赖 common 包
- 解决：在 `intent_avoidance_node.cpp` 中用匿名 namespace 内联了 5 个话题字符串常量，与 `topic.h` 保持一致

**问题2：测试中 `beamToBaseLink` 是 private 无法访问**
- 原因：该静态函数原本放在 `private` 区段，测试代码无法调用
- 解决：移动到 `public` 区段，因为它是纯工具函数，供外部测试和复用是合理的

**问题3：`control_generator.cpp` 未使用参数 warning**
- 原因：`user_intent` 和 `actual_speed` 参数声明了但函数体内未引用
- 解决：参数名改为 `/*user_intent*/` 和 `/*actual_speed*/` 注释掉参数名，消除 `-Wunused-parameter` 警告

### 测试问题

**问题4：BeamToBaseLink 测试断言 `range < 1.5` 失败，实际值 1.709**
- 原因：雷达帧下 range=1.0m 的障碍物，变换到 base_link 后距离 = hypot(1.69, 0.255) ≈ 1.709m。最初写测试时直接用 1.5 作为上限，忽略了安装偏移使实际距离变长。
- 修正：`EXPECT_NEAR(range, 1.709, 0.01)` 使用精确预期值

### 架构决策

**决策1：新建独立模块而非修改 scene_node**
- 原因：scene_node 依赖链重（PCL/TF2/Eigen），且避障逻辑与点云聚类/跟踪强耦合。新模块只需 LaserScan 的 range 数组，无需点云处理。
- 结果：intent_avoidance 仅 5 个依赖，约 800 行 C++，完全自包含。

**决策2：不做 TF2 变换，用标定参数做几何变换**
- 原因：雷达固定安装，URDF 中的 x/y/yaw 永不改变，三角函数即可完成变换。省掉 TF2 的 Buffer 查找和异步等待。
- 需要回答的核心问题："不做TF转换可以确认左右雷达扫描出的通行区域和车辆的位置信息吗？"——答案是可以，因为 URDF 提供了精确的安装参数。

**决策3：用角度扇区而非笛卡尔坐标描述可通行空间**
- 原因：避障决策本质是"往哪个方向转"，角度扇区天然对应转向方向，比 (x,y) 坐标更直接。
