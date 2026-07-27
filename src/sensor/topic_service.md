# Sensor 模块 Topic 和服务文档

本文档详细说明了 sensor 模块中使用的所有 ROS2 topic、服务(Service)、QoS 策略以及使用场景。

## 目录

- [Topic 列表](#topic列表)
  - [发布 Topic](#发布topic)
  - [订阅 Topic](#订阅topic)
- [服务(Service)列表](#服务service列表)
  - [提供的服务](#提供的服务)
  - [使用的服务客户端](#使用的服务客户端)
- [QoS 策略说明](#qos策略说明)
- [使用场景说明](#使用场景说明)

---

## Topic 列表

### 发布 Topic

#### 1. GPS 数据

- **Topic 名称**: `/sensor/gps/data`
- **消息类型**: `sensor_msgs::msg::NavSatFix`
- **QoS 策略**: 队列深度 30
- **发布组件**: `sensor::gps::GpsComponent`
- **使用场景**: 发布 GPS 定位数据，包括经纬度、海拔高度等信息，用于车辆定位和导航
- **发布频率**: 根据 GPS 模块数据更新频率（通常 1-10Hz）

#### 2. IMU 数据

- **Topic 名称**: `/sensor/imu/data`
- **消息类型**: `sensor_msgs::msg::Imu`
- **QoS 策略**: 队列深度 10
- **发布组件**: `sensor::imu::ImuComponent`
- **使用场景**: 发布惯性测量单元数据，包括角速度、线性加速度、姿态四元数等，用于姿态估计和运动控制
- **发布频率**: 10Hz（可配置参数 `publish_rate`）

#### 3. IMU 温度

- **Topic 名称**: `imu/temperature`
- **消息类型**: `sensor_msgs::msg::Temperature`
- **QoS 策略**: 队列深度 10
- **发布组件**: `sensor::imu::ImuComponent`
- **使用场景**: 发布 IMU 温度数据，用于温度监控和设备健康检查
- **发布频率**: 10Hz（与 IMU 数据同步）

#### 4. 手柄控制指令

- **Topic 名称**: `/handle/control`
- **消息类型**: `geometry_msgs::msg::Twist`
- **QoS 策略**: 队列深度 10
- **发布组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 发布从 CAN 总线接收的手柄控制指令，转换为 ROS2 标准速度控制消息，供上层控制模块使用
- **发布频率**: 根据 CAN 数据接收频率（通常 50-100Hz）

#### 5. 车辆速度信息

- **Topic 名称**: `/car/speed`
- **消息类型**: `interface::msg::Control`
- **QoS 策略**: 队列深度 10
- **发布组件**: `sensor::chassis::CanAdapter`
- **使用场景**: 发布车辆速度、档位等底盘信息
- **发布频率**: 根据 CAN 数据更新频率

#### 6. 车辆速度信息(带时间戳)

- **Topic 名称**: `/wheel/twist`
- **消息类型**: `geometry_msgs::msg::TwistStamped`
- **QoS 策略**: 队列深度 10
- **发布组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 发布带时间戳的车辆速度信息，包括线速度和角速度，用于里程计和定位
- **发布频率**: 根据 CAN 数据接收频率

#### 7. 编码器数据

- **Topic 名称**: `/car/encoder`
- **消息类型**: `interface::msg::Encoder`
- **QoS 策略**: 队列深度 10
- **发布组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 发布左右轮编码器圈数，用于里程计和位置估计
- **消息字段**:
  - `left_wheel_count`: 左轮编码器圈数
  - `right_wheel_count`: 右轮编码器圈数
- **发布频率**: 根据 CAN 数据接收频率

#### 8. 轮式里程计

- **Topic 名称**: `/sensor/wheel_odom`
- **消息类型**: `nav_msgs::msg::Odometry`
- **QoS 策略**: 队列深度 30
- **发布组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 发布基于左右轮编码器积分的里程计数据，包括平面位姿（x、y、yaw）及线速度、角速度，用于车辆相对定位、导航与传感器融合
- **消息字段**:
  - `header.frame_id`: 里程计坐标系（默认 `odom`）
  - `child_frame_id`: 车体坐标系（默认 `base_link`）
  - `pose.pose`: 平面位置与姿态（四元数，仅绕 Z 轴）
  - `twist.twist`: 线速度 `linear.x`、角速度 `angular.z`（EMA 滤波）
  - `pose.covariance` / `twist.covariance`: 时变协方差
- **配置参数**（节点 `handle`）:
  - `wheel_odom_wheel_radius_m`: 轮半径（m），默认 0.126
  - `wheel_odom_wheel_separation_m`: 轮距（m），默认 0.612
  - `wheel_odom_counts_per_revolution`: 每圈编码器计数，默认 278511
  - `wheel_odom_frame_id` / `wheel_odom_child_frame_id`: 坐标系名称
- **发布频率**: 与 CAN 编码器帧（`0x21a`）接收频率一致

#### 9. 倒车状态

- **Topic 名称**: `/sensor/backup/car`
- **消息类型**: `interface::msg::BackupCar`
- **QoS 策略**: 队列深度 10
- **发布组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 发布车辆倒车状态，用于倒车摄像头和倒车安全控制
- **消息字段**:
  - `is_backup_car`: 是否倒车 (true=倒车, false=前进)
- **发布频率**: 状态变化时发布

#### 10. 车辆信息

- **Topic 名称**: `/vlago/car/info`
- **消息类型**: `interface::msg::CarInfo`
- **QoS 策略**: 队列深度 10
- **发布组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 发布车辆综合信息，包括速度、电量、档位、按键状态等，供上层应用和 UI 显示使用
- **消息字段**:
  - `vx`: 前进速度
  - `w`: 旋转速度
  - `speed`: 速度状态
  - `electricquantity`: 电量百分比
  - `remainingmileage`: 剩余里程(km)
  - `gear`: 档位 (1-3)
  - `helpcar`: 助行按键状态
  - `sos`: 紧急呼叫状态
  - `turnswitch`: 转向灯状态
  - `lighttype`: 灯光类型
  - `lockswitch`: 锁车状态
  - `health`: 健康计状态
  - `getcharging`: 充电状态
  - `speedkmh`: 速度(km/h)
  - `left_brakests`: 左刹车状态
  - `right_brakests`: 右刹车状态
- **发布频率**: 1Hz（定时发布）

#### 11. 健康计启动信号

- **Topic 名称**: `/sensor/handle/control`
- **消息类型**: `interface::msg::HealthStart`
- **QoS 策略**: 队列深度 10
- **发布组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 发布健康计采集启动/停止信号，触发健康数据采集
- **消息字段**:
  - `status`: true=开始采集, false=停止采集
- **发布频率**: 按键触发时发布

#### 12. 减速带告警

- **Topic 名称**: `/speed_bump/status`
- **消息类型**: `std_msgs::msg::Int32`
- **QoS 策略**: 队列深度 10
- **发布组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 发布减速带检测告警，触发减速控制和安全提醒
- **消息值**: 1=检测到减速带

#### 13. 下坡告警

- **Topic 名称**: `/downhill/status`
- **消息类型**: `std_msgs::msg::Int32`
- **QoS 策略**: 队列深度 10
- **发布组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 发布下坡检测告警，触发速度限制和安全控制
- **消息值**: 1=检测到下坡

#### 14. 上坡告警

- **Topic 名称**: `/uphill/status`
- **消息类型**: `std_msgs::msg::Int32`
- **QoS 策略**: 队列深度 10
- **发布组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 发布上坡检测告警，触发动力调整和安全控制
- **消息值**: 1=检测到上坡

#### 15. 舞蹈秀启动

- **Topic 名称**: `/scene/dance/key`
- **消息类型**: `std_msgs::msg::Int32`
- **QoS 策略**: 队列深度 10
- **发布组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 发布舞蹈秀按键触发信号，启动舞蹈场景模式
- **消息值**: 1=启动, 0=停止

#### 16. 摇杆数据

- **Topic 名称**: `/sensor/joystick`
- **消息类型**: `interface::msg::Joystick`
- **QoS 策略**: 队列深度 10
- **发布组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 发布摇杆操作数据，用于手动控制和调试
- **消息字段**:
  - `rocker_x`: X 轴摇杆值 (-100 到 100)
  - `rocker_y`: Y 轴摇杆值 (-100 到 100)
  - `rocker_direction`: 摇杆方向 (0-360 度)
  - `rocker_angle`: 摇杆角度 (-180 到 180 度)

#### 17. H264 视频流

- **Topic 名称**: `/sensor/backup/camera/h264`
- **消息类型**: `ffmpeg_image_transport_msgs::msg::FFMPEGPacket`
- **QoS 策略**: 队列深度 1, best_effort 模式
- **发布组件**: `sensor::camera::H264UVC`
- **使用场景**: 发布 H264 编码的视频流数据，用于实时视频显示和录制
- **QoS 说明**: 使用 best_effort 模式适合视频流传输，允许丢帧保证实时性
- **发布频率**: 视频帧率（通常 25-30fps）

#### 18. CAN 数据帧

- **Topic 名称**: `/can/frames`
- **消息类型**: `can_msgs::msg::Frame`
- **QoS 策略**: 队列深度 100
- **发布组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 发布原始 CAN 总线数据帧，供数据录制和调试使用
- **QoS 说明**: 使用较大的队列深度以应对高频 CAN 数据（通常 500-1000Hz）
- **消息字段**:
  - `id`: CAN 帧 ID
  - `dlc`: 数据长度
  - `data`: 数据内容（最多 8 字节）
  - `is_extended`: 是否为扩展帧
  - `is_rtr`: 是否为远程帧
  - `is_error`: 是否为错误帧

#### 19. 错误事件

- **Topic 名称**: `/error_events`
- **消息类型**: `interface::msg::HexData`
- **QoS 策略**: 队列深度 10
- **发布组件**: `sensor::Global`
- **使用场景**: 发布系统错误事件，包括 IMU 错误、CAN 总线错误等，用于错误监控和诊断
- **发布频率**: 错误发生时发布

---

### 订阅 Topic

#### 1. 导航控制指令

- **Topic 名称**: `/nav/control`
- **消息类型**: `geometry_msgs::msg::Twist`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `sensor::chassis::CanAdapter`
- **使用场景**: 接收来自导航系统(nav2)的速度控制指令，转发到底盘 CAN 总线执行
- **处理逻辑**: 接收导航指令后转换为 CAN 总线命令发送给底盘控制器

#### 2. 健康计数据

- **Topic 名称**: `/sensor/health/data`
- **消息类型**: `interface::msg::Health`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `sensor::chassis::CanAdapter`
- **使用场景**: 接收来自上控的健康计采集数据（心率、血氧、血压等），通过 CAN 总线发送给手柄显示
- **消息字段**:
  - `lowpressure`: 低压
  - `hightension`: 高压
  - `heartrate`: 心率
  - `bloodoxygen`: 血氧
  - `microcirculation`: 微循环
  - `acdata`: 心律波形数据

#### 3. 充电状态

- **Topic 名称**: `/vlago/recharge/status`
- **消息类型**: `interface::msg::RechargeStatus`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 接收充电桩控制模块的充电状态信息，通过 CAN 总线发送给手柄显示
- **消息字段**:
  - `status`: 充电状态
    - 1: 搜索充电站
    - 2: 未找到充电站
    - 3: 找到充电站并前往
    - 4: 检测到障碍物
    - 5: 开始充电
    - 6: 停止充电

#### 4. 减速带检测结果

- **Topic 名称**: `/speedbump_marker_array`
- **消息类型**: `visualization_msgs::msg::MarkerArray`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 接收感知模块检测到的减速带位置信息，触发减速和安全控制

#### 5. 坡道检测状态

- **Topic 名称**: `/perception/podao_detect_status`
- **消息类型**: `std_msgs::msg::Int32`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `sensor::chassis::HandleCanAdapter`
- **使用场景**: 接收感知模块的坡道检测状态，触发相应的速度限制和安全控制
- **消息值说明**:
  - 100: 正常
  - 102: 缓坡
  - 103: 上坡
  - 104: 下坡

#### 6. 窄道结束信号

- **Topic 名称**: `/narrow/end`
- **消息类型**: `std_msgs::msg::Empty`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `sensor::chassis::CanAdapter`
- **使用场景**: 接收窄道场景结束信号，重置相关状态

#### 7. 音频播放指令

- **Topic 名称**: `/vlago/audio/play`
- **消息类型**: `std_msgs::msg::Int32`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `sensor::chassis::CanAdapter`
- **使用场景**: 接收音频播放指令，通过 CAN 总线控制手柄播放音频

#### 8. 倒车信息

- **Topic 名称**: `/sensor/backup/car`
- **消息类型**: `interface::msg::BackupCar`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `sensor::camera::CameraComponent`
- **使用场景**: 接收倒车状态信息，控制摄像头录制（当前已注释）

---

## 服务(Service)列表

### 提供的服务

#### 1. 档位控制服务

- **服务名称**: `/sensor/vlago/gear`
- **服务类型**: `interface::srv::GearsControls`
- **提供组件**: `sensor::chassis::ChassisComponent`
- **使用场景**: 提供档位、助行、紧急呼叫、锁车等控制功能
- **请求字段**:
  - `gear`: 档位 (1-3)
  - `helpcar`: 助行按键状态
  - `sos`: 紧急呼叫状态
  - `lock`: 锁车状态
- **响应字段**:
  - `code`: 返回码 (0=成功)

#### 2. TFT 屏幕控制服务

- **服务名称**: `/sensor/vlago/lamp_language_screen`
- **服务类型**: `interface::srv::TftControls`
- **提供组件**: `sensor::chassis::ChassisComponent`
- **使用场景**: 控制手柄 TFT 屏幕的转向灯和灯光显示
- **请求字段**:
  - `turns_switch`: 转向灯开关状态
  - `light_type`: 灯光类型
- **响应字段**:
  - `code`: 返回码 (0=成功)

#### 3. 数据录制控制服务

- **服务名称**:
  - `/sensor/recorder/start` - 开始录制
  - `/sensor/recorder/stop` - 停止录制
  - `/sensor/recorder/lidar/start` - 开始雷达数据录制
  - `/sensor/recorder/lidar/stop` - 停止雷达数据录制
- **服务类型**: `std_srvs::srv::Empty`
- **提供组件**: `sensor::recorder::SensorDataRecorder`
- **使用场景**: 控制传感器数据录制功能，包括摄像头、IMU、GPS、CAN 和雷达数据

---

### 使用的服务客户端

#### 1. 模式使能服务

- **服务名称**: `/vlago/mode/enable`
- **服务类型**: `interface::srv::KeyValueInt`
- **使用组件**: `sensor::chassis::CanAdapter`
- **使用场景**: 调用决策管理模块的模式使能服务，开启/关闭辅助模式、巡航模式等
- **请求字段**:
  - `name`: 模式名称 ("auxiliary", "cruise"等)
  - `value`: 模式值 (0=关闭, 1=开启)
- **响应字段**:
  - `code`: 返回码
  - `message`: 返回消息

---

## QoS 策略说明

### QoS 配置原则

1. **实时性要求高的数据**（如控制指令）

   - 使用可靠的 QoS 策略
   - 队列深度适中（通常 10）
   - 示例: `/handle/control`, `/nav/control`

2. **高频数据流**（如 CAN 数据、视频流）

   - 使用 best_effort 模式允许丢帧
   - 队列深度较大（50-100）
   - 示例: `/can/frames` (队列深度 100), `/sensor/backup/camera/h264` (best_effort)

3. **状态和配置数据**（如车辆信息）

   - 使用可靠的 QoS 策略
   - 队列深度较小（通常 10）
   - 示例: `/vlago/car/info`, `/sensor/handle/control`

4. **传感器数据**（如 GPS、IMU）
   - 根据更新频率设置队列深度
   - GPS 使用较大队列深度(30)以应对可能的延迟
   - IMU 使用标准队列深度(10)

### 常见 QoS 配置

| QoS 配置 | 队列深度 | 可靠性      | 使用场景           |
| -------- | -------- | ----------- | ------------------ |
| 标准配置 | 10       | Reliable    | 控制指令、状态信息 |
| 高频数据 | 30-100   | Reliable    | GPS、CAN 数据      |
| 视频流   | 1        | Best Effort | H264 视频流        |

---

## 使用场景说明

### 场景 1: 车辆控制流程

1. **手柄控制**: 用户操作手柄 → CAN 总线接收 → `/handle/control`发布 → 控制执行模块订阅
2. **导航控制**: 导航模块规划路径 → `/nav/control`发布 → Sensor 模块订阅 → 转发到 CAN 总线
3. **速度反馈**: CAN 总线接收速度反馈 → `/wheel/twist`发布 → 定位和导航模块使用

### 场景 2: 传感器数据采集

1. **GPS 定位**: GPS 模块 → `/sensor/gps/data`发布 → 定位和导航模块订阅
2. **IMU 姿态**: IMU 模块 → `/sensor/imu/data`发布 → 定位和融合模块订阅
3. **轮式里程计**: CAN 编码器帧 → `/sensor/wheel_odom`发布 → 定位、导航与融合模块订阅
4. **视频录制**: 摄像头 → H264 编码 → `/sensor/backup/camera/h264`发布 → 录制和显示模块订阅

### 场景 3: 安全控制

1. **减速带检测**: 感知模块检测 → `/speedbump_marker_array`发布 → Sensor 模块订阅 → 触发减速 → `/speed_bump/status`发布
2. **坡道检测**: 感知模块检测 → `/perception/podao_detect_status`发布 → Sensor 模块订阅 → 触发速度限制 → `/downhill/status`或`/uphill/status`发布
3. **倒车检测**: CAN 总线检测倒车 → `/sensor/backup/car`发布 → 触发倒车摄像头和安全控制

### 场景 4: 健康计集成

1. **启动采集**: 用户按健康计按键 → CAN 总线接收 → `/sensor/handle/control`发布 → 健康计模块订阅
2. **数据上传**: 健康计模块采集 → `/sensor/health/data`发布 → Sensor 模块订阅 → 转发到 CAN 总线显示

### 场景 5: 充电管理

1. **充电状态**: 充电桩控制模块 → `/vlago/recharge/status`发布 → Sensor 模块订阅 → 转发到 CAN 总线显示
2. **电量监控**: CAN 总线接收电量信息 → 车辆信息整合 → `/vlago/car/info`发布 → UI 和云端订阅

### 场景 6: 数据录制

1. **开始录制**: 通过服务调用 `/sensor/recorder/start` → 开始录制摄像头、IMU、GPS、CAN 数据
2. **数据存储**: 所有传感器数据写入 rosbag2 文件，支持后续回放和分析

---

## 注意事项

1. **CAN 总线频率**: CAN 总线数据频率很高（500-1000Hz），订阅`/can/frames`时注意处理性能
2. **视频流带宽**: H264 视频流数据量较大，注意网络带宽限制
3. **QoS 兼容性**: 发布和订阅的 QoS 策略需要兼容，否则可能无法建立连接
4. **时间同步**: 多个传感器数据需要考虑时间同步，使用 ROS2 的时间戳
5. **错误处理**: 传感器异常时会发布错误事件到`/error_events`，需要监控和处理

---

## 更新日志
