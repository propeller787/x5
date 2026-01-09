# Scene 模块 Topic 和服务文档

本文档详细说明了 scene_node 和 scene_services 模块中使用的所有 ROS2 topic、服务(Service)、QoS 策略以及使用场景。

## 目录

- [Topic 列表](#topic列表)
  - [Scene Node 发布 Topic](#scene-node-发布-topic)
  - [Scene Node 订阅 Topic](#scene-node-订阅-topic)
  - [Scene Services 发布 Topic](#scene-services-发布-topic)
  - [Scene Services 订阅 Topic](#scene-services-订阅-topic)
- [服务(Service)列表](#服务service列表)
  - [Scene Node 提供的服务](#scene-node-提供的服务)
  - [Scene Services 提供的服务](#scene-services-提供的服务)
  - [使用的服务客户端](#使用的服务客户端)
- [模式管理说明](#模式管理说明)
- [QoS 策略说明](#qos策略说明)
- [使用场景说明](#使用场景说明)

---

## Topic 列表

### Scene Node 发布 Topic

#### 1. 车辆控制指令

- **Topic 名称**: `/nav/control`
- **消息类型**: `geometry_msgs::msg::Twist`
- **QoS 策略**: 队列深度 10
- **发布组件**: `SceneNode`
- **使用场景**: 发布经过安全干预的车辆控制指令，供 sensor 模块订阅并转发到底盘
- **发布频率**: 根据控制指令生成频率（通常 10-50Hz）
- **控制逻辑**:
  - **辅助模式 (Auxiliary)**: 通过决策器处理，使用执行器进行安全干预
  - **标准模式 (Standard)**: 关闭执行器，直接透传摇杆控制指令
  - **巡航模式 (Cruise)**: 记录最大线速度，持续保持该速度，角速度随摇杆变化

#### 2. 左雷达聚类结果

- **Topic 名称**: `/scene_auxiliary/left_laser/clusters`
- **消息类型**: `sensor_msgs::msg::PointCloud2`
- **QoS 策略**: 队列深度 10
- **发布组件**: `Clustering`
- **使用场景**: 发布左雷达检测到的障碍物聚类点云，用于可视化调试

#### 3. 右雷达聚类结果

- **Topic 名称**: `/scene_auxiliary/right_laser/clusters`
- **消息类型**: `sensor_msgs::msg::PointCloud2`
- **QoS 策略**: 队列深度 10
- **发布组件**: `Clustering`
- **使用场景**: 发布右雷达检测到的障碍物聚类点云

#### 4. 左倾斜雷达聚类结果

- **Topic 名称**: `/scene_auxiliary/left_tilt/clusters`
- **消息类型**: `sensor_msgs::msg::PointCloud2`
- **QoS 策略**: 队列深度 10
- **发布组件**: `Clustering`
- **使用场景**: 发布左倾斜雷达检测到的障碍物聚类点云

#### 5. 右倾斜雷达聚类结果

- **Topic 名称**: `/scene_auxiliary/right_tilt/clusters`
- **消息类型**: `sensor_msgs::msg::PointCloud2`
- **QoS 策略**: 队列深度 10
- **发布组件**: `Clustering`
- **使用场景**: 发布右倾斜雷达检测到的障碍物聚类点云

#### 6. 后雷达聚类结果

- **Topic 名称**: `/laser/back_cluster`
- **消息类型**: `sensor_msgs::msg::PointCloud2`
- **QoS 策略**: 队列深度 10
- **发布组件**: `Clustering`
- **使用场景**: 发布后雷达检测到的障碍物聚类点云

#### 7. 辅助模式告警（前向）

- **Topic 名称**: `/vlago/auxiliary/alarm`
- **消息类型**: `interface::msg::AuxiliaryAlarm`
- **QoS 策略**: 队列深度 10
- **发布组件**: `DecisionMaker`
- **使用场景**: 发布前向安全告警，包括碰撞风险、紧急制动等
- **发布频率**: 告警触发时发布

#### 8. 辅助模式告警（倒车）

- **Topic 名称**: `/vlago/backcar/alarm`
- **消息类型**: `interface::msg::AuxiliaryAlarm`
- **QoS 策略**: 队列深度 10
- **发布组件**: `DecisionMaker`
- **使用场景**: 发布倒车安全告警
- **发布频率**: 告警触发时发布

---

### Scene Node 订阅 Topic

#### 1. 左雷达扫描数据

- **Topic 名称**: `/scan/left`
- **消息类型**: `sensor_msgs::msg::LaserScan`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `LidarDriver`
- **使用场景**: 接收左雷达扫描数据，转换为点云进行处理

#### 2. 右雷达扫描数据

- **Topic 名称**: `/scan/right`
- **消息类型**: `sensor_msgs::msg::LaserScan`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `LidarDriver`
- **使用场景**: 接收右雷达扫描数据，转换为点云进行处理

#### 3. 左倾斜雷达扫描数据

- **Topic 名称**: `/scan/left_tilt`
- **消息类型**: `sensor_msgs::msg::LaserScan`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `LidarDriver`
- **使用场景**: 接收左倾斜雷达扫描数据

#### 4. 右倾斜雷达扫描数据

- **Topic 名称**: `/scan/right_tilt`
- **消息类型**: `sensor_msgs::msg::LaserScan`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `LidarDriver`
- **使用场景**: 接收右倾斜雷达扫描数据

#### 5. 后雷达扫描数据

- **Topic 名称**: `/scan/back`
- **消息类型**: `sensor_msgs::msg::LaserScan`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `LidarDriver`
- **使用场景**: 接收后雷达扫描数据

#### 6. 左倾斜点云数据

- **Topic 名称**: `/perception/left_tilt/pointcloud`
- **消息类型**: `sensor_msgs::msg::PointCloud2`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `LidarDriver`
- **使用场景**: 接收左倾斜点云数据，直接进行聚类处理

#### 7. 右倾斜点云数据

- **Topic 名称**: `/perception/right_tilt/pointcloud`
- **消息类型**: `sensor_msgs::msg::PointCloud2`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `LidarDriver`
- **使用场景**: 接收右倾斜点云数据，直接进行聚类处理

#### 8. 手柄控制指令

- **Topic 名称**: `/handle/control`
- **消息类型**: `geometry_msgs::msg::Twist`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `VehicleInterface`
- **使用场景**: 接收手柄控制指令，用于辅助模式和标准模式
- **处理逻辑**: 根据当前模式（辅助/标准/巡航）进行不同处理

#### 9. 车辆速度反馈

- **Topic 名称**: `/wheel/twist`
- **消息类型**: `geometry_msgs::msg::TwistStamped`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `VehicleInterface`
- **使用场景**: 接收车辆速度反馈，用于目标跟踪和速度控制

#### 10. 车辆信息

- **Topic 名称**: `/vlago/car/info`
- **消息类型**: `interface::msg::CarInfo`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `DecisionMaker`
- **使用场景**: 接收车辆综合信息，用于决策和控制

---

### Scene Services 发布 Topic

#### 1. 场景操作信息

- **Topic 名称**: `/service/operation/scenario`
- **消息类型**: `interface::msg::VlagoScene`
- **QoS 策略**: 队列深度 10
- **发布组件**: `SceneServices`
- **使用场景**: 发布场景操作信息，包括场景类型和开关状态
- **发布频率**: 模式切换时发布
- **消息字段**:
  - `scene`: 场景类型 (0=进电梯, 1=出电梯, 2=去花园, 3=回家)
  - `onoff`: 开关状态 (0=关闭, 1=开始)

#### 2. 模式状态报告

- **Topic 名称**: `/vlago/mode/status`
- **消息类型**: `interface::msg::ReportMode`
- **QoS 策略**: 队列深度 10
- **发布组件**: `SceneServices`
- **使用场景**: 报告模式切换状态，包括开启的模式和关闭的模式
- **发布频率**: 模式切换时发布
- **消息字段**:
  - `header`: 消息头
  - `mode_list`: 模式状态列表
    - `name`: 模式名称（"auxiliary", "standard", "cruise" 等）
    - `status`: 状态 (0=关闭, 1=开启)

#### 3. 充电状态

- **Topic 名称**: `/vlago/recharge/status`
- **消息类型**: `interface::msg::RechargeStatus`
- **QoS 策略**: 队列深度 10
- **发布组件**: `SceneServices`
- **使用场景**: 发布充电状态信息，供 sensor 模块订阅并转发到 CAN 总线
- **发布频率**: 充电状态变化时发布
- **状态值说明**:
  - 1: 搜索充电站
  - 2: 未找到充电站
  - 3: 找到充电站并前往
  - 4: 检测到障碍物
  - 5: 开始充电
  - 6: 停止充电

#### 4. 当前模式字符串

- **Topic 名称**: `/scene_services/mode`
- **消息类型**: `std_msgs::msg::String`
- **QoS 策略**: 队列深度 10
- **发布组件**: `SceneServices`
- **使用场景**: 定期发布当前模式字符串，用于状态监控
- **发布频率**: 1Hz（定时发布）
- **模式字符串值**: "Mode_zero", "auxiliary", "narrow", "standard", "cruise", "dance", "recharge", "followme", "autoindoor"

#### 5. 自动驾驶导航目标

- **Topic 名称**: `/yisbot/goto`
- **消息类型**: `std_msgs::msg::Int16`
- **QoS 策略**: 队列深度 10
- **发布组件**: `SceneServices`
- **使用场景**: 发布自动驾驶导航目标 ID，供导航模块订阅
- **发布频率**: 导航指令触发时发布

#### 6. 取消自动驾驶导航

- **Topic 名称**: `/yisbot/cancel`
- **消息类型**: `std_msgs::msg::Empty`
- **QoS 策略**: 队列深度 10
- **发布组件**: `SceneServices`
- **使用场景**: 发布取消导航信号
- **发布频率**: 取消导航时发布

#### 7. 充电控制指令

- **Topic 名称**: `/recharge/ctrl`
- **消息类型**: `std_msgs::msg::String`
- **QoS 策略**: 队列深度 10
- **发布组件**: `SceneServices`
- **使用场景**: 发布充电控制指令（"start" 或 "stop"）
- **发布频率**: 充电模式切换时发布

---

### Scene Services 订阅 Topic

#### 1. 充电反馈状态

- **Topic 名称**: `/recharge/feed`
- **消息类型**: `std_msgs::msg::Int32`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `SceneServices`
- **使用场景**: 接收充电模块的反馈状态，更新充电模式状态
- **状态值说明**:
  - 2: 未找到充电站 → 发布状态 2，关闭充电模式
  - 3: 找到充电站并前往 → 发布状态 3
  - 4: 检测到障碍物 → 发布状态 4，关闭充电模式
  - 5: 开始充电 → 发布状态 5，关闭充电模式
  - 6: 停止充电 → 发布状态 6，关闭充电模式

#### 2. 自动驾驶签到状态

- **Topic 名称**: `/vlago/auto_drive/checkin`
- **消息类型**: `std_msgs::msg::Bool`
- **QoS 策略**: 队列深度 10
- **订阅组件**: `SceneServices`
- **使用场景**: 接收自动驾驶签到状态，用于判断是否可以启动自动驾驶导航

---

## 服务(Service)列表

### Scene Node 提供的服务

#### 1. 辅助模式控制服务

- **服务名称**: `/scene/auxiliary`
- **服务类型**: `interface::srv::SceneControls`
- **提供组件**: `SceneNode`
- **使用场景**: 开启/关闭辅助模式（安全模式，使用执行器进行安全干预）
- **请求字段**:
  - `start`: 0=关闭, 1=开启
- **响应字段**:
  - `code`: 0=成功, 1=失败
  - `message`: 响应消息

#### 2. 标准模式控制服务

- **服务名称**: `/scene/standard`
- **服务类型**: `interface::srv::SceneControls`
- **提供组件**: `SceneNode`
- **使用场景**: 开启/关闭标准模式（关闭执行器，直接透传摇杆控制）
- **请求字段**:
  - `start`: 0=关闭, 1=开启
- **响应字段**:
  - `code`: 0=成功, 1=失败
  - `message`: 响应消息

#### 3. 巡航模式控制服务

- **服务名称**: `/scene/cruise`
- **服务类型**: `interface::srv::SceneControls`
- **提供组件**: `SceneNode`
- **使用场景**: 开启/关闭巡航模式（记录最大线速度，保持速度巡航）
- **请求字段**:
  - `start`: 0=关闭, 1=开启
- **响应字段**:
  - `code`: 0=成功, 1=失败
  - `message`: 响应消息
- **巡航模式特性**:
  - 自动记录摇杆设置的最大前进速度
  - 角速度随摇杆实时变化
  - 角速度过大时自动降低线速度防止倾倒
  - 检测到后退速度时自动退出
  - 8 秒内未收到大于 0 的速度自动退出

---

### Scene Services 提供的服务

#### 1. 充电控制服务

- **服务名称**: `/vlago/auto_charge`
- **服务类型**: `interface::srv::RechargeControls`
- **提供组件**: `SceneServices`
- **使用场景**: 开启/关闭充电模式，触发自动充电流程
- **请求字段**:
  - `on_off`: 0=关闭, 1=开启
- **响应字段**:
  - `code`: 0=成功, 非零=失败
  - `type`: 1=开始回充, 0=取消关闭回充
  - `message`: 失败描述

#### 2. 跟随模式控制服务

- **服务名称**: `/manage/followme`
- **服务类型**: `interface::srv::SceneControls`
- **提供组件**: `SceneServices`
- **使用场景**: 开启/关闭跟随模式
- **请求字段**:
  - `start`: 0=关闭, 1=开启
- **响应字段**:
  - `code`: 0=成功, 1=失败
  - `message`: 响应消息
- **处理逻辑**:
  - 开启时：关闭其他模式，启动跟随模式相关程序
  - 关闭时：关闭跟随模式，启动辅助驾驶模式

#### 3. 模式使能配置服务

- **服务名称**: `/vlago/setting`
- **服务类型**: `interface::srv::ModeEnable`
- **提供组件**: `SceneServices`
- **使用场景**: 批量配置多个模式的使能状态
- **请求字段**:
  - `narrow`: 窄道模式使能
  - `auxiliary`: 辅助模式使能
  - `followme`: 跟随模式使能
- **响应字段**:
  - `code`: 0=成功
  - `message`: 响应消息
- **处理逻辑**:
  - 如果禁用某个模式且该模式当前处于激活状态，则切换到默认模式
  - 如果启用辅助模式且当前为标准模式，则切换到辅助模式
  - 如果禁用辅助模式且当前为辅助模式，则切换到标准模式

#### 4. 单模式使能控制服务

- **服务名称**: `/vlago/mode/enable`
- **服务类型**: `interface::srv::KeyValueInt`
- **提供组件**: `SceneServices`
- **使用场景**: 单独控制某个模式的使能状态
- **请求字段**:
  - `name`: 模式名称（"auxiliary", "narrow", "cruise", "followme" 等）
  - `value`: 0=禁用, 1=启用
- **响应字段**:
  - `code`: 0=成功
  - `message`: 响应消息
- **支持的模式名称**:
  - "auxiliary": 辅助模式
  - "narrow": 窄道模式
  - "cruise": 巡航模式
  - "followme": 跟随模式

#### 5. 自动驾驶模式控制服务

- **服务名称**: `/vlago/autoDriveMode`
- **服务类型**: `interface::srv::SceneControls`
- **提供组件**: `SceneServices`
- **使用场景**: 开启/关闭自动驾驶模式
- **请求字段**:
  - `start`: 0=关闭, 1=开启
- **响应字段**:
  - `code`: 0=成功
  - `message`: 响应消息

#### 6. 启动自动驾驶导航服务

- **服务名称**: `/vlago/startAutoDrive`
- **服务类型**: `interface::srv::AutoindoorNav`
- **提供组件**: `SceneServices`
- **使用场景**: 启动自动驾驶导航到指定目标点
- **请求字段**:
  - `start`: 1（导航 goto）
  - `id`: 目的地 ID
- **响应字段**:
  - `code`: 0=成功, 1=未签到不可用, -1=其他模式锁定
  - `message`: 响应消息
- **前置条件**:
  - 必须已签到（`autobot_indoor_checkin_` 为 true）
  - 当前模式不能是锁定模式（Narrow, Cruise, Dance, Followme）

#### 7. 停止自动驾驶导航服务

- **服务名称**: `/vlago/stopAutoDrive`
- **服务类型**: `interface::srv::AutoindoorNav`
- **提供组件**: `SceneServices`
- **使用场景**: 停止/取消自动驾驶导航
- **请求字段**:
  - `start`: 0（取消 cannel）
  - `id`: 目的地 ID（未使用）
- **响应字段**:
  - `code`: 0=成功
  - `message`: 响应消息

#### 8. 窄道模式反馈控制服务

- **服务名称**: `/feed/narrow/ctrl`
- **服务类型**: `interface::srv::SceneControls`
- **提供组件**: `SceneServices`
- **使用场景**: 接收窄道模块的反馈，控制窄道模式状态
- **请求字段**:
  - `start`: 0=停止, 1=启动, 2=暂停
- **响应字段**:
  - `code`: 0=成功
  - `message`: 响应消息

#### 9. 充电反馈控制服务

- **服务名称**: `/feed/recharge/ctrl`
- **服务类型**: `interface::srv::SceneControls`
- **提供组件**: `SceneServices`
- **使用场景**: 接收充电模块的反馈，更新充电状态
- **请求字段**:
  - `start`: 状态值（2=未找到, 3=找到并前往, 4=检测到障碍物, 5=开始充电, 6=停止充电）
- **响应字段**:
  - `code`: 1=成功
  - `message`: 响应消息

#### 10. 跟随模式反馈控制服务

- **服务名称**: `/feed/follwme/ctrl`
- **服务类型**: `interface::srv::SceneControls`
- **提供组件**: `SceneServices`
- **使用场景**: 接收跟随模块的反馈
- **请求字段**:
  - `start`: 0=停止, 1=启动
- **响应字段**:
  - `code`: 1=成功
  - `message`: 响应消息

#### 11. 舞蹈模式反馈控制服务

- **服务名称**: `/feed/dance/ctrl`
- **服务类型**: `interface::srv::SceneControls`
- **提供组件**: `SceneServices`
- **使用场景**: 接收舞蹈模块的反馈，控制舞蹈模式状态
- **请求字段**:
  - `start`: 0=停止, 1=启动, 2=暂停
- **响应字段**:
  - `code`: 0=成功
  - `message`: 响应消息

#### 12. 巡航模式反馈控制服务

- **服务名称**: `/manage/cruise`
- **服务类型**: `interface::srv::SceneControls`
- **提供组件**: `SceneServices`
- **使用场景**: 接收巡航模式的反馈，控制巡航模式状态
- **请求字段**:
  - `start`: 0=停止, 1=启动, 2=暂停
- **响应字段**:
  - `code`: 0=成功
  - `message`: 响应消息

---

### 使用的服务客户端

#### 1. Scene Services 调用的服务

Scene Services 作为服务客户端调用 Scene Node 提供的服务：

- **服务名称**: `/scene/auxiliary` - 调用辅助模式控制
- **服务名称**: `/scene/standard` - 调用标准模式控制
- **服务名称**: `/scene/cruise` - 调用巡航模式控制

#### 2. Scene Node 使用的服务客户端

- **服务名称**: `/vlago/mode/enable`
- **服务类型**: `interface::srv::KeyValueInt`
- **使用组件**: `SceneNode`
- **使用场景**: 巡航模式自动关闭时调用此服务通知决策管理模块

---

## 模式管理说明

### 支持的模式类型

Scene Services 支持以下模式：

- **Mode_zero**: 初始模式
- **Mode_Auxiliary**: 辅助模式（安全模式）
- **Mode_Narrow**: 窄道模式
- **Mode_Standard**: 标准模式
- **Mode_Cruise**: 巡航模式
- **Mode_Dance**: 舞蹈模式
- **Mode_Recharge**: 充电模式
- **Mode_Followme**: 跟随模式
- **Mode_Autoindoor**: 自动驾驶模式

### 模式优先级和互斥规则

1. **锁定模式**（不能被其他模式切换出去）:

   - `Mode_Dance`: 舞蹈模式
   - `Mode_Followme`: 跟随模式
   - `Mode_Recharge`: 充电模式
   - `Mode_Autoindoor`: 自动驾驶模式

2. **默认模式**:

   - 如果辅助模式启用，默认模式为 `Mode_Auxiliary`
   - 如果辅助模式禁用，默认模式为 `Mode_Standard`

3. **模式切换规则**:
   - 切换到新模式时，先停止旧模式，再启动新模式
   - 如果新模式的使能被禁用，切换会失败
   - 某些模式（Narrow, Dance, Followme, Autoindoor）不需要调用 scene_node 的服务，直接切换即可

### 模式使能管理

每个模式都有独立的使能状态，存储在 `mode_enable_` 数组中：

- 禁用某个模式时，如果该模式当前激活，会切换到默认模式
- 启用辅助模式时，如果当前为标准模式，会自动切换到辅助模式
- 禁用辅助模式时，如果当前为辅助模式，会自动切换到标准模式

---

## QoS 策略说明

### QoS 配置原则

1. **实时性要求高的数据**（如控制指令）

   - 使用可靠的 QoS 策略
   - 队列深度适中（通常 10）
   - 示例: `/nav/control`, `/handle/control`

2. **雷达和点云数据**

   - 使用可靠的 QoS 策略
   - 队列深度 10
   - 示例: `/scan/left`, `/scene_auxiliary/left_laser/clusters`

3. **状态和配置数据**（如模式状态、充电状态）
   - 使用可靠的 QoS 策略
   - 队列深度 10
   - 示例: `/vlago/mode/status`, `/vlago/recharge/status`

### 常见 QoS 配置

| QoS 配置 | 队列深度 | 可靠性   | 使用场景           |
| -------- | -------- | -------- | ------------------ |
| 标准配置 | 10       | Reliable | 控制指令、状态信息 |
| 雷达数据 | 10       | Reliable | 雷达扫描、点云数据 |

---

## 使用场景说明

### 场景 1: 辅助模式使用流程

1. **开启辅助模式**: 调用 `/scene/auxiliary` 服务（start=1）
2. **安全干预**: Scene Node 接收摇杆控制指令，通过决策器进行安全干预
3. **控制输出**: 经过干预的控制指令发布到 `/nav/control`
4. **关闭辅助模式**: 调用 `/scene/auxiliary` 服务（start=0）

### 场景 2: 巡航模式使用流程

1. **开启巡航模式**: 调用 `/scene/cruise` 服务（start=1）
2. **速度记录**: 系统记录用户通过摇杆设置的最大前进速度
3. **持续巡航**: 系统保持记录的最大速度，角速度随摇杆变化
4. **自动退出**: 检测到后退速度或 8 秒无速度时自动退出
5. **手动关闭**: 调用 `/scene/cruise` 服务（start=0）

### 场景 3: 充电模式流程

1. **开启充电模式**: 调用 `/vlago/auto_charge` 服务（on_off=1）
2. **搜索充电站**: 发布充电状态 1（搜索中）
3. **找到充电站**: 发布充电状态 3（前往中）
4. **开始充电**: 发布充电状态 5（开始充电），关闭充电模式
5. **状态反馈**: 通过 `/recharge/feed` topic 接收充电模块反馈

### 场景 4: 自动驾驶导航流程

1. **签到检查**: 确保 `/vlago/auto_drive/checkin` 为 true
2. **模式切换**: 如果不在自动驾驶模式，自动切换到自动驾驶模式
3. **发送导航目标**: 调用 `/vlago/startAutoDrive` 服务，指定目标 ID
4. **导航执行**: 发布 `/yisbot/goto` topic，导航模块执行导航
5. **取消导航**: 调用 `/vlago/stopAutoDrive` 服务或发布 `/yisbot/cancel` topic

### 场景 5: 模式切换流程

1. **模式切换请求**: 调用相应模式的服务或 Scene Services 的服务
2. **停止旧模式**: Scene Services 调用 scene_node 停止当前模式
3. **启动新模式**: Scene Services 调用 scene_node 启动新模式
4. **状态报告**: 发布 `/vlago/mode/status` 报告模式切换
5. **模式更新**: 更新 `/scene_services/mode` topic

### 场景 6: 雷达数据处理流程

1. **雷达数据接收**: LidarDriver 订阅雷达扫描数据 (`/scan/left`, `/scan/right` 等)
2. **点云转换**: 将激光扫描数据转换为点云格式
3. **点云聚类**: Clustering 组件对点云进行聚类处理
4. **障碍物识别**: 聚类结果发布到 `/scene_auxiliary/*/clusters` topics
5. **目标跟踪**: Tracking 组件对聚类结果进行跟踪
6. **决策制定**: DecisionMaker 进行碰撞检测和速度干预决策
7. **控制输出**: 最终控制指令发布到 `/nav/control`

---

## 注意事项

1. **模式互斥**: 某些模式（如 Dance, Followme, Recharge, Autoindoor）是锁定模式，不能从外部切换出去
2. **模式使能**: 切换模式前需要确保目标模式的使能状态为 true
3. **服务超时**: 调用 scene_node 服务时有 5 秒超时，如果服务不可用会失败
4. **线程安全**: Scene Services 使用互斥锁保护模式切换操作，避免并发问题
5. **默认模式**: 禁用辅助模式时会自动切换到标准模式，反之亦然
6. **巡航模式超时**: 巡航模式在 8 秒内未收到大于 0 的速度会自动退出
7. **充电反馈**: 充电模块通过 `/recharge/feed` topic 反馈状态，Scene Services 根据反馈自动更新充电模式状态

---

## 更新日志

- 2025-01-XX: 初始版本文档
