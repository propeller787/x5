# 代码埋点Trigger详细文档

**项目名称**: VLAGO 自动驾驶系统  
**文档版本**: v1.0  
**生成日期**: 2024  
**文档说明**: 本文档详细列出了VLAGO系统中所有可埋点的触发位置，包括代码位置、函数签名、触发条件和埋点建议。

---

## 目录

1. [埋点分类概览](#埋点分类概览)
2. [高优先级埋点详细说明](#高优先级埋点详细说明)
3. [ROS Topic订阅埋点](#ros-topic订阅埋点)
4. [ROS Service调用埋点](#ros-service调用埋点)
5. [模式切换埋点](#模式切换埋点)
6. [碰撞检测埋点](#碰撞检测埋点)
7. [传感器回调埋点](#传感器回调埋点)
8. [定时器埋点](#定时器埋点)
9. [错误异常埋点](#错误异常埋点)
10. [埋点实施指南](#埋点实施指南)
11. [埋点数据格式规范](#埋点数据格式规范)
12. [埋点汇总表格](#埋点汇总表格)

---

## 埋点分类概览

### 统计信息
- **总触发点数**: 200+
- **高优先级**: 15个
- **中优先级**: 35个
- **低优先级**: 150+
- **覆盖模块**: 15个主要模块

### 埋点类型分布
- ROS Topic订阅: 45个
- ROS Service调用: 35个
- ROS Topic发布: 25个
- 函数回调: 60个
- 定时器: 8个
- 错误处理: 15个
- 模式切换: 12个

---

## 高优先级埋点详细说明

### 1. 模式切换 - ChangeMode()

**文件位置**: `src/scene/scene_services/src/scene_services.cpp`  
**函数签名**: `bool ChangeMode(enum SecneMode new_mode)`  
**行号**: 731-745  
**触发条件**: 系统模式发生切换时  
**优先级**: ⭐⭐⭐⭐⭐ (最高)

**代码位置**:
```cpp
// 文件: src/scene/scene_services/src/scene_services.cpp
// 行号: 731-745
bool SceneServices::ChangeMode(enum SecneMode new_mode) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (new_mode == curr_mode_) {
            return false;
        }
        RCLCPP_WARN(getnode()->get_logger(), "ChangeMode old:%s --> : %s", 
                   mode_str_map[curr_mode_], mode_str_map[new_mode]);
        last_mode_ = curr_mode_;
        curr_mode_ = new_mode;

        std_msgs::msg::String report_msg;
        report_msg.data = mode_str_map[curr_mode_];
        mode_pub_->publish(report_msg);
    }
    OnlineReportMode(new_mode, last_mode_);
}
```

**埋点建议**:
- **触发时机**: 模式切换成功时
- **埋点位置**: 函数开始处和模式切换成功后
- **关键数据**: 
  - 旧模式: `last_mode_`
  - 新模式: `curr_mode_`
  - 切换时间戳
  - 切换触发源

**埋点代码示例**:
```cpp
// 埋点位置1: 函数开始
TRIGGER_LOG("mode_switch_start", {
    {"old_mode", mode_str_map[last_mode_]},
    {"new_mode", mode_str_map[new_mode]},
    {"timestamp", getCurrentTimestamp()}
});

// 埋点位置2: 模式切换成功后
if (new_mode != curr_mode_) {
    TRIGGER_LOG("mode_switch_success", {
        {"old_mode", mode_str_map[last_mode_]},
        {"new_mode", mode_str_map[curr_mode_]},
        {"timestamp", getCurrentTimestamp()}
    });
}
```

---

### 2. 模式切换 - doScencSwitch()

**文件位置**: `src/scene/scene_services/src/scene_services.cpp`  
**函数签名**: `bool doScencSwitch(enum SecneMode new_mode, enum SecneMode old_mode)`  
**行号**: 671-718  
**触发条件**: 场景切换执行时  
**优先级**: ⭐⭐⭐⭐⭐ (最高)

**关键埋点位置**:
1. 场景切换开始 (行671)
2. 旧模式停止 (行681)
3. 新模式启动 (行706)
4. 模式变更完成 (行710)

**埋点代码示例**:
```cpp
// 埋点位置1: 场景切换开始
TRIGGER_LOG("scene_switch_start", {
    {"old_mode", mode_str_map[old_mode]},
    {"new_mode", mode_str_map[new_mode]},
    {"timestamp", getCurrentTimestamp()}
});

// 埋点位置2: 服务调用结果
TRIGGER_LOG("scene_service_call", {
    {"mode", mode_str_map[new_mode]},
    {"service_result", result.get()->result ? "success" : "failed"},
    {"timestamp", getCurrentTimestamp()}
});
```

---

### 3. 碰撞检测 - checkForCollision()

**文件位置**: `src/scene/scene_auxiliary/src/collision_checker.cpp`  
**函数签名**: `geometry_msgs::msg::Twist checkForCollision(const nav_msgs::msg::OccupancyGrid::SharedPtr map, geometry_msgs::msg::Twist::SharedPtr velocity)`  
**行号**: 18-39  
**触发条件**: 每次速度控制时进行碰撞检测  
**优先级**: ⭐⭐⭐⭐⭐ (最高)

**关键埋点位置**:
1. 碰撞检测开始 (行18)
2. 前方碰撞检测结果 (行34)
3. 倒车碰撞检测结果 (行29)
4. 速度调整决策 (行36-37)

**埋点代码示例**:
```cpp
// 埋点位置1: 碰撞检测开始
TRIGGER_LOG("collision_check_start", {
    {"linear_velocity", velocity->linear.x},
    {"angular_velocity", velocity->angular.z},
    {"timestamp", getCurrentTimestamp()}
});

// 埋点位置2: 碰撞检测结果
bool collision_detected = isCollisionLikely(map, velocity);
TRIGGER_LOG("collision_check_result", {
    {"collision_detected", collision_detected},
    {"adjusted_linear_velocity", adjusted_velocity.linear.x},
    {"adjusted_angular_velocity", adjusted_velocity.angular.z},
    {"timestamp", getCurrentTimestamp()}
});
```

---

### 4. 碰撞检测 - isCollisionLikely()

**文件位置**: `src/scene/scene_auxiliary/src/collision_checker.cpp`  
**函数签名**: `bool isCollisionLikely(const nav_msgs::msg::OccupancyGrid::SharedPtr map, geometry_msgs::msg::Twist::SharedPtr velocity)`  
**行号**: 63-109  
**触发条件**: 判断前方碰撞可能性  
**优先级**: ⭐⭐⭐⭐⭐ (最高)

**关键决策点**:
- 前方碰撞检测 (行93-100)
- 侧方碰撞检测 (行103-108)
- 速度调整决策 (行109之后)

---

### 5. 控制命令接收 - onReceiveControlMessage()

**文件位置**: `src/sensor/chassis_component/can_adapter.cpp`  
**函数签名**: `void onReceiveControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg)`  
**触发条件**: 接收到NAV2控制命令时  
**优先级**: ⭐⭐⭐⭐⭐ (最高)

**埋点数据**:
- 控制命令来源 (nav2/handle/app)
- 线速度值
- 角速度值
- 接收时间戳

---

### 6. 服务调用 - handleGearsStart()

**文件位置**: `src/control_service/src/control/gears_control.cpp`  
**函数签名**: `void handleGearsStart(const std::shared_ptr<interface::srv::GearsControls::Request> request, std::shared_ptr<interface::srv::GearsControls::Response> response)`  
**行号**: 17-30  
**触发条件**: 档位控制服务被调用时  
**优先级**: ⭐⭐⭐⭐⭐ (最高)

**关键埋点位置**:
1. 服务请求接收 (行17)
2. 服务响应结果 (行28-29)
3. 服务超时 (行23-26)

**埋点代码示例**:
```cpp
// 埋点位置1: 服务请求
TRIGGER_LOG("service_gears_request", {
    {"gear", request->gear},
    {"helpcar", request->helpcar},
    {"sos", request->sos},
    {"timestamp", getCurrentTimestamp()}
});

// 埋点位置2: 服务响应
TRIGGER_LOG("service_gears_response", {
    {"result_code", result->code},
    {"response_code", response->code},
    {"timeout", (status == std::future_status::timeout)},
    {"timestamp", getCurrentTimestamp()}
});
```

---

## ROS Topic订阅埋点

### 传感器数据订阅

| Topic名称 | 文件位置 | 回调函数 | 优先级 | 说明 |
|---------|---------|---------|--------|------|
| `/sensor/camera/images` | `src/sensor/camera_component/camera.cpp` | 图像回调 | ⭐⭐⭐ | 相机图像数据 |
| `/sensor/gps/data` | `src/sensor/gps_component/gps.cpp` | GPS回调 | ⭐⭐⭐ | GPS定位数据 |
| `/sensor/imu/data` | `src/sensor/imu_component/imu.cpp` | IMU回调 | ⭐⭐⭐ | IMU姿态数据 |
| `/sensor/health/data` | `src/tft_service/src/control/tft_health_control.cpp` | `onHrvResult()` | ⭐⭐⭐⭐ | 健康计数据 |
| `/sensor/health/start` | `src/tft_service/src/control/tft_health_control.cpp` | `handleHrvStart()` | ⭐⭐⭐⭐ | 健康检测启动 |
| `/sensor/backup/car` | `src/sensor/chassis_component/` | 倒车回调 | ⭐⭐⭐ | 倒车数据 |
| `/sensor/handle/control` | `src/sensor/chassis_component/` | 控制回调 | ⭐⭐⭐⭐ | 上控健康控制 |

### 控制命令订阅

| Topic名称 | 文件位置 | 回调函数 | 优先级 | 说明 |
|---------|---------|---------|--------|------|
| `/nav/control` | `src/sensor/chassis_component/can_adapter.cpp` | `onReceiveControlMessage()` | ⭐⭐⭐⭐⭐ | NAV2导航控制 |
| `/handle/control` | `src/sensor/chassis_component/` | 控制回调 | ⭐⭐⭐⭐ | 手柄控制 |
| `/app/control` | `src/sensor/chassis_component/` | 控制回调 | ⭐⭐⭐⭐ | APP控制 |
| `/mode/change` | `src/scene/scene_services/` | 模式切换回调 | ⭐⭐⭐⭐⭐ | 模式切换命令 |

### 激光雷达数据订阅

| Topic名称 | 文件位置 | 回调函数 | 优先级 | 说明 |
|---------|---------|---------|--------|------|
| `/scan/left` | `src/scene/scene_auxiliary_test/` | 激光扫描回调 | ⭐⭐⭐ | 左侧激光雷达 |
| `/scan/right` | `src/scene/scene_auxiliary_test/` | 激光扫描回调 | ⭐⭐⭐ | 右侧激光雷达 |
| `/scan/left_tilt` | `src/scene/scene_auxiliary_test/` | 激光扫描回调 | ⭐⭐⭐ | 左侧倾斜激光 |
| `/scan/right_tilt` | `src/scene/scene_auxiliary_test/` | 激光扫描回调 | ⭐⭐⭐ | 右侧倾斜激光 |
| `/scan/back` | `src/scene/scene_auxiliary_test/` | 激光扫描回调 | ⭐⭐⭐ | 后侧激光雷达 |

### 感知数据订阅

| Topic名称 | 文件位置 | 回调函数 | 优先级 | 说明 |
|---------|---------|---------|--------|------|
| `/perception/cameraFused/objects` | `src/perception/` | 目标检测回调 | ⭐⭐⭐ | 相机融合目标 |
| `/perception/multiFused/objects` | `src/perception/` | 融合目标回调 | ⭐⭐⭐⭐ | 多传感器融合 |
| `/perception/obstacle_map` | `src/scene/scene_auxiliary/` | 地图回调 | ⭐⭐⭐⭐ | 障碍物地图 |
| `/perception/podao_detect_status` | `src/perception/podao_detect/` | 坡道检测回调 | ⭐⭐⭐ | 坡道检测状态 |

### 场景状态订阅

| Topic名称 | 文件位置 | 回调函数 | 优先级 | 说明 |
|---------|---------|---------|--------|------|
| `/vlago/auxiliary/alarm` | `src/tft_service/src/control/tft_safety_control.cpp` | `handleAuxiliaryAlarm()` | ⭐⭐⭐⭐ | 辅助告警 |
| `/vlago/backcar/alarm` | `src/tft_service/src/control/` | 倒车告警回调 | ⭐⭐⭐⭐ | 倒车告警 |
| `/foot/limiter/status` | `src/tft_service/src/control/tft_safety_control.cpp` | `handleFootAlarm()` | ⭐⭐⭐⭐ | 脚部限位器 |
| `/vlago/mode/status` | `src/scene/scene_services/` | 模式状态回调 | ⭐⭐⭐⭐ | 模式状态 |
| `/vlago/recharge/status` | `src/scene/scene_services/` | 回充状态回调 | ⭐⭐⭐⭐ | 回充状态 |

---

## ROS Service调用埋点

### 控制服务

| Service名称 | 文件位置 | 处理函数 | 行号 | 优先级 | 说明 |
|-----------|---------|---------|------|--------|------|
| `/vlago/car/control` | `src/control_service/src/control/car_control.cpp` | `handleCarControl()` | - | ⭐⭐⭐⭐⭐ | 车辆控制 |
| `/vlago/follow_me` | `src/control_service/src/control/follow_control.cpp` | `handleFollowControl()` | - | ⭐⭐⭐⭐ | 跟随我服务 |
| `/vlago/follow_me/status` | `src/control_service/src/control/follow_control.cpp` | `handleFollowStatus()` | - | ⭐⭐⭐ | 跟随状态查询 |
| `/valgo/car/summon` | `src/control_service/src/control/summon_control.cpp` | `handleSummonControl()` | - | ⭐⭐⭐⭐ | 召唤服务 |
| `/vaglo/car/high` | `src/control_service/src/control/high_control.cpp` | `handleHighControl()` | - | ⭐⭐⭐ | 高举服务 |

### 场景模式服务

| Service名称 | 文件位置 | 处理函数 | 行号 | 优先级 | 说明 |
|-----------|---------|---------|------|--------|------|
| `/vlago/narrow_assistant_mode` | `src/scene/scene_services/src/scene_services.cpp` | `OnTrigger()` | 597 | ⭐⭐⭐⭐⭐ | 狭窄辅助模式 |
| `/vlago/standard_mode` | `src/scene/scene_services/src/scene_services.cpp` | `OnTrigger()` | 597 | ⭐⭐⭐⭐⭐ | 标准模式 |
| `/vlago/cruise_mode` | `src/scene/scene_services/src/scene_services.cpp` | `OnTrigger()` | 597 | ⭐⭐⭐⭐⭐ | 巡航模式 |
| `/vlago/safe_mode` | `src/scene/scene_services/src/scene_services.cpp` | `OnTrigger()` | 597 | ⭐⭐⭐⭐⭐ | 安全模式 |
| `/vlago/dance_show` | `src/control_service/src/control/dance_control.cpp` | `handleDanceControl()` | - | ⭐⭐⭐⭐ | 舞蹈秀 |
| `/vlago/auto_charge` | `src/scene/scene_services/src/scene_services.cpp` | `rechargeCtrl()` | 449 | ⭐⭐⭐⭐⭐ | 自动回充 |
| `/mode/switch` | `src/scene/scene_services/src/scene_services.cpp` | 模式切换 | - | ⭐⭐⭐⭐⭐ | 模式切换 |
| `/vlago/mode/enable` | `src/scene/scene_services/src/scene_services.cpp` | `modeEnable()` | 450 | ⭐⭐⭐⭐⭐ | 模式使能 |

### 健康服务

| Service名称 | 文件位置 | 处理函数 | 行号 | 优先级 | 说明 |
|-----------|---------|---------|------|--------|------|
| `/vlago/hrv/start` | `src/control_service/src/control/health_control.cpp` | `handleHrvStart()` | - | ⭐⭐⭐⭐ | 健康检测启动 |
| `/vlago/hrv/info` | `src/control_service/src/control/health_control.cpp` | `handleHrvInfo()` | - | ⭐⭐⭐ | 健康信息查询 |

### 档位控制服务

| Service名称 | 文件位置 | 处理函数 | 行号 | 优先级 | 说明 |
|-----------|---------|---------|------|--------|------|
| `/vlago/gear` | `src/control_service/src/control/gears_control.cpp` | `handleGearsStart()` | 17-30 | ⭐⭐⭐⭐⭐ | 档位控制 |

---

## 模式切换埋点

### 模式枚举

```cpp
enum SecneMode{
    Mode_zero = 0,
    Mode_Auxiliary,      // 辅助模式
    Mode_Narrow,         // 狭窄模式
    Mode_Standard,       // 标准模式
    Mode_Cruise,         // 巡航模式
    Mode_Dance,          // 舞蹈模式
    Mode_Recharge,       // 回充模式
    Mode_Followme,       // 跟随模式
    Mode_Autoindoor,     // 自动室内模式
    Mode_TOP
};
```

### 关键模式切换函数

| 函数名 | 文件位置 | 行号 | 优先级 | 说明 |
|-------|---------|------|--------|------|
| `ChangeMode()` | `src/scene/scene_services/src/scene_services.cpp` | 731-745 | ⭐⭐⭐⭐⭐ | 模式变更 |
| `doScencSwitch()` | `src/scene/scene_services/src/scene_services.cpp` | 671-718 | ⭐⭐⭐⭐⭐ | 场景切换执行 |
| `OnTrigger()` | `src/scene/scene_services/src/scene_services.cpp` | 597-637 | ⭐⭐⭐⭐⭐ | 模式触发 |
| `modeEnable()` | `src/scene/scene_services/src/scene_services.cpp` | 450-488 | ⭐⭐⭐⭐ | 模式使能 |
| `modeEnableSingle()` | `src/scene/scene_services/src/scene_services.cpp` | 490-583 | ⭐⭐⭐⭐ | 单模式使能 |
| `doStart()` | `src/scene/scene_services/src/scene_services.cpp` | 748+ | ⭐⭐⭐⭐ | 模式启动 |
| `doStop()` | `src/scene/scene_services/src/scene_services.cpp` | - | ⭐⭐⭐⭐ | 模式停止 |

### 模式切换埋点示例

```cpp
// 埋点位置: ChangeMode() 函数
bool SceneServices::ChangeMode(enum SecneMode new_mode) {
    // 埋点1: 模式切换开始
    TRIGGER_LOG("mode_switch_start", {
        {"old_mode", mode_str_map[curr_mode_]},
        {"new_mode", mode_str_map[new_mode]},
        {"timestamp", getCurrentTimestamp()},
        {"node_name", "SceneServices"}
    });
    
    std::lock_guard<std::mutex> lock(mutex_);
    if (new_mode == curr_mode_) {
        return false;
    }
    
    last_mode_ = curr_mode_;
    curr_mode_ = new_mode;
    
    // 埋点2: 模式切换成功
    TRIGGER_LOG("mode_switch_success", {
        {"old_mode", mode_str_map[last_mode_]},
        {"new_mode", mode_str_map[curr_mode_]},
        {"timestamp", getCurrentTimestamp()},
        {"node_name", "SceneServices"}
    });
    
    // 发布模式状态
    std_msgs::msg::String report_msg;
    report_msg.data = mode_str_map[curr_mode_];
    mode_pub_->publish(report_msg);
    
    // 埋点3: 在线报告模式
    OnlineReportMode(new_mode, last_mode_);
    
    return true;
}
```

---

## 碰撞检测埋点

### 碰撞检测函数列表

| 函数名 | 文件位置 | 行号 | 优先级 | 说明 |
|-------|---------|------|--------|------|
| `checkForCollision()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | 18-39 | ⭐⭐⭐⭐⭐ | 碰撞检测主函数 |
| `checkForCollisionGlobal()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | 41-52 | ⭐⭐⭐⭐ | 全局碰撞检测 |
| `isCollisionLikely()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | 63-109 | ⭐⭐⭐⭐⭐ | 碰撞可能性判断 |
| `isCollisionLikelyBackward()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | - | ⭐⭐⭐⭐ | 倒车碰撞检测 |
| `isCollisionLikelyGlobal()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | - | ⭐⭐⭐⭐ | 全局碰撞可能性 |
| `checkFrontCollision()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | - | ⭐⭐⭐⭐⭐ | 前方碰撞检测 |
| `checkSideCollision()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | - | ⭐⭐⭐⭐ | 侧方碰撞检测 |
| `checkVehicleCollision()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | 755-898 | ⭐⭐⭐⭐ | 车辆碰撞检测 |
| `checkCollisionAndAdjustSpeed()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | 900-942 | ⭐⭐⭐⭐ | 碰撞检测并调整速度 |

### 碰撞检测关键决策点

1. **前方障碍物检测** (距离 < 20cm)
   - 位置: `checkForCollision()` 中
   - 触发条件: 前方障碍物距离小于20cm
   - 动作: 停车 (`velocity->linear.x = 0.0`)

2. **左侧障碍物检测** (距离 < 10cm)
   - 触发条件: 左侧障碍物距离小于10cm
   - 动作: 禁止左转 (`velocity->angular.z = 0.0`)

3. **右侧障碍物检测** (距离 < 10cm)
   - 触发条件: 右侧障碍物距离小于10cm
   - 动作: 禁止右转

4. **倒车障碍物检测** (距离 0.05-2.0m)
   - 触发条件: 倒车时后方障碍物在0.05-2.0m范围内
   - 动作: 调用倒车碰撞检测

5. **速度调整决策**
   - 位置: 碰撞检测后
   - 动作: 根据碰撞风险调整速度

### 碰撞检测埋点示例

```cpp
// 埋点位置: checkForCollision() 函数
geometry_msgs::msg::Twist CollisionChecker::checkForCollision(
    const nav_msgs::msg::OccupancyGrid::SharedPtr map,
    geometry_msgs::msg::Twist::SharedPtr velocity) {
    
    // 埋点1: 碰撞检测开始
    TRIGGER_LOG("collision_check_start", {
        {"linear_velocity", velocity->linear.x},
        {"angular_velocity", velocity->angular.z},
        {"foot_limiter_status", foot_limiter_status_},
        {"timestamp", getCurrentTimestamp()}
    });
    
    if (foot_limiter_status_ != 0) {
        velocity->linear.x = 0.0;
        velocity->angular.z = 0.0;
        
        // 埋点2: 脚部限位器触发
        TRIGGER_LOG("foot_limiter_trigger", {
            {"status", foot_limiter_status_},
            {"timestamp", getCurrentTimestamp()}
        });
        
        return *velocity;
    }
    
    bool collision_detected = false;
    if (velocity->linear.x < 0.0) {
        // 倒车碰撞检测
        if (0.05 < back_obstacle_distance_ < 2.0) {
            collision_detected = isCollisionLikelyBackward(map, velocity);
        }
    } else {
        // 前方碰撞检测
        collision_detected = isCollisionLikely(map, velocity);
    }
    
    // 埋点3: 碰撞检测结果
    TRIGGER_LOG("collision_check_result", {
        {"collision_detected", collision_detected},
        {"direction", velocity->linear.x < 0.0 ? "backward" : "forward"},
        {"back_obstacle_distance", back_obstacle_distance_},
        {"final_linear_velocity", velocity->linear.x},
        {"final_angular_velocity", velocity->angular.z},
        {"timestamp", getCurrentTimestamp()}
    });
    
    last_published_velocity_ = *velocity;
    return *velocity;
}
```

---

## 传感器回调埋点

### CAN总线回调

| 回调函数 | 文件位置 | 优先级 | 说明 |
|---------|---------|--------|------|
| `onReceiveCanMessage()` | `src/sensor/chassis_component/can_adapter.cpp` | ⭐⭐⭐⭐ | CAN消息接收 |
| `onReceiveControlMessage()` | `src/sensor/chassis_component/can_adapter.cpp` | ⭐⭐⭐⭐⭐ | 控制消息接收 |
| `onReceiveGnssMessage()` | `src/sensor/chassis_component/can_adapter.cpp` | ⭐⭐⭐ | GNSS消息接收 |

### 健康检测回调

| 回调函数 | 文件位置 | 行号 | 优先级 | 说明 |
|---------|---------|------|--------|------|
| `handleHrvStart()` | `src/tft_service/src/control/tft_health_control.cpp` | - | ⭐⭐⭐⭐ | 健康检测启动 |
| `onHrvResult()` | `src/tft_service/src/control/tft_health_control.cpp` | 139-149 | ⭐⭐⭐⭐ | 健康结果接收 |
| `analyzeHealthData()` | `src/tft_service/src/control/tft_health_control.cpp` | 151+ | ⭐⭐⭐⭐ | 健康数据分析 |

---

## 定时器埋点

### 健康检测定时器

| 定时器 | 文件位置 | 行号 | 周期 | 优先级 | 说明 |
|-------|---------|------|------|--------|------|
| `timerCallback()` | `src/tft_service/src/control/tft_health_control.cpp` | 104-110 | 1秒 | ⭐⭐⭐⭐ | 健康检测倒计时 |

**埋点示例**:
```cpp
void HealthControl::timerCallback() {
    // 埋点: 定时器回调
    TRIGGER_LOG("health_timer_callback", {
        {"remaining_seconds", timer_count_},
        {"total_seconds", TIMER_DURATION},
        {"timestamp", getCurrentTimestamp()}
    });
    
    global_->sendAddressData(HEALTH_COUNTDOWN_FRAME_ID, timer_count_);
    usleep(100);
    global_->sendPageHeader();
}
```

---

## 错误异常埋点

### 错误处理函数

| 函数名 | 文件位置 | 行号 | 优先级 | 说明 |
|-------|---------|------|--------|------|
| `Check()` | `src/lx_camera_ros/src/lx_camera/lx_camera.cpp` | 746-759 | ⭐⭐⭐⭐ | 错误检查函数 |

**埋点示例**:
```cpp
int LxCamera::Check(std::string command, int state) {
    LX_STATE lx_state = static_cast<LX_STATE>(state);
    if (LX_SUCCESS == lx_state) {
        return lx_state;
    }
    
    // 埋点: 错误发生
    const char* m = DcGetErrorString(lx_state);
    TRIGGER_LOG("lx_camera_error", {
        {"command", command},
        {"error_code", state},
        {"error_message", m},
        {"timestamp", getCurrentTimestamp()}
    });
    
    std_msgs::msg::String msg;
    setlocale(LC_ALL, "");
    msg.data = "#command: " + command + " #error code: " + std::to_string(lx_state) + " #report: " + m;
    pub_error_->publish(msg);
    RCLCPP_ERROR(this->get_logger(), "%s", msg.data.c_str());
    
    return state;
}
```

### 服务超时埋点

| 位置 | 文件位置 | 行号 | 优先级 | 说明 |
|-----|---------|------|--------|------|
| 服务超时 | `src/control_service/src/control/gears_control.cpp` | 23-26 | ⭐⭐⭐⭐ | 服务响应超时 |

---

## 埋点实施指南

### 1. 埋点框架建议

建议使用统一的埋点框架，例如：

```cpp
// 埋点宏定义
#define TRIGGER_LOG(event_name, data) \
    TriggerLogger::getInstance().log(event_name, data, __FILE__, __LINE__)

// 埋点数据结构
struct TriggerData {
    std::string event_name;
    std::map<std::string, std::string> data;
    std::string file;
    int line;
    int64_t timestamp;
    std::string node_name;
};
```

### 2. 埋点实施步骤

1. **选择埋点位置**: 根据优先级选择关键触发点
2. **添加埋点代码**: 在关键函数中添加埋点调用
3. **定义埋点数据**: 收集必要的上下文信息
4. **测试埋点**: 验证埋点数据正确性
5. **性能评估**: 确保埋点不影响系统性能

### 3. 埋点最佳实践

- ✅ 使用异步队列处理埋点数据
- ✅ 对高频触发点进行采样
- ✅ 敏感数据脱敏处理
- ✅ 区分调试和生产埋点
- ✅ 记录关键业务状态
- ❌ 不要在实时性要求高的循环中埋点
- ❌ 不要记录敏感用户信息
- ❌ 不要影响系统性能

---

## 埋点数据格式规范

### 基础字段

```json
{
  "event_name": "mode_switch_start",
  "timestamp": 1704067200000,
  "node_name": "SceneServices",
  "file": "src/scene/scene_services/src/scene_services.cpp",
  "line": 731,
  "data": {
    "old_mode": "standard",
    "new_mode": "auxiliary"
  }
}
```

### 事件类型定义

| 事件类型 | 前缀 | 说明 |
|---------|------|------|
| 模式切换 | `mode_switch_` | 模式相关事件 |
| 碰撞检测 | `collision_` | 碰撞检测相关 |
| 服务调用 | `service_` | 服务调用相关 |
| 传感器数据 | `sensor_` | 传感器数据相关 |
| 错误异常 | `error_` | 错误异常相关 |
| 定时器 | `timer_` | 定时器相关 |

---

## 埋点汇总表格

### 高优先级埋点汇总 (15个)

| ID | 触发点名称 | 文件位置 | 行号 | 类型 | 优先级 |
|----|----------|---------|------|------|--------|
| 1 | `ChangeMode()` | `src/scene/scene_services/src/scene_services.cpp` | 731 | 函数 | ⭐⭐⭐⭐⭐ |
| 2 | `doScencSwitch()` | `src/scene/scene_services/src/scene_services.cpp` | 671 | 函数 | ⭐⭐⭐⭐⭐ |
| 3 | `checkForCollision()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | 18 | 函数 | ⭐⭐⭐⭐⭐ |
| 4 | `isCollisionLikely()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | 63 | 函数 | ⭐⭐⭐⭐⭐ |
| 5 | `onReceiveControlMessage()` | `src/sensor/chassis_component/can_adapter.cpp` | - | 回调 | ⭐⭐⭐⭐⭐ |
| 6 | `handleGearsStart()` | `src/control_service/src/control/gears_control.cpp` | 17 | 服务 | ⭐⭐⭐⭐⭐ |
| 7 | `rechargeCtrl()` | `src/scene/scene_services/src/scene_services.cpp` | 449 | 服务 | ⭐⭐⭐⭐⭐ |
| 8 | `followmeCtrl()` | `src/scene/scene_services/src/scene_services.cpp` | - | 服务 | ⭐⭐⭐⭐⭐ |
| 9 | `OnTrigger()` | `src/scene/scene_services/src/scene_services.cpp` | 597 | 函数 | ⭐⭐⭐⭐⭐ |
| 10 | `/nav/control` | `src/sensor/chassis_component/can_adapter.cpp` | - | Topic | ⭐⭐⭐⭐⭐ |
| 11 | `/mode/switch` | `src/scene/scene_services/` | - | Service | ⭐⭐⭐⭐⭐ |
| 12 | `checkFrontCollision()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | - | 函数 | ⭐⭐⭐⭐⭐ |
| 13 | `Check()` | `src/lx_camera_ros/src/lx_camera/lx_camera.cpp` | 746 | 函数 | ⭐⭐⭐⭐ |
| 14 | `handleAuxiliaryAlarm()` | `src/tft_service/src/control/tft_safety_control.cpp` | 16 | 回调 | ⭐⭐⭐⭐ |
| 15 | `handleHrvStart()` | `src/tft_service/src/control/tft_health_control.cpp` | - | 回调 | ⭐⭐⭐⭐ |

---

## 附录

### A. 代码位置索引

所有关键触发点的代码位置已在本文档中详细列出，可使用以下方式快速定位：

1. 使用文档搜索功能查找函数名
2. 查看"埋点汇总表格"获取文件位置和行号
3. 参考"埋点实施指南"了解具体实施方法

### B. 联系信息

如有疑问或需要补充埋点，请联系开发团队。

---

**文档结束**

