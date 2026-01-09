# 埋点Trigger快速参考表

## 高优先级埋点 (必须实施)

| ID | 触发点 | 文件位置 | 行号 | 类型 | 说明 |
|----|-------|---------|------|------|------|
| H1 | `ChangeMode()` | `src/scene/scene_services/src/scene_services.cpp` | 731 | 函数 | 模式变更 |
| H2 | `doScencSwitch()` | `src/scene/scene_services/src/scene_services.cpp` | 671 | 函数 | 场景切换执行 |
| H3 | `checkForCollision()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | 18 | 函数 | 碰撞检测主函数 |
| H4 | `isCollisionLikely()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | 63 | 函数 | 碰撞可能性判断 |
| H5 | `onReceiveControlMessage()` | `src/sensor/chassis_component/can_adapter.cpp` | - | 回调 | 控制消息接收 |
| H6 | `handleGearsStart()` | `src/control_service/src/control/gears_control.cpp` | 17 | 服务 | 档位控制 |
| H7 | `rechargeCtrl()` | `src/scene/scene_services/src/scene_services.cpp` | 449 | 服务 | 回充控制 |
| H8 | `followmeCtrl()` | `src/scene/scene_services/src/scene_services.cpp` | - | 服务 | 跟随控制 |
| H9 | `OnTrigger()` | `src/scene/scene_services/src/scene_services.cpp` | 597 | 函数 | 模式触发 |
| H10 | `/nav/control` | `src/sensor/chassis_component/can_adapter.cpp` | - | Topic订阅 | NAV2控制 |
| H11 | `/mode/switch` | `src/scene/scene_services/` | - | Service | 模式切换服务 |
| H12 | `checkFrontCollision()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | - | 函数 | 前方碰撞检测 |
| H13 | `Check()` | `src/lx_camera_ros/src/lx_camera/lx_camera.cpp` | 746 | 函数 | 错误检查 |
| H14 | `handleAuxiliaryAlarm()` | `src/tft_service/src/control/tft_safety_control.cpp` | 16 | 回调 | 辅助告警 |
| H15 | `handleHrvStart()` | `src/tft_service/src/control/tft_health_control.cpp` | - | 回调 | 健康检测启动 |

## ROS Topic订阅埋点

| Topic | 文件位置 | 回调函数 | 优先级 |
|-------|---------|---------|--------|
| `/sensor/camera/images` | `src/sensor/camera_component/camera.cpp` | 图像回调 | ⭐⭐⭐ |
| `/sensor/gps/data` | `src/sensor/gps_component/gps.cpp` | GPS回调 | ⭐⭐⭐ |
| `/sensor/imu/data` | `src/sensor/imu_component/imu.cpp` | IMU回调 | ⭐⭐⭐ |
| `/sensor/health/data` | `src/tft_service/src/control/tft_health_control.cpp` | `onHrvResult()` | ⭐⭐⭐⭐ |
| `/sensor/health/start` | `src/tft_service/src/control/tft_health_control.cpp` | `handleHrvStart()` | ⭐⭐⭐⭐ |
| `/nav/control` | `src/sensor/chassis_component/can_adapter.cpp` | `onReceiveControlMessage()` | ⭐⭐⭐⭐⭐ |
| `/handle/control` | `src/sensor/chassis_component/` | 控制回调 | ⭐⭐⭐⭐ |
| `/app/control` | `src/sensor/chassis_component/` | 控制回调 | ⭐⭐⭐⭐ |
| `/mode/change` | `src/scene/scene_services/` | 模式切换回调 | ⭐⭐⭐⭐⭐ |
| `/scan/left` | `src/scene/scene_auxiliary_test/` | 激光扫描回调 | ⭐⭐⭐ |
| `/scan/right` | `src/scene/scene_auxiliary_test/` | 激光扫描回调 | ⭐⭐⭐ |
| `/perception/multiFused/objects` | `src/perception/` | 融合目标回调 | ⭐⭐⭐⭐ |
| `/perception/obstacle_map` | `src/scene/scene_auxiliary/` | 地图回调 | ⭐⭐⭐⭐ |
| `/vlago/auxiliary/alarm` | `src/tft_service/src/control/tft_safety_control.cpp` | `handleAuxiliaryAlarm()` | ⭐⭐⭐⭐ |
| `/vlago/backcar/alarm` | `src/tft_service/src/control/` | 倒车告警回调 | ⭐⭐⭐⭐ |
| `/foot/limiter/status` | `src/tft_service/src/control/tft_safety_control.cpp` | `handleFootAlarm()` | ⭐⭐⭐⭐ |
| `/vlago/mode/status` | `src/scene/scene_services/` | 模式状态回调 | ⭐⭐⭐⭐ |
| `/vlago/recharge/status` | `src/scene/scene_services/` | 回充状态回调 | ⭐⭐⭐⭐ |

## ROS Service调用埋点

| Service | 文件位置 | 处理函数 | 优先级 |
|---------|---------|---------|--------|
| `/vlago/car/control` | `src/control_service/src/control/car_control.cpp` | `handleCarControl()` | ⭐⭐⭐⭐⭐ |
| `/vlago/follow_me` | `src/control_service/src/control/follow_control.cpp` | `handleFollowControl()` | ⭐⭐⭐⭐ |
| `/valgo/car/summon` | `src/control_service/src/control/summon_control.cpp` | `handleSummonControl()` | ⭐⭐⭐⭐ |
| `/vlago/narrow_assistant_mode` | `src/scene/scene_services/src/scene_services.cpp` | `OnTrigger()` | ⭐⭐⭐⭐⭐ |
| `/vlago/standard_mode` | `src/scene/scene_services/src/scene_services.cpp` | `OnTrigger()` | ⭐⭐⭐⭐⭐ |
| `/vlago/cruise_mode` | `src/scene/scene_services/src/scene_services.cpp` | `OnTrigger()` | ⭐⭐⭐⭐⭐ |
| `/vlago/safe_mode` | `src/scene/scene_services/src/scene_services.cpp` | `OnTrigger()` | ⭐⭐⭐⭐⭐ |
| `/vlago/dance_show` | `src/control_service/src/control/dance_control.cpp` | `handleDanceControl()` | ⭐⭐⭐⭐ |
| `/vlago/auto_charge` | `src/scene/scene_services/src/scene_services.cpp` | `rechargeCtrl()` | ⭐⭐⭐⭐⭐ |
| `/mode/switch` | `src/scene/scene_services/src/scene_services.cpp` | 模式切换 | ⭐⭐⭐⭐⭐ |
| `/vlago/mode/enable` | `src/scene/scene_services/src/scene_services.cpp` | `modeEnable()` | ⭐⭐⭐⭐⭐ |
| `/vlago/gear` | `src/control_service/src/control/gears_control.cpp` | `handleGearsStart()` | ⭐⭐⭐⭐⭐ |
| `/vlago/hrv/start` | `src/control_service/src/control/health_control.cpp` | `handleHrvStart()` | ⭐⭐⭐⭐ |

## 碰撞检测埋点

| 函数名 | 文件位置 | 行号 | 优先级 |
|-------|---------|------|--------|
| `checkForCollision()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | 18 | ⭐⭐⭐⭐⭐ |
| `checkForCollisionGlobal()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | 41 | ⭐⭐⭐⭐ |
| `isCollisionLikely()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | 63 | ⭐⭐⭐⭐⭐ |
| `isCollisionLikelyBackward()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | - | ⭐⭐⭐⭐ |
| `checkFrontCollision()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | - | ⭐⭐⭐⭐⭐ |
| `checkSideCollision()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | - | ⭐⭐⭐⭐ |
| `checkVehicleCollision()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | 755 | ⭐⭐⭐⭐ |
| `checkCollisionAndAdjustSpeed()` | `src/scene/scene_auxiliary/src/collision_checker.cpp` | 900 | ⭐⭐⭐⭐ |

## 模式切换埋点

| 函数名 | 文件位置 | 行号 | 优先级 |
|-------|---------|------|--------|
| `ChangeMode()` | `src/scene/scene_services/src/scene_services.cpp` | 731 | ⭐⭐⭐⭐⭐ |
| `doScencSwitch()` | `src/scene/scene_services/src/scene_services.cpp` | 671 | ⭐⭐⭐⭐⭐ |
| `OnTrigger()` | `src/scene/scene_services/src/scene_services.cpp` | 597 | ⭐⭐⭐⭐⭐ |
| `modeEnable()` | `src/scene/scene_services/src/scene_services.cpp` | 450 | ⭐⭐⭐⭐ |
| `modeEnableSingle()` | `src/scene/scene_services/src/scene_services.cpp` | 490 | ⭐⭐⭐⭐ |
| `doStart()` | `src/scene/scene_services/src/scene_services.cpp` | 748+ | ⭐⭐⭐⭐ |
| `doStop()` | `src/scene/scene_services/src/scene_services.cpp` | - | ⭐⭐⭐⭐ |

## 定时器埋点

| 定时器 | 文件位置 | 行号 | 周期 | 优先级 |
|-------|---------|------|------|--------|
| `timerCallback()` | `src/tft_service/src/control/tft_health_control.cpp` | 104 | 1秒 | ⭐⭐⭐⭐ |
| `mode_pub_timer_` | `src/scene/scene_services/src/scene_services.cpp` | - | - | ⭐⭐⭐ |
| `mock_recharge_timer_` | `src/scene/scene_services/src/scene_services.cpp` | - | - | ⭐⭐⭐ |

## 错误异常埋点

| 函数名 | 文件位置 | 行号 | 优先级 |
|-------|---------|------|--------|
| `Check()` | `src/lx_camera_ros/src/lx_camera/lx_camera.cpp` | 746 | ⭐⭐⭐⭐ |
| 服务超时 | `src/control_service/src/control/gears_control.cpp` | 23 | ⭐⭐⭐⭐ |

## 埋点实施优先级说明

- ⭐⭐⭐⭐⭐ **最高优先级**: 必须立即实施，影响系统核心功能
- ⭐⭐⭐⭐ **高优先级**: 建议尽快实施，影响重要业务逻辑
- ⭐⭐⭐ **中优先级**: 可根据需要实施，用于监控和调试

## 快速查找指南

1. **按模块查找**: 查看文件路径中的模块名
2. **按类型查找**: Topic订阅、Service调用、函数回调等
3. **按优先级查找**: 优先实施高优先级埋点
4. **详细说明**: 参考 `代码埋点Trigger详细文档.md`

---

**最后更新**: 2024  
**文档版本**: v1.0

