/**
 * @file light_scene_controller.cpp
 *
 * 实现要点：
 * - 灯效通过 lightTypeTopic 发布枚举值 light_type_type；publishLightTypeLocked 与上次相同则不下发，减少总线/灯控负载。
 * - applyScene 切换场景时会清零 last_published_light_type_ 并 flush，保证新场景一定会发一帧（避免与旧场景“恰好同色”被跳过）。
 * - 碰撞预警 updateCollisionWarningStatus：有预警时覆盖为闪烁红并记下进入前的灯效；无预警时在仍处于覆盖态下启动 2s 单次定时器，
 *   从最后一次“无预警”回调起算，到期恢复；期间再次有预警会取消定时器。
 * - 名称后缀 Locked 表示调用方已持有 mutex_，或由同文件内仅被 Locked 函数调用的路径保证。
 */

#include "control/light_scene_controller.h"
#include "tft_state_machine.h"
#include <utils/topic.h>
#include <chrono>
#include <cmath>
#include <std_msgs/msg/int32.hpp>

namespace tft_service {

namespace {

/** 车速绝对值大于该阈值 (km/h) 视为有速度 → 常亮橙，否则常亮红 */
constexpr float kJoystickSpeedThresholdKmh = 0.1f;
/** 速度在该阈值内认为等于 0.0（速度已在状态机中保留 1 位小数） */
constexpr float kZeroSpeedEpsilonKmh = 0.001f;

/**
 * 充电场景灯色：≤10% 红，(10%, 80%] 黄，(80%, 100%] 绿；输入裁剪到 [0,100]。
 */
light_type_type chargingLightFromPercent(int pct) {
    if (pct < 0) {
        pct = 0;
    }
    if (pct > 100) {
        pct = 100;
    }
    if (pct <= 10) {
        return LIGHT_TYPE_TYPE_CONSTANT_RED;
    }
    if (pct <= 80) {
        return LIGHT_TYPE_TYPE_CONSTANT_YELLOW;
    }
    return LIGHT_TYPE_TYPE_CONSTANT_GREEN;
}
}  // namespace

LightSceneController& LightSceneController::instance() {
    // Meyers 单例；首次调用在进程内完成静态初始化。
    static LightSceneController inst;
    return inst;
}

void LightSceneController::init(rclcpp::Node* node) {
    // 保存 node_ 供 create_wall_timer 使用；发布者在此创建，故 updateCollisionWarningStatus 在 init 之前调用会直接 return。
    std::lock_guard<std::mutex> lock(mutex_);
    node_ = node;
    if (node_) {
        light_scene_type_pub_ = node_->create_publisher<std_msgs::msg::Int32>(lightTypeTopic, 10);
    }
}

void LightSceneController::publishJoystickLightFromSpeedLocked() {
    // 车速来自 TftStateMachine，与摇杆场景业务一致；仅应在 current_scene_==JOYSTICK 的路径上调用。
    auto& sm = TftStateMachine::instance();
    float speed_kmh = sm.getSpeedKmh();
    //const std::string joystick_event = sm.getJoystickEvent();
    // 明确兜底：摇杆回中且速度为 0.0 时，灯效必须是常亮红。
    // if (joystick_event == "center" && std::fabs(speed_kmh) <= kZeroSpeedEpsilonKmh && current_scene_ == LIGHT_TYPE_SCENE_TYPE_JOYSTICK) {
    //     publishLightTypeLocked(LIGHT_TYPE_TYPE_CONSTANT_RED);
    //     return;
    // }
    if (std::fabs(speed_kmh) > kJoystickSpeedThresholdKmh) {
        publishLightTypeLocked(LIGHT_TYPE_TYPE_CONSTANT_ORANGE);
    } else {
        publishLightTypeLocked(LIGHT_TYPE_TYPE_CONSTANT_RED);
    }
}

void LightSceneController::refreshJoystickLightIfCurrentScene() {
    // 外部在车速话题更新时调用；非摇杆场景直接返回，避免误改灯效。
    if (current_scene_ != LIGHT_TYPE_SCENE_TYPE_JOYSTICK) {
        return;
    }
    publishJoystickLightFromSpeedLocked();
}

void LightSceneController::publishLightTypeLocked(light_type_type type) {
    // 去重：硬件/灯控侧若对重复帧敏感，可避免刷屏；若需强制重发同值，可先将 last_published_light_type_ 清零（见 applyScene / 恢复逻辑）。
    if (last_published_light_type_ == type) {
        return;
    }
    last_published_light_type_ = type;
    if (!light_scene_type_pub_) {
        return;
    }
    std_msgs::msg::Int32 msg;
    msg.data = static_cast<int32_t>(type);
    light_scene_type_pub_->publish(msg);
}

bool LightSceneController::isCruiseOrBluetoothScene() const {
    // 只读场景，仍加锁与写入路径互斥。
    std::lock_guard<std::mutex> lock(mutex_);
    return current_scene_ == LIGHT_TYPE_SCENE_TYPE_CRUISE || current_scene_ == LIGHT_TYPE_SCENE_TYPE_BLUETOOTH;
}

light_type_scene_type LightSceneController::getCurrentScene() const {
    // 与 applyScene 写入同一变量，读侧加锁保证可见性，避免与灯效更新交错读到半状态。
    std::lock_guard<std::mutex> lock(mutex_);
    return current_scene_;
}

void LightSceneController::applyScene(light_type_scene_type scene) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_scene_ = scene;
    cancelRecoveryTimerLocked();
    collision_warning_override_ = false;
    // 场景切换后强制重新发布（避免 last_published 与新区所需灯效相同而跳过下发）
    last_published_light_type_ = static_cast<light_type_type>(0);
    // 离开巡航/蓝牙/摇杆时清零四向碰撞缓存与“曾预警”标记，与历史避障逻辑保持一致，避免旧状态带入新场景。
    if (scene != LIGHT_TYPE_SCENE_TYPE_CRUISE && scene != LIGHT_TYPE_SCENE_TYPE_BLUETOOTH &&
        scene != LIGHT_TYPE_SCENE_TYPE_JOYSTICK) {
        for (int i = 0; i < 4; ++i) {
            collision_levels_[i] = 0;
        }
        had_obstacle_warning_active_ = false;
    }
    flushLightLocked();
}

void LightSceneController::cancelRecoveryTimerLocked() {
    // cancel 后 reset，避免悬空 shared_ptr 重复触发。
    if (recovery_timer_) {
        recovery_timer_->cancel();
        recovery_timer_.reset();
    }
}

void LightSceneController::scheduleRecoverAfterObstacleClearedLocked() {
    if (!node_) {
        collision_warning_override_ = false;
        if (static_cast<int>(light_type_before_warning_) == 0) {
            last_published_light_type_ = static_cast<light_type_type>(0);
            flushLightLocked();
        } else {
            publishLightTypeLocked(light_type_before_warning_);
        }
        return;
    }
    cancelRecoveryTimerLocked();
    recovery_timer_ = node_->create_wall_timer(std::chrono::seconds(2), [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!collision_warning_override_) {
            return;
        }
        cancelRecoveryTimerLocked();
        collision_warning_override_ = false;
        publishLightTypeLocked(light_type_before_warning_);
    });
}

void LightSceneController::applyCruiseOrBluetoothOnSceneEnterLocked() {
    // 巡航、蓝牙共用“常亮蓝”作为场景常态（与碰撞预警时的闪烁红区分）。
    publishLightTypeLocked(LIGHT_TYPE_TYPE_CONSTANT_BLUE);
}

void LightSceneController::applyJoystickOnSceneEnterLocked() {
    // 进入摇杆场景或 flush 时复用与车速联动的橙/红逻辑。
    publishJoystickLightFromSpeedLocked();
}

void LightSceneController::refreshChargingLightIfCurrentScene() {
    // 电量变化时由业务调用；非充电场景不动作。
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_scene_ != LIGHT_TYPE_SCENE_TYPE_CHARGING) {
        return;
    }
    flushLightLocked();
}

void LightSceneController::flushLightLocked() {
    // 各场景默认灯效：智能避障/倒车/抱闸失效为闪烁红；巡航与蓝牙为常亮蓝；回充常亮蓝；驻车/推行常亮红；摇杆按车速；充电按电量分档。
    auto& sm = TftStateMachine::instance();
    switch (current_scene_) {
        case LIGHT_TYPE_SCENE_TYPE_SMART_AVOIDANCE:
        case LIGHT_TYPE_SCENE_TYPE_BACKWARD:
        case LIGHT_TYPE_SCENE_TYPE_BRAKE_FAILURE:
            publishLightTypeLocked(LIGHT_TYPE_TYPE_FLASH_RED);
            break;
        case LIGHT_TYPE_SCENE_TYPE_CRUISE:
        case LIGHT_TYPE_SCENE_TYPE_BLUETOOTH:
            applyCruiseOrBluetoothOnSceneEnterLocked();
            break;
        case LIGHT_TYPE_SCENE_TYPE_RECHARGE:
            publishLightTypeLocked(LIGHT_TYPE_TYPE_CONSTANT_BLUE);
            break;
        case LIGHT_TYPE_SCENE_TYPE_PARKING:
        case LIGHT_TYPE_SCENE_TYPE_PUSHING:
            publishLightTypeLocked(LIGHT_TYPE_TYPE_CONSTANT_RED);
            break;
        case LIGHT_TYPE_SCENE_TYPE_JOYSTICK:
            applyJoystickOnSceneEnterLocked();
            break;
        case LIGHT_TYPE_SCENE_TYPE_CHARGING: {
            auto info = sm.getCarInfo();
            const int pct = info ? static_cast<int>(info->electricquantity) : 0;
            publishLightTypeLocked(chargingLightFromPercent(pct));
            break;
        }
        default:
            // 未列出的场景类型：不改变灯效（保持现状或由其它逻辑驱动）。
            break;
    }
}

void LightSceneController::updateCollisionWarningStatus(bool has_warning) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!light_scene_type_pub_) {
        return;
    }
    if (has_warning) {
        if (!collision_warning_override_) {
            light_type_before_warning_ = last_published_light_type_;
        }
        collision_warning_override_ = true;
        publishLightTypeLocked(LIGHT_TYPE_TYPE_FLASH_RED);
        scheduleRecoverAfterObstacleClearedLocked();
        return;
    }
    if (!collision_warning_override_) {
        return;
    }
    scheduleRecoverAfterObstacleClearedLocked();
}

}  // namespace tft_service
