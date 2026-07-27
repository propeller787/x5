/**
 * @file light_scene_controller.h
 * @brief 灯效场景单例：按 light_type_scene_type 映射灯效；碰撞预警时强制闪烁红，全向无预警持续 2s 后恢复预警前灯效。
 */
#pragma once

#include "control/define.h"
#include <cstdint>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace tft_service {

class LightSceneController {
public:
    /** 返回全局单例，线程安全由调用方与内部 mutex 配合保证。 */
    static LightSceneController& instance();

    LightSceneController(const LightSceneController&) = delete;
    LightSceneController& operator=(const LightSceneController&) = delete;

    /**
     * 绑定 ROS 节点，创建灯效发布者与恢复用定时器。
     * 必须在任何依赖定时器或发布者的接口之前调用。
     */
    void init(rclcpp::Node* node);

    /**
     * 按场景应用灯效（对外主入口）。
     * 会切换 current_scene_、取消碰撞预警覆盖与恢复定时器，并按场景刷新灯色；充电场景会读取 TftStateMachine 当前电量。
     */
    void applyScene(light_type_scene_type scene);

    /** 当前是否为定速巡航或蓝牙遥控场景（供业务侧分支判断）。 */
    bool isCruiseOrBluetoothScene() const;

    /**
     * 更新碰撞预警状态（由四向等级聚合后的 bool 驱动）。
     * @param has_warning true 时强制闪烁红，并在首次进入预警时保存当前灯效；false 时在仍处于预警覆盖下启动/重置 2s 无预警计时，到期后恢复保存的灯效。
     */
    void updateCollisionWarningStatus(bool has_warning);

    /** 车辆电量变化时调用：若当前为充电场景则按电量区间重算红/黄/绿灯。 */
    void refreshChargingLightIfCurrentScene();

    /** 车速变化时调用：若当前为摇杆场景则按阈值重算常亮橙或常亮红。 */
    void refreshJoystickLightIfCurrentScene();

    /** 当前灯效场景类型（与 applyScene 最后一次生效一致）。 */
    light_type_scene_type getCurrentScene() const;

private:
    LightSceneController() = default;

    /** 在已持 mutex_ 前提下，按 current_scene_ 刷新应显示的灯效。 */
    void flushLightLocked();

    /** 发布灯效至 lightTypeTopic；与 last_published_light_type_ 相同时跳过，避免重复下发。调用方须已持 mutex_。 */
    void publishLightTypeLocked(light_type_type type);

    /** 摇杆场景：根据当前车速发布常亮橙（有速度）或常亮红（近似静止）。须已持 mutex_。 */
    void publishJoystickLightFromSpeedLocked();

    /** 巡航/蓝牙场景进入或刷新时的常态灯：常亮蓝。须已持 mutex_。 */
    void applyCruiseOrBluetoothOnSceneEnterLocked();

    /** 摇杆场景进入时：按车速发布橙/红。须已持 mutex_。 */
    void applyJoystickOnSceneEnterLocked();

    /**
     * 碰撞预警已清除（has_warning 为 false）时调度：从最后一次无预警起算 2s 后恢复 light_type_before_warning_ 对应灯效；期间再次预警会取消该定时器。
     * 须已持 mutex_。
     */
    void scheduleRecoverAfterObstacleClearedLocked();

    /** 取消无预警恢复定时器并释放句柄。须已持 mutex_。 */
    void cancelRecoveryTimerLocked();

    mutable std::mutex mutex_;
    rclcpp::Node* node_{nullptr};
    light_type_scene_type current_scene_{LIGHT_TYPE_SCENE_TYPE_JOYSTICK};
    int16_t collision_levels_[4]{};
    rclcpp::TimerBase::SharedPtr recovery_timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr light_scene_type_pub_;
    /** 上次下发灯效类型；用于避免重复下发相同灯效 */
    light_type_type last_published_light_type_{};
    /** 进入碰撞预警覆盖前一刻的灯效，用于 2s 无预警后恢复 */
    light_type_type light_type_before_warning_{};
    /** 当前是否由碰撞预警强制为闪烁红（与场景常态灯分离） */
    bool collision_warning_override_{false};
    /** 曾出现过预警；全向无预警期间每次 setCollision 都会重建 2s 恢复定时器（从最后一次收到起算），定时器触发后清 false */
    bool had_obstacle_warning_active_{false};
};

}  // namespace tft_service
