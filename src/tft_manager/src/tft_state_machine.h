/**
 * @file tft_state_machine.h
 * @brief 状态机：订阅 /vlago/car/info、/scene_services/mode、/error_events，缓存整车数据、场景模式与设备异常，供其他类随时查询。
 */

#pragma once
#include <interface/msg/car_info.hpp>
#include <interface/msg/hex_data.hpp>
#include <interface/msg/joystick.hpp>
#include <std_msgs/msg/string.hpp>
#include <interface/msg/collision_warning.hpp>
#include <interface/msg/vehicle_bind_status.hpp>
#include <utils/warning_type.h>
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <interface/srv/scene_controls.hpp>
#include "control/define.h"

namespace tft_service {

class PageDialogManager;

/** 错误码格式 SSMMEE1EE2：SS=系统(0x01~0x04,0x10~0x16...)，MM=模块，EE1=错误类型，EE2=0x01发生/0x02解除 */
struct ErrorEventEntry {
    uint8_t system;   // SS: 0x01系统基础 0x02can0 0x03can1 0x04can2 0x10IMU 0x11GPS 0x12~0x16单线雷达#1~5 等
    uint8_t module;  // MM
    uint8_t code;    // EE1
    bool occurred;   // true=发生(EE2=0x01) false=解除(EE2=0x02)
};

/** 从 SS 字节取系统名称（用于显示） */
const char* errorEventSystemName(uint8_t system);

/** 是否为 CAN 类系统：SS 0x02 can0, 0x03 can1, 0x04 can2 */
inline bool isCanSystem(uint8_t system) {
    return system >= 0x02 && system <= 0x04;
}

/** 是否为单线雷达类系统：SS 0x12~0x16 单线雷达#1左~#5后 */
inline bool isLidarSystem(uint8_t system) {
    return system >= 0x12 && system <= 0x16;
}

/**
 * @brief 状态机单例：记录整车数据（CarInfo）、场景模式、设备异常（/error_events），
 *        其他类通过 instance() 随时查看。TFT 显示速度来自 /wheel/twist（与 sensor 线速度一致，km/h = |vx|*3.6）。
 */
class TftStateMachine {
public:
    static constexpr std::size_t kReverseCurvePointCount = 6;
    using GuidePoint = std::pair<int16_t, int16_t>;
    using GuideCurve = std::array<GuidePoint, kReverseCurvePointCount>;

    struct ReverseGuideCurves {
        GuideCurve left{};
        GuideCurve right{};
        bool valid{false};
    };

    static TftStateMachine& instance();

    /**
     * @brief 在节点内订阅 topic，必须在 TftService 构造时调用一次。
     * @param node 本节点（用于 create_subscription）
     */
    void init(rclcpp::Node* node);

    /** 最近一次整车数据（未收到过则 nullopt） */
    std::optional<interface::msg::CarInfo> getCarInfo() const;

    /** 当前场景模式字符串：standard / cruise / helpcar / bluetooth / hollowme / recharge，未收到过为空串 */
    std::string getSceneMode() const;

    /** 当前处于“发生”状态的错误码集合（每个元素为 24 位：SS<<16|MM<<8|EE1） */
    std::set<uint32_t> getActiveErrorCodes() const;

    /** 按系统(SS)分组的当前发生中的错误列表 */
    std::map<uint8_t, std::vector<ErrorEventEntry>> getActiveErrorsBySystem() const;

    /** 是否存在 CAN 类异常（can0/can1/can2 任一有发生中的错误） */
    bool hasCanError() const;

    /** 是否存在单线雷达类异常（雷达#1~#5 任一有发生中的错误） */
    bool hasLidarError() const;

    /** 是否存在指定系统(SS)的异常 */
    bool hasErrorOfSystem(uint8_t system) const;

    /** 摇杆是否处于倒车状态：rocker_x < -0.01 连续 5 次触发 */
    bool hasJoystickBackwardEvent() const;

    /** 当前摇杆事件：forward / backward / center */
    std::string getJoystickEvent() const;

    /** 当前抱闸状态：0 抱闸关闭 1 抱闸打开 */
    int getBrakeStatus() const { return brake_status_; }

    /** 是否为摇杆模式：true 摇杆模式 false 非摇杆模式 */
    bool isStandardMode() const { return scene_mode_ == "standard"; }

    /** 是否为推行模式：true 推行模式 false 摇杆模式 */
    bool isPushingMode() const { return scene_mode_ == "helpcar"; }
    /** 是否已初始化（已调用 init） */
    bool isInitialized() const { return node_ != nullptr; }
    /*是否为线充模式：true 线充模式 false 基站充 */
    bool isLineChargingMode() const { return charging_status_ == 1; }
    /** 是否为驻车模式：true 驻车模式 false 摇杆模式 */
    bool isParkingMode() const { return gear_icon_ == 2; }

    int getGearIcon() const { return gear_icon_; }
    /** 是否为向后行拨动摇杆：true 向后行拨动摇杆 false 向后行 */
    bool isBackwardMovement() const { return joystick_event_ == "backward"; }

    /** 设置 scene_mode_str_ 变化时的回调（方案二：直接回调，无 topic） */
    void setOnSceneModeStrChanged(std::function<void(const std::string&)> cb) { on_scene_mode_str_changed_ = std::move(cb); }
    /** 自主回充场景：scene_services/mode == recharge 进入/退出 */
    void setOnRechargeSceneChanged(std::function<void(bool)> cb) { on_recharge_scene_changed_ = std::move(cb); }
    // 剩余里程 回调函数
    void setOnRemainingMileageChanged(std::function<void(int)> cb) { on_remaining_mileage_changed_ = std::move(cb); }
    // 剩余电量 回调函数
    void setOnRemainingElectricityChanged(std::function<void(int)> cb) { on_remaining_electricity_changed_ = std::move(cb); }
    // 速度 回调函数
    void setOnSpeedChanged(std::function<void(float)> cb) { on_speed_changed_ = std::move(cb); }

    // 档位图标 回调函数
    void setOnGearIconChanged(std::function<void(int)> cb) { on_gear_icon_changed_ = std::move(cb); }
    // 安全功能状态 回调函数
    void setOnAuxiliaryStatusChanged(std::function<void(bool)> cb) { on_auxiliary_status_changed_ = std::move(cb); }
    // 巡航模式 回调函数
    void setOnCruiseModeChanged(std::function<void(bool)> cb) { on_cruise_mode_changed_ = std::move(cb); }
    // 蓝牙控车状态 回调函数
    void setOnBluetoothCtrlModeChanged(std::function<void(int)> cb) { on_bluetooth_ctrl_mode_changed_ = std::move(cb); }
    // 充电状态 回调函数
    void setOnChargingStatusChanged(std::function<void(int)> cb) { on_charging_status_changed_ = std::move(cb); }
    // 制动/减速状态 回调函数
    void setOnForwardBrakeDecelerationChanged(std::function<void(int)> cb) { on_forward_brake_deceleration_changed_ = std::move(cb); }
    void setOnBackwardBrakeDecelerationChanged(std::function<void(int)> cb) { on_backward_brake_deceleration_changed_ = std::move(cb); }
    void setOnLeftBrakeDecelerationChanged(std::function<void(int)> cb) { on_left_brake_deceleration_changed_ = std::move(cb); }
    void setOnRightBrakeDecelerationChanged(std::function<void(int)> cb) { on_right_brake_deceleration_changed_ = std::move(cb); }
    // 抱闸状态 回调函数
    void setOnBrakeStatusChanged(std::function<void(int)> cb) { on_brake_status_changed_ = std::move(cb); }

    // 摇杆失效类型 回调函数
    void setOnJoystickFailureTypeChanged(std::function<void(joystick_failure_type)> cb) { on_joystick_failure_type_changed_ = std::move(cb); }
    // 设备绑定状态 回调函数
    void setOnDeviceBindStatusChanged(std::function<void(int, const std::string&, const std::string&)> cb) { on_device_bind_status_changed_ = std::move(cb); }
    // 倒车引导线（左右两条贝塞尔曲线，各6点）回调
    void setOnReverseGuideCurvesChanged(std::function<void(const ReverseGuideCurves&)> cb) {
        on_reverse_guide_curves_changed_ = std::move(cb);
    }
    TftStateMachine(const TftStateMachine&) = delete;
    TftStateMachine& operator=(const TftStateMachine&) = delete;
    std::string getSceneModeStr() const { return scene_mode_str_; }
    float getSpeedKmh() const { return speed_; }
    /** /wheel/twist 角速度 angular.z，单位 rad/s */
    float getWheelAngularRadPerSec() const { return wheel_angular_z_; }
    ReverseGuideCurves getReverseGuideCurves() const;

    /**
     * @brief 语言切换后调用：根据当前语言重新计算档位/场景模式字符串，
     *        更新内部成员变量并返回新的 scene_mode_gear_str_（可直接发送给 TFT）。
     */
    std::string refreshLanguageStrings();

    void setDeviceBindStatus(int status, const std::string& user_id) { device_bind_status_ = status; device_bind_user_id_ = user_id; }

    void setPageDialogManager(PageDialogManager* mgr) { page_dialog_mgr_ = mgr; }
    // 动态调整D、P档灯光
    void adaptGearIconLight();
    // 根据充电状态调整灯光
    void adaptChargingLight(const int &charing_status);
    // 动态调整抱闸灯光
    void adaptBrakeLight();
private:
    TftStateMachine() = default;

    void onCarInfo(const interface::msg::CarInfo::SharedPtr msg);
    /** sensor 发布的底盘速度 TwistStamped，linear.x 为 m/s */
    void onWheelTwistStamped(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void onSceneMode(const std_msgs::msg::String::SharedPtr msg);
    void onErrorEvent(const interface::msg::HexData::SharedPtr msg);
    void onJoystick(const interface::msg::Joystick::SharedPtr msg);
    void onAuxiliaryStatus(const std_msgs::msg::Bool::SharedPtr msg);
    void onCollisionWarning(const interface::msg::CollisionWarning::SharedPtr msg);
    void onDeviceBindStatus(const interface::msg::VehicleBindStatus::SharedPtr msg);
    void updateReverseGuideCurves(int16_t rocker_x, int16_t rocker_y);
    static GuidePoint computeCubicBezierPoint(const GuidePoint& p0, const GuidePoint& p1,
                                              const GuidePoint& p2, const GuidePoint& p3, float t);
    // 摇杆失效类型判断
    void checkJoystickFailureType();
    // 档位数字转字符串
    std::string gearNumberToStr(int gear) const;
    //档位图标
    int gearNumberToIcon(int lockswitch) const;

private:
    mutable std::mutex mutex_;
    rclcpp::Node* node_{nullptr};
    std::optional<interface::msg::CarInfo> car_info_;
    std::string scene_mode_;
    /** 当前发生中的错误（24 位 key：SS<<16|MM<<8|EE1） */
    std::set<uint32_t> active_error_codes_;
    /** 摇杆事件：forward=前进(连续5次rocker_x>0.01) backward=倒车(连续5次rocker_x<-0.01) center=回正(连续5次-0.01~0.01) */
    std::string joystick_event_{"center"};
    /** 摇杆前进：rocker_x > 0.01 的连续次数，>=5 触发前进事件 */
    int joystick_forward_consecutive_{0};
    /** 摇杆倒车：rocker_x < -0.01 的连续次数，>=5 触发倒车事件 */
    int joystick_backward_consecutive_{0};
    /** 摇杆回正：-0.01~0.01 的连续次数，>=5 触发回正事件 */
    int joystick_center_consecutive_{0};
    /** 摇杆横向：rocker_y < -0.01 ||rocker_y > 0.01 的连续次数，>=5 触发横向事件 */
    int joystick_horizontal_consecutive_{0};
    /** int16 阈值：0.01 对应 1，-0.01 对应 -1（范围 -100~100） */
    static constexpr int16_t kJoystickForwardThreshold = 1;
    static constexpr int16_t kJoystickBackwardThreshold = -1;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr joystick_event_pub_;
    rclcpp::Subscription<interface::msg::CarInfo>::SharedPtr car_info_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_twist_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scene_mode_sub_;
    rclcpp::Subscription<interface::msg::HexData>::SharedPtr error_event_sub_;
    rclcpp::Subscription<interface::msg::Joystick>::SharedPtr joystick_sub_;
    rclcpp::Client<interface::srv::SceneControls>::SharedPtr manage_recharge_client_;
    rclcpp::Client<interface::srv::SceneControls>::SharedPtr manage_followme_client_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auxiliary_status_sub_;
    rclcpp::Subscription<interface::msg::VehicleBindStatus>::SharedPtr device_bind_status_sub_;
    bool auxiliary_status_{true};
    std::function<void(bool)> on_auxiliary_status_changed_;
    // 1.0.0版本，文案显示没有摇杆模式了，只有驾驶模式
    std::string scene_mode_str_ {"驾驶模式"}; // 驾驶模式/推行模式/驻车中
    std::string scene_gear_str_ {}; // 初始化为空
    std::string scene_mode_gear_str_ {"一档 | 驾驶模式"}; // 一档 | 驾驶模式/推行模式/驻车中
    std::function<void(const std::string&)> on_scene_mode_str_changed_;
    std::function<void(bool)> on_recharge_scene_changed_;
    int remaining_mileage_{0};
    int remaining_electricity_{0};
    float speed_{0};
    float wheel_angular_z_{0};
    std::function<void(int)> on_remaining_mileage_changed_;
    std::function<void(int)> on_remaining_electricity_changed_;
    std::function<void(float)> on_speed_changed_;
    //档位图标
    int gear_icon_{0}; // 0 D 1 R 2 P
    std::function<void(int)> on_gear_icon_changed_;
    // 巡航模式 0 关闭 1 开启
    bool cruise_mode_{false};
    std::function<void(bool)> on_cruise_mode_changed_;
    // 蓝牙控车状态
    bool bluetooth_ctrl_mode_{false}; // true 开启 false 关闭
    std::function<void(int)> on_bluetooth_ctrl_mode_changed_;
    // 充电状态
    int charging_status_{0}; // 0 未充电 1 线充 2 基站充
    std::function<void(int)> on_charging_status_changed_;
    // 回充状态
    bool recharge_status_{false}; // true 回充中 false 回充完成
    std::function<void(int)> on_recharge_status_changed_;
    // 推行状态
    bool pushing_status_{false}; // true 推行中 false 推行完成
    std::function<void(bool)> on_pushing_status_changed_;
    // 标准状态
    bool standard_status_{false}; // true 标准状态 false 非标准状态
    // 制动/减速状态
    rclcpp::Subscription<interface::msg::CollisionWarning>::SharedPtr collision_warning_sub_;
    interface::msg::CollisionWarning collision_warning_{};  ///< 最近一次碰撞警告（按方向更新）
    std::function<void(int)> on_forward_brake_deceleration_changed_;// 前向制动/减速
    std::function<void(int)> on_backward_brake_deceleration_changed_;// 后向制动/减速
    std::function<void(int)> on_left_brake_deceleration_changed_;// 左向制动/减速
    std::function<void(int)> on_right_brake_deceleration_changed_;// 右向制动/减速

    // 抱闸状态
    int brake_status_{0}; // 0 抱闸关闭 1 抱闸打开(异常状态)
    std::function<void(int)> on_brake_status_changed_;

    // 摇杆失效类型
    std::function<void(joystick_failure_type)> on_joystick_failure_type_changed_;
    // 设备绑定状态
    int device_bind_status_{0}; // 0 未绑定 1 已绑定 2 绑定失败
    std::function<void(int, const std::string&, const std::string&)> on_device_bind_status_changed_;
    std::function<void(const ReverseGuideCurves&)> on_reverse_guide_curves_changed_;
    
    std::string device_bind_user_id_{""}; // 设备绑定用户ID
    ReverseGuideCurves reverse_guide_curves_{};

    PageDialogManager* page_dialog_mgr_{nullptr};
};

}  // namespace tft_service
