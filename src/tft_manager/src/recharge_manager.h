/**
 * @file recharge_manager.h
 * @brief 回充 / 充电相关：订阅回充状态话题、充电页切换、回充触屏读响应（上滑 0x1104 留在 TftService）。
 */

#pragma once

#include "control/define.h"
#include "tft_state_machine.h"
#include <interface/msg/bluetooth_status.hpp>
#include <interface/msg/joystick.hpp>
#include <interface/srv/scene_controls.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <functional>
#include <mutex>

namespace tft_service {

class PageDialogManager;
//回充状态 1 前往基站中 2 未找到充电站 100 成功
enum class RechargeStatus : int32_t {
    Searching = 1,
    NotFound = 2,
    Success = 100,
};

class TftAudioPublisher;

/**
 * @brief 回充与充电展示独立管理（与 OtaManager 并列，由 TftService 注入 TFT/页面回调）。
 */
class RechargeManager {
public:
    static RechargeManager& instance();

    void init(rclcpp::Node* node);
    bool isInitialized() const { return node_ != nullptr; }

    void setTftAudio(TftAudioPublisher* audio) { tft_audio_ = audio; }

    void setPageDialogManager(PageDialogManager* mgr);

    void setOnSwitchPage(std::function<void(int page_id)> cb) { on_switch_page_ = std::move(cb); }
    void setTftConnectedPredicate(std::function<bool()> pred) { tft_connected_ = std::move(pred); }
    /** 返回 true 时允许根据充电状态切换充电相关页面（与原先：仅主页时切换一致） */
    void setChargingPageUpdateGate(std::function<bool()> gate) { charging_page_gate_ = std::move(gate); }

    /** Modbus 读响应：回充按钮等（不含上滑 0x1104） */
    void onReadResponse(uint16_t reg, uint16_t value);

    /** 在 tft_audio_ 与 TFT 回调注册后调用，挂接 TftStateMachine */
    void attachStateMachineCallbacks();

    RechargeManager(const RechargeManager&) = delete;
    RechargeManager& operator=(const RechargeManager&) = delete;

private:
    RechargeManager() = default;

    void onRechargeStatusForAudio(const std_msgs::msg::Int32::SharedPtr msg);
    void onBluetoothStatus(const interface::msg::BluetoothStatus::SharedPtr msg);
    void onRechargeButtonTouch(uint16_t reg, uint16_t value);

    /**
     * 回充过程中蓝牙断开或离开基站：关闭回充并弹窗。
     * 曾处于基站充电且电量未满：recharge_exit；否则按原因弹 recharge_block。
     */
    void evaluateExitRechargeOnBluetoothLost(bool prev_status, bool prev_key, bool prev_charging);
    /** 驻车时若收到巡航意图、摇杆前后推、双抱闸打开，则请求关闭回充（与物理按键巡航弹窗逻辑独立） */
    void evaluateExitRechargeOnDriverOverride();
    void onCruiseModeSignalForExit(const std_msgs::msg::Int32::SharedPtr msg);
    void onJoystickForRechargeExit(const interface::msg::Joystick::SharedPtr msg);

private:
    std::mutex mutex_;
    rclcpp::Node* node_{nullptr};
    TftAudioPublisher* tft_audio_{nullptr};
    PageDialogManager* page_dialog_mgr_{nullptr};
    // 回充client
    rclcpp::Client<interface::srv::SceneControls>::SharedPtr recharge_client_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr recharge_status_sub_;
    rclcpp::Subscription<interface::msg::BluetoothStatus>::SharedPtr bluetooth_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cruise_mode_signal_sub_;
    rclcpp::Subscription<interface::msg::Joystick>::SharedPtr joystick_recharge_exit_sub_;
    rclcpp::TimerBase::SharedPtr recharge_exit_eval_timer_;
    int last_cruise_mode_signal_{0};
    int16_t last_joystick_x_{0};
    bool recharge_exit_override_latched_{false};
    bool bluetooth_recharge_exit_latched_{false};
    std::function<void(int)> on_switch_page_;
    std::function<bool()> tft_connected_;
    std::function<bool()> charging_page_gate_;
    int last_charging_status_for_audio_{-1};

    // 蓝牙状态
    bool bluetooth_status_{false}; // true 连接成功 false 未连接
    int bluetooth_voltage_{0}; // 电压值
    bool bluetooth_key_status_{false}; // true 按下 false 未按下
    bool bluetooth_charging_status_{false}; // true 充电中 false 未充电
};

}  // namespace tft_service
