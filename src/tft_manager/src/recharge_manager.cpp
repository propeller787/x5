/**
 * @file recharge_manager.cpp
 * @brief RechargeManager 实现。
 */

#include "recharge_manager.h"
#include "dialog/page_dialog_manager.h"
#include "tft_audio.h"
#include "tft_state_machine.h"
#include <utils/topic.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstdint>

namespace tft_service {

namespace {
/** 回充退出：摇杆前后推阈值（与状态机 1/-1 防抖不同，按产品 |x|>10） */
constexpr int16_t kRechargeExitJoystickThreshold = 10;
/** 静止判定：与 service / 灯效阈值一致 */
constexpr float kStationarySpeedKmh = 0.1f;
constexpr float kStationaryAngularRadPerSec = 0.001f;

bool isVehicleStationary(const TftStateMachine& sm) {
    return std::fabs(sm.getSpeedKmh()) <= kStationarySpeedKmh &&
           std::fabs(sm.getWheelAngularRadPerSec()) <= kStationaryAngularRadPerSec;
}
}  // namespace

RechargeManager& RechargeManager::instance() {
    static RechargeManager inst;
    return inst;
}

void RechargeManager::init(rclcpp::Node* node) {
    if (!node) return;
    node_ = node;

    auto qos = rclcpp::QoS(10);
    recharge_status_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        rechargeStateTopic, qos,
        [this](const std_msgs::msg::Int32::SharedPtr msg) { onRechargeStatusForAudio(msg); });
    bluetooth_status_sub_ = node_->create_subscription<interface::msg::BluetoothStatus>(
        bluetoothStatusTopic, qos,
        [this](const interface::msg::BluetoothStatus::SharedPtr msg) { onBluetoothStatus(msg); });
    recharge_client_ = node_->create_client<interface::srv::SceneControls>(manageRechargeService);
    // cruise_mode_signal_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    //     cruiseModeTopic, qos,
    //     [this](const std_msgs::msg::Int32::SharedPtr msg) { onCruiseModeSignalForExit(msg); });
    // joystick_recharge_exit_sub_ = node_->create_subscription<interface::msg::Joystick>(
    //     joystickTopic, qos,
    //     [this](const interface::msg::Joystick::SharedPtr msg) { onJoystickForRechargeExit(msg); });
    // recharge_exit_eval_timer_ = node_->create_wall_timer(
    //     std::chrono::milliseconds(50),
    //     [this]() { evaluateExitRechargeOnDriverOverride(); });
    RCLCPP_INFO(rclcpp::get_logger("RechargeManager"), "RechargeManager initialized");
}

void RechargeManager::onRechargeStatusForAudio(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!msg) return;
    int status = msg->data;
    if (!page_dialog_mgr_) return;
    if (!tft_connected_ || !tft_connected_()) return;

    if (status == static_cast<int>(RechargeStatus::Searching)) {
        page_dialog_mgr_->switchToPage(GO_TO_BASE_STATION_PAGE_ID);
    } else if (status == static_cast<int>(RechargeStatus::NotFound)) {
        if (!on_switch_page_) return;
        //关闭回充
        auto request = std::make_shared<interface::srv::SceneControls::Request>();
        request->start = 0;
        recharge_client_->async_send_request(request);
        page_dialog_mgr_->switchToPage(MAIN_PAGE_ID);
        //50ms后显示弹窗
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        page_dialog_mgr_->showDialog("recharge_block", recharge_failure_type::RECHARGE_FAILURE_TYPE_MOVE_TO_BASE);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        // 返回首页
    } else if (status == static_cast<int>(RechargeStatus::Success)) {
        //关闭回充
        auto request = std::make_shared<interface::srv::SceneControls::Request>();
        request->start = 0;
        recharge_client_->async_send_request(request);
        page_dialog_mgr_->switchToPage(MAIN_PAGE_ID);

    }
}

void RechargeManager::setPageDialogManager(PageDialogManager* mgr) {
    page_dialog_mgr_ = mgr;
    page_dialog_mgr_->setOnRechargeRetryCallback([this](TouchActionType action, const std::string& dialog_id) {
        if (action == TouchActionType::Retry && dialog_id == "recharge_block") {
            onRechargeButtonTouch(TOUCH_EVENT_RECHARGE_START, 0);
        }
    });
}

void RechargeManager::onBluetoothStatus(const interface::msg::BluetoothStatus::SharedPtr msg) {
    if (!msg) return;
    const bool prev_status = bluetooth_status_;
    const bool prev_key = bluetooth_key_status_;
    const bool prev_charging = bluetooth_charging_status_;
    bluetooth_status_ = msg->status;
    bluetooth_voltage_ = msg->voltage;
    bluetooth_key_status_ = msg->key_status;
    bluetooth_charging_status_ = msg->charging_status;
    evaluateExitRechargeOnBluetoothLost(prev_status, prev_key, prev_charging);
}

void RechargeManager::evaluateExitRechargeOnBluetoothLost(bool prev_status, bool prev_key,
                                                          bool prev_charging) {
    if (!recharge_client_ || !node_) return;
    TftStateMachine& sm = TftStateMachine::instance();
    const bool charging_became_false = prev_charging && !bluetooth_charging_status_;

    // 标准模式：基站充电状态 true→false 且车辆静止时提示回充已退出
    if (sm.isStandardMode()) {
        if (charging_became_false && isVehicleStationary(sm)) {
            if (tft_connected_ && tft_connected_() && page_dialog_mgr_) {
                page_dialog_mgr_->switchToPage(MAIN_PAGE_ID);
                sleep(1);
                page_dialog_mgr_->showDialog("recharge_exit", 0);
                if (tft_audio_) {
                    tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
                }
            }
        }
        std::lock_guard<std::mutex> lock(mutex_);
        bluetooth_recharge_exit_latched_ = false;
        return;
    }

    if (sm.getSceneMode() != "recharge") {
        std::lock_guard<std::mutex> lock(mutex_);
        bluetooth_recharge_exit_latched_ = false;
        return;
    }
    if (bluetooth_status_ && bluetooth_key_status_) return;

    const bool status_became_false = prev_status && !bluetooth_status_;
    const bool key_became_false = prev_key && !bluetooth_key_status_;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (bluetooth_recharge_exit_latched_) return;
        bluetooth_recharge_exit_latched_ = true;
    }

    int battery_pct = 100;
    if (auto info_opt = sm.getCarInfo()) {
        battery_pct = static_cast<int>(info_opt->electricquantity);
    }
    const bool show_charging_interrupt_dialog =
        prev_charging && battery_pct < 100 && (status_became_false || key_became_false);

    RCLCPP_INFO(rclcpp::get_logger("RechargeManager"),
                "Exit recharge: prev_charging=%d battery=%d%% status %d->%d key %d->%d dialog=%d",
                static_cast<int>(prev_charging), battery_pct,
                static_cast<int>(prev_status), static_cast<int>(bluetooth_status_),
                static_cast<int>(prev_key), static_cast<int>(bluetooth_key_status_),
                static_cast<int>(show_charging_interrupt_dialog));
    auto request = std::make_shared<interface::srv::SceneControls::Request>();
    request->start = 0;
    recharge_client_->async_send_request(request);
    
    if (!tft_connected_ || !tft_connected_()) return;
    if (!page_dialog_mgr_) return;
    page_dialog_mgr_->switchToPage(MAIN_PAGE_ID);
    if (show_charging_interrupt_dialog) {
        page_dialog_mgr_->showDialog("recharge_exit", 0);
    } else if (!bluetooth_status_ || !bluetooth_key_status_) {
        page_dialog_mgr_->showDialog("recharge_exit", 0);
    }
    if (tft_audio_) {
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
    }
}

void RechargeManager::onCruiseModeSignalForExit(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!msg) return;
    std::lock_guard<std::mutex> lock(mutex_);
    last_cruise_mode_signal_ = msg->data;
}

void RechargeManager::onJoystickForRechargeExit(const interface::msg::Joystick::SharedPtr msg) {
    if (!msg) return;
    std::lock_guard<std::mutex> lock(mutex_);
    last_joystick_x_ = static_cast<int16_t>(msg->rocker_x);
}

void RechargeManager::evaluateExitRechargeOnDriverOverride() {
    if (!recharge_client_ || !node_) return;
    TftStateMachine& sm = TftStateMachine::instance();
    if (sm.getSceneMode() != "recharge") {
        std::lock_guard<std::mutex> lock(mutex_);
        recharge_exit_override_latched_ = false;
        return;
    }

    int cruise_sig = 0;
    int16_t joy_x = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cruise_sig = last_cruise_mode_signal_;
        joy_x = last_joystick_x_;
    }

    if (cruise_sig == 0) return;
    if (!(joy_x > kRechargeExitJoystickThreshold || joy_x < -kRechargeExitJoystickThreshold)) return;

    auto info_opt = sm.getCarInfo();
    if (!info_opt) return;
    const auto& info = *info_opt;
    // CarInfo：lockswitch 1=锁车/驻车；left/right_brakests 1=手动抱闸打开
    if (info.lockswitch != 1) return;
    if (info.left_brakests != 1 || info.right_brakests != 1) return;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (recharge_exit_override_latched_) return;
        recharge_exit_override_latched_ = true;
    }

    RCLCPP_INFO(rclcpp::get_logger("RechargeManager"),
                "Exit recharge: parked + cruise signal + joystick x=%d + brakes open", static_cast<int>(joy_x));
    auto request = std::make_shared<interface::srv::SceneControls::Request>();
    request->start = 0;
    recharge_client_->async_send_request(request);
}

void RechargeManager::onReadResponse(uint16_t reg, uint16_t value) {
    onRechargeButtonTouch(reg, value);
}

void RechargeManager::onRechargeButtonTouch(uint16_t reg, uint16_t value) {
    if (!tft_connected_ || !tft_connected_()) return;
    if (reg != TOUCH_EVENT_RECHARGE_START) return;
    if (!page_dialog_mgr_) return;
    TftStateMachine& sm = TftStateMachine::instance();
    auto info_opt = sm.getCarInfo();
    if (!info_opt) {
        RCLCPP_INFO(rclcpp::get_logger("RechargeManager"), "onRechargeButtonTouch info_opt: null");
        return;
    }
    const auto& info = *info_opt;
    RCLCPP_INFO(rclcpp::get_logger("RechargeManager"), "onRechargeButtonTouch info: %d", info.getcharging);
    if (info.getcharging == 1) {
        RCLCPP_INFO(rclcpp::get_logger("RechargeManager"), "onRechargeButtonTouch getcharging == 1");
        page_dialog_mgr_->showDialog("recharge_block", recharge_failure_type::RECHARGE_FAILURE_TYPE_POWER_OFF);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if (info.getcharging == 2) {
        RCLCPP_INFO(rclcpp::get_logger("RechargeManager"), "onRechargeButtonTouch getcharging == 2");
        page_dialog_mgr_->showDialog("recharge_block", recharge_failure_type::RECHARGE_FAILURE_TYPE_CHARGING);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if (info.lockswitch == 1) {
        RCLCPP_INFO(rclcpp::get_logger("RechargeManager"), "onRechargeButtonTouch lockswitch == 1");
        page_dialog_mgr_->showDialog("recharge_block", recharge_failure_type::RECHARGE_FAILURE_TYPE_PARKING_RELEASED);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if(bluetooth_status_ == false) {
        RCLCPP_INFO(rclcpp::get_logger("RechargeManager"), "onRechargeButtonTouch bluetooth status: %d", bluetooth_status_);
        page_dialog_mgr_->showDialog("recharge_block", recharge_failure_type::RECHARGE_FAILURE_TYPE_MOVE_TO_BASE);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if (bluetooth_key_status_ == false) {
        RCLCPP_INFO(rclcpp::get_logger("RechargeManager"), "onRechargeButtonTouch bluetooth key status: %d", bluetooth_key_status_);
        page_dialog_mgr_->showDialog("recharge_block", recharge_failure_type::RECHARGE_FAILURE_TYPE_BASE_LEAVED);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if (sm.isPushingMode()) {
        RCLCPP_INFO(rclcpp::get_logger("RechargeManager"), "onRechargeButtonTouch pushing mode");
        page_dialog_mgr_->showDialog("recharge_block", recharge_failure_type::RECHARGE_FAILURE_TYPE_DRIVING_MODE);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if (info.left_brakests == 1 || info.right_brakests == 1) {
        RCLCPP_INFO(rclcpp::get_logger("RechargeManager"), "onRechargeButtonTouch brake closed");
        page_dialog_mgr_->showDialog("recharge_block", recharge_failure_type::RECHARGE_FAILURE_TYPE_BRAKE_CLOSED);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("RechargeManager"), "onRechargeButtonTouch start recharge");
    auto request = std::make_shared<interface::srv::SceneControls::Request>();
    request->start = 1;
    recharge_client_->async_send_request(request);
    
}

void RechargeManager::attachStateMachineCallbacks() {
    TftStateMachine::instance().setOnRechargeSceneChanged([this](bool in_recharge) {
        if (!tft_audio_) return;
        if (in_recharge) {
            tft_audio_->playTtsCode(TTS_CODE_TYPE_RECHARGE);
        } else {
            tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        }
    });

    TftStateMachine::instance().setOnChargingStatusChanged([this](int status) {
        if (tft_audio_ && last_charging_status_for_audio_ >= 0 &&
            last_charging_status_for_audio_ == static_cast<int>(ChargingConnectStatus::NotCharging) &&
            (status == static_cast<int>(ChargingConnectStatus::BaseStationCharging))) {
            tft_audio_->playTtsCode(TTS_CODE_TYPE_CHARGING);
        }
        last_charging_status_for_audio_ = status;

        if (!tft_connected_ || !tft_connected_()) return;
        // 进入线充/基站充页时沿用 gate（仅主页才跟 CarInfo 切充电 UI）；
        // 结束充电须从充电页回到主页，若此处也检查 gate，在充电页上 gate 为 false 会永远不返回主页。
        if (status != 0 && charging_page_gate_ && !charging_page_gate_()) return;

        if (!page_dialog_mgr_) return;
        switch (status) {
            case 0:
                page_dialog_mgr_->switchToPage(MAIN_PAGE_ID);
                break;
            case 1:
                page_dialog_mgr_->switchToPage(LINE_CHARGE_PAGE_ID);
                break;
            case 2:
                page_dialog_mgr_->switchToPage(BASE_STATION_CHARGE_PAGE_ID);
                break;
            default:
                break;
        }
    });
}

}  // namespace tft_service
