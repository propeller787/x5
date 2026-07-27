/**
 * @file physical_button_handler.cpp
 * @brief 物理按键与弹窗联动实现
 */

#include "physical_button_handler.h"
#include "dialog/page_dialog_config.h"
#include <utils/topic.h>

namespace tft_service {

PhysicalButtonHandler::PhysicalButtonHandler(rclcpp::Node* node, PageDialogManager* dialog_mgr, TftAudioPublisher* tft_audio)
    : node_(node)
    , dialog_mgr_(dialog_mgr)
    , tft_audio_(tft_audio)
    , logger_(rclcpp::get_logger("PhysicalButtonHandler"))
{
    if (!node_ || !dialog_mgr_) return;

    // 接收到 retry 动作时，关闭弹窗并重新尝试上一个动作（依旧通过 onSafetyFunction/onCruiseMode 逻辑判断）
    dialog_mgr_->setOnActionCallback([this](TouchActionType action, const std::string& dialog_id) {
        if (action == TouchActionType::Retry && dialog_id == "obstacle_avoidance_block") {
            auto m = std::make_shared<std_msgs::msg::Int32>();
            m->data = last_safety_function_intent_;
            onSafetyFunction(m);
        } else if (action == TouchActionType::Retry && dialog_id == "cruise_block") {
            auto m = std::make_shared<std_msgs::msg::Int32>();
            m->data = last_cruise_mode_intent_;
            onCruiseMode(m);
        }
    });

    auto qos = rclcpp::QoS(10);
    safety_function_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        safetyFunctionTopic, qos,
        [this](const std_msgs::msg::Int32::SharedPtr msg) { onSafetyFunction(msg); });
    cruise_mode_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        cruiseModeTopic, qos,
        [this](const std_msgs::msg::Int32::SharedPtr msg) { onCruiseMode(msg); });
    cruise_process_state_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        cursorAlarmTopic, qos,
        [this](const std_msgs::msg::Int32::SharedPtr msg) { onCruiseProcessState(msg); });

    scene_auxiliary_client_ = node_->create_client<interface::srv::SceneControls>(sceneAuxiliaryService);
    manage_cruise_client_ = node_->create_client<interface::srv::SceneControls>(manageCruiseService);
}

bool PhysicalButtonHandler::isHelpcarMode() const {
    return TftStateMachine::instance().isPushingMode();
}

bool PhysicalButtonHandler::isHandBrakeOpen() const {
    return TftStateMachine::instance().getBrakeStatus();
}

bool PhysicalButtonHandler::isLineCharging() const {
    return TftStateMachine::instance().isLineChargingMode();
}

bool PhysicalButtonHandler::isParking() const {
   return TftStateMachine::instance().isParkingMode();
}

bool PhysicalButtonHandler::isBackwardMovement() const {
    return TftStateMachine::instance().isBackwardMovement();
}

void PhysicalButtonHandler::callSceneAuxiliary(int start) {
    if (!scene_auxiliary_client_->service_is_ready()) {
        RCLCPP_WARN(logger_, "sceneAuxiliaryService not ready");
        return;
    }
    auto req = std::make_shared<interface::srv::SceneControls::Request>();
    req->start = static_cast<int16_t>(start);
    scene_auxiliary_client_->async_send_request(req);
}

void PhysicalButtonHandler::callManageCruise(int start) {
    if (!manage_cruise_client_->service_is_ready()) {
        RCLCPP_WARN(logger_, "manageCruiseService not ready");
        return;
    }
    auto req = std::make_shared<interface::srv::SceneControls::Request>();
    req->start = static_cast<int16_t>(start);
    manage_cruise_client_->async_send_request(req);
}

void PhysicalButtonHandler::onSafetyFunction(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!msg || !dialog_mgr_) return;
    last_safety_function_intent_ = msg->data;
    if (isHandBrakeOpen()) {
        dialog_mgr_->showDialog("obstacle_avoidance_block", physical_button_failure_type::PHYSICAL_BUTTON_FAILURE_TYPE_BRAKE_CLOSED);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if (isHelpcarMode()) {
        dialog_mgr_->showDialog("obstacle_avoidance_block", physical_button_failure_type::PHYSICAL_BUTTON_FAILURE_TYPE_DRIVING_MODE);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    callSceneAuxiliary(msg->data);
}

void PhysicalButtonHandler::onCruiseMode(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!msg || !dialog_mgr_) return;
    last_cruise_mode_intent_ = msg->data;
    if (isHandBrakeOpen()) {
        dialog_mgr_->showDialog("cruise_block", cruise_mode_failure_type::CRUISE_MODE_FAILURE_TYPE_BRAKE_CLOSED);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if (isHelpcarMode()) {
        dialog_mgr_->showDialog("cruise_block", cruise_mode_failure_type::CRUISE_MODE_FAILURE_TYPE_DRIVING_MODE);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if (isLineCharging()) {
        dialog_mgr_->showDialog("cruise_block", cruise_mode_failure_type::CRUISE_MODE_FAILURE_TYPE_POWER_OFF);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if (isParking()) {
        dialog_mgr_->showDialog("cruise_block", cruise_mode_failure_type::CRUISE_MODE_FAILURE_TYPE_PARKING_RELEASED);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if (isBackwardMovement()) {
        dialog_mgr_->showDialog("cruise_block", cruise_mode_failure_type::CRUISE_MODE_FAILURE_TYPE_FORWARD_MOVEMENT);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    callManageCruise(msg->data);
}

void PhysicalButtonHandler::onCruiseProcessState(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!msg || !dialog_mgr_) return;
    int v = msg->data;
    if (v == 1) {
        dialog_mgr_->showDialog("cruise_exit", cruise_process_state_type::CRUISE_PROCESS_STATE_TYPE_EXIT);
        return;
    }else if (v == 2) {
        dialog_mgr_->showDialog("cruise_exit", cruise_process_state_type::CRUISE_PROCESS_STATE_TYPE_ACTIVE);
        return;
    }
}

}  // namespace tft_service
