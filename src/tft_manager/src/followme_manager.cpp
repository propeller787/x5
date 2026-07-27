/**
 * @file followme_manager.cpp
 * @brief FollowmeManager 实现。
 */

#include "followme_manager.h"
#include "dialog/page_dialog_manager.h"
#include "tft_audio.h"
#include <utils/topic.h>

namespace tft_service {

FollowmeManager& FollowmeManager::instance()
{
    static FollowmeManager inst;
    return inst;
}

void FollowmeManager::init(rclcpp::Node* node)
{
    if (!node)
        return;
    node_ = node;

    auto qos = rclcpp::QoS(10);
    followme_status_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        followmeStatusTopic, qos,
        [this](const std_msgs::msg::Int32::SharedPtr msg) { onFollowmeStatusForAudio(msg); });
    followme_client_ = node_->create_client<interface::srv::SceneControls>(manageFollowmeService);
    RCLCPP_INFO(rclcpp::get_logger("FollowmeManager"), "FollowmeManager initialized");
}

void FollowmeManager::setPageDialogManager(PageDialogManager* mgr)
{
    page_dialog_mgr_ = mgr;
}

void FollowmeManager::onReadResponse(uint16_t reg, uint16_t /*value*/)
{
    if (!tft_connected_ || !tft_connected_())
        return;
    if (reg != TOUCH_EVENT_SMART_FOLLOW_START_FOLLOW)
        return;
    if (!page_dialog_mgr_)
        return;
    TftStateMachine& sm = TftStateMachine::instance();
    auto info_opt = sm.getCarInfo();
    if (!info_opt) {
        RCLCPP_INFO(rclcpp::get_logger("FollowmeManager"), "onReadResponse info_opt: null");
        return;
    }
    const auto& info = *info_opt;
    RCLCPP_INFO(rclcpp::get_logger("FollowmeManager"), "onReadResponse car info getcharging: %d", info.getcharging);
    if (info.getcharging == 1) {
        RCLCPP_INFO(rclcpp::get_logger("FollowmeManager"), "followme blocked: power off / line charge");
        page_dialog_mgr_->showDialog("recharge_block", recharge_failure_type::RECHARGE_FAILURE_TYPE_POWER_OFF);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if (info.getcharging == 2) {
        RCLCPP_INFO(rclcpp::get_logger("FollowmeManager"), "followme blocked: charging");
        page_dialog_mgr_->showDialog("recharge_block", recharge_failure_type::RECHARGE_FAILURE_TYPE_CHARGING);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if (info.lockswitch == 1) {
        RCLCPP_INFO(rclcpp::get_logger("FollowmeManager"), "followme blocked: parking lock");
        page_dialog_mgr_->showDialog("recharge_block", recharge_failure_type::RECHARGE_FAILURE_TYPE_PARKING_RELEASED);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if (sm.isPushingMode()) {
        RCLCPP_INFO(rclcpp::get_logger("FollowmeManager"), "followme blocked: pushing mode");
        page_dialog_mgr_->showDialog("recharge_block", recharge_failure_type::RECHARGE_FAILURE_TYPE_DRIVING_MODE);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if (info.left_brakests == 1 || info.right_brakests == 1) {
        RCLCPP_INFO(rclcpp::get_logger("FollowmeManager"), "followme blocked: manual brake");
        page_dialog_mgr_->showDialog("recharge_block", recharge_failure_type::RECHARGE_FAILURE_TYPE_BRAKE_CLOSED);
        tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        return;
    }
    if (!followme_client_) {
        RCLCPP_WARN(rclcpp::get_logger("FollowmeManager"), "followme client not initialized");
        return;
    }
    auto request = std::make_shared<interface::srv::SceneControls::Request>();
    request->start = 1;
    followme_client_->async_send_request(request);
}

void FollowmeManager::onFollowmeStatusForAudio(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (!msg)
        return;
    int status = msg->data;
    if (!page_dialog_mgr_)
        return;
    if (!tft_connected_ || !tft_connected_())
        return;
    if (status == static_cast<int>(FollowmeStatus::Following)) {
        if (tft_audio_)
            tft_audio_->playTtsCode(TTS_CODE_TYPE_FOLLOWING);
        page_dialog_mgr_->switchToPage(SMART_FOLLOW_START_FOLLOW_PAGE_ID);
    } else if (status == static_cast<int>(FollowmeStatus::Stopped)) {
        page_dialog_mgr_->switchToPage(MAIN_PAGE_ID);
        if (followme_client_) {
            auto request = std::make_shared<interface::srv::SceneControls::Request>();
            request->start = 0;
            followme_client_->async_send_request(request);
        }
    } else if (status == static_cast<int>(FollowmeStatus::Searching)) { // 寻人中
        page_dialog_mgr_->switchToPage(SMART_FOLLOW_ACTIVE_PAGE_ID);
    }
}

} // namespace tft_service