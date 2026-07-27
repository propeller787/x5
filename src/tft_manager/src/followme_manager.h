/*
* @file followme_manager.h
* @brief 智能跟随管理器
* @author likang
* @date 2026-04-03
*/

#pragma once

#include "control/define.h"
#include "tft_state_machine.h"
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <mutex>
#include <std_msgs/msg/int32.hpp>
#include <interface/srv/scene_controls.hpp>
#include <atomic>
#include <thread>

namespace tft_service {

class TftAudioPublisher;

enum class FollowmeStatus {
    Searching = 0, // 寻人中
    Following = 1, // 跟随中
    Stopped = 2, // 跟随结束
};

class PageDialogManager;
class FollowmeManager {
public:
    static FollowmeManager& instance();
    void init(rclcpp::Node* node);
    bool isInitialized() const { return node_ != nullptr; }
    void setTftAudio(TftAudioPublisher* audio) { tft_audio_ = audio; }
    void setPageDialogManager(PageDialogManager* mgr);
    void setOnSwitchPage(std::function<void(int page_id)> cb) { on_switch_page_ = std::move(cb); }
    void setTftConnectedPredicate(std::function<bool()> pred) { tft_connected_ = std::move(pred); }
    void setChargingPageUpdateGate(std::function<bool()> gate) { charging_page_gate_ = std::move(gate); }
    /* Modbus 读响应：智能跟随按钮等 */
    void onReadResponse(uint16_t reg, uint16_t value);
    FollowmeManager(const FollowmeManager&) = delete;
    FollowmeManager& operator=(const FollowmeManager&) = delete;
private:
    FollowmeManager() = default;

    void onFollowmeStatusForAudio(const std_msgs::msg::Int32::SharedPtr msg);

    rclcpp::Node* node_{nullptr};
    TftAudioPublisher* tft_audio_{nullptr};
    PageDialogManager* page_dialog_mgr_{nullptr};
    std::function<void(int page_id)> on_switch_page_{};
    std::function<bool()> tft_connected_{};
    std::function<bool()> charging_page_gate_{};
    rclcpp::Client<interface::srv::SceneControls>::SharedPtr followme_client_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr followme_status_sub_;
    std::mutex mutex_;
    std::thread thread_;
    std::atomic<bool> is_running_{false};
    std::atomic<bool> is_initialized_{false};
};

}