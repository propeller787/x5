/**
 * @file physical_button_handler.h
 * @brief 物理按键与弹窗联动：订阅 /vlago/physical/button/failure、/vlago/safety/function、
 *        /vlago/cruise/mode，按条件弹窗或调用服务。
 */

#pragma once

#include "dialog/page_dialog_manager.h"
#include "tft_audio.h"
#include "tft_state_machine.h"
#include <interface/srv/scene_controls.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <std_msgs/msg/int32.hpp>

namespace tft_service {

/**
 * @brief 物理按键处理器：根据 CarInfo、场景模式判断是否弹窗，否则调用对应服务。
 */
class PhysicalButtonHandler {
public:
    // explicit PhysicalButtonHandler(rclcpp::Node* node, PageDialogManager* dialog_mgr);

    explicit PhysicalButtonHandler(rclcpp::Node* node, PageDialogManager* dialog_mgr, TftAudioPublisher* tft_audio);
    ~PhysicalButtonHandler() = default;

private:
    void onSafetyFunction(const std_msgs::msg::Int32::SharedPtr msg);
    void onCruiseMode(const std_msgs::msg::Int32::SharedPtr msg);
    void onCruiseProcessState(const std_msgs::msg::Int32::SharedPtr msg);
    // 是否为推行模式
    bool isHelpcarMode() const;
    // 是否为手动抱闸打开
    bool isHandBrakeOpen() const;
    // 是否为线充模式
    bool isLineCharging() const;
    // 是否为驻车模式
    bool isParking() const;

    // 是否为向后行拨动摇杆
    bool isBackwardMovement() const;

    void callSceneAuxiliary(int start);
    void callManageCruise(int start);

    rclcpp::Node* node_{nullptr};
    PageDialogManager* dialog_mgr_{nullptr};
    // 新增功能退出的语音提示
    TftAudioPublisher* tft_audio_{nullptr};
    rclcpp::Logger logger_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr safety_function_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cruise_mode_sub_;
    rclcpp::Client<interface::srv::SceneControls>::SharedPtr scene_auxiliary_client_;
    rclcpp::Client<interface::srv::SceneControls>::SharedPtr manage_cruise_client_;
    // 巡航过程状态
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cruise_process_state_sub_;
    int last_safety_function_intent_{0};   ///< 上次安全功能意图（用于 obstacle_avoidance_block retry）
    int last_cruise_mode_intent_{0};       ///< 上次巡航模式意图（用于 cruise_block retry）
};

}  // namespace tft_service
