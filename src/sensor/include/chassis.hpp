#pragma once

#include "muduo/net/EventLoop.h"
#include "muduo/net/EventLoopThread.h"
#include <interface/srv/gears_controls.hpp>
#include <interface/srv/tft_controls.hpp>
#include <interface/msg/vehicle_bind_status.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <dirent.h>
#include <unistd.h>
#include <sys/types.h>
#include <fstream>
#include <string>
#include <algorithm>
#include <cctype>
#include <iostream>
#include <mutex>
#include "define.h"
namespace sensor::chassis {

class CanTransreceiver;
class CanAdapter;

class ChassisComponent : public rclcpp::Node {
  public:
    ChassisComponent(const rclcpp::NodeOptions &options);
    ~ChassisComponent();
    
    // 获取当前模式类型
    sensor::ModeType getCurrentModeType() const;

  private:
    void setupConfigures();
    void setupTansreceivers();
    void setupAdapters();
    void onTimer();
    void sendCarInfo();

  private:
    void handleGearControls(const std::shared_ptr<interface::srv::GearsControls::Request> request,
                            std::shared_ptr<interface::srv::GearsControls::Response> response);
    void handleTftControls(const std::shared_ptr<interface::srv::TftControls::Request> request,
                           std::shared_ptr<interface::srv::TftControls::Response> response);
    void modeCallback(const std_msgs::msg::String::SharedPtr msg);
    void auxiliaryStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void upgradePrepareCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void maxGearCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void signStatusCallback(const interface::msg::VehicleBindStatus::SharedPtr msg);
    void languageCallback(const std_msgs::msg::Int32::SharedPtr msg);
    
    sensor::ModeType getModeType(const std::string &mode_str);

  private:
    std::unordered_map<std::string, std::shared_ptr<CanTransreceiver>> can_devs_;
    std::unordered_map<std::string, std::shared_ptr<CanAdapter>> adapters_;
    std::shared_ptr<rclcpp::Executor> adapter_executor_;  // 在 ChassisAdapterThread 中 spin HandleCanAdapter
    rclcpp::CallbackGroup::SharedPtr cb_group_;            // 让多线程 executor 把 chassis 的回调放在独立线程
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr car_info_timer_;
    rclcpp::Service<interface::srv::GearsControls>::SharedPtr gears_control_service_;
    rclcpp::Service<interface::srv::TftControls>::SharedPtr tft_control_service_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auxiliary_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr upgrade_prepare_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr max_gear_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr language_sub_;
    rclcpp::Subscription<interface::msg::VehicleBindStatus>::SharedPtr sign_status_sub_;
    int16_t cell_count_{0};
    bool is_process_{false};
    sensor::ModeType current_mode_type_{sensor::ModeType::MODE_TYPE_STANDARD};  // 当前模式类型：1=巡航 2=回充 3=推行 4=标准
    sensor::SafetyStatus current_safety_status_{sensor::SafetyStatus::SAFETY_STATUS_ON};  // 当前安全功能状态：0=关闭 1=开启
    mutable std::mutex state_mutex_;  // 保护 current_mode_type_ / current_fafety_status_ 的互斥锁
    int bluetooth_mac_count_{0};
    bool send_cell_pack_next_{false};

    int max_gear_{-1};
    int language_{-1};
    int sign_status_{-1};
};

} // namespace sensor::chassis