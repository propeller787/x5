#pragma once

#include "muduo/net/EventLoop.h"
#include "muduo/net/EventLoopThread.h"
#include <interface/srv/gears_controls.hpp>
#include <interface/srv/tft_controls.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <rclcpp/rclcpp.hpp>
#include <dirent.h>
#include <unistd.h>
#include <sys/types.h>
#include <fstream>
#include <string>
#include <algorithm>
#include <cctype>
#include <iostream>
namespace sensor::chassis {

class CanTransreceiver;
class CanAdapter;

class ChassisComponent : public rclcpp::Node {
  public:
    ChassisComponent(const rclcpp::NodeOptions &options);
    ~ChassisComponent();

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
    bool isProcessRunning(const std::string &processName);

  private:
    std::unordered_map<std::string, std::shared_ptr<CanTransreceiver>> can_devs_;
    std::unordered_map<std::string, std::shared_ptr<CanAdapter>> adapters_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr car_info_timer_;
    rclcpp::Service<interface::srv::GearsControls>::SharedPtr gears_control_service_;
    rclcpp::Service<interface::srv::TftControls>::SharedPtr tft_control_service_;
    int16_t cell_conut_{0};
    bool is_process_{false};
    int process_check_count_{0};  // 进程检查次数计数器
    bool cached_process_status_{false};  // 缓存最后一次检查的结果
};

} // namespace sensor::chassis