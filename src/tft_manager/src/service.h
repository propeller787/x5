#pragma once

#include "control/control.h"
#include "control/tft_alarm_control.h"
#include "control/tft_car_control.h"
#include "control/tft_charger_control.h"
#include "control/tft_cruise_control.h"
#include "control/tft_health_control.h"
#include "control/tft_safety_control.h"
#include "muduo/net/EventLoop.h"
#include "muduo/net/EventLoopThread.h"
#include "tft/tft_module.h"
#include <atomic>
#include <chrono>
#include <future>
#include <interface/msg/control.hpp>
#include <muduo/net/TimerId.h>
#include <rclcpp/rclcpp.hpp>
#include <utils/topic.h>
using namespace tft_service::control;

namespace tft_service {
class TftService : public rclcpp::Node {
public:
    TftService(const rclcpp::NodeOptions& options);

private:
    void setupControls();

private:
    rclcpp::Logger logger_;
    std::unordered_map<std::string, std::shared_ptr<Control>> controls_;
};

}