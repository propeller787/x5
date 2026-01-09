#pragma once
#include "../global.h"
#include "define.h"
#include <deque>
#include <interface/msg/auxiliary_alarm.hpp>
#include <interface/msg/backup_car.hpp>
#include <interface/msg/car_info.hpp>
#include <interface/msg/control.hpp>
#include <interface/msg/health.hpp>
#include <interface/msg/health_start.hpp>
#include <interface/srv/car_controls.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <utils/topic.h>
namespace tft_service::control {
using namespace std::placeholders;
class Control {
  public:
    Control(const std::string &name);

  public:
    tft_service::Global *global_;

  protected:
    rclcpp::Logger logger_;

  public:
    const std::string name_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr audio_pub_{nullptr};
};

} // namespace tft_service::control