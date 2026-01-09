#pragma once
// 安全预警
#include "control.h"
#include <interface/msg/auxiliary_alarm.hpp>
#include <chrono>

namespace tft_service::control {
class SafetyControl : public Control {
  public:
    SafetyControl();
    ~SafetyControl();

  private:
    void handleAuxiliaryAlarm(const interface::msg::AuxiliaryAlarm msg);
    void handleFootAlarm(const std_msgs::msg::Int32 msg);

  private:
    rclcpp::Subscription<interface::msg::AuxiliaryAlarm>::SharedPtr sub_auxiliary_alarm_{nullptr};
    int16_t index_ararm_{0};
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_foot_alarm_{nullptr};
    int16_t foot_alarm_level_{0};
    std::chrono::steady_clock::time_point last_aux_alarm_time_{};
};
} // namespace tft_service::control