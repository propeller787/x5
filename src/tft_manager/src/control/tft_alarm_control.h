#pragma once
// 状态报警
#include "control.h"

namespace tft_service::control {
class AlarmControl : public Control {
  public:
    AlarmControl();
    ~AlarmControl();

  private:
    void alarmCallback(const interface::msg::AuxiliaryAlarm::SharedPtr msg);

  private:
    rclcpp::Subscription<interface::msg::AuxiliaryAlarm>::SharedPtr auxiliary_alarm_sub_back_{nullptr};
    int16_t index_{0};
    // rclcpp::Subscription<interface::msg::AuxiliaryAlarm>::SharedPtr auxiliary_alarm_sub_front_;
};
} // namespace tft_service::control