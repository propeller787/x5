#pragma once
// 巡航
#include "control.h"
#include <interface/msg/named_status.hpp>
#include <interface/msg/report_mode.hpp>
#include <std_msgs/msg/int32.hpp>
namespace tft_service::control {
class CruiseControl : public Control {
public:
    CruiseControl();
    ~CruiseControl();

private:
    void handleReportMode(const interface::msg::ReportMode msg);
    void handleCruiseAlarm(const std_msgs::msg::Int32 msg);
    void handleModeStatusStr(const std_msgs::msg::String msg);

private:
    rclcpp::Subscription<interface::msg::ReportMode>::SharedPtr mode_status_ptr_ { nullptr };
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_status_str_ptr_ { nullptr };
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cruise_alarm_sub_; // 巡航前方障碍物报警
private:
    bool cruise_status_ = false; // 巡航状态
};
}