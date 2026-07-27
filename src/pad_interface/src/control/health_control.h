#pragma once
#include "control.h"

namespace service::control {

class HealthControl : public Control {

public:
    HealthControl();
    ~HealthControl();

private:
    void handleHrvStart(const std::shared_ptr<interface::srv::HrvStart::Request> request,
        std::shared_ptr<interface::srv::HrvStart::Response> response);
    void handleHrvInfo(const std::shared_ptr<interface::srv::HrvInfo::Request> request,
        std::shared_ptr<interface::srv::HrvInfo::Response> response);
    void sendHrvStop();
    void onHrvResult(const interface::msg::Health msg);

private:
    rclcpp::Subscription<interface::msg::Health>::SharedPtr health_ptr_ { nullptr };
    interface::msg::Health health_;
    bool status_ = false; // true 采集中
    std::deque<int16_t> acdata_;
    rclcpp::Publisher<interface::msg::HealthStart>::SharedPtr health_sta_ptr_ { nullptr };
    rclcpp::Service<interface::srv::HrvStart>::SharedPtr hrv_start_ { nullptr };
    std::atomic<bool> is_triggered_ = false; // 使用 atomic 保证线程安全
    std::future<void> future_callback;
    rclcpp::Service<interface::srv::HrvInfo>::SharedPtr hrv_info_ { nullptr };
};

}