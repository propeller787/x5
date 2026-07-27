#pragma once
#include "control.h"

namespace service::control {
class TftControl : public Control {
public:
    TftControl();
    ~TftControl();

private:
    void handleTftControl(const std::shared_ptr<interface::srv::TftControls::Request> request,
        std::shared_ptr<interface::srv::TftControls::Response> response);
private:
    rclcpp::Service<interface::srv::TftControls>::SharedPtr tft_start_ { nullptr };
    rclcpp::Client<interface::srv::TftControls>::SharedPtr tft_client_ { nullptr };
};
}

