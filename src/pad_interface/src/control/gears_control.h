#pragma once
#include "control.h"
#include <cmath>

namespace service::control {

class GearsControl : public Control {
public:
    GearsControl();
    ~GearsControl();

private:
    void handleGearsStart(const std::shared_ptr<interface::srv::GearsControls::Request> request,
        std::shared_ptr<interface::srv::GearsControls::Response> response);

private:
    rclcpp::Service<interface::srv::GearsControls>::SharedPtr gears_start_ { nullptr };
    rclcpp::Client<interface::srv::GearsControls>::SharedPtr gears_control_client_ { nullptr };
};

}