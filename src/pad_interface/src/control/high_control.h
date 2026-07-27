#pragma once
#include "control.h"

namespace service::control {

class HighControl : public Control {

public:
    HighControl();

private:
    void handleHigh(const std::shared_ptr<interface::srv::High::Request> request,
        std::shared_ptr<interface::srv::High::Response> response);

private:
    // high控制
    rclcpp::Service<interface::srv::High>::SharedPtr car_high_ { nullptr };
};

}