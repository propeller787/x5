#pragma once
#include "control.h"
namespace service::control {

class DanceControl : public Control {
public:
    DanceControl();
    ~DanceControl();
private:
    void handleDanceControl(const std::shared_ptr<interface::srv::DanceControls::Request> request,
        std::shared_ptr<interface::srv::DanceControls::Response> response);
private:
    rclcpp::Service<interface::srv::DanceControls>::SharedPtr dance_control_ { nullptr };
    int16_t type_ { 0 };
    int16_t musicalscore_ { 0 };
};

}
