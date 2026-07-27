#pragma once
#include "control.h"
namespace service::control {

class IdControl : public Control 
{
public:
    IdControl();
    ~IdControl();

private:
    void handleIdControls(const std::shared_ptr<interface::srv::IdControls::Request> request,
        std::shared_ptr<interface::srv::IdControls::Response> response);

private:
    rclcpp::Service<interface::srv::IdControls>::SharedPtr id_controls_ { nullptr };
};

}
