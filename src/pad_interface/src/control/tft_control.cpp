#include "tft_control.h"

namespace service::control {

TftControl::TftControl()
    : Control("tft_control")
{
    auto global = service::Global::getInstance();
    tft_start_ = global->node_->create_service<interface::srv::TftControls>(tftControlService, std::bind(&TftControl::handleTftControl, this, _1, _2));
    tft_client_ = global->node_->create_client<interface::srv::TftControls>(sensorTftControlService);
}

TftControl::~TftControl()
{
}

void TftControl::handleTftControl(const std::shared_ptr<interface::srv::TftControls::Request> request,
    std::shared_ptr<interface::srv::TftControls::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "tft_control: %d, %d", request->turns_switch, request->light_type);
    tft_client_->async_send_request(request);
    response->set__code(0);
}

}
