#include "gears_control.h"

namespace service::control {
using namespace std::placeholders;
GearsControl::GearsControl()
    : Control("gears")
{
    auto global = service::Global::getInstance();
    gears_start_ = global->node_->create_service<interface::srv::GearsControls>(gearsControlService, std::bind(&GearsControl::handleGearsStart, this, _1, _2));
    gears_control_client_ = global->node_->create_client<interface::srv::GearsControls>(sensorGearControlService);
}

GearsControl::~GearsControl()
{
}

void GearsControl::handleGearsStart(const std::shared_ptr<interface::srv::GearsControls::Request> request,
    std::shared_ptr<interface::srv::GearsControls::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("GearsControl"), "Gear: %d, Helpcar: %d, SOS: %d", request->gear, request->helpcar, request->sos);
    auto futer = gears_control_client_->async_send_request(request);
    auto status = futer.wait_for(std::chrono::seconds(1));
    if(status == std::future_status::timeout) {
        RCLCPP_ERROR(rclcpp::get_logger("GearsControl"), "Service response timeout");
        response->code = 0;  // 超时返回-1
        return;
    }
    auto result = futer.get();
    response->code = (result->code == 0) ? 0 : -1;
}

} // namespace service::control
