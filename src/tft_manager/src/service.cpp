#include "service.h"
using namespace tft_service::control;
namespace tft_service {

using namespace interface::msg;
using namespace interface::srv;

TftService::TftService(const rclcpp::NodeOptions& options)
    : rclcpp::Node("TftService", options)
    , logger_(rclcpp::get_logger("TftService"))
{
    setupControls();
}

void TftService::setupControls()
{
    controls_["carControl"] = std::make_shared<CarControl>();
    controls_["alarmControl"] = std::make_shared<AlarmControl>();
    controls_["chargerControl"] = std::make_shared<ChargerControl>();
    controls_["cruiseControl"] = std::make_shared<CruiseControl>();
    controls_["safetyControl"] = std::make_shared<SafetyControl>();
}

}