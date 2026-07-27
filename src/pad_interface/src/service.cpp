#include "service.h"

namespace service {

using namespace interface::msg;
using namespace interface::srv;
static muduo::net::EventLoopThread ControlServiceThread;
ControlService::ControlService(const rclcpp::NodeOptions& options)
    : rclcpp::Node("ControlService", options)
    , logger_(rclcpp::get_logger("ControlService"))
{
    setupControls();
}

void ControlService::setupControls()
{
    controls_["carControl"] = std::make_shared<CarControl>();
    controls_["followControl"] = std::make_shared<FollowControl>();
    controls_["healthControl"] = std::make_shared<HealthControl>();
    controls_["operationControl"] = std::make_shared<OperationControl>();
    controls_["summonControl"] = std::make_shared<SummonControl>();
    controls_["highControl"] = std::make_shared<HighControl>();
    controls_["idControl"] = std::make_shared<IdControl>();
    controls_["gearsControl"] = std::make_shared<GearsControl>();
    // controls_["danceControl"] = std::make_shared<DanceControl>();
    controls_["tftControl"] = std::make_shared<TftControl>();
    controls_["sceneControl"] = std::make_shared<SceneControl>();
    // controls_["waypointControl"] = std::make_shared<WaypointControl>();
}

}