#pragma once
#include <interface/srv/car_controls.hpp>
#include <interface/srv/follow_me.hpp>
#include <interface/srv/follow_status.hpp>
#include <interface/srv/hrv_info.hpp>
#include <interface/srv/hrv_start.hpp>
#include <interface/srv/operation_scenario.hpp>
#include <interface/srv/operation_status.hpp>

#include "control/car_control.h"
#include "control/follow_control.h"
#include "control/health_control.h"
#include "control/high_control.h"
#include "control/operation_control.h"
#include "control/summon_control.h"
#include "control/waypoint_control.h"
#include "control/id_control.h"
#include "control/gears_control.h"
#include "control/dance_control.h"
#include "control/tft_control.h"
#include "control/scene_control.h"
#include <atomic>
#include <chrono>
#include <future>
#include <interface/msg/control.hpp>
#include <interface/msg/health.hpp>
#include <interface/msg/health_start.hpp>

#include "muduo/net/EventLoop.h"
#include "muduo/net/EventLoopThread.h"
#include <muduo/net/TimerId.h>
#include <rclcpp/rclcpp.hpp>
#include <utils/topic.h>

using namespace service::control;

namespace service {
class ControlService : public rclcpp::Node {
public:
    ControlService(const rclcpp::NodeOptions& options);

private:
    void setupControls();

private:
    rclcpp::Logger logger_;
    std::unordered_map<std::string, std::shared_ptr<Control>> controls_;
};

}