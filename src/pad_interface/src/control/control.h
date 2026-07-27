#pragma once
#include "../global.h"
#include <deque>
#include <interface/msg/car_info.hpp>
#include <interface/msg/control.hpp>
#include <interface/msg/health.hpp>
#include <interface/msg/health_start.hpp>
#include <interface/msg/nav2_follow_me.hpp>
#include <interface/msg/objects.hpp>
#include <interface/msg/vlago_scene.hpp>
#include <interface/srv/car_controls.hpp>
#include <interface/srv/dance_controls.hpp>
#include <interface/srv/follow_me.hpp>
#include <interface/srv/follow_status.hpp>
#include <interface/srv/gears_controls.hpp>
#include <interface/srv/high.hpp>
#include <interface/srv/hrv_info.hpp>
#include <interface/srv/hrv_start.hpp>
#include <interface/srv/id_controls.hpp>
#include <interface/srv/mode_switch.hpp>
#include <interface/srv/operation_scenario.hpp>
#include <interface/srv/operation_status.hpp>
#include <interface/srv/scene_controls.hpp>
#include <interface/srv/summon.hpp>
#include <interface/srv/summon_status.hpp>
#include <interface/srv/tft_controls.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <utils/topic.h>
namespace service::control {
struct Point {
    double x, y;
};

struct Rect {
    Point top_left;
    Point bottom_right;
};
using namespace std::placeholders;
class Control {
public:
    Control(const std::string& name);

protected:
    rclcpp::Logger logger_;

public:
    int id_ = 10000;
    const std::string name_;
    rclcpp::Client<interface::srv::ModeSwitch>::SharedPtr mode_switch_client_;
};

}