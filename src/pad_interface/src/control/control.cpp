#include "control.h"

using namespace service::control;

Control::Control(const std::string& name)
    : logger_(rclcpp::get_logger(name))
    , name_(name)
{
}