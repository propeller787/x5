#include "control.h"

using namespace tft_service::control;

Control::Control(const std::string &name) : logger_(rclcpp::get_logger(name)), name_(name) {
    global_ = tft_service::Global::getInstance();
    audio_pub_ = global_->node_->create_publisher<std_msgs::msg::Int32>(audioTopic, 10);
}