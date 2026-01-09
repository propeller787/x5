#include "global.h"
#include <rclcpp/rclcpp.hpp>
#include <utils/topic.h>

using std::placeholders::_1;
namespace sensor {
Global* Global::instance = nullptr;
std::mutex Global::mtx;
Global::Global(void)
{
    node_ = std::make_shared<rclcpp::Node>("sensor");
    hex_data_pub_ = node_->create_publisher<interface::msg::HexData>(errorEventTopic, 10);
}

void Global::sendHexData(const interface::msg::HexData msg)
{
    hex_data_pub_->publish(msg);
}

void Global::loadParametersFromYAML(const std::string& file_path)
{

}

void Global::printParameters()
{
    RCLCPP_INFO(node_->get_logger(), "Loaded parameters:");
    auto parameters_and_prefixes = node_->list_parameters({}, 1);
    for (const auto& parameter : parameters_and_prefixes.names) {
        RCLCPP_INFO(node_->get_logger(), "%s : %d", parameter.c_str(), node_->get_parameter(parameter).get_type());
    }
}

void Global::registerCarinfoCB(CarinfoCB cb)
{
    slots_carinfo_.emplace_back(signal_carinfo_.connect(std::move(cb)));
}

void Global::onCarInforResult(const interface::msg::CarInformation msg)
{
    car_information_ = msg;
    signal_carinfo_.call(std::move(msg));
}

}