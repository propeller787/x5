#include "shared_control_assist/shared_control_assist_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<shared_control_assist::SharedControlAssistNode>());
    rclcpp::shutdown();
    return 0;
}
