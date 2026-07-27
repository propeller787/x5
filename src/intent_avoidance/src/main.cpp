#include "intent_avoidance/intent_avoidance_node.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<intent_avoidance::IntentAvoidanceNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
