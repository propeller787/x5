#include "lidar_obstacle_scantrend.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarObstacleScanTrend>("lidar_obstacle_scantrend");
    RCLCPP_INFO(node->get_logger(), "lidar_obstacle_scantrend started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
