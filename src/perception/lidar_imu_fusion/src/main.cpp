#include "lidar_imu_fusion/lidar_imu_fusion_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_imu_fusion::ThreeLidarImuFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
