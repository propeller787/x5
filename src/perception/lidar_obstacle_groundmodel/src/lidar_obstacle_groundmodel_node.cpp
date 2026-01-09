#include <rclcpp/rclcpp.hpp>
#include "lidar_obstacle_groundmodel.h"

int main(int argc, char ** argv)
{
  /* code */
  auto logger = rclcpp::get_logger("lidar_obastacle_groundmodel");
  RCLCPP_INFO(logger, "lidar_obastacle_groundmodel extract start!!!");
  rclcpp::init(argc, argv);

  //node point
  auto node = std::make_shared<Lidar_Obstacle_GroundModel>("Lidar_Obstacle");
  //log
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
