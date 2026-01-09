#include <rclcpp/rclcpp.hpp>
#include "scene_services.h"

int main(int argc, char ** argv)
{
  /* code */
  auto logger = rclcpp::get_logger("scene_service");
  RCLCPP_INFO(logger, "scene_service extract start!!!");
  rclcpp::init(argc, argv);

  //node point
  auto node = std::make_shared<service::scene::SceneServices>("scene_service");
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}