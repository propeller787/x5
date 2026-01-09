#include <rclcpp/rclcpp.hpp>
#include "podao_detect.h"

int main(int argc, char ** argv)
{
  /* code */
  auto logger = rclcpp::get_logger("podao_detect");
  RCLCPP_INFO(logger, "podao_detect start!!!");
  rclcpp::init(argc, argv);

  //node point
  auto node = std::make_shared<Podao_Detect>("Podao_Detect");
  //log
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
