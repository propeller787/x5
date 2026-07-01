#include "localization/odom_fusion_node.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<localization::OdomFusionNode>());
  rclcpp::shutdown();
  return 0;
}
