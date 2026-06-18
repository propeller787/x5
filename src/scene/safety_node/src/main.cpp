/**
 * @file main.cpp
 * @brief safety_node 进程入口
 *
 * 启动流程：
 *   rclcpp::init → make_shared<SafetyNode> → attachComponents → spin
 */
#include "safety_node/safety_node.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SafetyNode>();
    node->attachComponents();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("safety_node"), "运行异常: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
