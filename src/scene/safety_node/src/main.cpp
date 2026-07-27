/**
 * @file main.cpp
 * @brief safety_node 进程入口
 *
 * 启动流程：
 *   rclcpp::init → make_shared<SafetyNode> → attachComponents → spin
 *
 * 方案 7：runtime.multithreaded_executor=true 时使用 3 线程 MultiThreadedExecutor，
 * 控制环 / 感知回调 / 后台（地图+可视化+看门狗）由三个互斥回调组隔离，
 * 慢回调不再推迟安全控制环。默认 false（单线程，与旧行为逐位一致），
 * TSan + 长稳验证通过后翻转生产默认值。
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
    bool multithreaded = false;
    if (node->has_parameter("runtime.multithreaded_executor")) {
      multithreaded =
        node->get_parameter("runtime.multithreaded_executor").as_bool();
    }
    if (multithreaded) {
      rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions{}, 3);
      executor.add_node(node);
      executor.spin();
    } else {
      rclcpp::spin(node);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("safety_node"), "运行异常: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
