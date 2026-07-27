#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "postback_node.hpp"

int main(int argc, char**argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<postback::PostbackNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("postback_node"), "节点初始化失败：%s", e.what());
        return -1;
    }
    rclcpp::shutdown();
    return 0;
}