// main.cpp
#include "event_input.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

class WheelchairStateMachineNode : public rclcpp::Node
{
public:
    WheelchairStateMachineNode()
        : Node("recharge")
    {
        event_handler_ = std::make_unique<EventInputHandler>(this);
        event_handler_->initializeModules();

        // 显示系统状态
        event_handler_->displaySystemStatus();

        RCLCPP_WARN(this->get_logger(), "智能轮椅状态机已启动");
    }

private:
    std::unique_ptr<EventInputHandler> event_handler_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WheelchairStateMachineNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}