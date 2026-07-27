#pragma once

#include "intent_avoidance/scan_processor.h"
#include "intent_avoidance/free_space_analyzer.h"
#include "intent_avoidance/intent_detector.h"
#include "intent_avoidance/avoidance_engine.h"
#include "intent_avoidance/control_generator.h"
#include "intent_avoidance/types.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <interface/msg/joystick.hpp>

#include <memory>

namespace intent_avoidance {

class IntentAvoidanceNode : public rclcpp::Node {
public:
    explicit IntentAvoidanceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // 订阅回调
    void onLeftScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void onRightScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void onJoystick(const interface::msg::Joystick::SharedPtr msg);
    void onWheelSpeed(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    // 定时处理（主控制循环）
    void onControlTimer();

    // 组件
    std::unique_ptr<ScanProcessor> left_processor_;
    std::unique_ptr<ScanProcessor> right_processor_;
    std::unique_ptr<FreeSpaceAnalyzer> free_space_analyzer_;
    std::unique_ptr<IntentDetector> intent_detector_;
    std::unique_ptr<AvoidanceEngine> avoidance_engine_;
    std::unique_ptr<ControlGenerator> control_generator_;

    // 缓存最新数据
    UserIntent latest_intent_;
    SpeedInfo latest_speed_;
    bool intent_received_ = false;

    // 订阅
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr left_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr right_scan_sub_;
    rclcpp::Subscription<interface::msg::Joystick>::SharedPtr joystick_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_speed_sub_;

    // 发布
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_pub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace intent_avoidance
