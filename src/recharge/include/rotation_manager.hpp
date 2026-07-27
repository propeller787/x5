// rotation_manager.hpp
#ifndef ROTATION_MANAGER_HPP
#define ROTATION_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <std_msgs/msg/string.hpp>
#include<std_msgs/msg/int32.hpp>
#include<std_msgs/msg/bool.h>
#include "recharge_status.hpp"

class RotationManager
{
public:
    RotationManager(rclcpp::Node *node);

    // 核心接口
    void startRotation();
    void stopRotation();

    // 状态查询
    bool isRotating() const { return is_rotating_; }

    // 定时器回调
    void rotationTimerCallback();

    // 获取旋转持续时间
    double getRotationDuration() const;

    // 检查是否需要停止旋转（旋转超时）
    bool shouldStopRotation() const;

private:
    rclcpp::Node *node_;
    // ROS发布者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rotation_timeout_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rotation_timeout_pub1_;//对外
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rotation_timeout_pub2_;//对外
    // 定时器
    rclcpp::TimerBase::SharedPtr rotation_timer_;
    rclcpp::TimerBase::SharedPtr rotation_timeout_timer_;

    // 状态变量
    bool is_rotating_;
    rclcpp::Time rotation_start_time_;
    double rotation_timeout_; // 旋转超时时间（秒）

    // 控制参数
    double angular_velocity_; // 角速度
    double linear_velocity_;  // 线速度

    // 发布旋转控制命令
    void publishRotationCommand();

    // 记录日志
    void logInfo(const std::string &message);
    void logWarning(const std::string &message);
    void logError(const std::string &message);

};

#endif // ROTATION_MANAGER_HPP