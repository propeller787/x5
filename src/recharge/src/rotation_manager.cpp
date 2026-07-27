// rotation_manager.cpp
#include "rotation_manager.hpp"
#include <rclcpp/logging.hpp>
#include "../../common/utils/topic.h"

using namespace std::chrono_literals;

RotationManager::RotationManager(rclcpp::Node *node)
    : node_(node), is_rotating_(false), rotation_timeout_(40.0) // 旋转40秒超时
      ,
      angular_velocity_(0.15) // 角速度0.3 rad/s
      ,
      linear_velocity_(0.0) // 线速度0 m/s
{
    // 创建控制命令发布者
    cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        nav2ControlTopic, 10);
    rotation_timeout_pub1_ = node_->create_publisher<std_msgs::msg::Int32>(
        rechargeStateTopic, 10);
    rotation_timeout_pub2_=node_->create_publisher<std_msgs::msg::Int32>("/vlago/recharge/feedback",10);
    rotation_timeout_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/wheelchair/rotation_timeout", 10);
    logInfo("旋转管理器初始化完成");
}

void RotationManager::startRotation()
{
    if (is_rotating_)
    {
        logWarning("旋转已经在进行中");
        return;
    }

    is_rotating_ = true;
    rotation_start_time_ = node_->now();

    // 创建旋转定时器（100ms周期）
    rotation_timer_ = node_->create_wall_timer(
        100ms,
        [this]()
        {
            this->rotationTimerCallback();
        });

    // 创建旋转超时定时器
    rotation_timeout_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(rotation_timeout_),
        [this]()
        {
            logInfo("旋转超时，自动停止旋转");
             auto timeout_msg = std_msgs::msg::String();
            timeout_msg.data = "rotation_timeout";
            rotation_timeout_pub_->publish(timeout_msg);
            stopRotation();
            auto timeout_msg1 = std_msgs::msg::Int32();
            timeout_msg1.data = RECHARGE_EVENT_STATION_NOT_FOUND;
            rotation_timeout_pub1_->publish(timeout_msg1); 
            auto timeout_msg2 = std_msgs::msg::Int32();
            timeout_msg2.data = RECHARGE_FEEDBACK_FAILED_TIMEOUT;
            rotation_timeout_pub2_->publish(timeout_msg2);
        });

    logInfo("开始旋转 - 角速度: " + std::to_string(angular_velocity_) +
            " rad/s, 持续时间: " + std::to_string(rotation_timeout_) + "秒");
}

void RotationManager::stopRotation()
{
    if (!is_rotating_)
    {
        logWarning("旋转未启动，无需停止");
        return;
    }

    // 停止定时器
    if (rotation_timer_)
    {
        rotation_timer_->cancel();
        rotation_timer_.reset();
    }

    if (rotation_timeout_timer_)
    {
        rotation_timeout_timer_->cancel();
        rotation_timeout_timer_.reset();
    }

    // 发送停止命令
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_publisher_->publish(stop_cmd);

    // 更新状态
    is_rotating_ = false;

    double duration = getRotationDuration();
    logInfo("停止旋转 - 持续时间: " + std::to_string(duration) + "秒");
}
void RotationManager::rotationTimerCallback()
{
    if (!is_rotating_)
    {
        return;
    }

    publishRotationCommand();
}

void RotationManager::publishRotationCommand()
{
    geometry_msgs::msg::Twist rotation_cmd;
    rotation_cmd.linear.x = linear_velocity_;
    rotation_cmd.angular.z = angular_velocity_;

    cmd_vel_publisher_->publish(rotation_cmd);

    // 每2秒记录一次旋转状态
    static int log_counter = 0;
    if (log_counter++ % 20 == 0) // 100ms * 20 = 2秒
    {
        double elapsed = (node_->now() - rotation_start_time_).seconds();
        logInfo("旋转中 - 已旋转: " + std::to_string(elapsed) +
                "秒, 角速度: " + std::to_string(angular_velocity_) + " rad/s");
    }
}

double RotationManager::getRotationDuration() const
{
    if (!is_rotating_)
    {
        return 0.0;
    }

    return (node_->now() - rotation_start_time_).seconds();
}

bool RotationManager::shouldStopRotation() const
{
    if (!is_rotating_)
    {
        return false;
    }

    double elapsed = getRotationDuration();
    return elapsed >= rotation_timeout_;
}

void RotationManager::logInfo(const std::string &message)
{
    RCLCPP_INFO(node_->get_logger(), "[RotationManager] %s", message.c_str());
}

void RotationManager::logWarning(const std::string &message)
{
    RCLCPP_WARN(node_->get_logger(), "[RotationManager] %s", message.c_str());
}

void RotationManager::logError(const std::string &message)
{
    RCLCPP_ERROR(node_->get_logger(), "[RotationManager] %s", message.c_str());
}