/**
 * @file vehicle_interface.cpp
 * @author likang (likang@freego.com)
 * @brief 车辆接口实现 - 管理车辆状态和控制输入
 * 
 * 功能说明:
 * 1. 接收摇杆控制输入 (/handle/control)
 * 2. 接收轮速反馈数据 (/wheel/twist)
 * 3. 维护和更新车辆状态
 * 4. 通过回调通知状态变化
 * 
 * 数据流向:
 * 摇杆/轮速话题 → VehicleInterface → 状态回调 → DecisionMaker
 * 
 * @version 0.1
 * @date 2025-10-16
 * @copyright Copyright (c) 2025 Freego Technology Co., Ltd.
 */

#include "vehicle_interface.h"
#include "../../../common/utils/topic.h"
#include <mutex>

namespace {
    constexpr const char* LOG_PREFIX = "[VehicleInterface]";
    constexpr int QUEUE_SIZE = 10;  ///< ROS2订阅队列大小
}

// ==================== 构造与析构 ====================

/**
 * @brief 构造函数 - 初始化车辆接口
 * 
 * 初始化流程:
 * 1. 初始化车辆状态为零
 * 2. 创建摇杆控制订阅器
 * 3. 创建轮速反馈订阅器
 */
VehicleInterface::VehicleInterface() : rclcpp::Node("vehicle_interface") {
    RCLCPP_INFO(this->get_logger(), "%s 开始初始化车辆接口...", LOG_PREFIX);
    
    // 初始化车辆状态为零值
    current_state_.linear_velocity = 0.0;
    current_state_.angular_velocity = 0.0;
    current_state_.wheel_speed_left = 0.0;
    current_state_.wheel_speed_right = 0.0;
    current_state_.timestamp = this->now();
    
    RCLCPP_INFO(this->get_logger(), "%s 车辆状态已初始化为零", LOG_PREFIX);
    
    // 创建摇杆控制订阅器
    handle_control_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        handleControlTopic, QUEUE_SIZE,
        std::bind(&VehicleInterface::handleControlCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "%s   ✓ 摇杆控制订阅器创建成功 (话题: %s)", 
                LOG_PREFIX, handleControlTopic);
    
    // 创建轮速反馈订阅器
    wheel_speed_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        carWheelSpeedStaTopic, QUEUE_SIZE,
        std::bind(&VehicleInterface::wheelSpeedCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "%s   ✓ 轮速反馈订阅器创建成功 (话题: %s)", 
                LOG_PREFIX, carWheelSpeedStaTopic);
    
    RCLCPP_INFO(this->get_logger(), "%s 车辆接口初始化完成", LOG_PREFIX);
}

/**
 * @brief 析构函数 - 清理资源
 */
VehicleInterface::~VehicleInterface() {
    RCLCPP_INFO(this->get_logger(), "%s 车辆接口正在析构...", LOG_PREFIX);
}

// ==================== 公共接口方法 ====================

/**
 * @brief 获取当前车辆状态
 * 
 * @return VehicleState 当前车辆状态的副本 (线程安全)
 * 
 * 注意:
 * - 使用互斥锁保证线程安全
 * - 返回的是状态副本，不受后续修改影响
 */
VehicleState VehicleInterface::getCurrentState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;  // 返回副本
}

/**
 * @brief 设置车辆状态变化回调函数
 * 
 * @param callback 回调函数，当车辆状态更新时被调用
 * 
 * 使用场景:
 * - DecisionMaker需要实时获取车辆状态
 * - 用于目标位置预测和速度干预决策
 * 
 * 注意:
 * - 只有摇杆控制数据会触发此回调
 * - 轮速反馈数据不会触发此回调
 */
void VehicleInterface::setStateChangeCallback(std::function<void(const VehicleState&)> callback) {
    state_change_callback_ = callback;
    RCLCPP_DEBUG(this->get_logger(), "%s 状态变化回调函数已设置", LOG_PREFIX);
}

/**
 * @brief 设置轮速反馈回调函数
 * 
 * @param callback 回调函数，参数为左轮速度和右轮速度
 * 
 * 使用场景:
 * - Tracking模块需要轮速数据进行目标位置预测
 * - 当雷达数据缺失时，基于轮速反推目标位置
 */
void VehicleInterface::setWheelSpeedCallback(std::function<void(double, double)> callback) {
    wheel_speed_callback_ = callback;
    RCLCPP_DEBUG(this->get_logger(), "%s 轮速反馈回调函数已设置", LOG_PREFIX);
}

// ==================== 私有回调方法 ====================

/**
 * @brief 摇杆控制数据回调
 * 
 * @param msg 摇杆控制消息 (Twist格式)
 * 
 * 消息格式:
 * - linear.x: 线速度 (m/s, 前进为正)
 * - angular.z: 角速度 (rad/s, 左转为正)
 * 
 * 处理流程:
 * 1. 加锁保护状态更新
 * 2. 提取速度信息并更新车辆状态
 * 3. 更新时间戳
 * 4. 触发状态变化回调 (通知DecisionMaker)
 * 
 * 注意:
 * - 摇杆控制是驾驶员的期望速度
 * - DecisionMaker可能会对此进行干预
 */
void VehicleInterface::handleControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!msg) {
        RCLCPP_WARN(this->get_logger(), "%s 收到空的摇杆控制消息", LOG_PREFIX);
        return;
    }
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 更新摇杆控制的速度信息
    current_state_.linear_velocity = msg->linear.x;
    current_state_.angular_velocity = msg->angular.z;
    current_state_.timestamp = this->now();
    
    RCLCPP_DEBUG(this->get_logger(), 
        "%s 收到摇杆控制: 线速度=%.3f m/s, 角速度=%.3f rad/s", 
        LOG_PREFIX, current_state_.linear_velocity, current_state_.angular_velocity);
    
    // 触发状态变化回调 (通知DecisionMaker进行速度干预判断)
    if (state_change_callback_) {
        state_change_callback_(current_state_);
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "%s 状态变化回调未设置，摇杆控制数据无法传递", LOG_PREFIX);
    }
}

/**
 * @brief 轮速反馈数据回调
 * 
 * @param msg 轮速反馈消息 (TwistStamped格式)
 * 
 * 消息格式:
 * - twist.linear.x: 左轮速度 (m/s)
 * - twist.linear.y: 右轮速度 (m/s)
 * 
 * 处理流程:
 * 1. 加锁保护状态更新
 * 2. 提取左右轮速度信息
 * 3. 更新时间戳
 * 4. 触发轮速变化回调 (传递给Tracking模块)
 * 
 * 用途:
 * - 实际轮速用于预测目标位置 (相对于车辆的运动)
 * - 当雷达数据缺失时，基于轮速反推目标位置
 * - 检测车辆实际运动状态
 * 
 * 注意:
 * - 轮速数据不再触发状态变化回调
 * - 轮速数据通过专门的轮速回调传递给Tracking模块
 */
void VehicleInterface::wheelSpeedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (!msg) {
        RCLCPP_WARN(this->get_logger(), "%s 收到空的轮速反馈消息", LOG_PREFIX);
        return;
    }
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 更新轮速反馈信息
    // 数据格式约定: linear.x=左轮速度, linear.y=右轮速度
    current_state_.wheel_speed_left = msg->twist.linear.x;
    current_state_.wheel_speed_right = msg->twist.linear.y;
    current_state_.timestamp = this->now();
    
    RCLCPP_DEBUG(this->get_logger(), 
        "%s 收到轮速反馈: 左轮=%.3f m/s, 右轮=%.3f m/s", 
        LOG_PREFIX, current_state_.wheel_speed_left, current_state_.wheel_speed_right);
    
    // 触发轮速变化回调 (传递给Tracking模块)
    if (wheel_speed_callback_) {
        wheel_speed_callback_(current_state_.wheel_speed_left, current_state_.wheel_speed_right);
    }
}
