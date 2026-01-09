/**
 * @file vehicle_interface.h
 * @author likang (likang@freego.com)
 * @brief 车辆接口 - 管理车辆状态和控制输入
 * 
 * 功能概述:
 * - 订阅摇杆控制话题获取驾驶员输入
 * - 订阅轮速反馈话题获取实际车速
 * - 维护统一的车辆状态结构
 * - 通过回调机制通知状态变化
 * 
 * @version 0.1
 * @date 2025-10-16
 * @copyright Copyright (c) 2025 Freego Technology Co., Ltd.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <functional>
#include <mutex>

/**
 * @struct VehicleState
 * @brief 车辆状态数据结构
 * 
 * 包含车辆的速度和时间戳信息，用于：
 * 1. 决策器进行速度干预
 * 2. 目标跟踪器预测障碍物位置
 * 3. 系统状态监控
 */
struct VehicleState {
    double linear_velocity = 0.0;   ///< 线速度 (m/s, 前进为正)
    double angular_velocity = 0.0;  ///< 角速度 (rad/s, 左转为正)
    double wheel_speed_left = 0.0;  ///< 左轮实际速度 (m/s)
    double wheel_speed_right = 0.0; ///< 右轮实际速度 (m/s)
    rclcpp::Time timestamp;         ///< 状态更新时间戳
};

/**
 * @class VehicleInterface
 * @brief 车辆接口类 - 统一管理车辆状态和控制输入
 * 
 * 设计职责:
 * 1. 数据采集: 接收摇杆控制和轮速反馈
 * 2. 状态管理: 维护统一的车辆状态
 * 3. 线程安全: 使用互斥锁保护状态访问
 * 4. 事件通知: 通过回调通知状态变化
 * 
 * 使用示例:
 * @code
 * auto vehicle_interface = std::make_shared<VehicleInterface>();
 * vehicle_interface->setStateChangeCallback([](const VehicleState& state) {
 *     // 处理状态变化
 * });
 * VehicleState current = vehicle_interface->getCurrentState();
 * @endcode
 */
class VehicleInterface : public rclcpp::Node {
public:
    // ==================== 构造与析构 ====================
    
    /**
     * @brief 构造函数 - 初始化订阅器和状态
     */
    VehicleInterface();
    
    /**
     * @brief 析构函数 - 清理资源
     */
    ~VehicleInterface();

    // ==================== 公共接口 ====================
    
    /**
     * @brief 获取当前车辆状态 (线程安全)
     * @return VehicleState 当前状态的副本
     */
    VehicleState getCurrentState() const;
    
    /**
     * @brief 设置状态变化回调函数
     * @param callback 回调函数，参数为最新的车辆状态
     * 
     * 注意: 只有摇杆控制数据会触发此回调
     */
    void setStateChangeCallback(std::function<void(const VehicleState&)> callback);
    
    /**
     * @brief 设置轮速反馈回调函数
     * @param callback 回调函数，参数为左轮速度和右轮速度
     * 
     * 注意: 轮速反馈数据通过此回调传递给Tracking模块
     */
    void setWheelSpeedCallback(std::function<void(double, double)> callback);

private:
    // ==================== 私有回调方法 ====================
    
    /**
     * @brief 摇杆控制数据回调
     * @param msg 摇杆控制消息 (包含线速度和角速度)
     */
    void handleControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    /**
     * @brief 轮速反馈数据回调
     * @param msg 轮速反馈消息 (包含左右轮速度)
     */
    void wheelSpeedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    
    // ==================== 成员变量 ====================
    
    // ROS2订阅器
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr handle_control_sub_;       ///< 摇杆控制订阅器
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_speed_sub_;  ///< 轮速反馈订阅器
    
    // 状态管理
    mutable std::mutex state_mutex_;   ///< 状态访问互斥锁
    VehicleState current_state_;       ///< 当前车辆状态
    
    // 回调函数
    std::function<void(const VehicleState&)> state_change_callback_;  ///< 状态变化回调 (仅摇杆控制触发)
    std::function<void(double, double)> wheel_speed_callback_;       ///< 轮速反馈回调 (传递给Tracking)
};