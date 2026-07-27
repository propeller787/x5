/**
 * @file smooth_filter.h
 * @author likang (likang@freego.com)
 * @brief 平滑滤波器 - 用于控制输出的平滑处理
 * @version 0.1
 * @date 2025-10-16
 * 
 * @copyright Copyright (c) 2025
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "speed_reduction_kalman_filter.h"
#include <chrono>
#include <deque>
#include <mutex>
#include <algorithm>
#include <numeric>

/** RATE_LIMITED 模式：每周期线速度/角速度最大变化量及线加速度上限（与 TFT 舒适/运动参数对应） */
struct SmoothCommandLimits {
    double max_linear_delta = 0.01; // 每周期 |Δvx| 上限 (m/s)
    double max_angular_delta = 0.01; // 每周期 |Δωz| 上限 (rad/s)
    /** 线加速度 (m/s²)；与 max_linear_delta 同时生效时取 min(Δ, a·dt)。设较大值则主要由 max_linear_delta 限制 */
    double max_linear_accel_m_s2 = 100.0;
};

// 平滑滤波类型
enum class FilterType {
    MOVING_AVERAGE,     // 移动平均
    EXPONENTIAL,        // 指数平滑
    MEDIAN,             // 中值滤波
    RATE_LIMITED,       // 速率限制：前向速度每次最多增加0.01，减速不做限制
    KALMAN_DECELERATION // 卡尔曼滤波：仅在减速时平滑，保持加速响应
};

class SmoothFilter {
public:
    SmoothFilter();
    ~SmoothFilter();
    
    // 设置滤波器类型和参数
    void setFilterType(FilterType type);
    void setWindowSize(size_t window_size);
    void setAlpha(double alpha);  // 指数平滑系数
    /** 仅 RATE_LIMITED：设置每周期线/角速度变化与线加速度上限（由 DecisionMaker 按 TFT 模式刷新） */
    void setCommandLimits(const SmoothCommandLimits& limits);

    // 滤波处理
    geometry_msgs::msg::Twist filter(const geometry_msgs::msg::Twist& input);
    
    // 重置滤波器
    void reset();
    
    // 获取滤波器状态
    bool isEmpty() const;
    size_t getCurrentSize() const;

private:
    // 移动平均滤波
    geometry_msgs::msg::Twist movingAverageFilter(const geometry_msgs::msg::Twist& input);
    
    // 指数平滑滤波
    geometry_msgs::msg::Twist exponentialFilter(const geometry_msgs::msg::Twist& input);
    
    // 中值滤波
    geometry_msgs::msg::Twist medianFilter(const geometry_msgs::msg::Twist& input);
    
    // 速率限制滤波：前向速度每次最多增加0.01，减速不做限制
    geometry_msgs::msg::Twist rateLimitedFilter(const geometry_msgs::msg::Twist& input);

    // 卡尔曼减速滤波：只在减速时进行平滑，避免速度突降
    geometry_msgs::msg::Twist kalmanDecelerationFilter(const geometry_msgs::msg::Twist& input);
    
    // 滤波器参数
    FilterType filter_type_;
    size_t window_size_;
    double alpha_;
    
    // 数据缓冲区
    mutable std::mutex data_mutex_;
    std::deque<geometry_msgs::msg::Twist> linear_buffer_;
    std::deque<geometry_msgs::msg::Twist> angular_buffer_;
    
    // 指数平滑的上一状态
    geometry_msgs::msg::Twist last_filtered_;
    bool has_last_filtered_;

    // 卡尔曼滤波器状态
    SpeedReductionKalmanFilter speed_kf_;
    std::chrono::steady_clock::time_point last_kalman_time_;
    bool has_last_kalman_time_;

    // RATE_LIMITED：指令变化率与计时
    SmoothCommandLimits command_limits_;
    std::chrono::steady_clock::time_point last_rate_limit_time_;
    bool has_rate_limit_time_ = false;
};
