/**
 * @file speed_reduction_kalman_filter.h
 * @author likang (likang@freego.com)
 * @version 0.1
 * @date 2025-12-08
 * 
 * @copyright Copyright (c) 2025
 * @brief 速度减少变换用的卡尔曼滤波器（仅在减速时平滑）
 */
#pragma once

#include <Eigen/Dense>
#include <cmath>

class SpeedReductionKalmanFilter {
public:
    struct Config {
        double process_noise = 0.8;      // 过程噪声标量
        double measurement_noise = 0.4;  // 测量噪声标量
        double initial_variance = 1.0;   // 初始协方差
        double min_dt = 0.005;           // 最小时间步，防止数值发散
        double default_dt = 0.02;        // 缺省时间步
    };

    SpeedReductionKalmanFilter();
    explicit SpeedReductionKalmanFilter(const Config& cfg);

    // 传入目标速度与时间步，返回平滑后的速度
    double filter(double measured_speed, double dt_seconds);

    // 重置滤波器
    void reset(double initial_speed = 0.0);

    // 更新配置
    void setConfig(const Config& cfg);

    // 获取上一次滤波结果
    double lastFilteredSpeed() const;
    bool hasResult() const;

private:
    void initializeState(double initial_speed);

    Config config_;
    bool initialized_;
    bool has_result_;
    double last_filtered_speed_;

    Eigen::Vector2d state_; // [速度, 加速度]
    Eigen::Matrix2d covariance_;
    Eigen::Matrix<double, 1, 2> H_;
    Eigen::Matrix<double, 1, 1> R_;
};

