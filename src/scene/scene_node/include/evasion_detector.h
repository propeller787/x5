/**
 * @file evasion_detector.h
 * @author likang (likang@freego.com)
 * @brief 操作者避让意图检测器 - 根据轮速反馈推断转向/前进，衰减前向制动干预
 *
 * 设计动机:
 *   当前向 TTC/距离触发减速或制动时，若车体已在大幅度转向（左或右避让），
 *   说明障碍物可能不在未来行驶轨迹上。此时如果仍然全力制动，会导致操控突兀。
 *   本模块根据轮速反馈（与 Tracking 中 wheel twist 一致）检测实际避让运动，输出衰减因子 [0, 1]。
 *
 * 判定逻辑:
 *   1. 车体正在转向：|omega_fb| >= angular_threshold（omega_fb 为轮速反馈角速度 rad/s）
 *   2. 前进运动存在：v_fb > 0（v_fb 为轮速反馈线速度 m/s）
 *   3. 前向障碍物不在固定停车距离内（固定停车距离内不衰减，确保安全底线）
 *   4. 满足以上条件时，根据 |omega_fb| 的大小计算衰减因子：
 *      - |omega_fb| = angular_threshold 时，factor = 1.0（不衰减）
 *      - |omega_fb| = angular_full_evasion 时，factor = min_factor（最大衰减）
 *      - 中间线性插值
 *   5. 对前向 speed_limit 做: relaxed_limit = limit + (max_limit - limit) * (1 - factor)
 *      即转向越大，前向限速越接近 max_speed_limit（不限速）
 *
 * @version 0.1
 * @date 2026-04-06
 * @copyright Copyright (c) 2026 Freego Technology Co., Ltd.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

/**
 * @struct EvasionDetectorParams
 * @brief 避让意图检测可调参数
 */
struct EvasionDetectorParams {
    bool enable = true;                       ///< 总开关
    double angular_threshold = 0.15;          ///< 开始认为"正在转向避让"的角速度阈值 (rad/s)
    double angular_full_evasion = 0.5;        ///< 完全避让时的角速度 (rad/s)，达到此值时衰减因子最小
    double min_factor = 0.2;                  ///< 最大衰减时保留的干预比例 [0,1]，0=完全放行，1=不衰减
    double safe_distance_override = 0.0;      ///< 距离 <= 此值时不衰减（使用 fixed_stopping_distance_forward）
};

/**
 * @class EvasionDetector
 * @brief 操作者避让意图检测器
 *
 * 无状态、无锁：每次调用只依赖当前输入，不缓存历史，可在任意线程安全调用。
 */
class EvasionDetector {
public:
    explicit EvasionDetector(rclcpp::Logger logger);

    /**
     * @brief 更新参数
     */
    void updateParams(const EvasionDetectorParams& params);

    /**
     * @brief 计算前向干预衰减因子
     *
     * @param feedback_angular_z 轮速反馈角速度 (rad/s，左转为正)，与 Tracking::getWheelSpeedAngularVelocity 一致
     * @param feedback_linear_x  轮速反馈线速度 (m/s，前进为正)，与 Tracking::getWheelSpeedLinearVelocity 一致
     * @param forward_distance    前向最小障碍物距离 (m)
     * @return 衰减因子 [min_factor, 1.0]
     *         1.0 = 不衰减（保持原始干预）
     *         min_factor = 最大衰减（车体已明显转向避让）
     */
    double computeEvasionFactor(double feedback_angular_z,
                                double feedback_linear_x,
                                double forward_distance) const;

    /**
     * @brief 对前向速度限制应用避让衰减
     *
     * @param original_limit 原始计算的前向速度上限 (m/s)
     * @param max_speed_limit 系统最大速度 (m/s)
     * @param factor 避让衰减因子（由 computeEvasionFactor 返回）
     * @return 放松后的速度上限 (m/s)
     */
    double applyEvasionRelax(double original_limit, double max_speed_limit, double factor) const;

private:
    rclcpp::Logger logger_;
    EvasionDetectorParams params_;
};
