/**
 * @file tracking.h
 * @author likang (likang@freego.com)
 * @brief 轮速反馈缓存（阶段 A 瘦身，原 Tracking 目标跟踪已移除）
 * @version 0.2
 * @date 2026-06-05
 *
 * @copyright Copyright (c) 2025
 */
#pragma once

/**
 * @class Tracking
 * @brief 轮速反馈缓存，供 DecisionMaker 读取 /wheel/twist 同源速度
 *
 * 历史命名保留为 Tracking，避免大范围重命名；职责等价于 WheelSpeedCache。
 */
class Tracking {
public:
    Tracking();
    ~Tracking();

    void updateWheelSpeed(double linear_velocity, double angular_velocity);

    double getWheelSpeedLinearVelocity() const;
    double getWheelSpeedAngularVelocity() const;

private:
    double wheel_speed_linear_velocity_{0.0};
    double wheel_speed_angular_velocity_{0.0};
};
