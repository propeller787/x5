/**
 * @file tracking.cpp
 * @author likang (likang@freego.com)
 * @brief 轮速反馈缓存实现
 * @version 0.2
 * @date 2026-06-05
 *
 * @copyright Copyright (c) 2025
 */

#include "tracking.h"

Tracking::Tracking() = default;

Tracking::~Tracking() = default;

void Tracking::updateWheelSpeed(double linear_velocity, double angular_velocity)
{
    wheel_speed_linear_velocity_ = linear_velocity;
    wheel_speed_angular_velocity_ = angular_velocity;
}

double Tracking::getWheelSpeedLinearVelocity() const
{
    return wheel_speed_linear_velocity_;
}

double Tracking::getWheelSpeedAngularVelocity() const
{
    return wheel_speed_angular_velocity_;
}
