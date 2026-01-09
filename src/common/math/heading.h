// Copyright 2016 Baidu Inc. All Rights Reserved.
// Author: Heng, Liang (hengliang@baidu.com)
//
// Helper functions that converts heading from/to a quaternion.
//
// We have permanently defined that the heading of car is zero when the car is pointing to east, and
// positive when the car turns left. This function guarantees correct heading calculation,
// regardless any future changes in the definition of vehicle reference frame.

#pragma once

#include "Eigen/Geometry"
#include "math/euler_angles_zxy.h"
#include "math/math_utils.h"

#include <cmath>

namespace decision::math {

// Returns heading (in radians) in [-PI, PI).
inline double quaternion_to_heading(double qw, double qx, double qy, double qz)
{
    EulerAnglesZXYd euler_angles(qw, qx, qy, qz);
    // Needs adding pi/2 to yaw because when the vehicle reference frame is RFU, yaw is zero when
    // the car is pointing to north.
    return normalize_angle(euler_angles.yaw + M_PI_2);
}

// Similar to above but takes a Quaternion object.
template <typename T>
inline double quaternion_to_heading(const Eigen::Quaternion<T>& q)
{
    return quaternion_to_heading(q.w(), q.x(), q.y(), q.z());
}

// Returns a quaternion with zero roll, zero pitch, and the specified heading.
template <typename T>
inline Eigen::Quaternion<T> heading_to_quaternion(double heading)
{
    // Needs deducting pi/2 from heading because the vehicle reference frame is RFU.
    EulerAnglesZXY<T> euler_angles(heading - M_PI_2);
    return euler_angles.to_quaternion();
}

}
