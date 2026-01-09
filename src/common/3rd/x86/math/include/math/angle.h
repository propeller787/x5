// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// Author: hengliang

// This file defines the Angle class, which uses an integer to represent an angle, and supports
// commonly-used operations such as addition, subtraction, and trigonometry. Using an integer gives
// the following advantages over the traditional floating-point representations:
// - A signed integer automatically wraps an angle to the interval [-pi, pi).
// - It makes the delta angle calculation easier and safer. For example, the difference between -179
//   degrees and 179 degrees longitude is automatically 2 degrees.
// - It uses less storage to achieve similar level of accuracy: Angle8 < Angle16 < float < Angle32 <
//   double < Angle64, where < means "less precise than."
// - Integer operations are more efficient.
// - Angle8 and Angle16 allow super fast trigonometric functions via a 64-KiB lookup table.
//
// It is recommended to use the Angle class as much as possible, especially for the following
// scenarios:
// - Latitude/longitude: Angle32 can achieve < 1 cm resolution.
// - Euler angles: Angle16 is precise enough for localization/object detection.
//
// Usage examples: Please refer to angle_test.cpp.

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace decision::math {

template <typename T>
class Angle {
public:
    static_assert(std::numeric_limits<T>::is_integer && std::numeric_limits<T>::is_signed,
        "T must be a signed integer type");

    static constexpr T RAW_PI = std::numeric_limits<T>::min();
    static constexpr T RAW_PI_2 = -(std::numeric_limits<T>::min() >> 1);
    static constexpr double DEG_TO_RAW = RAW_PI / -180.0;
    static constexpr double RAD_TO_RAW = RAW_PI * -M_1_PI;
    static constexpr double RAW_TO_DEG = -180.0 / RAW_PI;
    static constexpr double RAW_TO_RAD = -M_PI / RAW_PI;

    // Construct an object from an angle in degrees.
    static Angle from_deg(double value)
    {
        Angle a(lround(value * DEG_TO_RAW));
        return a;
    }

    // Construct an object from an angle in radians.
    static Angle from_rad(double value)
    {
        Angle a(lround(value * RAD_TO_RAW));
        return a;
    }

    explicit Angle(T value = 0)
        : _value(value)
    {
    }

    double to_deg() const
    {
        return _value * RAW_TO_DEG;
    }

    double to_rad() const
    {
        return _value * RAW_TO_RAD;
    }

    T raw() const
    {
        return _value;
    }

    Angle operator+=(Angle other)
    {
        _value += other._value;
        return *this;
    }

    Angle operator-=(Angle other)
    {
        _value -= other._value;
        return *this;
    }

    template <typename Scalar>
    Angle operator*=(Scalar s)
    {
        _value = lround(_value * s);
        return *this;
    }

    template <typename Scalar>
    Angle operator/=(Scalar s)
    {
        _value = lround(_value / s);
        return *this;
    }

private:
    T _value;
};

using Angle8 = Angle<int8_t>;
using Angle16 = Angle<int16_t>;
using Angle32 = Angle<int32_t>;
using Angle64 = Angle<int64_t>;

template <typename T>
Angle<T> operator+(Angle<T> lhs, Angle<T> rhs)
{
    lhs += rhs;
    return lhs;
}

template <typename T>
Angle<T> operator-(Angle<T> lhs, Angle<T> rhs)
{
    lhs -= rhs;
    return lhs;
}

template <typename T, typename Scalar>
Angle<T> operator*(Angle<T> lhs, Scalar rhs)
{
    lhs *= rhs;
    return lhs;
}

template <typename T, typename Scalar>
Angle<T> operator*(Scalar lhs, Angle<T> rhs)
{
    rhs *= lhs;
    return rhs;
}

template <typename T, typename Scalar>
Angle<T> operator/(Angle<T> lhs, Scalar rhs)
{
    lhs /= rhs;
    return lhs;
}

template <typename T>
double operator/(Angle<T> lhs, Angle<T> rhs)
{
    return double(lhs.raw()) / rhs.raw();
}

template <typename T>
bool operator==(Angle<T> lhs, Angle<T> rhs)
{
    return lhs.raw() == rhs.raw();
}

template <typename T>
bool operator!=(Angle<T> lhs, Angle<T> rhs)
{
    return !(lhs == rhs);
}

// Fast trigonometric functions. Single precision is sufficient for Angle16 and Angle8.
float sin(Angle16 a);
float cos(Angle16 a);
float tan(Angle16 a);
float sin(Angle8 a);
float cos(Angle8 a);
float tan(Angle8 a);

}
