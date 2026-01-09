// Copyright 2018 Baidu Inc. All Rights Reserved
// Author: Fuxiangyu (fuxiangyu@baidu.com)
//
// Description: supporting for 3D element

#pragma once

// #include <cmath>
// #include <string>
#include "math/vec2d.h"

namespace decision::math {

class Vec3d {
public:
    constexpr Vec3d() noexcept
        : Vec3d(0, 0, 0)
    {
    }
    constexpr Vec3d(const double x, const double y, const double z) noexcept
        : _x(x)
        , _y(y)
        , _z(z)
    {
    }
    static Vec3d create_unit_vec(const double yaw_angle, const double pitch_angle);

    double x() const { return _x; }
    double y() const { return _y; }
    double z() const { return _z; }
    double yaw_angle() const { return atan2(_y, _x); }
    void set_x(const double x) { _x = x; }
    void set_y(const double y) { _y = y; }
    void set_z(const double z) { _z = z; }
    void normalize();

    double length() const;
    double length_sqr() const;
    double distance_to(const Vec3d& other) const;
    double distance_sqr_to(const Vec3d& other) const;

    Vec3d operator+(const Vec3d& other) const;
    Vec3d operator-(const Vec3d& other) const;
    Vec3d operator*(const double ratio) const;
    Vec3d operator/(const double ratio) const;
    Vec3d& operator+=(const Vec3d& other);
    Vec3d& operator-=(const Vec3d& other);
    Vec3d& operator*=(const double ratio);
    Vec3d& operator/=(const double ratio);
    bool operator==(const Vec3d& other) const;

    Vec3d cross_prod(const Vec3d& other) const;
    double inner_prod(const Vec3d& other) const;

    Vec2d vec2d() const { return Vec2d(_x, _y); }
    std::string debug_string() const;

protected:
    double _x = 0.0;
    double _y = 0.0;
    double _z = 0.0;
};

Vec3d operator*(const double ratio, const Vec3d& vec);

}
