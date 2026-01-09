#pragma once

#include "math/box2d.h"
#include "math/polygon2d.h"
#include "math/vec2d.h"
#include "math/vec3d.h"

namespace decision::math {

constexpr double kMathEpsilon = 1e-10;

inline double cross_prod(const Vec2d& point1, const Vec2d& point2, const Vec2d& point3)
{
    return (point2.x() - point1.x()) * (point3.y() - point1.y()) - (point3.x() - point1.x()) * (point2.y() - point1.y());
}

inline Vec3d cross_prod(const Vec3d& point1, const Vec3d& point2, const Vec3d& point3)
{
    Vec3d v12 = point2 - point1;
    Vec3d v13 = point3 - point1;
    return v12.cross_prod(v13);
}

inline double inner_prod(const Vec2d& point1, const Vec2d& point2, const Vec2d& point3)
{
    return (point2.x() - point1.x()) * (point3.x() - point1.x()) + (point2.y() - point1.y()) * (point3.y() - point1.y());
}

// Wrap angle to [0, 2 * PI).
inline double wrap_angle(const double angle)
{
    const double new_angle = fmod(angle, M_PI * 2.0);
    return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
}

// Normalize angle to [-PI, PI).
inline double normalize_angle(const double angle)
{
    const double new_angle = fmod(angle + M_PI, M_PI * 2.0);
    return (new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle) - M_PI;
}

template <typename T>
inline T sqr(const T value)
{
    return value * value;
}

}

#include <iostream>
#define CHECK(v) (void)(v)
#define CHECK_NOTNULL(v) (void)(v)
#define CHECK_GE(a, b)
#define CHECK_GT(a, b)