#include "math/vec2d.h"
#include "math/math_utils.h"

#include <cmath>
#include <sstream>

using namespace decision::math;

Vec2d Vec2d::create_unit_vec(const double angle)
{
    return Vec2d(cos(angle), sin(angle));
}

double Vec2d::length() const
{
    return hypot(_x, _y);
}

double Vec2d::length_sqr() const
{
    return _x * _x + _y * _y;
}

double Vec2d::distance_to(const Vec2d& other) const
{
    return hypot(_x - other._x, _y - other._y);
}

double Vec2d::distance_sqr_to(const Vec2d& other) const
{
    const double dx = _x - other._x;
    const double dy = _y - other._y;
    return dx * dx + dy * dy;
}

void Vec2d::normalize()
{
    const double l = length();
    if (l > kMathEpsilon) {
        _x /= l;
        _y /= l;
    }
}

Vec2d Vec2d::operator+(const Vec2d& other) const
{
    return Vec2d(_x + other.x(), _y + other.y());
}

Vec2d Vec2d::operator-(const Vec2d& other) const
{
    return Vec2d(_x - other.x(), _y - other.y());
}

Vec2d Vec2d::operator*(const double ratio) const
{
    return Vec2d(_x * ratio, _y * ratio);
}

Vec2d Vec2d::operator/(const double ratio) const
{
    CHECK_GT(std::abs(ratio), kMathEpsilon);
    return Vec2d(_x / ratio, _y / ratio);
}

Vec2d& Vec2d::operator+=(const Vec2d& other)
{
    _x += other.x();
    _y += other.y();
    return *this;
}

Vec2d& Vec2d::operator-=(const Vec2d& other)
{
    _x -= other.x();
    _y -= other.y();
    return *this;
}

Vec2d& Vec2d::operator*=(const double ratio)
{
    _x *= ratio;
    _y *= ratio;
    return *this;
}

Vec2d& Vec2d::operator/=(const double ratio)
{
    CHECK_GT(std::abs(ratio), kMathEpsilon);
    _x /= ratio;
    _y /= ratio;
    return *this;
}

bool Vec2d::operator==(const Vec2d& other) const
{
    return (std::abs(_x - other.x()) < kMathEpsilon && std::abs(_y - other.y()) < kMathEpsilon);
}

Vec2d Vec2d::rotate(const double angle) const
{
    return Vec2d(_x * cos(angle) - _y * sin(angle),
        _x * sin(angle) + _y * cos(angle));
}

double Vec2d::cross_prod(const Vec2d& other) const
{
    return _x * other.y() - _y * other.x();
}

double Vec2d::inner_prod(const Vec2d& other) const
{
    return _x * other.x() + _y * other.y();
}

namespace decision::math {

Vec2d operator*(const double ratio, const Vec2d& vec)
{
    return vec * ratio;
}

}

std::string Vec2d::debug_string() const
{
    std::ostringstream sout;
    sout << "vec2d ( x = " << _x << "  y = " << _y << " )";
    sout.flush();
    return sout.str();
}
