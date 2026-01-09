#include "math/vec3d.h"
#include "math/math_utils.h"

#include <sstream>

using namespace decision::math;

Vec3d Vec3d::create_unit_vec(const double yaw_angle, const double pitch_angle)
{
    double x = cos(pitch_angle) * cos(yaw_angle);
    double y = cos(pitch_angle) * sin(yaw_angle);
    double z = sin(pitch_angle);
    return Vec3d(x, y, z);
}

double Vec3d::length() const
{
    return hypot(hypot(_x, _y), _z);
}

double Vec3d::length_sqr() const
{
    return _x * _x + _y * _y + _z * _z;
}

double Vec3d::distance_to(const Vec3d& other) const
{
    return hypot(hypot(_x - other.x(), _y - other.y()), _z - other.z());
}

double Vec3d::distance_sqr_to(const Vec3d& other) const
{
    const double dx = _x - other._x;
    const double dy = _y - other._y;
    const double dz = _z - other._z;
    return dx * dx + dy * dy + dz * dz;
}

void Vec3d::normalize()
{
    const double l = length();
    if (l > kMathEpsilon) {
        _x /= l;
        _y /= l;
        _z /= l;
    }
}

Vec3d Vec3d::operator+(const Vec3d& other) const
{
    return Vec3d(_x + other.x(), _y + other.y(), _z + other.z());
}

Vec3d Vec3d::operator-(const Vec3d& other) const
{
    return Vec3d(_x - other.x(), _y - other.y(), _z - other.z());
}

Vec3d Vec3d::operator*(const double ratio) const
{
    return Vec3d(_x * ratio, _y * ratio, _z * ratio);
}

Vec3d Vec3d::operator/(const double ratio) const
{
    CHECK_GT(std::abs(ratio), kMathEpsilon);
    return Vec3d(_x / ratio, _y / ratio, _z / ratio);
}

Vec3d& Vec3d::operator+=(const Vec3d& other)
{
    _x += other.x();
    _y += other.y();
    _z += other.z();
    return *this;
}

Vec3d& Vec3d::operator-=(const Vec3d& other)
{
    _x -= other.x();
    _y -= other.y();
    _z -= other.z();
    return *this;
}

Vec3d& Vec3d::operator*=(const double ratio)
{
    _x *= ratio;
    _y *= ratio;
    _z *= ratio;
    return *this;
}

Vec3d& Vec3d::operator/=(const double ratio)
{
    CHECK_GT(std::abs(ratio), kMathEpsilon);
    _x /= ratio;
    _y /= ratio;
    _z /= ratio;
    return *this;
}

bool Vec3d::operator==(const Vec3d& other) const
{
    return (std::abs(_x - other.x()) < kMathEpsilon && std::abs(_y - other.y()) < kMathEpsilon && std::abs(_z - other.z()) < kMathEpsilon);
}

Vec3d Vec3d::cross_prod(const Vec3d& other) const
{
    double x = _y * other.z() - _z * other.y();
    double y = _z * other.x() - _x * other.z();
    double z = _x * other.y() - _y * other.x();
    return Vec3d(x, y, z);
}

double Vec3d::inner_prod(const Vec3d& other) const
{
    return _x * other.x() + _y * other.y() + _z * other.z();
}

Vec3d operator*(const double ratio, const Vec3d& vec)
{
    return vec * ratio;
}

std::string Vec3d::debug_string() const
{
    std::ostringstream sout;
    sout << "vec3d ( x = " << _x << "  y = " << _y << "  z = " << _z << " )";
    sout.flush();
    return sout.str();
}
