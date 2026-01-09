#pragma once

#include <cmath>
#include <string>

namespace decision::math {

class Vec2d {
public:
    constexpr Vec2d() noexcept
        : Vec2d(0, 0)
    {
    }
    constexpr Vec2d(const double x, const double y) noexcept
        : _x(x)
        , _y(y)
    {
    }
    static Vec2d create_unit_vec(const double angle);

    double x() const { return _x; }
    double y() const { return _y; }
    double angle() const { return atan2(_y, _x); }
    void set_x(const double x) { _x = x; }
    void set_y(const double y) { _y = y; }
    void normalize();

    double length() const;
    double length_sqr() const;
    double distance_to(const Vec2d& other) const;
    double distance_sqr_to(const Vec2d& other) const;

    Vec2d operator+(const Vec2d& other) const;
    Vec2d operator-(const Vec2d& other) const;
    Vec2d operator*(const double ratio) const;
    Vec2d operator/(const double ratio) const;
    Vec2d& operator+=(const Vec2d& other);
    Vec2d& operator-=(const Vec2d& other);
    Vec2d& operator*=(const double ratio);
    Vec2d& operator/=(const double ratio);
    bool operator==(const Vec2d& other) const;

    //! rotate the vector by angle.
    Vec2d rotate(const double angle) const;

    double cross_prod(const Vec2d& other) const;
    double inner_prod(const Vec2d& other) const;

    std::string debug_string() const;

protected:
    double _x = 0.0;
    double _y = 0.0;
};

Vec2d operator*(const double ratio, const Vec2d& vec);

}
