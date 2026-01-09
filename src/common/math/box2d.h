#pragma once

#include "math/aabox2d.h"
#include "math/segment2d.h"
#include "math/vec2d.h"

#include <string>
#include <vector>

namespace decision::math {

class Box2d {
public:
    Box2d(const Vec2d& center, const double heading,
        const double length, const double width);
    Box2d(const Segment2d& axis, const double width);
    explicit Box2d(const AABox2d& aabox);
    static Box2d create_aa_box(const Vec2d& one_corner, const Vec2d& opponent_corner);

    const Vec2d& center() const { return _center; }
    double center_x() const { return _center.x(); }
    double center_y() const { return _center.y(); }
    double length() const { return _length; }
    double width() const { return _width; }
    double half_length() const { return _half_length; }
    double half_width() const { return _half_width; }
    double heading() const { return _heading; }
    double cos_heading() const { return _cos_heading; }
    double sin_heading() const { return _sin_heading; }
    double area() const { return _length * _width; }
    double diagonal() const { return hypot(_length, _width); }

    void get_all_corners(std::vector<Vec2d>* const corners) const;

    bool is_point_in(const Vec2d& point) const;
    bool is_point_on_boundary(const Vec2d& point) const;

    double distance_to(const Vec2d& point) const;
    double distance_to(const Segment2d& segment) const;
    double distance_to(const Box2d& box) const;

    bool has_overlap(const Segment2d& segment) const;
    bool has_overlap(const Box2d& box) const;

    AABox2d get_aa_box() const;

    void rotate_from_center(double rotate_angle);
    void shift(const Vec2d& shift_vec);

    std::string debug_string() const;

protected:
    Vec2d _center;
    double _length = 0.0;
    double _width = 0.0;
    double _half_length = 0.0;
    double _half_width = 0.0;
    double _heading = 0.0;
    double _cos_heading = 1.0;
    double _sin_heading = 0.0;
};

}
