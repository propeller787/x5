#pragma once

#include "math/vec2d.h"

#include <string>
#include <vector>

namespace decision::math {

class AABox2d {
public:
    AABox2d() = default;
    AABox2d(const Vec2d& center, const double length, const double width);
    AABox2d(const Vec2d& one_corner, const Vec2d& opponent_corner);
    AABox2d(const std::vector<Vec2d>& points);

    const Vec2d& center() const { return _center; }
    double center_x() const { return _center.x(); }
    double center_y() const { return _center.y(); }
    double length() const { return _length; }
    double width() const { return _width; }
    double half_length() const { return _half_length; }
    double half_width() const { return _half_width; }
    double area() const { return _length * _width; }
    double min_x() const { return _center.x() - _half_length; }
    double max_x() const { return _center.x() + _half_length; }
    double min_y() const { return _center.y() - _half_width; }
    double max_y() const { return _center.y() + _half_width; }

    // Get all corners in counter clockwise order.
    void get_all_corners(std::vector<Vec2d>* const corners) const;

    bool is_point_in(const Vec2d& point) const;
    bool is_point_on_boundary(const Vec2d& point) const;

    double distance_to(const Vec2d& point) const;
    double distance_to(const AABox2d& box) const;

    bool has_overlap(const AABox2d& box) const;

    // Shift the center of AABox by the input vector.
    void shift(const Vec2d& shift_vec);

    void merge_from(const AABox2d& other_box);
    void merge_from(const Vec2d& other_point);

    std::string debug_string() const;

protected:
    Vec2d _center;
    double _length = 0.0;
    double _width = 0.0;
    double _half_length = 0.0;
    double _half_width = 0.0;
};

}
