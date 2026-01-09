#pragma once

#include "math/vec2d.h"

#include <string>

namespace decision::math {

class Segment2d {
public:
    Segment2d();
    Segment2d(const Vec2d& start, const Vec2d& end);

    const Vec2d& start() const { return _start; }
    const Vec2d& end() const { return _end; }
    const Vec2d& unit_direction() const { return _unit_direction; }
    Vec2d center() const { return (_start + _end) / 2.0; }
    double heading() const { return _heading; }
    double cos_heading() const { return _unit_direction.x(); }
    double sin_heading() const { return _unit_direction.y(); }

    double length() const;
    double length_sqr() const;

    double distance_to(const Vec2d& point) const;
    double distance_to(const Vec2d& point, Vec2d* const nearest_pt) const;
    double distance_sqr_to(const Vec2d& point) const;
    double distance_sqr_to(const Vec2d& point, Vec2d* const nearest_pt) const;

    bool is_point_in(const Vec2d& point) const;

    bool has_intersect(const Segment2d& other_segment) const;
    bool get_intersect(const Segment2d& other_segment, Vec2d* const point) const;

    double project_onto_unit(const Vec2d& point) const;
    double product_onto_unit(const Vec2d& point) const;
    double get_perpendicular_foot(const Vec2d& point, Vec2d* const foot_point) const;

    std::string debug_string() const;

protected:
    Vec2d _start;
    Vec2d _end;
    Vec2d _unit_direction;
    double _heading = 0.0;
    double _length = 0.0;
};

}
