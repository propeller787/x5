// Copyright 2018 Baidu Inc. All Rights Reserved
// Author: Fuxiangyu (fuxiangyu@baidu.com)
//
// Description: segment for 3d

#pragma once

#include "math/segment2d.h"
#include "math/vec3d.h"

namespace decision::math {

class Segment3d {
public:
    Segment3d();
    Segment3d(const Vec3d& start, const Vec3d& end);

    const Vec3d& start() const { return _start; }
    const Vec3d& end() const { return _end; }
    const Vec3d& unit_direction() const { return _unit_direction; }
    Vec3d center() const { return (_start + _end) / 2.0; }

    double length() const;
    double length_sqr() const;

    double distance_to(const Vec3d& point) const;
    double distance_to(const Vec3d& point, Vec3d* const nearest_pt) const;
    double distance_sqr_to(const Vec3d& point) const;
    double distance_sqr_to(const Vec3d& point, Vec3d* const nearest_pt) const;

    bool is_point_in(const Vec3d& point) const;

    bool has_intersect(const Segment3d& other_segment) const;
    bool get_intersect(const Segment3d& other_segment, Vec3d* const point) const;

    double project_onto_unit(const Vec3d& point) const;
    double product_onto_unit(const Vec3d& point) const;
    double get_perpendicular_foot(const Vec3d& point, Vec3d* const foot_point) const;

    Segment2d segment2d() const;

    std::string debug_string() const;

protected:
    Vec3d _start;
    Vec3d _end;
    Vec3d _unit_direction;
    double _length = 0.0;
};

}
