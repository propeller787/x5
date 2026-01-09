// Copyright 2018 Baidu Inc. All Rights Reserved
// Author: Fuxiangyu (fuxiangyu@baidu.com)
//
// Description: aabox for 3d

#pragma once

#include "math/aabox2d.h"
#include "math/vec3d.h"

namespace decision::math {

class AABox3d {
public:
    AABox3d() = default;
    AABox3d(const Vec3d& center, const double length, const double width, const double height);
    AABox3d(const Vec3d& one_corner, const Vec3d& opponent_corner);
    AABox3d(const std::vector<Vec3d>& points);

    AABox2d aabox2d() const;
    const Vec3d& center() const { return _center; }
    double length() const { return _length; }
    double width() const { return _width; }
    double height() const { return _height; }
    double min_x() const { return _center.x() - _half_length; }
    double max_x() const { return _center.x() + _half_length; }
    double min_y() const { return _center.y() - _half_width; }
    double max_y() const { return _center.y() + _half_width; }
    double min_z() const { return _center.z() - _half_height; }
    double max_z() const { return _center.z() + _half_height; }

    // Get all corners in counter clockwise order.
    void get_all_corners(std::vector<Vec3d>* const corners) const;

    bool is_point_in(const Vec3d& point) const;
    bool is_point_on_boundary(const Vec3d& point) const;

    double distance_to(const Vec3d& point) const;

    // Shift the center of AABox by the input vector.
    void shift(const Vec3d& shift_vec);

    void merge_from(const AABox3d& other_box);

    std::string debug_string() const;

protected:
    Vec3d _center;
    double _length = 0.0;
    double _width = 0.0;
    double _height = 0.0;
    double _half_length = 0.0;
    double _half_width = 0.0;
    double _half_height = 0.0;
};

}
