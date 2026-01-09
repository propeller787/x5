// Copyright 2018 Baidu Inc. All Rights Reserved
// Author: Fuxiangyu (fuxiangyu@baidu.com)
//
// Description: polygon for 3d point, the polygon is constructed on 2d space, but the point is 3d

#pragma once

#include "math/polygon2d.h"
#include "math/segment3d.h"
#include "math/vec2d.h"
#include "math/vec3d.h"

namespace decision::math {

class Polygon3d {
public:
    Polygon3d() = default;
    explicit Polygon3d(const std::vector<Vec3d>& points);

    Polygon2d polygon2d() const;
    const std::vector<Vec3d>& points() const { return _points; }
    const std::vector<Segment3d>& segments() const { return _segments; }

    std::string debug_string() const;

protected:
    void build_from_points();

protected:
    inline size_t next(size_t at) const;
    inline size_t prev(size_t at) const;

    std::vector<Vec3d> _points;
    std::vector<Vec2d> _points2d;
    std::vector<Segment3d> _segments;
};

}
