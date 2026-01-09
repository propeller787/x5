#include "math/polygon3d.h"
#include "math/math_utils.h"

#include <algorithm>
#include <sstream>

using namespace decision::math;

Polygon3d::Polygon3d(const std::vector<Vec3d>& points)
    : _points(points)
{
    build_from_points();
}

Polygon2d Polygon3d::polygon2d() const
{
    return Polygon2d(_points2d);
}

inline size_t Polygon3d::next(size_t at) const
{
    return at >= _points.size() - 1 ? 0 : at + 1;
}

inline size_t Polygon3d::prev(size_t at) const
{
    return at == 0 ? _points.size() - 1 : at - 1;
}

void Polygon3d::build_from_points()
{
    auto num_points = _points.size();
    CHECK_GE(num_points, 3u);

    // remove duplicates
    std::vector<Vec3d> points_tmp(std::move(_points));
    _points.clear();
    _points2d.clear();
    for (size_t i = 0; i < num_points; ++i) {
        Vec3d vec3d(points_tmp[i].x(), points_tmp[i].y(), points_tmp[i].z());
        auto pos = std::find(_points.begin(), _points.end(), vec3d);
        if (pos == _points.end()) {
            _points2d.emplace_back(vec3d.x(), vec3d.y());
            _points.emplace_back(points_tmp[i]);
        }
    }
    num_points = _points.size();
    CHECK_GE(num_points, 3u);

    // Make sure the points are in ccw order.
    double area = 0.0;
    for (size_t i = 1; i < num_points; ++i) {
        area += cross_prod(_points2d[0], _points2d[i - 1], _points2d[i]);
    }
    if (area < 0) {
        area = -area;
        std::reverse(_points2d.begin(), _points2d.end());
        std::reverse(_points.begin(), _points.end());
    }
    CHECK_GT(area, kMathEpsilon);

    // Construct segments.
    _segments.reserve(num_points);
    for (size_t i = 0; i < num_points; ++i) {
        _segments.emplace_back(_points[i], _points[next(i)]);
    }
}

std::string Polygon3d::debug_string() const
{
    std::ostringstream sout;
    sout << "polygon3d (  num_points = " << _points.size() << "  points = (";
    for (const auto& pt : _points) {
        sout << " " << pt.debug_string();
    }
    sout.flush();
    return sout.str();
}
