#include "math/aabox2d.h"
#include "math/math_utils.h"

#include <cmath>
#include <sstream>

using namespace decision::math;

AABox2d::AABox2d(const Vec2d& center, const double length, const double width)
    : _center(center)
    , _length(length)
    , _width(width)
    , _half_length(length / 2.0)
    , _half_width(width / 2.0)
{
    CHECK_GT(_length, -kMathEpsilon);
    CHECK_GT(_width, -kMathEpsilon);
}

AABox2d::AABox2d(const Vec2d& one_corner, const Vec2d& opponent_corner)
    : AABox2d((one_corner + opponent_corner) / 2.0,
        std::abs(one_corner.x() - opponent_corner.x()),
        std::abs(one_corner.y() - opponent_corner.y()))
{
}

AABox2d::AABox2d(const std::vector<Vec2d>& points)
{
    CHECK(!points.empty());
    double min_x = points[0].x();
    double max_x = points[0].x();
    double min_y = points[0].y();
    double max_y = points[0].y();
    for (const auto& point : points) {
        min_x = std::min(min_x, point.x());
        max_x = std::max(max_x, point.x());
        min_y = std::min(min_y, point.y());
        max_y = std::max(max_y, point.y());
    }

    _center = { (min_x + max_x) / 2.0, (min_y + max_y) / 2.0 };
    _length = max_x - min_x;
    _width = max_y - min_y;
    _half_length = _length / 2.0;
    _half_width = _width / 2.0;
}

void AABox2d::get_all_corners(std::vector<Vec2d>* const corners) const
{
    CHECK_NOTNULL(corners)->clear();
    corners->reserve(4);
    corners->emplace_back(_center.x() + _half_length, _center.y() - _half_width);
    corners->emplace_back(_center.x() + _half_length, _center.y() + _half_width);
    corners->emplace_back(_center.x() - _half_length, _center.y() + _half_width);
    corners->emplace_back(_center.x() - _half_length, _center.y() - _half_width);
}

bool AABox2d::is_point_in(const Vec2d& point) const
{
    return std::abs(point.x() - _center.x()) <= _half_length + kMathEpsilon && std::abs(point.y() - _center.y()) <= _half_width + kMathEpsilon;
}

bool AABox2d::is_point_on_boundary(const Vec2d& point) const
{
    const double dx = std::abs(point.x() - _center.x());
    const double dy = std::abs(point.y() - _center.y());
    return (std::abs(dx - _half_length) <= kMathEpsilon && dy <= _half_width + kMathEpsilon) || (std::abs(dy - _half_width) <= kMathEpsilon && dx <= _half_length + kMathEpsilon);
}

double AABox2d::distance_to(const Vec2d& point) const
{
    const double dx = std::abs(point.x() - _center.x()) - _half_length;
    const double dy = std::abs(point.y() - _center.y()) - _half_width;
    if (dx <= 0.0) {
        return std::max(0.0, dy);
    }
    if (dy <= 0.0) {
        return dx;
    }
    return hypot(dx, dy);
}

double AABox2d::distance_to(const AABox2d& box) const
{
    const double dx = std::abs(box.center_x() - _center.x()) - box.half_length() - _half_length;
    const double dy = std::abs(box.center_y() - _center.y()) - box.half_width() - _half_width;
    if (dx <= 0.0) {
        return std::max(0.0, dy);
    }
    if (dy <= 0.0) {
        return dx;
    }
    return hypot(dx, dy);
}

bool AABox2d::has_overlap(const AABox2d& box) const
{
    return std::abs(box.center_x() - _center.x()) <= box.half_length() + _half_length && std::abs(box.center_y() - _center.y()) <= box.half_width() + _half_width;
}

void AABox2d::shift(const Vec2d& shift_vec)
{
    _center += shift_vec;
}

void AABox2d::merge_from(const AABox2d& other_box)
{
    const double x1 = std::min(min_x(), other_box.min_x());
    const double x2 = std::max(max_x(), other_box.max_x());
    const double y1 = std::min(min_y(), other_box.min_y());
    const double y2 = std::max(max_y(), other_box.max_y());
    _center = Vec2d((x1 + x2) / 2.0, (y1 + y2) / 2.0);
    _length = x2 - x1;
    _width = y2 - y1;
    _half_length = _length / 2.0;
    _half_width = _width / 2.0;
}

void AABox2d::merge_from(const Vec2d& other_point)
{
    const double x1 = std::min(min_x(), other_point.x());
    const double x2 = std::max(max_x(), other_point.x());
    const double y1 = std::min(min_y(), other_point.y());
    const double y2 = std::max(max_y(), other_point.y());
    _center = Vec2d((x1 + x2) / 2.0, (y1 + y2) / 2.0);
    _length = x2 - x1;
    _width = y2 - y1;
    _half_length = _length / 2.0;
    _half_width = _width / 2.0;
}

std::string AABox2d::debug_string() const
{
    std::ostringstream sout;
    sout << "aabox2d ( center = " << _center.debug_string()
         << "  length = " << _length
         << "  width = " << _width << " )";
    sout.flush();
    return sout.str();
}
