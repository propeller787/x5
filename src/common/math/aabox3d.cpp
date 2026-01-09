#include "math/aabox3d.h"
#include "math/math_utils.h"

#include <sstream>

using namespace decision::math;

AABox3d::AABox3d(const Vec3d& center, const double length, const double width, const double height)
    : _center(center)
    , _length(length)
    , _width(width)
    , _height(height)
    , _half_length(length / 2.0)
    , _half_width(width / 2.0)
    , _half_height(height / 2.0)
{
    CHECK_GT(_length, -kMathEpsilon);
    CHECK_GT(_width, -kMathEpsilon);
    CHECK_GT(_height, -kMathEpsilon);
}

AABox3d::AABox3d(const Vec3d& one_corner, const Vec3d& opponent_corner)
    : AABox3d((one_corner + opponent_corner) / 2.0,
        std::abs(one_corner.x() - opponent_corner.x()),
        std::abs(one_corner.y() - opponent_corner.y()),
        std::abs(one_corner.z() - opponent_corner.z()))
{
}

AABox3d::AABox3d(const std::vector<Vec3d>& points)
{
    CHECK(!points.empty());
    double min_x = points[0].x();
    double max_x = points[0].x();
    double min_y = points[0].y();
    double max_y = points[0].y();
    double min_z = points[0].z();
    double max_z = points[0].z();
    for (const auto& point : points) {
        min_x = std::min(min_x, point.x());
        max_x = std::max(max_x, point.x());
        min_y = std::min(min_y, point.y());
        max_y = std::max(max_y, point.y());
        min_z = std::min(min_z, point.z());
        max_z = std::max(max_z, point.z());
    }

    _center = { (min_x + max_x) / 2.0, (min_y + max_y) / 2.0, (min_z + max_z) / 2.0 };
    _length = max_x - min_x;
    _width = max_y - min_y;
    _height = max_z - min_z;
    _half_length = _length / 2.0;
    _half_width = _width / 2.0;
    _half_height = _height / 2.0;
}

AABox2d AABox3d::aabox2d() const
{
    return AABox2d({ _center.x(), _center.y() }, _length, _width);
}

void AABox3d::get_all_corners(std::vector<Vec3d>* const corners) const
{
    CHECK_NOTNULL(corners)->clear();
    corners->reserve(8);
    corners->emplace_back(_center.x() - _half_length,
        _center.y() - _half_width,
        _center.z() - _half_height);
    corners->emplace_back(_center.x() + _half_length,
        _center.y() - _half_width,
        _center.z() - _half_height);
    corners->emplace_back(_center.x() + _half_length,
        _center.y() + _half_width,
        _center.z() - _half_height);
    corners->emplace_back(_center.x() - _half_length,
        _center.y() + _half_width,
        _center.z() - _half_height);

    corners->emplace_back(_center.x() - _half_length,
        _center.y() - _half_width,
        _center.z() + _half_height);
    corners->emplace_back(_center.x() + _half_length,
        _center.y() - _half_width,
        _center.z() + _half_height);
    corners->emplace_back(_center.x() + _half_length,
        _center.y() + _half_width,
        _center.z() + _half_height);
    corners->emplace_back(_center.x() - _half_length,
        _center.y() + _half_width,
        _center.z() + _half_height);
}

bool AABox3d::is_point_in(const Vec3d& point) const
{
    return std::abs(point.x() - _center.x()) <= _half_length + kMathEpsilon && std::abs(point.y() - _center.y()) <= _half_width + kMathEpsilon && std::abs(point.z() - _center.z()) <= _half_height + kMathEpsilon;
}

bool AABox3d::is_point_on_boundary(const Vec3d& point) const
{
    const double dx = std::abs(point.x() - _center.x());
    const double dy = std::abs(point.y() - _center.y());
    const double dz = std::abs(point.z() - _center.z());
    return (std::abs(dx - _half_length) <= kMathEpsilon && std::abs(dz - _half_height) <= kMathEpsilon && dy <= _half_width + kMathEpsilon) || (std::abs(dy - _half_width) <= kMathEpsilon && std::abs(dz - _half_height) <= kMathEpsilon && dx <= _half_length + kMathEpsilon) || (std::abs(dx - _half_length) <= kMathEpsilon && std::abs(dy - _half_width) <= kMathEpsilon && dz <= _half_height + kMathEpsilon);
}

double AABox3d::distance_to(const Vec3d& point) const
{
    const double dx = std::abs(point.x() - _center.x()) - _half_length;
    const double dy = std::abs(point.y() - _center.y()) - _half_width;
    const double dz = std::abs(point.z() - _center.z()) - _half_height;
    return std::sqrt(std::max(0.0, dx) * std::max(0.0, dx) + std::max(0.0, dy) * std::max(0.0, dy) + std::max(0.0, dz) * std::max(0.0, dz));
}

void AABox3d::shift(const Vec3d& shift_vec)
{
    _center += shift_vec;
}

void AABox3d::merge_from(const AABox3d& other_box)
{
    const double x1 = std::min(min_x(), other_box.min_x());
    const double x2 = std::max(max_x(), other_box.max_x());
    const double y1 = std::min(min_y(), other_box.min_y());
    const double y2 = std::max(max_y(), other_box.max_y());
    const double z1 = std::min(min_z(), other_box.min_z());
    const double z2 = std::max(max_z(), other_box.max_z());
    _center = Vec3d((x1 + x2) / 2.0, (y1 + y2) / 2.0, (z1 + z2) / 2.0);
    _length = x2 - x1;
    _width = y2 - y1;
    _height = z2 - z1;
    _half_length = _length / 2.0;
    _half_width = _width / 2.0;
    _half_height = _height / 2.0;
}

std::string AABox3d::debug_string() const
{
    std::ostringstream sout;
    sout << "aabox3d ( center = " << _center.debug_string()
         << "  length = " << _length
         << "  width = " << _width
         << "  height = " << _height << " )";
    sout.flush();
    return sout.str();
}
