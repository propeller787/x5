#include "math/segment2d.h"
#include "math/math_utils.h"

#include <algorithm>
#include <cmath>
#include <sstream>

using namespace decision::math;

namespace {

bool is_within(double val, double bound1, double bound2)
{
    if (bound1 > bound2) {
        std::swap(bound1, bound2);
    }
    return val >= bound1 - kMathEpsilon && val <= bound2 + kMathEpsilon;
}

} // namespace

Segment2d::Segment2d()
{
    _unit_direction = Vec2d(1, 0);
}

Segment2d::Segment2d(const Vec2d& start, const Vec2d& end)
    : _start(start)
    , _end(end)
{

    const double dx = _end.x() - _start.x();
    const double dy = _end.y() - _start.y();
    _length = hypot(dx, dy);
    _unit_direction = (_length <= kMathEpsilon ? Vec2d(0, 0) : Vec2d(dx / _length, dy / _length));
    _heading = _unit_direction.angle();
}

double Segment2d::length() const
{
    return _length;
}

double Segment2d::length_sqr() const
{
    return _length * _length;
}

double Segment2d::distance_to(const Vec2d& point) const
{
    if (_length <= kMathEpsilon) {
        return point.distance_to(_start);
    }
    const double x0 = point.x() - _start.x();
    const double y0 = point.y() - _start.y();
    const double proj = x0 * _unit_direction.x() + y0 * _unit_direction.y();
    if (proj <= 0.0) {
        return hypot(x0, y0);
    }
    if (proj >= _length) {
        return point.distance_to(_end);
    }
    return std::abs(x0 * _unit_direction.y() - y0 * _unit_direction.x());
}

double Segment2d::distance_to(const Vec2d& point, Vec2d* const nearest_pt) const
{
    CHECK_NOTNULL(nearest_pt);
    if (_length <= kMathEpsilon) {
        *nearest_pt = _start;
        return point.distance_to(_start);
    }
    const double x0 = point.x() - _start.x();
    const double y0 = point.y() - _start.y();
    const double proj = x0 * _unit_direction.x() + y0 * _unit_direction.y();
    if (proj < 0.0) {
        *nearest_pt = _start;
        return hypot(x0, y0);
    }
    if (proj > _length) {
        *nearest_pt = _end;
        return point.distance_to(_end);
    }
    *nearest_pt = _start + _unit_direction * proj;
    return std::abs(x0 * _unit_direction.y() - y0 * _unit_direction.x());
}

double Segment2d::distance_sqr_to(const Vec2d& point) const
{
    if (_length <= kMathEpsilon) {
        return point.distance_sqr_to(_start);
    }
    const double x0 = point.x() - _start.x();
    const double y0 = point.y() - _start.y();
    const double proj = x0 * _unit_direction.x() + y0 * _unit_direction.y();
    if (proj <= 0.0) {
        return sqr(x0) + sqr(y0);
    }
    if (proj >= _length) {
        return point.distance_sqr_to(_end);
    }
    return sqr(x0 * _unit_direction.y() - y0 * _unit_direction.x());
}

double Segment2d::distance_sqr_to(const Vec2d& point, Vec2d* const nearest_pt) const
{
    CHECK_NOTNULL(nearest_pt);
    if (_length <= kMathEpsilon) {
        *nearest_pt = _start;
        return point.distance_sqr_to(_start);
    }
    const double x0 = point.x() - _start.x();
    const double y0 = point.y() - _start.y();
    const double proj = x0 * _unit_direction.x() + y0 * _unit_direction.y();
    if (proj <= 0.0) {
        *nearest_pt = _start;
        return sqr(x0) + sqr(y0);
    }
    if (proj >= _length) {
        *nearest_pt = _end;
        return point.distance_sqr_to(_end);
    }
    *nearest_pt = _start + _unit_direction * proj;
    return sqr(x0 * _unit_direction.y() - y0 * _unit_direction.x());
}

bool Segment2d::is_point_in(const Vec2d& point) const
{
    if (_length <= kMathEpsilon) {
        return std::abs(point.x() - _start.x()) <= kMathEpsilon && std::abs(point.y() - _start.y()) <= kMathEpsilon;
    }
    const double prod = cross_prod(point, _start, _end);
    if (std::abs(prod) > kMathEpsilon) {
        return false;
    }
    return is_within(point.x(), _start.x(), _end.x()) && is_within(point.y(), _start.y(), _end.y());
}

double Segment2d::project_onto_unit(const Vec2d& point) const
{
    return _unit_direction.inner_prod(point - _start);
}

double Segment2d::product_onto_unit(const Vec2d& point) const
{
    return _unit_direction.cross_prod(point - _start);
}

bool Segment2d::has_intersect(const Segment2d& other_segment) const
{
    Vec2d point;
    return get_intersect(other_segment, &point);
}

bool Segment2d::get_intersect(const Segment2d& other_segment, Vec2d* const point) const
{
    CHECK_NOTNULL(point);
    if (is_point_in(other_segment.start())) {
        *point = other_segment.start();
        return true;
    }
    if (is_point_in(other_segment.end())) {
        *point = other_segment.end();
        return true;
    }
    if (other_segment.is_point_in(_start)) {
        *point = _start;
        return true;
    }
    if (other_segment.is_point_in(_end)) {
        *point = _end;
        return true;
    }
    if (_length <= kMathEpsilon || other_segment.length() <= kMathEpsilon) {
        return false;
    }
    const double cc1 = cross_prod(_start, _end, other_segment.start());
    const double cc2 = cross_prod(_start, _end, other_segment.end());
    if (cc1 * cc2 >= -kMathEpsilon) {
        return false;
    }
    const double cc3 = cross_prod(other_segment.start(), other_segment.end(), _start);
    const double cc4 = cross_prod(other_segment.start(), other_segment.end(), _end);
    if (cc3 * cc4 >= -kMathEpsilon) {
        return false;
    }
    const double ratio = cc4 / (cc4 - cc3);
    *point = Vec2d(_start.x() * ratio + _end.x() * (1.0 - ratio),
        _start.y() * ratio + _end.y() * (1.0 - ratio));
    return true;
}

// return distance with perpendicular foot point.
double Segment2d::get_perpendicular_foot(const Vec2d& point, Vec2d* const foot_point) const
{
    CHECK_NOTNULL(foot_point);
    if (_length <= kMathEpsilon) {
        *foot_point = _start;
        return point.distance_to(_start);
    }
    const double x0 = point.x() - _start.x();
    const double y0 = point.y() - _start.y();
    const double proj = x0 * _unit_direction.x() + y0 * _unit_direction.y();
    *foot_point = _start + _unit_direction * proj;
    return std::abs(x0 * _unit_direction.y() - y0 * _unit_direction.x());
}

std::string Segment2d::debug_string() const
{
    std::ostringstream sout;
    sout << "segment2d ( start = " << _start.debug_string()
         << "  end = " << _end.debug_string() << " )";
    sout.flush();
    return sout.str();
}
