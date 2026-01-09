#include "math/segment3d.h"
#include "math/math_utils.h"

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

Segment3d::Segment3d()
{
    _unit_direction = Vec3d(1, 0, 0);
}

Segment3d::Segment3d(const Vec3d& start, const Vec3d& end)
    : _start(start)
    , _end(end)
{

    const double dx = _end.x() - _start.x();
    const double dy = _end.y() - _start.y();
    const double dz = _end.z() - _start.z();
    _length = hypot(hypot(dx, dy), dz);
    _unit_direction = (_length <= kMathEpsilon ? Vec3d(0, 0, 0) : Vec3d(dx / _length, dy / _length, dz / _length));
}

double Segment3d::length() const
{
    return _length;
}

double Segment3d::length_sqr() const
{
    return _length * _length;
}

double Segment3d::distance_to(const Vec3d& point) const
{
    if (_length <= kMathEpsilon) {
        return point.distance_to(_start);
    }
    const double x0 = point.x() - _start.x();
    const double y0 = point.y() - _start.y();
    const double z0 = point.z() - _start.z();
    const double proj = x0 * _unit_direction.x() + y0 * _unit_direction.y() + z0 * _unit_direction.z();
    if (proj <= 0.0) {
        return hypot(hypot(x0, y0), z0);
    }
    if (proj >= _length) {
        return point.distance_to(_end);
    }
    Vec3d vec1 = Vec3d(x0, y0, z0);
    Vec3d c_p = _unit_direction.cross_prod(vec1);
    return c_p.length();
}

double Segment3d::distance_to(const Vec3d& point, Vec3d* const nearest_pt) const
{
    CHECK_NOTNULL(nearest_pt);
    if (_length <= kMathEpsilon) {
        *nearest_pt = _start;
        return point.distance_to(_start);
    }
    const double x0 = point.x() - _start.x();
    const double y0 = point.y() - _start.y();
    const double z0 = point.z() - _start.z();
    const double proj = x0 * _unit_direction.x() + y0 * _unit_direction.y() + z0 * _unit_direction.z();
    if (proj < 0.0) {
        *nearest_pt = _start;
        return hypot(hypot(x0, y0), z0);
    }
    if (proj > _length) {
        *nearest_pt = _end;
        return point.distance_to(_end);
    }
    *nearest_pt = _start + _unit_direction * proj;
    Vec3d vec1 = Vec3d(x0, y0, z0);
    Vec3d c_p = _unit_direction.cross_prod(vec1);
    return c_p.length();
}

double Segment3d::distance_sqr_to(const Vec3d& point) const
{
    if (_length <= kMathEpsilon) {
        return point.distance_sqr_to(_start);
    }
    const double x0 = point.x() - _start.x();
    const double y0 = point.y() - _start.y();
    const double z0 = point.z() - _start.z();
    const double proj = x0 * _unit_direction.x() + y0 * _unit_direction.y() + z0 * _unit_direction.z();
    if (proj <= 0.0) {
        return sqr(x0) + sqr(y0) + sqr(z0);
    }
    if (proj >= _length) {
        return point.distance_sqr_to(_end);
    }
    Vec3d vec1 = Vec3d(x0, y0, z0);
    Vec3d c_p = _unit_direction.cross_prod(vec1);

    return c_p.length_sqr();
}

double Segment3d::distance_sqr_to(const Vec3d& point, Vec3d* const nearest_pt) const
{
    CHECK_NOTNULL(nearest_pt);
    if (_length <= kMathEpsilon) {
        *nearest_pt = _start;
        return point.distance_sqr_to(_start);
    }
    const double x0 = point.x() - _start.x();
    const double y0 = point.y() - _start.y();
    const double z0 = point.z() - _start.z();
    const double proj = x0 * _unit_direction.x() + y0 * _unit_direction.y() + z0 * _unit_direction.z();
    if (proj < 0.0) {
        *nearest_pt = _start;
        return sqr(x0) + sqr(y0) + sqr(z0);
    }
    if (proj > _length) {
        *nearest_pt = _end;
        return point.distance_sqr_to(_end);
    }
    *nearest_pt = _start + _unit_direction * proj;
    Vec3d vec1 = Vec3d(x0, y0, z0);
    Vec3d c_p = _unit_direction.cross_prod(vec1);
    return c_p.length_sqr();
}

bool Segment3d::is_point_in(const Vec3d& point) const
{
    if (_length <= kMathEpsilon) {
        return std::abs(point.x() - _start.x()) <= kMathEpsilon && std::abs(point.y() - _start.y()) <= kMathEpsilon && std::abs(point.z() - _start.z()) <= kMathEpsilon;
    }
    const double prod = (_unit_direction.cross_prod(point - _start)).length();
    if (std::abs(prod) > kMathEpsilon) {
        return false;
    }
    return is_within(point.x(), _start.x(), _end.x()) && is_within(point.y(), _start.y(), _end.y()) && is_within(point.z(), _start.z(), _end.z());
}

double Segment3d::project_onto_unit(const Vec3d& point) const
{
    return _unit_direction.inner_prod(point - _start);
}

double Segment3d::product_onto_unit(const Vec3d& point) const
{
    Vec3d c_p = _unit_direction.cross_prod(point - _start);
    return c_p.length();
}

bool Segment3d::has_intersect(const Segment3d& other_segment) const
{
    Vec3d point;
    return get_intersect(other_segment, &point);
}

bool Segment3d::get_intersect(const Segment3d& other_segment, Vec3d* const point) const
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
    Vec3d con_perpendicular = _unit_direction.cross_prod(other_segment.unit_direction());
    Vec3d cross_line = other_segment.end() - _start;
    const double mix_prod = con_perpendicular.inner_prod(cross_line);
    // not in common plane
    if (fabs(mix_prod) > kMathEpsilon) {
        return false;
    }
    Vec3d v1 = cross_prod(_start, _end, other_segment.start());
    v1.normalize();
    Vec3d v2 = cross_prod(_start, _end, other_segment.end());
    v1.normalize();
    const double cc1 = v1.inner_prod(v2);
    if (cc1 >= -kMathEpsilon) {
        return false;
    }
    Vec3d v3 = cross_prod(other_segment.start(), other_segment.end(), _start);
    v3.normalize();
    Vec3d v4 = cross_prod(other_segment.start(), other_segment.end(), _end);
    v4.normalize();
    const double cc2 = v3.inner_prod(v4);

    if (cc2 >= -kMathEpsilon) {
        return false;
    }
    const double ratio = v3.length() / (v3.length() + v4.length());
    *point = _start * (1 - ratio) + _end * ratio;
    return true;
}

// return distance with perpendicular foot point.
double Segment3d::get_perpendicular_foot(const Vec3d& point, Vec3d* const foot_point) const
{
    CHECK_NOTNULL(foot_point);
    if (_length <= kMathEpsilon) {
        *foot_point = _start;
        return point.distance_to(_start);
    }
    const double x0 = point.x() - _start.x();
    const double y0 = point.y() - _start.y();
    const double z0 = point.z() - _start.z();
    const double proj = x0 * _unit_direction.x() + y0 * _unit_direction.y() + z0 * _unit_direction.z();
    *foot_point = _start + _unit_direction * proj;
    return std::abs(_unit_direction.cross_prod(Vec3d(x0, y0, z0)).length());
}

Segment2d Segment3d::segment2d() const
{
    Vec2d start(_start.x(), _start.y());
    Vec2d end(_end.x(), _end.y());
    return Segment2d(start, end);
}

std::string Segment3d::debug_string() const
{
    std::ostringstream sout;
    sout << "Segment3d ( start = " << _start.debug_string()
         << "  end = " << _end.debug_string() << " )";
    sout.flush();
    return sout.str();
}
