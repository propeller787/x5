#include "math/polygon2d.h"
#include "math/math_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

using namespace decision::math;

Polygon2d::Polygon2d(const Box2d& box)
{
    box.get_all_corners(&_points);
    build_from_points();
}

Polygon2d::Polygon2d(std::vector<Vec2d> points)
    : _points(std::move(points))
{
    build_from_points();
}

double Polygon2d::distance_to(const Vec2d& point) const
{
    CHECK_GE(_points.size(), 3u);
    if (is_point_in(point)) {
        return 0.0;
    }
    double distance = std::numeric_limits<double>::infinity();
    for (int i = 0; i < _num_points; ++i) {
        distance = std::min(distance, _segments[i].distance_to(point));
    }
    return distance;
}

double Polygon2d::distance_sqr_to(const Vec2d& point) const
{
    CHECK_GE(_points.size(), 3u);
    if (is_point_in(point)) {
        return 0.0;
    }
    double distance_sqr = std::numeric_limits<double>::infinity();
    for (int i = 0; i < _num_points; ++i) {
        distance_sqr = std::min(distance_sqr, _segments[i].distance_sqr_to(point));
    }
    return distance_sqr;
}

double Polygon2d::distance_to(const Segment2d& segment) const
{
    if (segment.length() <= kMathEpsilon) {
        return distance_to(segment.start());
    }
    CHECK_GE(_points.size(), 3u);
    if (is_point_in(segment.center())) {
        return 0.0;
    }
    if (std::any_of(_segments.begin(), _segments.end(),
            [&](const Segment2d& poly_seg) {
                return poly_seg.has_intersect(segment);
            })) {
        return 0.0;
    }

    double distance = std::min(distance_to(segment.start()), distance_to(segment.end()));
    for (int i = 0; i < _num_points; ++i) {
        distance = std::min(distance, segment.distance_to(_points[i]));
    }
    return distance;
}

double Polygon2d::distance_to(const Box2d& box) const
{
    CHECK_GE(_points.size(), 3u);
    return distance_to(Polygon2d(box));
}

double Polygon2d::distance_to(const Polygon2d& polygon) const
{
    CHECK_GE(_points.size(), 3u);
    CHECK_GE(polygon.num_points(), 3);

    if (is_point_in(polygon.points()[0])) {
        return 0.0;
    }
    if (polygon.is_point_in(_points[0])) {
        return 0.0;
    }
    double distance = std::numeric_limits<double>::infinity();
    for (int i = 0; i < _num_points; ++i) {
        distance = std::min(distance, polygon.distance_to(_segments[i]));
    }
    return distance;
}

double Polygon2d::distance_to_boundary(const Vec2d& point) const
{
    double distance = std::numeric_limits<double>::infinity();
    for (int i = 0; i < _num_points; ++i) {
        distance = std::min(distance, _segments[i].distance_to(point));
    }
    return distance;
}

bool Polygon2d::is_point_on_boundary(const Vec2d& point) const
{
    CHECK_GE(_points.size(), 3u);
    return std::any_of(_segments.begin(), _segments.end(),
        [&](const Segment2d& poly_seg) {
            return poly_seg.is_point_in(point);
        });
}

bool Polygon2d::is_point_in(const Vec2d& point) const
{
    CHECK_GE(_points.size(), 3u);
    if (is_point_on_boundary(point)) {
        return true;
    }
    int j = _num_points - 1;
    int c = 0;
    for (int i = 0; i < _num_points; ++i) {
        if ((_points[i].y() > point.y()) != (_points[j].y() > point.y())) {
            const double side = cross_prod(point, _points[i], _points[j]);
            if (_points[i].y() < _points[j].y() ? side > 0.0 : side < 0.0) {
                ++c;
            }
        }
        j = i;
    }
    return c & 1;
}

bool Polygon2d::has_overlap(const Polygon2d& polygon) const
{
    CHECK_GE(_points.size(), 3u);
    return distance_to(polygon) <= kMathEpsilon;
}

bool Polygon2d::is_contain(const Segment2d& segment) const
{
    if (segment.length() <= kMathEpsilon) {
        return is_point_in(segment.start());
    }
    CHECK_GE(_points.size(), 3u);
    if (!is_point_in(segment.start())) {
        return false;
    }
    if (!is_point_in(segment.end())) {
        return false;
    }
    if (!_is_convex) {
        std::vector<Segment2d> overlaps = get_all_overlaps(segment);
        double total_length = 0;
        for (const auto& overlap_seg : overlaps) {
            total_length += overlap_seg.length();
        }
        return total_length >= segment.length() - kMathEpsilon;
    }
    return true;
}

bool Polygon2d::is_contain(const Polygon2d& polygon) const
{
    CHECK_GE(_points.size(), 3u);
    if (_area < polygon.area() - kMathEpsilon) {
        return false;
    }
    if (!is_point_in(polygon.points()[0])) {
        return false;
    }
    const auto& segments = polygon.segments();
    return std::all_of(segments.begin(), segments.end(),
        [&](const Segment2d& segment) {
            return is_contain(segment);
        });
}

int Polygon2d::next(int at) const
{
    return at >= _num_points - 1 ? 0 : at + 1;
}

int Polygon2d::prev(int at) const
{
    return at == 0 ? _num_points - 1 : at - 1;
}

void Polygon2d::build_from_points()
{
    _num_points = _points.size();
    CHECK_GE(_num_points, 3);

    // Make sure the points are in ccw order.
    _area = 0.0;
    for (int i = 1; i < _num_points; ++i) {
        _area += cross_prod(_points[0], _points[i - 1], _points[i]);
    }
    if (_area < 0) {
        _area = -_area;
        std::reverse(_points.begin(), _points.end());
    }
    _area /= 2.0;
    CHECK_GT(_area, kMathEpsilon);

    // Construct segments.
    _segments.reserve(_num_points);
    for (int i = 0; i < _num_points; ++i) {
        _segments.emplace_back(_points[i], _points[next(i)]);
    }

    // Check convexity.
    _is_convex = true;
    for (int i = 0; i < _num_points; ++i) {
        if (cross_prod(_points[prev(i)], _points[i], _points[next(i)]) <= -kMathEpsilon) {
            _is_convex = false;
            break;
        }
    }

    // Compute aabox.
    _min_x = _points[0].x();
    _max_x = _points[0].x();
    _min_y = _points[0].y();
    _max_y = _points[0].y();
    for (const auto& point : _points) {
        _min_x = std::min(_min_x, point.x());
        _max_x = std::max(_max_x, point.x());
        _min_y = std::min(_min_y, point.y());
        _max_y = std::max(_max_y, point.y());
    }
}

bool Polygon2d::compute_convex_hull(const std::vector<Vec2d>& points,
    Polygon2d* const polygon)
{
    CHECK_NOTNULL(polygon);
    const int n = points.size();
    if (n < 3) {
        return false;
    }
    std::vector<int> sorted_indices(n);
    for (int i = 0; i < n; ++i) {
        sorted_indices[i] = i;
    }
    std::sort(sorted_indices.begin(), sorted_indices.end(),
        [&](const int idx1, const int idx2) {
            const Vec2d& pt1 = points[idx1];
            const Vec2d& pt2 = points[idx2];
            const double dx = pt1.x() - pt2.x();
            if (std::abs(dx) > kMathEpsilon) {
                return dx < 0.0;
            }
            return pt1.y() < pt2.y();
        });
    int count = 0;
    std::vector<int> results;
    results.reserve(n);
    int last_count = 1;
    for (int i = 0; i < n + n; ++i) {
        if (i == n) {
            last_count = count;
        }
        const int idx = sorted_indices[(i < n) ? i : (n + n - 1 - i)];
        const Vec2d& pt = points[idx];
        while (count > last_count && cross_prod(points[results[count - 2]], points[results[count - 1]], pt) <= kMathEpsilon) {
            results.pop_back();
            --count;
        }
        results.push_back(idx);
        ++count;
    }
    --count;
    if (count < 3) {
        return false;
    }
    std::vector<Vec2d> result_points;
    result_points.reserve(count);
    for (int i = 0; i < count; ++i) {
        result_points.push_back(points[results[i]]);
    }

    // check if area > kMathEpsilon
    double area = 0.0;
    for (int i = 1; i < count; ++i) {
        area += cross_prod(result_points[0], result_points[i - 1], result_points[i]);
    }
    if (area < 0) {
        area = -area;
    }
    area /= 2.0;
    if (area > kMathEpsilon) {
        *polygon = Polygon2d(result_points);
        return true;
    } else {
        return false;
    }
}

bool Polygon2d::clip_convex_hull(const Segment2d& segment,
    std::vector<Vec2d>* const points)
{
    if (segment.length() <= kMathEpsilon) {
        return true;
    }
    CHECK_NOTNULL(points);
    const int n = points->size();
    if (n < 3) {
        return false;
    }
    std::vector<double> prod(n);
    std::vector<int> side(n);
    for (int i = 0; i < n; ++i) {
        prod[i] = cross_prod(segment.start(), segment.end(), (*points)[i]);
        if (std::abs(prod[i]) <= kMathEpsilon) {
            side[i] = 0;
        } else {
            side[i] = ((prod[i] < 0) ? -1 : 1);
        }
    }

    std::vector<Vec2d> new_points;
    for (int i = 0; i < n; ++i) {
        if (side[i] >= 0) {
            new_points.push_back((*points)[i]);
        }
        const int j = ((i == n - 1) ? 0 : (i + 1));
        if (side[i] * side[j] < 0) {
            const double ratio = prod[j] / (prod[j] - prod[i]);
            new_points.emplace_back((*points)[i].x() * ratio + (*points)[j].x() * (1.0 - ratio),
                (*points)[i].y() * ratio + (*points)[j].y() * (1.0 - ratio));
        }
    }

    points->swap(new_points);
    return points->size() >= 3;
}

bool Polygon2d::compute_overlap(const Polygon2d& other_polygon, Polygon2d* const overlap_polygon) const
{
    CHECK_GE(_points.size(), 3u);
    CHECK_NOTNULL(overlap_polygon);
    CHECK(_is_convex && other_polygon.is_convex());
    std::vector<Vec2d> points = other_polygon.points();
    for (int i = 0; i < _num_points; ++i) {
        if (!clip_convex_hull(_segments[i], &points)) {
            return false;
        }
    }
    return compute_convex_hull(points, overlap_polygon);
}

bool Polygon2d::has_overlap(const Segment2d& segment) const
{
    CHECK_GE(_points.size(), 3u);
    Vec2d first;
    Vec2d last;
    return get_overlap(segment, &first, &last);
}

bool Polygon2d::get_overlap(const Segment2d& segment, Vec2d* const first, Vec2d* const last) const
{
    CHECK_GE(_points.size(), 3u);
    CHECK_NOTNULL(first);
    CHECK_NOTNULL(last);

    if (segment.length() <= kMathEpsilon) {
        if (!is_point_in(segment.start())) {
            return false;
        }
        *first = segment.start();
        *last = segment.start();
        return true;
    }

    double min_proj = segment.length();
    double max_proj = 0;
    if (is_point_in(segment.start())) {
        *first = segment.start();
        min_proj = 0.0;
    }
    if (is_point_in(segment.end())) {
        *last = segment.end();
        max_proj = segment.length();
    }
    for (const auto& poly_seg : _segments) {
        Vec2d pt;
        if (poly_seg.get_intersect(segment, &pt)) {
            const double proj = segment.project_onto_unit(pt);
            if (proj < min_proj) {
                min_proj = proj;
                *first = pt;
            }
            if (proj > max_proj) {
                max_proj = proj;
                *last = pt;
            }
        }
    }
    return min_proj <= max_proj + kMathEpsilon;
}

std::vector<Segment2d> Polygon2d::get_all_overlaps(const Segment2d& segment) const
{
    CHECK_GE(_points.size(), 3u);

    if (segment.length() <= kMathEpsilon) {
        std::vector<Segment2d> overlaps;
        if (is_point_in(segment.start())) {
            overlaps.push_back(segment);
        }
        return overlaps;
    }
    std::vector<double> projections;
    if (is_point_in(segment.start())) {
        projections.push_back(0.0);
    }
    if (is_point_in(segment.end())) {
        projections.push_back(segment.length());
    }
    for (const auto& poly_seg : _segments) {
        Vec2d pt;
        if (poly_seg.get_intersect(segment, &pt)) {
            projections.push_back(segment.project_onto_unit(pt));
        }
    }
    std::sort(projections.begin(), projections.end());
    std::vector<std::pair<double, double>> overlaps;
    for (size_t i = 0; i + 1 < projections.size(); ++i) {
        const double start_proj = projections[i];
        const double end_proj = projections[i + 1];
        if (end_proj - start_proj <= kMathEpsilon) {
            continue;
        }
        const Vec2d reference_point = segment.start() + (start_proj + end_proj) / 2.0 * segment.unit_direction();
        if (!is_point_in(reference_point)) {
            continue;
        }
        if (overlaps.empty() || start_proj > overlaps.back().second + kMathEpsilon) {
            overlaps.emplace_back(start_proj, end_proj);
        } else {
            overlaps.back().second = end_proj;
        }
    }
    std::vector<Segment2d> overlap_segments;
    for (const auto& overlap : overlaps) {
        overlap_segments.emplace_back(
            segment.start() + overlap.first * segment.unit_direction(),
            segment.start() + overlap.second * segment.unit_direction());
    }
    return overlap_segments;
}

void Polygon2d::extreme_points(const double heading, Vec2d* const first, Vec2d* const last) const
{
    CHECK_GE(_points.size(), 3u);
    CHECK_NOTNULL(first);
    CHECK_NOTNULL(last);

    const Vec2d direction_vec = Vec2d::create_unit_vec(heading);
    double min_proj = std::numeric_limits<double>::infinity();
    double max_proj = -std::numeric_limits<double>::infinity();
    for (const auto& pt : _points) {
        const double proj = pt.inner_prod(direction_vec);
        if (proj < min_proj) {
            min_proj = proj;
            *first = pt;
        }
        if (proj > max_proj) {
            max_proj = proj;
            *last = pt;
        }
    }
}

AABox2d Polygon2d::aa_bounding_box() const
{
    return AABox2d({ _min_x, _min_y }, { _max_x, _max_y });
}

Box2d Polygon2d::bounding_box_with_heading(const double heading) const
{
    CHECK_GE(_points.size(), 3u);
    const Vec2d direction_vec = Vec2d::create_unit_vec(heading);
    Vec2d px1;
    Vec2d px2;
    Vec2d py1;
    Vec2d py2;
    extreme_points(heading, &px1, &px2);
    extreme_points(heading - M_PI_2, &py1, &py2);
    const double x1 = px1.inner_prod(direction_vec);
    const double x2 = px2.inner_prod(direction_vec);
    const double y1 = py1.cross_prod(direction_vec);
    const double y2 = py2.cross_prod(direction_vec);
    return Box2d((x1 + x2) / 2.0 * direction_vec + (y1 + y2) / 2.0 * Vec2d(direction_vec.y(), -direction_vec.x()),
        heading, x2 - x1, y2 - y1);
}

Box2d Polygon2d::min_area_bounding_box() const
{
    CHECK_GE(_points.size(), 3u);
    if (!_is_convex) {
        Polygon2d convex_polygon;
        compute_convex_hull(_points, &convex_polygon);
        CHECK(convex_polygon.is_convex());
        return convex_polygon.min_area_bounding_box();
    }
    double min_area = std::numeric_limits<double>::infinity();
    double min_area_at_heading = 0.0;
    int left_most = 0;
    int right_most = 0;
    int top_most = 0;
    for (int i = 0; i < _num_points; ++i) {
        const auto& segment = _segments[i];
        double proj = 0.0;
        double min_proj = segment.project_onto_unit(_points[left_most]);
        while ((proj = segment.project_onto_unit(_points[prev(left_most)])) < min_proj) {
            min_proj = proj;
            left_most = prev(left_most);
        }
        while ((proj = segment.project_onto_unit(_points[next(left_most)])) < min_proj) {
            min_proj = proj;
            left_most = next(left_most);
        }
        double max_proj = segment.project_onto_unit(_points[right_most]);
        while ((proj = segment.project_onto_unit(_points[prev(right_most)])) > max_proj) {
            max_proj = proj;
            right_most = prev(right_most);
        }
        while ((proj = segment.project_onto_unit(_points[next(right_most)])) > max_proj) {
            max_proj = proj;
            right_most = next(right_most);
        }
        double prod = 0.0;
        double max_prod = segment.product_onto_unit(_points[top_most]);
        while ((prod = segment.product_onto_unit(_points[prev(top_most)])) > max_prod) {
            max_prod = prod;
            top_most = prev(top_most);
        }
        while ((prod = segment.product_onto_unit(_points[next(top_most)])) > max_prod) {
            max_prod = prod;
            top_most = next(top_most);
        }
        const double area = max_prod * (max_proj - min_proj);
        if (area < min_area) {
            min_area = area;
            min_area_at_heading = segment.heading();
        }
    }
    return bounding_box_with_heading(min_area_at_heading);
}

Polygon2d Polygon2d::expand_by_distance(const double distance) const
{
    if (!_is_convex) {
        Polygon2d convex_polygon;
        compute_convex_hull(_points, &convex_polygon);
        CHECK(convex_polygon.is_convex());
        return convex_polygon.expand_by_distance(distance);
    }
    const double kMinAngle = 0.1;
    std::vector<Vec2d> points;
    for (int i = 0; i < _num_points; ++i) {
        const double start_angle = _segments[prev(i)].heading() - M_PI_2;
        const double end_angle = _segments[i].heading() - M_PI_2;
        const double diff = wrap_angle(end_angle - start_angle);
        if (diff <= kMathEpsilon) {
            points.push_back(_points[i] + Vec2d::create_unit_vec(start_angle) * distance);
        } else {
            const int count = static_cast<int>(diff / kMinAngle) + 1;
            for (int k = 0; k <= count; ++k) {
                const double angle = start_angle + diff * static_cast<double>(k) / static_cast<double>(count);
                points.push_back(_points[i] + Vec2d::create_unit_vec(angle) * distance);
            }
        }
    }
    Polygon2d new_polygon;
    CHECK(compute_convex_hull(points, &new_polygon));
    return new_polygon;
}

std::string Polygon2d::debug_string() const
{
    std::ostringstream sout;
    sout << "polygon2d (  num_points = " << _num_points << "  points = (";
    for (const auto& pt : _points) {
        sout << " " << pt.debug_string();
    }
    sout << " )  " << (_is_convex ? "convex" : "non-convex") << "  area = " << _area << " )";
    sout.flush();
    return sout.str();
}
