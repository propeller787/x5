#pragma once

#include "math/box2d.h"
#include "math/segment2d.h"
#include "math/vec2d.h"

#include <string>
#include <vector>

namespace decision::math{

class Polygon2d {
public:
    Polygon2d() = default;
    explicit Polygon2d(const Box2d& box);
    explicit Polygon2d(std::vector<Vec2d> points);

    const std::vector<Vec2d>& points() const { return _points; }
    const std::vector<Segment2d>& segments() const { return _segments; }
    int num_points() const { return _num_points; }
    bool is_convex() const { return _is_convex; }
    double area() const { return _area; }

    double distance_to_boundary(const Vec2d& point) const;
    double distance_to(const Vec2d& point) const;
    double distance_to(const Segment2d& segment) const;
    double distance_to(const Box2d& box) const;
    double distance_to(const Polygon2d& polygon) const;
    double distance_sqr_to(const Vec2d& point) const;

    bool is_point_in(const Vec2d& point) const;
    bool is_point_on_boundary(const Vec2d& point) const;

    bool is_contain(const Segment2d& segment) const;
    bool is_contain(const Polygon2d& polygon) const;

    static bool compute_convex_hull(const std::vector<Vec2d>& points, Polygon2d* const polygon);

    bool has_overlap(const Segment2d& segment) const;
    bool get_overlap(const Segment2d& segment, Vec2d* const first, Vec2d* const last) const;
    std::vector<Segment2d> get_all_overlaps(const Segment2d& segment) const;

    bool has_overlap(const Polygon2d& polygon) const;
    // Only compute overlaps between two convex polygons.
    bool compute_overlap(const Polygon2d& other_polygon, Polygon2d* const overlap_polygon) const;

    AABox2d aa_bounding_box() const;
    Box2d bounding_box_with_heading(const double heading) const;
    Box2d min_area_bounding_box() const;
    void extreme_points(const double heading, Vec2d* const first, Vec2d* const last) const;

    Polygon2d expand_by_distance(const double distance) const;

    std::string debug_string() const;

protected:
    void build_from_points();

protected:
    int next(int at) const;
    int prev(int at) const;

    // Remove point p if and only if outer_prod(segment.start, segment.end, p) < 0.
    static bool clip_convex_hull(const Segment2d& segment, std::vector<Vec2d>* const points);

    std::vector<Vec2d> _points;
    int _num_points = 0;
    std::vector<Segment2d> _segments;
    bool _is_convex = false;
    double _area = 0.0;
    double _min_x = 0.0;
    double _max_x = 0.0;
    double _min_y = 0.0;
    double _max_y = 0.0;
};

}
