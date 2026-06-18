/**
 * @file vehicle_footprint.cpp
 * @brief 梯形 / 双梯形 footprint 几何计算
 */
#include "safety_node/core/vehicle_footprint.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{

constexpr double kEps = 1e-12;

struct OffsetLine
{
  double px = 0.0;
  double py = 0.0;
  double dx = 0.0;
  double dy = 0.0;
};

OffsetLine makeOffsetLine(
  const std::pair<double, double> & a,
  const std::pair<double, double> & b,
  double margin_m,
  double outward_sign)
{
  OffsetLine line;
  line.dx = b.first - a.first;
  line.dy = b.second - a.second;
  const double len = std::hypot(line.dx, line.dy);
  if (len <= kEps) {
    return line;
  }
  const double nx = outward_sign * line.dy / len;
  const double ny = outward_sign * (-line.dx / len);
  line.px = a.first + nx * margin_m;
  line.py = a.second + ny * margin_m;
  return line;
}

double polygonSignedArea2(const FootprintPolygon & polygon)
{
  double twice_area = 0.0;
  const size_t n = polygon.vertices.size();
  for (size_t i = 0; i < n; ++i) {
    const auto & a = polygon.vertices[i];
    const auto & b = polygon.vertices[(i + 1U) % n];
    twice_area += a.first * b.second - b.first * a.second;
  }
  return twice_area;
}

bool intersectOffsetLines(
  const OffsetLine & a,
  const OffsetLine & b,
  double & x,
  double & y)
{
  const double denom = a.dx * b.dy - a.dy * b.dx;
  if (std::abs(denom) <= kEps) {
    return false;
  }
  const double rx = b.px - a.px;
  const double ry = b.py - a.py;
  const double t = (rx * b.dy - ry * b.dx) / denom;
  x = a.px + t * a.dx;
  y = a.py + t * a.dy;
  return std::isfinite(x) && std::isfinite(y);
}

}  // namespace

VehicleFootprint::VehicleFootprint(const AvoidanceParams & params)
: params_(params)
{
}

void VehicleFootprint::updateParams(const AvoidanceParams & params)
{
  params_ = params;
}

FootprintPolygon VehicleFootprint::buildPolygon(
  double x_rear,
  double x_front,
  double width_rear,
  double width_front)
{
  const double rear_half = std::max(0.0, width_rear) * 0.5;
  const double front_half = std::max(0.0, width_front) * 0.5;
  FootprintPolygon polygon;
  polygon.vertices = {{
    {x_rear, rear_half},
    {x_front, front_half},
    {x_front, -front_half},
    {x_rear, -rear_half},
  }};
  return polygon;
}

FootprintPolygon VehicleFootprint::buildDualTrapezoidPolygon(
  double x_rear,
  const FootprintSegmentParams & rear_segment,
  const FootprintSegmentParams & front_segment)
{
  const double x_joint = x_rear + std::max(0.0, rear_segment.length_m);
  const double x_front = x_joint + std::max(0.0, front_segment.length_m);
  const double rear_half = std::max(0.0, rear_segment.width_rear_m) * 0.5;
  const double joint_half = std::max(0.0, rear_segment.width_front_m) * 0.5;
  const double front_half = std::max(0.0, front_segment.width_front_m) * 0.5;

  FootprintPolygon polygon;
  polygon.vertices = {{
    {x_rear, rear_half},
    {x_joint, joint_half},
    {x_front, front_half},
    {x_front, -front_half},
    {x_joint, -joint_half},
    {x_rear, -rear_half},
  }};
  return polygon;
}

FootprintPolygon VehicleFootprint::offsetPolygonByEdgeMargins(
  const FootprintPolygon & polygon,
  const std::vector<double> & edge_margins_m)
{
  FootprintPolygon inflated;
  const size_t n = polygon.vertices.size();
  if (n < 3U || edge_margins_m.size() != n) {
    return polygon;
  }

  const double outward_sign = polygonSignedArea2(polygon) >= 0.0 ? 1.0 : -1.0;
  std::vector<OffsetLine> lines;
  lines.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    const auto & a = polygon.vertices[i];
    const auto & b = polygon.vertices[(i + 1U) % n];
    const double margin = std::max(0.0, edge_margins_m[i]);
    lines.push_back(makeOffsetLine(a, b, margin, outward_sign));
  }

  inflated.vertices.resize(n);
  for (size_t i = 0; i < n; ++i) {
    const OffsetLine & prev = lines[(i + n - 1U) % n];
    const OffsetLine & curr = lines[i];
    if (!intersectOffsetLines(prev, curr, inflated.vertices[i].first, inflated.vertices[i].second)) {
      inflated.vertices[i] = polygon.vertices[i];
    }
  }
  return inflated;
}

FootprintPolygon VehicleFootprint::inflateAxisAlignedTrapezoid(
  double x_rear,
  double x_front,
  double width_rear,
  double width_front,
  double front_margin,
  double back_margin,
  double side_margin)
{
  const double inflated_x_rear =
    x_rear - std::max(0.0, back_margin);
  const double inflated_x_front =
    x_front + std::max(0.0, front_margin);
  const double side = std::max(0.0, side_margin);
  return buildPolygon(
    inflated_x_rear,
    inflated_x_front,
    width_rear + 2.0 * side,
    width_front + 2.0 * side);
}

FootprintRect VehicleFootprint::boundingRect(const FootprintPolygon & polygon)
{
  FootprintRect rect;
  if (polygon.vertices.empty()) {
    return rect;
  }
  rect.x_min = rect.x_max = polygon.vertices.front().first;
  rect.y_min = rect.y_max = polygon.vertices.front().second;
  for (const auto & vertex : polygon.vertices) {
    rect.x_min = std::min(rect.x_min, vertex.first);
    rect.x_max = std::max(rect.x_max, vertex.first);
    rect.y_min = std::min(rect.y_min, vertex.second);
    rect.y_max = std::max(rect.y_max, vertex.second);
  }
  return rect;
}

FootprintPolygon VehicleFootprint::physicalPolygon() const
{
  const double x_rear = -params_.robot_rear_offset;
  if (params_.footprint_use_dual_trapezoid) {
    return buildDualTrapezoidPolygon(
      x_rear,
      params_.footprint_rear_segment,
      params_.footprint_front_segment);
  }

  const double x_front = params_.robot_length - params_.robot_rear_offset;
  return buildPolygon(
    x_rear, x_front,
    params_.robot_width_rear_m,
    params_.robot_width_front_m);
}

FootprintPolygon VehicleFootprint::inflatedPolygon() const
{
  const double front_margin = std::max(0.0, params_.footprint_front_margin);
  const double back_margin = std::max(0.0, params_.footprint_back_margin);
  const double side_margin = std::max(0.0, params_.footprint_side_margin);

  if (params_.footprint_use_dual_trapezoid) {
    const FootprintPolygon physical = physicalPolygon();
    if (physical.vertices.size() != 6U) {
      return physical;
    }
    // CCW：后左→分节左→前左→前右→分节右→后右
    const std::vector<double> edge_margins = {
      side_margin,
      side_margin,
      front_margin,
      side_margin,
      side_margin,
      back_margin,
    };
    return offsetPolygonByEdgeMargins(physical, edge_margins);
  }

  const double x_rear = -params_.robot_rear_offset;
  const double x_front = params_.robot_length - params_.robot_rear_offset;
  return inflateAxisAlignedTrapezoid(
    x_rear,
    x_front,
    params_.robot_width_rear_m,
    params_.robot_width_front_m,
    front_margin,
    back_margin,
    side_margin);
}

FootprintRect VehicleFootprint::physicalRect() const
{
  return boundingRect(physicalPolygon());
}

FootprintRect VehicleFootprint::inflatedRect() const
{
  return boundingRect(inflatedPolygon());
}

namespace
{

double minOrientedSide(
  double x, double y,
  const FootprintPolygon & polygon)
{
  if (polygon.vertices.size() < 3U) {
    return std::numeric_limits<double>::infinity();
  }

  double twice_area = 0.0;
  for (size_t i = 0; i < polygon.vertices.size(); ++i) {
    const auto & a = polygon.vertices[i];
    const auto & b = polygon.vertices[(i + 1U) % polygon.vertices.size()];
    twice_area += a.first * b.second - b.first * a.second;
  }
  const bool ccw = twice_area >= 0.0;
  double min_side = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < polygon.vertices.size(); ++i) {
    const auto & a = polygon.vertices[i];
    const auto & b = polygon.vertices[(i + 1U) % polygon.vertices.size()];
    const double edge_x = b.first - a.first;
    const double edge_y = b.second - a.second;
    const double edge_len = std::hypot(edge_x, edge_y);
    if (edge_len <= kEps) {
      continue;
    }
    const double rel_x = x - a.first;
    const double rel_y = y - a.second;
    const double signed_side = (edge_x * rel_y - edge_y * rel_x) / edge_len;
    const double oriented_side = ccw ? signed_side : -signed_side;
    min_side = std::min(min_side, oriented_side);
  }
  return min_side;
}

}  // namespace

bool VehicleFootprint::circleIntersectsPolygon(
  double x, double y, double obstacle_radius,
  const FootprintPolygon & polygon)
{
  const double radius = std::max(0.0, obstacle_radius);
  const double min_side = minOrientedSide(x, y, polygon);
  if (!std::isfinite(min_side)) {
    return false;
  }
  if (min_side >= -1e-9) {
    return true;
  }
  return (-min_side) <= radius + 1e-9;
}

bool VehicleFootprint::circleContainedInPolygon(
  double x, double y, double obstacle_radius,
  const FootprintPolygon & polygon)
{
  const double min_side = minOrientedSide(x, y, polygon);
  return std::isfinite(min_side) &&
         min_side + 1e-9 >= std::max(0.0, obstacle_radius);
}

bool VehicleFootprint::pointInPolygon(
  double x, double y, double obstacle_radius,
  const FootprintPolygon & polygon)
{
  return circleIntersectsPolygon(x, y, obstacle_radius, polygon);
}

double VehicleFootprint::clearanceToPolygon(
  double x, double y, double obstacle_radius,
  const FootprintPolygon & polygon)
{
  const double radius = std::max(0.0, obstacle_radius);
  const double min_side = minOrientedSide(x, y, polygon);
  if (!std::isfinite(min_side)) {
    return std::numeric_limits<double>::infinity();
  }
  if (min_side >= 0.0) {
    return min_side - radius;
  }
  return (-min_side) - radius;
}

FootprintPolygon VehicleFootprint::transformPolygon(
  const FootprintPolygon & polygon,
  const PredictedPose & pose)
{
  FootprintPolygon world;
  world.vertices.resize(polygon.vertices.size());
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  for (size_t i = 0; i < polygon.vertices.size(); ++i) {
    const double lx = polygon.vertices[i].first;
    const double ly = polygon.vertices[i].second;
    world.vertices[i].first = pose.x + c * lx - s * ly;
    world.vertices[i].second = pose.y + s * lx + c * ly;
  }
  return world;
}

std::vector<std::pair<double, double>> VehicleFootprint::samplePolygonInterior(
  const FootprintPolygon & polygon,
  double step_m)
{
  std::vector<std::pair<double, double>> samples;
  const FootprintRect bounds = boundingRect(polygon);
  const double step = std::max(0.01, step_m);
  samples.reserve(32);

  for (const auto & vertex : polygon.vertices) {
    samples.emplace_back(vertex.first, vertex.second);
  }

  for (double x = bounds.x_min; x <= bounds.x_max + kEps; x += step) {
    for (double y = bounds.y_min; y <= bounds.y_max + kEps; y += step) {
      if (circleContainedInPolygon(x, y, 0.0, polygon)) {
        samples.emplace_back(x, y);
      }
    }
  }
  return samples;
}

bool VehicleFootprint::pointInRect(
  double x, double y, double obstacle_radius,
  const FootprintRect & rect)
{
  return x + obstacle_radius >= rect.x_min && x - obstacle_radius <= rect.x_max &&
         y + obstacle_radius >= rect.y_min && y - obstacle_radius <= rect.y_max;
}

double VehicleFootprint::clearanceToRect(
  double x, double y, double obstacle_radius,
  const FootprintRect & rect)
{
  const double dx = std::max(
    {rect.x_min - (x + obstacle_radius), 0.0,
      (x - obstacle_radius) - rect.x_max});
  const double dy = std::max(
    {rect.y_min - (y + obstacle_radius), 0.0,
      (y - obstacle_radius) - rect.y_max});
  if (dx <= 0.0 && dy <= 0.0) {
    const double inside_x =
      std::min(x - obstacle_radius - rect.x_min, rect.x_max - (x + obstacle_radius));
    const double inside_y =
      std::min(y - obstacle_radius - rect.y_min, rect.y_max - (y + obstacle_radius));
    return -std::min(inside_x, inside_y);
  }
  return std::hypot(dx, dy);
}
