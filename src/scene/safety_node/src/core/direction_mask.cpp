/**
 * @file direction_mask.cpp
 * @brief L1 方向解耦掩码实现
 */
#include "safety_node/core/direction_mask.h"

#include <algorithm>
#include <cmath>

namespace
{

/** 障碍（base_link 坐标）变换到轨迹 pose 坐标系，与 SweptFootprintChecker 一致 */
void transformObstacleToPoseFrame(
  double ox, double oy,
  const PredictedPose & pose,
  double & lx, double & ly)
{
  const double dx = ox - pose.x;
  const double dy = oy - pose.y;
  const double c = std::cos(-pose.theta);
  const double s = std::sin(-pose.theta);
  lx = c * dx - s * dy;
  ly = s * dx + c * dy;
}

double signedClearanceAtPose(
  const ObstaclePrimitive & obs,
  const PredictedPose & pose,
  const FootprintPolygon & polygon)
{
  double lx = 0.0;
  double ly = 0.0;
  transformObstacleToPoseFrame(obs.x, obs.y, pose, lx, ly);
  return VehicleFootprint::signedClearanceToPolygon(lx, ly, obs.radius, polygon);
}

}  // namespace

namespace direction_mask
{

bool isNearObstacle(
  const ObstaclePrimitive & obs,
  const FootprintPolygon & inflated_polygon,
  double hard_stop_distance_m)
{
  if (!obs.valid) {
    return false;
  }
  const double clearance = VehicleFootprint::signedClearanceToPolygon(
    obs.x, obs.y, obs.radius, inflated_polygon);
  return std::isfinite(clearance) && clearance <= hard_stop_distance_m;
}

bool obstacleThreatenedAlongPoses(
  const ObstaclePrimitive & obs,
  const std::vector<PredictedPose> & poses,
  const FootprintPolygon & physical_polygon,
  double contact_buffer_m,
  double approach_eps_m)
{
  if (!obs.valid || poses.empty()) {
    return false;
  }
  const double entry_clearance =
    signedClearanceAtPose(obs, poses.front(), physical_polygon);
  if (!std::isfinite(entry_clearance)) {
    return false;
  }
  // 威胁线：尚未进入缓冲时为缓冲值；已在缓冲内/体内时为当前穿深——
  // 只有继续变差（跌破威胁线且降幅超死区）才构成威胁，远离/持平放行。
  const double threat_line =
    std::min(contact_buffer_m, entry_clearance) - std::max(0.0, approach_eps_m);
  for (size_t i = 1; i < poses.size(); ++i) {
    const double clearance = signedClearanceAtPose(obs, poses[i], physical_polygon);
    if (std::isfinite(clearance) && clearance < threat_line) {
      return true;
    }
  }
  return false;
}

std::vector<PredictedPose> linearProbePoses(
  double direction_sign,
  double distance_m,
  double step_m)
{
  std::vector<PredictedPose> poses;
  const double distance = std::max(0.0, distance_m);
  const double step = std::clamp(step_m, 0.005, std::max(0.005, distance));
  const double sign = direction_sign >= 0.0 ? 1.0 : -1.0;
  poses.push_back(PredictedPose{});
  double s = 0.0;
  while (s + 1e-9 < distance) {
    s = std::min(distance, s + step);
    poses.push_back({sign * s, 0.0, 0.0, 0.0});
  }
  return poses;
}

std::vector<PredictedPose> angularProbePoses(
  double direction_sign,
  double angle_rad,
  double step_rad)
{
  std::vector<PredictedPose> poses;
  const double angle = std::max(0.0, angle_rad);
  const double step = std::clamp(step_rad, 0.01, std::max(0.01, angle));
  const double sign = direction_sign >= 0.0 ? 1.0 : -1.0;
  poses.push_back(PredictedPose{});
  double a = 0.0;
  while (a + 1e-9 < angle) {
    a = std::min(angle, a + step);
    poses.push_back({0.0, 0.0, sign * a, 0.0});
  }
  return poses;
}

DirectionBlockMask computeDirectionBlockMask(
  const ObstacleField & field,
  const FootprintPolygon & physical_polygon,
  const FootprintPolygon & inflated_polygon,
  const AvoidanceParams & params)
{
  DirectionBlockMask mask;
  if (!params.direction_mask_enabled || field.obstacles.empty()) {
    return mask;
  }

  const auto forward_probe = linearProbePoses(
    1.0, params.direction_mask_probe_distance_m, params.direction_mask_probe_step_m);
  const auto backward_probe = linearProbePoses(
    -1.0, params.direction_mask_probe_distance_m, params.direction_mask_probe_step_m);
  const auto left_probe = angularProbePoses(
    1.0, params.direction_mask_probe_angle_rad, params.direction_mask_probe_step_rad);
  const auto right_probe = angularProbePoses(
    -1.0, params.direction_mask_probe_angle_rad, params.direction_mask_probe_step_rad);

  for (const auto & obs : field.obstacles) {
    if (!isNearObstacle(obs, inflated_polygon, params.hard_stop_distance_m)) {
      continue;
    }
    const double inflated_clearance = VehicleFootprint::signedClearanceToPolygon(
      obs.x, obs.y, obs.radius, inflated_polygon);
    const auto threat = [&](const std::vector<PredictedPose> & probe) {
        return obstacleThreatenedAlongPoses(
          obs, probe, physical_polygon,
          params.direction_mask_contact_buffer_m,
          params.direction_mask_approach_eps_m);
      };
    if (threat(forward_probe)) {
      mask.forward = true;
      mask.forward_clearance_m = std::min(mask.forward_clearance_m, inflated_clearance);
    }
    if (threat(backward_probe)) {
      mask.backward = true;
      mask.backward_clearance_m = std::min(mask.backward_clearance_m, inflated_clearance);
    }
    if (threat(left_probe)) {
      mask.left = true;
      mask.left_clearance_m = std::min(mask.left_clearance_m, inflated_clearance);
    }
    if (threat(right_probe)) {
      mask.right = true;
      mask.right_clearance_m = std::min(mask.right_clearance_m, inflated_clearance);
    }
  }
  return mask;
}

}  // namespace direction_mask
