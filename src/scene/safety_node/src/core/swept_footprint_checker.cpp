/**
 * @file swept_footprint_checker.cpp
 * @brief 轨迹扫掠 footprint 与 ObstacleField 碰撞检测
 *
 * L1 方向解耦：direction_mask_enabled 时对「近障」（t=0 位于硬停带内，
 * 含已在虚拟车体内）做方向豁免——整条轨迹不将其推向物理接触的近障不参与
 * min_distance / hard_stop / collision 归因。由此前进候选不再被后方/侧向
 * 近障在 t=0 的全向净空污染，倒车候选不再被前方近障判死。远障语义不变。
 */
#include "safety_node/core/swept_footprint_checker.h"

#include "safety_node/core/direction_mask.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{

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

}  // namespace

SweptFootprintChecker::SweptFootprintChecker(
  const AvoidanceParams & params,
  VehicleFootprint & footprint)
: params_(params)
  , footprint_(footprint)
{
}

void SweptFootprintChecker::updateParams(const AvoidanceParams & params)
{
  params_ = params;
  footprint_.updateParams(params);
}

CollisionCheckResult SweptFootprintChecker::checkTrajectory(
  const TrajectoryRollout & trajectory,
  const ObstacleField & field) const
{
  CollisionCheckResult result;
  const FootprintPolygon polygon = footprint_.inflatedPolygon();
  const bool directional = params_.direction_mask_enabled;
  const FootprintPolygon physical_polygon =
    directional ? footprint_.physicalPolygon() : FootprintPolygon{};

  // 预计算各位姿累计路径距离（原实现按位姿顺序累加，等价）
  std::vector<double> path_distance_at(trajectory.poses.size(), 0.0);
  for (size_t pi = 1; pi < trajectory.poses.size(); ++pi) {
    const auto & pose = trajectory.poses[pi];
    const auto & previous = trajectory.poses[pi - 1U];
    path_distance_at[pi] = path_distance_at[pi - 1U] +
      std::hypot(pose.x - previous.x, pose.y - previous.y);
  }
  if (!trajectory.poses.empty()) {
    result.path_length_m = path_distance_at.back();
  }

  // ── 方案 8 预裁剪：轨迹 AABB 外扩「Footprint 外接圆半径 + 裁剪带宽」──
  // 带外障碍的全程净空必然 > swept_cull_band_m（校验器保证带宽 ≥ 一切净空
  // 判据阈值），跳过它们不改变任何控制决策，仅使远障不再计入 min_distance
  //（日志中由有限值变为 +inf）。band ≤ 0 关闭（旧行为，供等价性单测对照）。
  const double cull_band = params_.swept_cull_band_m;
  double traj_min_x = 0.0;
  double traj_max_x = 0.0;
  double traj_min_y = 0.0;
  double traj_max_y = 0.0;
  double polygon_radius = 0.0;
  const bool cull_enabled = cull_band > 0.0 && !trajectory.poses.empty();
  if (cull_enabled) {
    traj_min_x = traj_max_x = trajectory.poses.front().x;
    traj_min_y = traj_max_y = trajectory.poses.front().y;
    for (const auto & pose : trajectory.poses) {
      traj_min_x = std::min(traj_min_x, pose.x);
      traj_max_x = std::max(traj_max_x, pose.x);
      traj_min_y = std::min(traj_min_y, pose.y);
      traj_max_y = std::max(traj_max_y, pose.y);
    }
    for (const auto & vertex : polygon.vertices) {
      polygon_radius = std::max(
        polygon_radius, std::hypot(vertex.first, vertex.second));
    }
  }

  for (const auto & obs : field.obstacles) {
    if (!obs.valid) {
      continue;
    }

    if (cull_enabled) {
      const double margin = polygon_radius + std::max(0.0, obs.radius) + cull_band;
      if (obs.x < traj_min_x - margin || obs.x > traj_max_x + margin ||
        obs.y < traj_min_y - margin || obs.y > traj_max_y + margin)
      {
        continue;
      }
    }

    // ── 近障方向豁免：t=0 已在硬停带内且整条轨迹不推向物理接触 → 整体跳过 ──
    if (directional && !trajectory.poses.empty()) {
      double entry_lx = 0.0;
      double entry_ly = 0.0;
      transformObstacleToPoseFrame(obs.x, obs.y, trajectory.poses.front(), entry_lx, entry_ly);
      const double entry_signed_clearance = VehicleFootprint::signedClearanceToPolygon(
        entry_lx, entry_ly, obs.radius, polygon);
      if (std::isfinite(entry_signed_clearance) &&
        entry_signed_clearance <= params_.hard_stop_distance_m &&
        !direction_mask::obstacleThreatenedAlongPoses(
          obs, trajectory.poses, physical_polygon,
          params_.direction_mask_contact_buffer_m,
          params_.direction_mask_approach_eps_m))
      {
        continue;
      }
    }

    for (size_t pi = 0; pi < trajectory.poses.size(); ++pi) {
      const auto & pose = trajectory.poses[pi];
      const double path_distance_m = path_distance_at[pi];
      double lx = 0.0;
      double ly = 0.0;
      transformObstacleToPoseFrame(obs.x, obs.y, pose, lx, ly);

      const double clearance =
        VehicleFootprint::clearanceToPolygon(lx, ly, obs.radius, polygon);
      if (clearance < result.min_distance) {
        result.min_distance = clearance;
      }
      if (clearance <= params_.hard_stop_distance_m &&
        path_distance_m < result.hard_stop_path_distance_m)
      {
        result.hard_stop_path_distance_m = path_distance_m;
        result.hard_stop_x = obs.x;
        result.hard_stop_y = obs.y;
      }

      if (VehicleFootprint::pointInPolygon(lx, ly, obs.radius, polygon)) {
        if (!result.collision || pose.t < result.ttc) {
          result.collision = true;
          result.ttc = pose.t;
          result.collision_x = obs.x;
          result.collision_y = obs.y;
          result.source = obs.source;
          result.pose_index = static_cast<int>(pi);
          result.collision_path_distance_m = path_distance_m;
        }
      }
    }
  }

  if (!result.collision) {
    result.ttc = std::numeric_limits<double>::infinity();
  }
  return result;
}
