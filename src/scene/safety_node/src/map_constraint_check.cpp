/**
 * @file map_constraint_check.cpp
 * @brief 沿轨迹采样 footprint 角点/中心，查询 LocalMap lethal
 */
#include "safety_node/map_constraint_check.h"

#include <cmath>
#include <limits>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{

/** 查询栅格综合代价 max(probability, inflated) */
float gridCostAt(
  const RollingLocalMapGrid & map,
  double x,
  double y)
{
  if (map.resolution_m <= 0.0 || map.cols == 0 || map.rows == 0) {
    return 1.0f;
  }
  const int col = static_cast<int>(std::floor((x - map.origin_x_m) / map.resolution_m));
  const int row = static_cast<int>(std::floor((y - map.origin_y_m) / map.resolution_m));
  if (col < 0 || row < 0 ||
    col >= static_cast<int>(map.cols) ||
    row >= static_cast<int>(map.rows))
  {
    return 1.0f;
  }
  const size_t idx = static_cast<size_t>(row) * map.cols + static_cast<size_t>(col);
  if (idx >= map.probability.size() || idx >= map.inflated.size()) {
    return 1.0f;
  }
  return std::max(map.probability[idx], map.inflated[idx]);
}

bool gridCellAt(
  const RollingLocalMapGrid & map,
  double x,
  double y,
  int & col,
  int & row,
  size_t & idx,
  float & cost)
{
  if (map.resolution_m <= 0.0 || map.cols == 0 || map.rows == 0) {
    return false;
  }
  col = static_cast<int>(std::floor((x - map.origin_x_m) / map.resolution_m));
  row = static_cast<int>(std::floor((y - map.origin_y_m) / map.resolution_m));
  if (col < 0 || row < 0 ||
    col >= static_cast<int>(map.cols) ||
    row >= static_cast<int>(map.rows))
  {
    return false;
  }
  idx = static_cast<size_t>(row) * map.cols + static_cast<size_t>(col);
  if (idx >= map.probability.size() || idx >= map.inflated.size()) {
    return false;
  }
  cost = std::max(map.probability[idx], map.inflated[idx]);
  return true;
}

std::vector<std::pair<double, double>> footprintSamples(
  const FootprintPolygon & local_polygon,
  const PredictedPose & pose,
  double resolution_m,
  bool check_corners)
{
  std::vector<std::pair<double, double>> samples;
  samples.emplace_back(pose.x, pose.y);
  if (!check_corners) {
    return samples;
  }

  const FootprintPolygon world_polygon =
    VehicleFootprint::transformPolygon(local_polygon, pose);
  const double step = std::max(resolution_m, 0.05);
  const auto local_samples =
    VehicleFootprint::samplePolygonInterior(local_polygon, step);
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  for (const auto & local_pt : local_samples) {
    samples.emplace_back(
      pose.x + c * local_pt.first - s * local_pt.second,
      pose.y + s * local_pt.first + c * local_pt.second);
  }
  for (const auto & vertex : world_polygon.vertices) {
    samples.emplace_back(vertex.first, vertex.second);
  }
  return samples;
}

}  // namespace

MapConstraintCheck::MapConstraintCheck(
  const AvoidanceParams & fp_params,
  const MapConstraintConfig & config)
: fp_params_(fp_params)
  , config_(config)
  , footprint_(fp_params)
{
}

void MapConstraintCheck::updateFootprintParams(const AvoidanceParams & fp_params)
{
  fp_params_ = fp_params;
  footprint_.updateParams(fp_params_);
}

void MapConstraintCheck::updateConfig(const MapConstraintConfig & config)
{
  config_ = config;
}

bool MapConstraintCheck::trajectoryLethal(
  const RollingLocalMapGrid & map,
  const std::vector<PredictedPose> & poses,
  double lethal_threshold,
  bool check_corners,
  MapConstraintResult & out,
  const std::vector<MapExemptCircle> * exemptions) const
{
  out = MapConstraintResult{};
  const FootprintPolygon local_polygon = footprint_.inflatedPolygon();
  const auto exempted = [exemptions](double x, double y) {
      if (exemptions == nullptr) {
        return false;
      }
      for (const auto & circle : *exemptions) {
        const double dx = x - circle.x;
        const double dy = y - circle.y;
        if (dx * dx + dy * dy <= circle.radius * circle.radius) {
          return true;
        }
      }
      return false;
    };

  for (const auto & pose : poses) {
    for (const auto & pt :
      footprintSamples(local_polygon, pose, map.resolution_m, check_corners))
    {
      if (exempted(pt.first, pt.second)) {
        continue;
      }
      const float cost = gridCostAt(map, pt.first, pt.second);
      out.p_max_on_path = std::max(out.p_max_on_path, static_cast<double>(cost));
      if (cost >= static_cast<float>(lethal_threshold)) {
        out.lethal = true;
        out.first_lethal_t = std::min(out.first_lethal_t, pose.t);
        const int col = static_cast<int>(
          std::floor((pt.first - map.origin_x_m) / map.resolution_m));
        const int row = static_cast<int>(
          std::floor((pt.second - map.origin_y_m) / map.resolution_m));
        out.lethal_cell_col = col;
        out.lethal_cell_row = row;
        return true;
      }
    }
  }
  return false;
}

bool MapConstraintCheck::trajectoryNoNewLethal(
  const RollingLocalMapGrid & map,
  const std::vector<PredictedPose> & poses,
  double lethal_threshold,
  bool check_corners,
  MapConstraintResult & out) const
{
  out = MapConstraintResult{};
  if (poses.empty()) {
    return true;    // 无运动 → 安全
  }

  const FootprintPolygon local_polygon = footprint_.inflatedPolygon();
  auto lethalCellsAtPose =
    [&](const PredictedPose & pose,
      std::unordered_set<size_t> & cells,
      double & max_cost) -> bool
    {
      cells.clear();
      max_cost = 0.0;
      for (const auto & pt :
        footprintSamples(local_polygon, pose, map.resolution_m, check_corners))
      {
        int col = -1;
        int row = -1;
        size_t idx = 0;
        float cost = 0.0f;
        if (!gridCellAt(map, pt.first, pt.second, col, row, idx, cost)) {
          return false;    // 越出地图 → 保守视为不安全
        }
        max_cost = std::max(max_cost, static_cast<double>(cost));
        if (cost >= static_cast<float>(lethal_threshold)) {
          cells.insert(idx);
        }
      }
      return true;
    };

  // 起始姿势的 lethal 集即豁免集（起始不 lethal 时为空 → 全程不得触任何 lethal）。
  std::unordered_set<size_t> initial_cells;
  double initial_max_cost = 0.0;
  if (!lethalCellsAtPose(poses.front(), initial_cells, initial_max_cost)) {
    return false;
  }
  out.p_max_on_path = initial_max_cost;

  for (size_t i = 1; i < poses.size(); ++i) {
    std::unordered_set<size_t> current_cells;
    double max_cost = 0.0;
    if (!lethalCellsAtPose(poses[i], current_cells, max_cost)) {
      return false;
    }
    out.p_max_on_path = std::max(out.p_max_on_path, max_cost);
    for (const size_t idx : current_cells) {
      if (initial_cells.count(idx) == 0U) {
        out.lethal = true;    // 踏入起始未占用的新 lethal → 恶化
        out.first_lethal_t = poses[i].t;
        return false;
      }
    }
  }
  return true;
}

bool MapConstraintCheck::trajectoryEscapesInitialLethal(
  const RollingLocalMapGrid & map,
  const std::vector<PredictedPose> & poses,
  double lethal_threshold,
  bool check_corners,
  double max_escape_time_s,
  MapConstraintResult & out) const
{
  out = MapConstraintResult{};
  if (poses.size() < 2 || max_escape_time_s <= 0.0) {
    return false;
  }

  const FootprintPolygon local_polygon = footprint_.inflatedPolygon();
  auto lethalCellsAtPose =
    [&](const PredictedPose & pose,
      std::unordered_set<size_t> & cells,
      double & max_cost) -> bool
    {
      cells.clear();
      max_cost = 0.0;
      for (const auto & pt :
        footprintSamples(local_polygon, pose, map.resolution_m, check_corners))
      {
        int col = -1;
        int row = -1;
        size_t idx = 0;
        float cost = 0.0f;
        if (!gridCellAt(map, pt.first, pt.second, col, row, idx, cost)) {
          return false;
        }
        max_cost = std::max(max_cost, static_cast<double>(cost));
        if (cost >= static_cast<float>(lethal_threshold)) {
          cells.insert(idx);
          if (out.lethal_cell_col < 0) {
            out.lethal_cell_col = col;
            out.lethal_cell_row = row;
          }
        }
      }
      return true;
    };

  std::unordered_set<size_t> initial_cells;
  double initial_max_cost = 0.0;
  if (!lethalCellsAtPose(poses.front(), initial_cells, initial_max_cost) ||
    initial_cells.empty())
  {
    return false;
  }

  out.lethal = true;
  out.first_lethal_t = poses.front().t;
  out.p_max_on_path = initial_max_cost;
  bool cleared = false;
  // L2 单调退离（默认关）：跟踪 footprint 对初始 lethal 集的覆盖格数是否逐位姿非增，
  // 及末位姿的残余覆盖数。全程不进新 lethal 由下方的子集检查保证。
  bool retreat_monotonic = true;
  size_t prev_count = initial_cells.size();
  size_t last_count = initial_cells.size();
  for (size_t i = 1; i < poses.size(); ++i) {
    std::unordered_set<size_t> current_cells;
    double max_cost = 0.0;
    if (!lethalCellsAtPose(poses[i], current_cells, max_cost)) {
      return false;
    }
    out.p_max_on_path = std::max(out.p_max_on_path, max_cost);

    for (const size_t idx : current_cells) {
      if (initial_cells.count(idx) == 0U) {
        return false;    // 进入新 lethal：任何模式下都否
      }
    }
    const size_t cur_count = current_cells.size();
    if (cur_count > prev_count) {
      retreat_monotonic = false;
    }
    prev_count = cur_count;
    last_count = cur_count;

    if (cleared && !current_cells.empty()) {
      return false;
    }
    if (current_cells.empty() && !cleared) {
      if (poses[i].t <= max_escape_time_s + 1e-9) {
        cleared = true;    // 窗内清空：原行为
      } else if (config_.retreat_escape_enabled && retreat_monotonic) {
        cleared = true;    // 慢清空但单调退离：放宽接受
      } else {
        return false;
      }
    }
  }
  if (cleared) {
    return true;
  }
  // 未完全清空，但单调退离且有足够进展 → 接受（仅在开关开启时）。
  if (config_.retreat_escape_enabled && retreat_monotonic &&
    static_cast<double>(initial_cells.size()) - static_cast<double>(last_count) >=
    std::max(1.0, config_.retreat_min_progress_cells))
  {
    return true;
  }
  return false;
}

MapConstraintResult MapConstraintCheck::queryTrajectory(
  const RollingLocalMapGrid & map,
  const std::vector<PredictedPose> & poses,
  double lethal_threshold,
  bool check_corners) const
{
  MapConstraintResult out;
  trajectoryLethal(map, poses, lethal_threshold, check_corners, out);
  return out;
}
