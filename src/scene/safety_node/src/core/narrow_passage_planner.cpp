/**
 * @file narrow_passage_planner.cpp
 * @brief P2 窄道提取与 Bézier 中线导引实现
 */
#include "safety_node/core/narrow_passage_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

NarrowPassagePlanner::NarrowPassagePlanner(const AvoidanceParams & params)
: params_(params)
{
}

void NarrowPassagePlanner::updateParams(const AvoidanceParams & params)
{
  params_ = params;
}

NarrowCorridorPlan NarrowPassagePlanner::plan(
  const ObstacleField & field,
  const geometry_msgs::msg::Twist & cmd,
  double vehicle_width_m) const
{
  NarrowCorridorPlan out;
  const NarrowPlannerParams & np = params_.narrow_planner;
  if (!np.enabled || cmd.linear.x <= 1e-3 || vehicle_width_m <= 0.0 ||
    field.obstacles.empty())
  {
    return out;
  }

  const double look_ahead = std::max(0.3, np.look_ahead_m);
  const double roi_half = std::max(0.5 * vehicle_width_m, np.roi_half_width_m);
  const double bin_size = std::clamp(np.bin_size_m, 0.05, look_ahead);
  const int bin_count = static_cast<int>(std::ceil(look_ahead / bin_size));

  // ── Step 1：左右壁内缘提取 ──
  std::vector<double> left_edge(static_cast<size_t>(bin_count), roi_half);
  std::vector<double> right_edge(static_cast<size_t>(bin_count), -roi_half);
  std::vector<bool> left_seen(static_cast<size_t>(bin_count), false);
  std::vector<bool> right_seen(static_cast<size_t>(bin_count), false);

  for (const auto & obs : field.obstacles) {
    if (!obs.valid || obs.x <= 0.0 || obs.x > look_ahead ||
      std::abs(obs.y) > roi_half)
    {
      continue;
    }
    const int idx = std::min(
      bin_count - 1, static_cast<int>(obs.x / bin_size));
    const size_t bi = static_cast<size_t>(idx);
    const double radius = std::max(0.0, obs.radius);
    if (obs.y >= 0.0) {
      left_edge[bi] = std::min(left_edge[bi], obs.y - radius);
      left_seen[bi] = true;
    } else {
      right_edge[bi] = std::max(right_edge[bi], obs.y + radius);
      right_seen[bi] = true;
    }
  }

  // ── Step 2：从车头向前累计连续可行段 ──
  const double min_gap =
    vehicle_width_m + 2.0 * std::max(0.0, np.min_side_margin_m);
  int last_ok = -1;
  double min_gap_seen = std::numeric_limits<double>::infinity();
  bool both_walls_seen = false;
  for (int i = 0; i < bin_count; ++i) {
    const size_t bi = static_cast<size_t>(i);
    const double gap = left_edge[bi] - right_edge[bi];
    if (gap < min_gap) {
      break;
    }
    last_ok = i;
    if (left_seen[bi] || right_seen[bi]) {
      min_gap_seen = std::min(min_gap_seen, gap);
      if (left_seen[bi] && right_seen[bi]) {
        both_walls_seen = true;
      }
    }
  }

  if (last_ok < 0 || !both_walls_seen || !std::isfinite(min_gap_seen)) {
    return out;
  }
  const double corridor_length =
    std::min(look_ahead, static_cast<double>(last_ok + 1) * bin_size);
  if (corridor_length + 1e-9 < np.min_corridor_len_m) {
    return out;
  }
  // 间隙过宽视为开阔区，不注入导引候选（避免干扰正常驾驶）
  if (min_gap_seen - vehicle_width_m > np.narrow_width_max_m) {
    return out;
  }

  // ── Step 3：中线与三次 Bézier ──
  // 单侧有墙的 bin：中线保持与所见墙至少半车宽 + 余量的距离，
  // 而不是用未观测侧的 ROI 边界平均（会把中线拉向开阔侧）。
  const double half_keepout = 0.5 * vehicle_width_m + np.min_side_margin_m;
  auto bin_center = [&](int i) {
      const size_t bi = static_cast<size_t>(i);
      if (left_seen[bi] && right_seen[bi]) {
        return 0.5 * (left_edge[bi] + right_edge[bi]);
      }
      if (left_seen[bi]) {
        return std::min(0.0, left_edge[bi] - half_keepout);
      }
      if (right_seen[bi]) {
        return std::max(0.0, right_edge[bi] + half_keepout);
      }
      return 0.0;
    };

  // 出口锚定在「最后一个观测到墙」的 bin：走廊末端之后的空 bin 属于
  // 出口开阔区，其中线会退化到 0 而误导航向。
  int exit_bin = -1;
  int prev_seen_bin = -1;
  for (int i = 0; i <= last_ok; ++i) {
    const size_t bi = static_cast<size_t>(i);
    if (left_seen[bi] || right_seen[bi]) {
      prev_seen_bin = exit_bin;
      exit_bin = i;
    }
  }
  if (exit_bin < 0) {
    return out;
  }
  const double exit_x = std::max(
    bin_size, (static_cast<double>(exit_bin) + 0.5) * bin_size);
  const double exit_y = bin_center(exit_bin);
  // 出口切线的 y 分量（x 分量不影响起点曲率，见下方 κ0 推导）
  double tangent_y = 0.0;
  if (prev_seen_bin >= 0) {
    const double dx =
      static_cast<double>(exit_bin - prev_seen_bin) * bin_size;
    const double dy = bin_center(exit_bin) - bin_center(prev_seen_bin);
    const double norm = std::hypot(dx, dy);
    if (norm > 1e-9) {
      tangent_y = dy / norm;
    }
  }

  // P0=(0,0) 入口航向 0 → P1=(d,0)；P3=出口，P2=P3−d·t_exit，d=exit_x/3。
  // 三次 Bézier 起点曲率解析解：κ0 = (2/3)·P2y / d²（推导：B'(0)=3(P1−P0)，
  // B''(0)=6(P0−2P1+P2)，κ = |B'×B''| / |B'|³，代入 P0=0、P1=(d,0)）。
  const double d = std::max(1e-3, exit_x / 3.0);
  const double p2y = exit_y - d * tangent_y;
  double kappa0 = (2.0 / 3.0) * p2y / (d * d);
  const double max_kappa = std::max(0.1, np.max_curvature_1_m);
  kappa0 = std::clamp(kappa0, -max_kappa, max_kappa);

  // ── Step 4：净空限速与居中候选 ──
  const double lateral_clearance = 0.5 * (min_gap_seen - vehicle_width_m);
  const double speed_cap = std::max(0.0, np.min_pass_speed_m_s) +
    std::max(0.0, np.speed_per_clearance) *
    std::max(0.0, lateral_clearance - np.min_side_margin_m);
  const double v = std::clamp(
    std::min(cmd.linear.x, speed_cap),
    0.0,
    params_.absolute_max_linear_speed);
  const double w = std::clamp(
    v * kappa0,
    -params_.absolute_max_angular_speed,
    params_.absolute_max_angular_speed);

  out.valid = true;
  out.corridor_width_m = min_gap_seen;
  out.corridor_length_m = corridor_length;
  out.lateral_clearance_m = lateral_clearance;
  out.curvature_1_m = kappa0;
  out.speed_cap_m_s = speed_cap;
  out.candidate.v = v;
  out.candidate.w = w;
  return out;
}
