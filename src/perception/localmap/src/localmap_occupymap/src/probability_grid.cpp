// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "localmap_occupymap/probability_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace localmap_occupymap
{

namespace
{
constexpr float kUnknownLogOdds = 0.0f;
constexpr float kOccupiedThreshold = 0.55f;
constexpr float kFreeThreshold = 0.45f;
}  // namespace

ProbabilityGrid::ProbabilityGrid(
  double resolution, int size_x, int size_y, double origin_x, double origin_y,
  float hit_probability, float miss_probability)
: resolution_(resolution),
  size_x_(size_x),
  size_y_(size_y),
  origin_x_(origin_x),
  origin_y_(origin_y),
  log_odds_(size_x * size_y, kUnknownLogOdds),
  last_hit_time_(size_x * size_y, 0.0),
  blind_zone_retention_time_(size_x * size_y, 0.0),
  update_marker_(size_x * size_y, 0),
  hit_log_odds_(probabilityToLogOdds(hit_probability)),
  miss_log_odds_(probabilityToLogOdds(miss_probability))
{
}

void ProbabilityGrid::reset()
{
  std::fill(log_odds_.begin(), log_odds_.end(), kUnknownLogOdds);
  std::fill(last_hit_time_.begin(), last_hit_time_.end(), 0.0);
  std::fill(blind_zone_retention_time_.begin(), blind_zone_retention_time_.end(), 0.0);
  std::fill(update_marker_.begin(), update_marker_.end(), 0);
}

void ProbabilityGrid::setOrigin(double origin_x, double origin_y)
{
  origin_x_ = origin_x;
  origin_y_ = origin_y;
}

void ProbabilityGrid::shiftOrigin(double new_origin_x, double new_origin_y)
{
  const int shift_x =
    static_cast<int>(std::lround((new_origin_x - origin_x_) / resolution_));
  const int shift_y =
    static_cast<int>(std::lround((new_origin_y - origin_y_) / resolution_));

  if (shift_x == 0 && shift_y == 0) {
    origin_x_ = new_origin_x;
    origin_y_ = new_origin_y;
    return;
  }

  std::vector<float> new_log_odds(size_x_ * size_y_, kUnknownLogOdds);
  std::vector<double> new_last_hit_time(size_x_ * size_y_, 0.0);
  std::vector<double> new_blind_zone_retention_time(size_x_ * size_y_, 0.0);
  std::vector<uint16_t> new_update_marker(size_x_ * size_y_, 0);

  for (int new_y = 0; new_y < size_y_; ++new_y) {
    for (int new_x = 0; new_x < size_x_; ++new_x) {
      const int old_x = new_x + shift_x;
      const int old_y = new_y + shift_y;
      if (!isInside(old_x, old_y)) {
        continue;
      }
      const int old_idx = index(old_x, old_y);
      const int new_idx = index(new_x, new_y);
      new_log_odds[new_idx] = log_odds_[old_idx];
      new_last_hit_time[new_idx] = last_hit_time_[old_idx];
      new_blind_zone_retention_time[new_idx] = blind_zone_retention_time_[old_idx];
      new_update_marker[new_idx] = 0;
    }
  }

  log_odds_ = std::move(new_log_odds);
  last_hit_time_ = std::move(new_last_hit_time);
  blind_zone_retention_time_ = std::move(new_blind_zone_retention_time);
  update_marker_ = std::move(new_update_marker);
  origin_x_ = new_origin_x;
  origin_y_ = new_origin_y;
}

bool ProbabilityGrid::worldToMap(double wx, double wy, int & mx, int & my) const
{
  mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  return isInside(mx, my);
}

void ProbabilityGrid::mapToWorld(int mx, int my, double & wx, double & wy) const
{
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

void ProbabilityGrid::setVehicleFootprint(
  double front_x, double rear_x, double half_width, double blind_zone_retention_sec)
{
  vehicle_front_x_ = front_x;
  vehicle_rear_x_ = rear_x;
  vehicle_half_width_ = half_width;
  blind_zone_retention_sec_ = blind_zone_retention_sec;
}

void ProbabilityGrid::setRobotPose(double x, double y, double yaw)
{
  robot_x_ = x;
  robot_y_ = y;
  robot_yaw_ = yaw;
}

void ProbabilityGrid::worldToRobotBase(double wx, double wy, double & bx, double & by) const
{
  const double dx = wx - robot_x_;
  const double dy = wy - robot_y_;
  const double cos_yaw = std::cos(robot_yaw_);
  const double sin_yaw = std::sin(robot_yaw_);
  bx = cos_yaw * dx + sin_yaw * dy;
  by = -sin_yaw * dx + cos_yaw * dy;
}

void ProbabilityGrid::fillGridOrigin(geometry_msgs::msg::Pose & origin) const
{
  double wx = 0.0;
  double wy = 0.0;
  mapToWorld(0, 0, wx, wy);
  origin.position.x = wx - resolution_ / 2.0;
  origin.position.y = wy - resolution_ / 2.0;
  origin.position.z = 0.0;
  origin.orientation.w = 1.0;
}

bool ProbabilityGrid::isInBlindZone(double wx, double wy) const
{
  double bx = 0.0;
  double by = 0.0;
  worldToRobotBase(wx, wy, bx, by);
  const bool inside_vehicle =
    bx >= vehicle_rear_x_ && bx <= vehicle_front_x_ &&
    by >= -vehicle_half_width_ && by <= vehicle_half_width_;
  return !inside_vehicle && bx <= vehicle_front_x_;
}

bool ProbabilityGrid::isOccupiedByHitProb(int idx) const
{
  return logOddsToProbability(log_odds_[idx]) >= kOccupiedThreshold;
}

bool ProbabilityGrid::isBlindZoneRetentionActive(
  int idx, double wx, double wy, double current_time) const
{
  if (!isInBlindZone(wx, wy)) {
    return false;
  }
  return blind_zone_retention_time_[idx] > 0.0 &&
         (current_time - blind_zone_retention_time_[idx]) <= blind_zone_retention_sec_;
}

void ProbabilityGrid::updateBlindZoneRetention(double timestamp)
{
  for (int y = 0; y < size_y_; ++y) {
    for (int x = 0; x < size_x_; ++x) {
      double wx = 0.0;
      double wy = 0.0;
      mapToWorld(x, y, wx, wy);
      const int idx = index(x, y);
      if (!isInBlindZone(wx, wy)) {
        blind_zone_retention_time_[idx] = 0.0;
        continue;
      }
      if (isOccupiedByHitProb(idx) && blind_zone_retention_time_[idx] == 0.0) {
        blind_zone_retention_time_[idx] = timestamp;
        expandUpdateBounds(x, y, 0);
      }
    }
  }
}

void ProbabilityGrid::insertRangeData(
  double origin_x, double origin_y, const std::vector<double> & hit_x,
  const std::vector<double> & hit_y, double timestamp, bool insert_free_space)
{
  update_x0_ = size_x_;
  update_y0_ = size_y_;
  update_xn_ = 0;
  update_yn_ = 0;

  int origin_mx = 0;
  int origin_my = 0;
  if (!worldToMap(origin_x, origin_y, origin_mx, origin_my)) {
    return;
  }
  expandUpdateBounds(origin_mx, origin_my, 0);

  std::vector<std::pair<int, int>> hit_cells;
  hit_cells.reserve(hit_x.size());

  for (size_t i = 0; i < hit_x.size(); ++i) {
    int hit_mx = 0;
    int hit_my = 0;
    if (!worldToMap(hit_x[i], hit_y[i], hit_mx, hit_my)) {
      continue;
    }
    applyHit(hit_mx, hit_my, timestamp);
    hit_cells.emplace_back(hit_mx, hit_my);
  }

  if (insert_free_space) {
    for (const auto & cell : hit_cells) {
      castRay(origin_mx, origin_my, cell.first, cell.second, timestamp);
    }
  }

  finishUpdate();
  updateBlindZoneRetention(timestamp);
  has_pending_update_ = update_xn_ > update_x0_ && update_yn_ > update_y0_;
}

void ProbabilityGrid::finishUpdate()
{
  for (auto & marker : update_marker_) {
    marker = 0;
  }
}

nav_msgs::msg::OccupancyGrid ProbabilityGrid::toOccupancyGrid(
  const std::string & frame_id, const rclcpp::Time & stamp, double stationary_retention_sec,
  bool robot_stationary, double current_time, double inflation_radius_m,
  double inscribed_radius_m, bool track_unknown_space, double cost_scaling_factor) const
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = stamp;
  grid.header.frame_id = frame_id;
  grid.info.resolution = static_cast<float>(resolution_);
  grid.info.width = static_cast<uint32_t>(size_x_);
  grid.info.height = static_cast<uint32_t>(size_y_);
  fillGridOrigin(grid.info.origin);
  grid.data.resize(
    static_cast<size_t>(size_x_ * size_y_), track_unknown_space ? static_cast<int8_t>(-1) : 0);

  const auto inflated = buildInflatedCostGrid(
    stationary_retention_sec, robot_stationary, current_time, inflation_radius_m,
    inscribed_radius_m, track_unknown_space, cost_scaling_factor);
  for (size_t i = 0; i < inflated.size(); ++i) {
    grid.data[i] = costToOccupancyValue(inflated[i]);
  }

  return grid;
}

std::uint8_t ProbabilityGrid::classifyNav2Cost(
  int idx, float probability, double wx, double wy, double stationary_retention_sec,
  bool robot_stationary, double current_time, bool track_unknown_space) const
{
  const std::uint8_t unknown_cost = track_unknown_space ? kCostUnknown : kCostFree;

  if (isBlindZoneRetentionActive(idx, wx, wy, current_time)) {
    return kCostLethal;
  }

  if (robot_stationary) {
    const bool recently_hit =
      last_hit_time_[idx] > 0.0 &&
      (current_time - last_hit_time_[idx]) <= stationary_retention_sec;
    if (!recently_hit) {
      if (probability < kFreeThreshold && log_odds_[idx] < -1e-3f) {
        return kCostFree;
      }
      return unknown_cost;
    }
  }

  if (probability >= kOccupiedThreshold) {
    return kCostLethal;
  }
  if (probability < kFreeThreshold && log_odds_[idx] < -1e-3f) {
    return kCostFree;
  }
  return unknown_cost;
}

std::uint8_t ProbabilityGrid::computeInflationCost(
  double dist_m, double inflation_radius_m, double inscribed_radius_m,
  double cost_scaling_factor)
{
  if (dist_m > inflation_radius_m) {
    return kCostFree;
  }
  if (dist_m <= inscribed_radius_m) {
    return kCostInscribed;
  }
  const double factor =
    std::exp(-cost_scaling_factor * (dist_m - inscribed_radius_m));
  return static_cast<std::uint8_t>(
    std::clamp((kCostInscribed - 1) * factor, 1.0, 252.0));
}

std::vector<std::uint8_t> ProbabilityGrid::buildInflatedCostGrid(
  double stationary_retention_sec, bool robot_stationary, double current_time,
  double inflation_radius_m, double inscribed_radius_m, bool track_unknown_space,
  double cost_scaling_factor) const
{
  const size_t cell_count = static_cast<size_t>(size_x_ * size_y_);
  const std::uint8_t default_cost = track_unknown_space ? kCostUnknown : kCostFree;
  std::vector<std::uint8_t> base_cost(cell_count, default_cost);
  std::vector<std::uint8_t> inflated_cost(cell_count, default_cost);

  for (int y = 0; y < size_y_; ++y) {
    for (int x = 0; x < size_x_; ++x) {
      const int idx = index(x, y);
      double wx = 0.0;
      double wy = 0.0;
      mapToWorld(x, y, wx, wy);
      const float probability = logOddsToProbability(log_odds_[idx]);
      base_cost[static_cast<size_t>(idx)] = classifyNav2Cost(
        idx, probability, wx, wy, stationary_retention_sec, robot_stationary, current_time,
        track_unknown_space);
    }
  }

  inflated_cost = base_cost;
  if (inflation_radius_m <= 0.0) {
    return inflated_cost;
  }

  const int inflation_cells =
    static_cast<int>(std::ceil(inflation_radius_m / resolution_));
  const double inscribed_radius = std::max(0.0, inscribed_radius_m);

  for (int y = 0; y < size_y_; ++y) {
    for (int x = 0; x < size_x_; ++x) {
      const int idx = index(x, y);
      if (base_cost[static_cast<size_t>(idx)] != kCostLethal) {
        continue;
      }

      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (!isInside(nx, ny)) {
            continue;
          }
          const double dist = std::hypot(dx * resolution_, dy * resolution_);
          if (dist > inflation_radius_m) {
            continue;
          }

          std::uint8_t cost = computeInflationCost(
            dist, inflation_radius_m, inscribed_radius, cost_scaling_factor);

          const int nidx = index(nx, ny);
          inflated_cost[static_cast<size_t>(nidx)] =
            std::max(inflated_cost[static_cast<size_t>(nidx)], cost);
        }
      }
    }
  }

  return inflated_cost;
}

nav2_msgs::msg::Costmap ProbabilityGrid::toCostmap(
  const std::string & frame_id, const rclcpp::Time & stamp, double stationary_retention_sec,
  bool robot_stationary, double current_time, const std::string & layer,
  double inflation_radius_m, double inscribed_radius_m, bool track_unknown_space,
  double cost_scaling_factor) const
{
  nav2_msgs::msg::Costmap costmap;
  costmap.header.stamp = stamp;
  costmap.header.frame_id = frame_id;
  costmap.metadata.map_load_time = stamp;
  costmap.metadata.update_time = stamp;
  costmap.metadata.layer = layer;
  costmap.metadata.resolution = static_cast<float>(resolution_);
  costmap.metadata.size_x = static_cast<uint32_t>(size_x_);
  costmap.metadata.size_y = static_cast<uint32_t>(size_y_);
  fillGridOrigin(costmap.metadata.origin);

  const auto inflated = buildInflatedCostGrid(
    stationary_retention_sec, robot_stationary, current_time, inflation_radius_m,
    inscribed_radius_m, track_unknown_space, cost_scaling_factor);
  costmap.data.assign(inflated.begin(), inflated.end());
  return costmap;
}

map_msgs::msg::OccupancyGridUpdate ProbabilityGrid::toOccupancyGridUpdate(
  const std::string & frame_id, const rclcpp::Time & stamp, double stationary_retention_sec,
  bool robot_stationary, double current_time, double inflation_radius_m,
  double inscribed_radius_m, bool track_unknown_space, double cost_scaling_factor) const
{
  map_msgs::msg::OccupancyGridUpdate update;
  update.header.stamp = stamp;
  update.header.frame_id = frame_id;

  if (!has_pending_update_) {
    return update;
  }

  const int inflation_cells =
    static_cast<int>(std::ceil(inflation_radius_m / resolution_));
  const int x0 = std::max(0, update_x0_ - inflation_cells);
  const int y0 = std::max(0, update_y0_ - inflation_cells);
  const int xn = std::min(size_x_, update_xn_ + inflation_cells);
  const int yn = std::min(size_y_, update_yn_ + inflation_cells);

  update.x = x0;
  update.y = y0;
  update.width = static_cast<uint32_t>(xn - x0);
  update.height = static_cast<uint32_t>(yn - y0);

  const auto inflated = buildInflatedCostGrid(
    stationary_retention_sec, robot_stationary, current_time, inflation_radius_m,
    inscribed_radius_m, track_unknown_space, cost_scaling_factor);

  update.data.reserve(static_cast<size_t>(update.width * update.height));
  for (int y = y0; y < yn; ++y) {
    for (int x = x0; x < xn; ++x) {
      update.data.push_back(costToOccupancyValue(
        inflated[static_cast<size_t>(index(x, y))]));
    }
  }
  return update;
}

void ProbabilityGrid::expandUpdateBounds(int mx, int my, int inflation_cells)
{
  update_x0_ = std::min(update_x0_, mx - inflation_cells);
  update_y0_ = std::min(update_y0_, my - inflation_cells);
  update_xn_ = std::max(update_xn_, mx + inflation_cells + 1);
  update_yn_ = std::max(update_yn_, my + inflation_cells + 1);
}

float ProbabilityGrid::clampProbability(float p)
{
  return std::clamp(p, kMinProbability, kMaxProbability);
}

float ProbabilityGrid::probabilityToLogOdds(float p)
{
  p = clampProbability(p);
  return std::log(p / (1.0f - p));
}

float ProbabilityGrid::logOddsToProbability(float l)
{
  return 1.0f / (1.0f + std::exp(-l));
}

int8_t ProbabilityGrid::probabilityToOccupancyValue(float p)
{
  return static_cast<int8_t>(std::lround(clampProbability(p) * 100.0f));
}

int ProbabilityGrid::index(int mx, int my) const
{
  return my * size_x_ + mx;
}

bool ProbabilityGrid::isInside(int mx, int my) const
{
  return mx >= 0 && mx < size_x_ && my >= 0 && my < size_y_;
}

void ProbabilityGrid::applyHit(int mx, int my, double timestamp)
{
  const int idx = index(mx, my);
  if ((update_marker_[idx] & kUpdateMarker) != 0) {
    return;
  }
  update_marker_[idx] |= kUpdateMarker;
  log_odds_[idx] += hit_log_odds_;
  log_odds_[idx] = std::clamp(
    log_odds_[idx], probabilityToLogOdds(kMinProbability),
    probabilityToLogOdds(kMaxProbability));
  last_hit_time_[idx] = timestamp;
  blind_zone_retention_time_[idx] = 0.0;
  expandUpdateBounds(mx, my, 0);
}

void ProbabilityGrid::applyMiss(int mx, int my, double wx, double wy, double current_time)
{
  const int idx = index(mx, my);
  if (isBlindZoneRetentionActive(idx, wx, wy, current_time)) {
    return;
  }
  if (isInBlindZone(wx, wy) && isOccupiedByHitProb(idx)) {
    return;
  }
  if ((update_marker_[idx] & kUpdateMarker) != 0) {
    return;
  }
  update_marker_[idx] |= kUpdateMarker;
  log_odds_[idx] += miss_log_odds_;
  log_odds_[idx] = std::clamp(
    log_odds_[idx], probabilityToLogOdds(kMinProbability),
    probabilityToLogOdds(kMaxProbability));
  expandUpdateBounds(mx, my, 0);
}

void ProbabilityGrid::castRay(int x0, int y0, int x1, int y1, double current_time)
{
  const auto cells = bresenhamLine(x0, y0, x1, y1);
  for (const auto & cell : cells) {
    double wx = 0.0;
    double wy = 0.0;
    mapToWorld(cell[0], cell[1], wx, wy);
    applyMiss(cell[0], cell[1], wx, wy, current_time);
  }
}

std::vector<std::vector<int>> ProbabilityGrid::bresenhamLine(int x0, int y0, int x1, int y1)
{
  std::vector<std::vector<int>> cells;
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  const int sx = (x0 < x1) ? 1 : -1;
  const int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int x = x0;
  int y = y0;

  while (true) {
    if (!(x == x0 && y == y0) && !(x == x1 && y == y1)) {
      cells.push_back({x, y});
    }
    if (x == x1 && y == y1) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }

  return cells;
}

}  // namespace localmap_occupymap
