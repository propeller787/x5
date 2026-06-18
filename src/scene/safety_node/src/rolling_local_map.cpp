/**
 * @file rolling_local_map.cpp
 * @brief RollingLocalMap 实现：落格、衰减、膨胀、查询
 */
#include "safety_node/rolling_local_map.h"

#include <algorithm>
#include <cmath>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace
{

constexpr double kEps = 1e-6;

float clamp01(float v)
{
  return std::max(0.0f, std::min(1.0f, v));
}

} // namespace

RollingLocalMap::RollingLocalMap(const LocalMapConfig & config)
: config_(config)
{
  initGrid();
}

void RollingLocalMap::updateConfig(const LocalMapConfig & config)
{
  config_ = config;
  initGrid();
}

void RollingLocalMap::initGrid()
{
  // 地图中心对齐 base_link，origin 为左下角
  const uint32_t cols =
    static_cast<uint32_t>(std::max(1.0, config_.width_m / config_.resolution_m));
  const uint32_t rows =
    static_cast<uint32_t>(std::max(1.0, config_.height_m / config_.resolution_m));
  const size_t n = static_cast<size_t>(cols) * rows;

  grid_.cols = cols;
  grid_.rows = rows;
  grid_.resolution_m = config_.resolution_m;
  grid_.origin_x_m = -config_.width_m * 0.5;
  grid_.origin_y_m = -config_.height_m * 0.5;
  grid_.lethal_threshold = static_cast<float>(config_.lethal_threshold);
  grid_.probability.assign(n, 0.0f);
  grid_.inflated.assign(n, 0.0f);
  last_hit_.assign(n, rclcpp::Time(0, 0, RCL_ROS_TIME));
}

void RollingLocalMap::reset()
{
  std::fill(grid_.probability.begin(), grid_.probability.end(), 0.0f);
  std::fill(grid_.inflated.begin(), grid_.inflated.end(), 0.0f);
  std::fill(last_hit_.begin(), last_hit_.end(), rclcpp::Time(0, 0, RCL_ROS_TIME));
  last_update_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

bool RollingLocalMap::worldToGrid(double x, double y, int & col, int & row) const
{
  col = static_cast<int>(std::floor((x - grid_.origin_x_m) / grid_.resolution_m));
  row = static_cast<int>(std::floor((y - grid_.origin_y_m) / grid_.resolution_m));
  return col >= 0 && row >= 0 &&
         col < static_cast<int>(grid_.cols) &&
         row < static_cast<int>(grid_.rows);
}

void RollingLocalMap::gridToWorld(int col, int row, double & x, double & y) const
{
  x = grid_.origin_x_m + (static_cast<double>(col) + 0.5) * grid_.resolution_m;
  y = grid_.origin_y_m + (static_cast<double>(row) + 0.5) * grid_.resolution_m;
}

size_t RollingLocalMap::cellIndex(int col, int row) const
{
  return static_cast<size_t>(row) * grid_.cols + static_cast<size_t>(col);
}

void RollingLocalMap::hitCell(int col, int row, double w_obs, const rclcpp::Time & stamp)
{
  const size_t idx = cellIndex(col, row);
  float & p = grid_.probability[idx];

  // 距上次命中的时间差 → 指数衰减系数
  double dt = 0.0;
  if (last_hit_[idx].nanoseconds() > 0 && last_update_.nanoseconds() > 0) {
    dt = (stamp - last_hit_[idx]).seconds();
  }
  const double decay = std::pow(0.5, dt / std::max(kEps, config_.decay_halflife_s));
  const float p_decay = static_cast<float>(p * decay);

  // 贝叶斯-lite 饱和累加
  p = clamp01(p_decay + static_cast<float>(w_obs) * (1.0f - p_decay));
  last_hit_[idx] = stamp;
}

void RollingLocalMap::integratePoints(
  const std::vector<std::tuple<double, double, double>> & points,
  const rclcpp::Time & stamp,
  double w_obs)
{
  // V1 全量重建：每周期先清空概率层
  if (config_.rebuild_each_frame) {
    std::fill(grid_.probability.begin(), grid_.probability.end(), 0.0f);
  }

  const double weight = (w_obs > 0.0) ? w_obs : config_.observation_weight;
  for (const auto & pt : points) {
    const double x = std::get<0>(pt);
    const double y = std::get<1>(pt);
    const double z = std::get<2>(pt);
    if (std::abs(z) > config_.tilt_z_threshold) {
      continue;
    }
    int col = 0;
    int row = 0;
    if (!worldToGrid(x, y, col, row)) {
      continue;
    }
    hitCell(col, row, weight, stamp);
  }
  last_update_ = stamp;
}

void RollingLocalMap::integratePrimitives(const ObstacleField & field, double w_obs)
{
  for (const auto & prim : field.obstacles) {
    if (!prim.valid) {
      continue;
    }
    int col = 0;
    int row = 0;
    if (!worldToGrid(prim.x, prim.y, col, row)) {
      continue;
    }
    hitCell(col, row, w_obs, field.stamp);
  }
}

void RollingLocalMap::pruneStale(const rclcpp::Time & now)
{
  if (config_.rebuild_each_frame) {
    return;
  }
  for (size_t i = 0; i < grid_.probability.size(); ++i) {
    if (last_hit_[i].nanoseconds() <= 0) {
      continue;
    }
    if ((now - last_hit_[i]).seconds() > config_.max_hit_age_s) {
      grid_.probability[i] = 0.0f;
      last_hit_[i] = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
  }
}

void RollingLocalMap::applyInflation()
{
  std::fill(grid_.inflated.begin(), grid_.inflated.end(), 0.0f);
  if (config_.inflation_radius_m <= 0.0) {
    grid_.inflated = grid_.probability;
    return;
  }
  const int radius_cells = static_cast<int>(
    std::ceil(config_.inflation_radius_m / grid_.resolution_m));

  for (uint32_t row = 0; row < grid_.rows; ++row) {
    for (uint32_t col = 0; col < grid_.cols; ++col) {
      const size_t idx = cellIndex(static_cast<int>(col), static_cast<int>(row));
      const float p = grid_.probability[idx];
      if (p < 0.5f) {
        continue;
      }
      // 对 hot cell 邻域做 falloff(d) = (1 - d/R)² 膨胀
      for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
        for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
          const int nc = static_cast<int>(col) + dc;
          const int nr = static_cast<int>(row) + dr;
          if (nc < 0 || nr < 0 ||
            nc >= static_cast<int>(grid_.cols) ||
            nr >= static_cast<int>(grid_.rows))
          {
            continue;
          }
          double cx = 0.0;
          double cy = 0.0;
          double nx = 0.0;
          double ny = 0.0;
          gridToWorld(static_cast<int>(col), static_cast<int>(row), cx, cy);
          gridToWorld(nc, nr, nx, ny);
          const double dist = std::hypot(nx - cx, ny - cy);
          if (dist > config_.inflation_radius_m) {
            continue;
          }
          const double falloff = std::pow(
            std::max(0.0, 1.0 - dist / config_.inflation_radius_m), 2.0);
          const size_t nidx = cellIndex(nc, nr);
          grid_.inflated[nidx] = std::max(
            grid_.inflated[nidx], static_cast<float>(p * falloff));
        }
      }
    }
  }
}

void RollingLocalMap::finalizeFrame(const rclcpp::Time & stamp)
{
  if (!config_.rebuild_each_frame && last_update_.nanoseconds() > 0) {
    const double dt = std::max(0.0, (stamp - last_update_).seconds());
    const double decay = std::pow(
      0.5, dt / std::max(kEps, config_.decay_halflife_s));
    for (auto & probability : grid_.probability) {
      probability = clamp01(static_cast<float>(probability * decay));
    }
  }
  pruneStale(stamp);
  applyInflation();
  grid_.stamp = stamp;
  last_update_ = stamp;
}

float RollingLocalMap::queryCost(double x, double y) const
{
  int col = 0;
  int row = 0;
  if (!worldToGrid(x, y, col, row)) {
    return 0.0f;
  }
  const size_t idx = cellIndex(col, row);
  return std::max(grid_.probability[idx], grid_.inflated[idx]);
}

bool RollingLocalMap::lethalQuery(double x, double y, double lethal_threshold) const
{
  return queryCost(x, y) >= static_cast<float>(lethal_threshold);
}

nav_msgs::msg::OccupancyGrid RollingLocalMap::toOccupancyGridMsg(const rclcpp::Time & stamp) const
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "base_link";
  msg.info.resolution = static_cast<float>(grid_.resolution_m);
  msg.info.width = grid_.cols;
  msg.info.height = grid_.rows;
  msg.info.origin.position.x = grid_.origin_x_m;
  msg.info.origin.position.y = grid_.origin_y_m;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.resize(grid_.probability.size());
  for (size_t i = 0; i < grid_.probability.size(); ++i) {
    const float cost = std::max(grid_.probability[i], grid_.inflated[i]);
    msg.data[i] = static_cast<int8_t>(std::min(100.0f, cost * 100.0f));
  }
  return msg;
}
