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
  publishSnapshot();
}

void RollingLocalMap::updateConfig(const LocalMapConfig & config)
{
  std::lock_guard<std::mutex> lock(ops_mutex_);
  config_ = config;
  initGrid();
  publishSnapshot();
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
  ego_residual_x_m_ = 0.0;
  ego_residual_y_m_ = 0.0;
  rotation_accum_rad_ = 0.0;

  // ── 方案 8.2 膨胀核查找表：格间距离只与 (dr,dc) 偏移有关，预计算 falloff，
  // 免去 applyInflation 内层每对格子的 gridToWorld + hypot 冗余计算。──
  inflation_kernel_.clear();
  if (config_.inflation_radius_m > 0.0 && config_.resolution_m > 0.0) {
    const int radius_cells = static_cast<int>(
      std::ceil(config_.inflation_radius_m / config_.resolution_m));
    for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
      for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
        const double dist =
          std::hypot(static_cast<double>(dc), static_cast<double>(dr)) *
          config_.resolution_m;
        if (dist > config_.inflation_radius_m) {
          continue;
        }
        const double falloff = std::pow(
          std::max(0.0, 1.0 - dist / config_.inflation_radius_m), 2.0);
        inflation_kernel_.emplace_back(dr, dc, static_cast<float>(falloff));
      }
    }
  }
}

void RollingLocalMap::reset()
{
  std::lock_guard<std::mutex> lock(ops_mutex_);
  std::fill(grid_.probability.begin(), grid_.probability.end(), 0.0f);
  std::fill(grid_.inflated.begin(), grid_.inflated.end(), 0.0f);
  std::fill(last_hit_.begin(), last_hit_.end(), rclcpp::Time(0, 0, RCL_ROS_TIME));
  last_update_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  ego_residual_x_m_ = 0.0;
  ego_residual_y_m_ = 0.0;
  rotation_accum_rad_ = 0.0;
  publishSnapshot();
}

void RollingLocalMap::publishSnapshot()
{
  auto copy = std::make_shared<RollingLocalMapGrid>(grid_);
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_ = std::move(copy);
}

std::shared_ptr<const RollingLocalMapGrid> RollingLocalMap::snapshot() const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_;
}

void RollingLocalMap::shiftGrid(int dc, int dr)
{
  // 内容位移 (dc, dr)：new[r][c] = old[r - dr][c - dc]，暴露区域清零。
  // probability / inflated（派生层）/ last_hit_ 三层必须同步搬移，
  // 否则旧位置的膨胀残留会继续参与 queryCost 的 max 合成（假 lethal）。
  const int cols = static_cast<int>(grid_.cols);
  const int rows = static_cast<int>(grid_.rows);
  const auto shift_layer = [&](std::vector<float> & layer) {
      shift_scratch_.assign(layer.size(), 0.0f);
      for (int r = 0; r < rows; ++r) {
        const int src_r = r - dr;
        if (src_r < 0 || src_r >= rows) {
          continue;
        }
        for (int c = 0; c < cols; ++c) {
          const int src_c = c - dc;
          if (src_c < 0 || src_c >= cols) {
            continue;
          }
          shift_scratch_[cellIndex(c, r)] = layer[cellIndex(src_c, src_r)];
        }
      }
      layer.swap(shift_scratch_);
    };
  shift_layer(grid_.probability);
  shift_layer(grid_.inflated);

  shift_scratch_hits_.assign(last_hit_.size(), rclcpp::Time(0, 0, RCL_ROS_TIME));
  for (int r = 0; r < rows; ++r) {
    const int src_r = r - dr;
    if (src_r < 0 || src_r >= rows) {
      continue;
    }
    for (int c = 0; c < cols; ++c) {
      const int src_c = c - dc;
      if (src_c < 0 || src_c >= cols) {
        continue;
      }
      shift_scratch_hits_[cellIndex(c, r)] = last_hit_[cellIndex(src_c, src_r)];
    }
  }
  last_hit_.swap(shift_scratch_hits_);
}

void RollingLocalMap::applyEgoMotion(double dx_m, double dy_m, double dtheta_rad)
{
  std::lock_guard<std::mutex> lock(ops_mutex_);
  // rebuild 模式每帧重建天然无错位；补偿关闭时保持旧行为（逐位一致）。
  if (config_.rebuild_each_frame || !config_.ego_motion_compensation) {
    return;
  }
  if (config_.resolution_m <= 0.0) {
    return;
  }
  // 世界点固定，自车在旧 base_link 系前移 (dx,dy) ⇒ 栅格内容反向平移 (−dx,−dy)。
  ego_residual_x_m_ += -dx_m;
  ego_residual_y_m_ += -dy_m;
  const int shift_c = static_cast<int>(std::lround(ego_residual_x_m_ / config_.resolution_m));
  const int shift_r = static_cast<int>(std::lround(ego_residual_y_m_ / config_.resolution_m));
  if (shift_c != 0 || shift_r != 0) {
    ego_residual_x_m_ -= shift_c * config_.resolution_m;
    ego_residual_y_m_ -= shift_r * config_.resolution_m;
    shiftGrid(shift_c, shift_r);
  }
  // 旋转不做栅格重采样（误差大、开销高）：累计角行程超阈值时全图概率乘惩罚
  // 衰减——转弯多则记忆快速失效，直行时记忆完整（见 LocalMapConfig 注释）。
  rotation_accum_rad_ += std::abs(dtheta_rad);
  if (config_.rotation_decay_threshold_rad > 0.0 &&
    rotation_accum_rad_ >= config_.rotation_decay_threshold_rad)
  {
    const float factor = static_cast<float>(
      std::clamp(config_.rotation_decay_factor, 0.0, 1.0));
    for (auto & probability : grid_.probability) {
      probability *= factor;
    }
    for (auto & inflated : grid_.inflated) {
      inflated *= factor;
    }
    rotation_accum_rad_ = 0.0;
  }
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
  std::lock_guard<std::mutex> lock(ops_mutex_);
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
  std::lock_guard<std::mutex> lock(ops_mutex_);
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
  // 方案 8.2：邻域偏移与 falloff 已在 initGrid 预计算为查找表，
  // 内层不再做 gridToWorld + hypot（格间距离只与偏移有关）。
  for (uint32_t row = 0; row < grid_.rows; ++row) {
    for (uint32_t col = 0; col < grid_.cols; ++col) {
      const size_t idx = cellIndex(static_cast<int>(col), static_cast<int>(row));
      const float p = grid_.probability[idx];
      if (p < 0.5f) {
        continue;
      }
      // 对 hot cell 邻域做 falloff(d) = (1 - d/R)² 膨胀
      for (const auto & [dr, dc, falloff] : inflation_kernel_) {
        const int nc = static_cast<int>(col) + dc;
        const int nr = static_cast<int>(row) + dr;
        if (nc < 0 || nr < 0 ||
          nc >= static_cast<int>(grid_.cols) ||
          nr >= static_cast<int>(grid_.rows))
        {
          continue;
        }
        const size_t nidx = cellIndex(nc, nr);
        grid_.inflated[nidx] = std::max(grid_.inflated[nidx], p * falloff);
      }
    }
  }
}

void RollingLocalMap::finalizeFrame(const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(ops_mutex_);
  // 全局时间衰减：仅对未被当前帧 hitCell 更新的格子生效。
  // hitCell 内部已对被命中格子做了贝叶斯衰减+融合（按距上次命中时间），
  // 此处再衰减会导致双重衰减。当前调用流程中 integratePoints 设置
  // last_update_=stamp，本函数 dt=0 → 衰减系数=1.0，不会双重衰减。
  // 以下条件保留以防御未来调用方式变化。
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
  // 方案 7：向控制环/可视化线程发布只读快照（原子交换 shared_ptr）
  publishSnapshot();
}

float RollingLocalMap::queryCost(double x, double y) const
{
  std::lock_guard<std::mutex> lock(ops_mutex_);
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

nav_msgs::msg::OccupancyGrid RollingLocalMap::gridToOccupancyMsg(
  const RollingLocalMapGrid & grid, const rclcpp::Time & stamp)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "base_link";
  msg.info.resolution = static_cast<float>(grid.resolution_m);
  msg.info.width = grid.cols;
  msg.info.height = grid.rows;
  msg.info.origin.position.x = grid.origin_x_m;
  msg.info.origin.position.y = grid.origin_y_m;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.resize(grid.probability.size());
  for (size_t i = 0; i < grid.probability.size(); ++i) {
    const float cost = std::max(grid.probability[i], grid.inflated[i]);
    msg.data[i] = static_cast<int8_t>(std::min(100.0f, cost * 100.0f));
  }
  return msg;
}

nav_msgs::msg::OccupancyGrid RollingLocalMap::toOccupancyGridMsg(const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lock(ops_mutex_);
  return gridToOccupancyMsg(grid_, stamp);
}
