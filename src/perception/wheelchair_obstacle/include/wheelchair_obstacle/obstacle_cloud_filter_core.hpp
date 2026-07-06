#ifndef WHEELCHAIR_OBSTACLE__OBSTACLE_CLOUD_FILTER_CORE_HPP_
#define WHEELCHAIR_OBSTACLE__OBSTACLE_CLOUD_FILTER_CORE_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace wheelchair_obstacle
{

struct FilterPoint
{
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
};

struct RangeConfig
{
  double range_min = 0.05;
  double range_max = 3.00;
  double z_min = -0.30;
  double z_max = 0.80;
};

struct RoiConfig
{
  double x_min = -1.20;
  double x_max = 2.50;
  double y_min = -1.50;
  double y_max = 1.50;
};

struct PolygonMask
{
  std::string name;
  std::vector<std::pair<double, double>> polygon;
};

struct IsolationConfig
{
  double neighbor_radius = 0.08;
  int min_neighbors = 2;
  double grid_resolution = 0.05;
  double skip_isolation_range = 0.50;
};

struct VoxelConfig
{
  double voxel_size_xy = 0.04;
  bool keep_nearest = true;
};

struct FilterStats
{
  std::size_t input = 0;
  std::size_t after_finite_range = 0;
  std::size_t after_roi = 0;
  std::size_t after_self_mask = 0;
  std::size_t after_isolation = 0;
  std::size_t after_voxel = 0;
};

class ObstacleCloudFilterCore
{
public:
  ObstacleCloudFilterCore() = default;

  void set_range_config(const RangeConfig & cfg) {range_ = cfg;}
  void set_roi_config(const RoiConfig & cfg) {roi_ = cfg;}
  void set_self_masks(const std::vector<PolygonMask> & masks) {masks_ = masks;}
  void set_isolation_config(const IsolationConfig & cfg) {isolation_ = cfg;}
  void set_voxel_config(const VoxelConfig & cfg) {voxel_ = cfg;}

  void set_self_mask_enabled(bool enabled) {self_mask_enabled_ = enabled;}
  void set_isolation_enabled(bool enabled) {isolation_enabled_ = enabled;}
  void set_voxel_enabled(bool enabled) {voxel_enabled_ = enabled;}

  const RangeConfig & range_config() const {return range_;}
  const RoiConfig & roi_config() const {return roi_;}
  const std::vector<PolygonMask> & self_masks() const {return masks_;}
  const IsolationConfig & isolation_config() const {return isolation_;}
  const VoxelConfig & voxel_config() const {return voxel_;}

  static bool is_finite(const FilterPoint & p)
  {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  }

  bool pass_range_and_z(const FilterPoint & p) const
  {
    const double dx = static_cast<double>(p.x);
    const double dy = static_cast<double>(p.y);
    const double dz = static_cast<double>(p.z);
    const double r = std::hypot(dx, dy);
    if (r < range_.range_min || r > range_.range_max) {
      return false;
    }
    if (dz < range_.z_min || dz > range_.z_max) {
      return false;
    }
    return true;
  }

  bool pass_roi(const FilterPoint & p) const
  {
    const double x = static_cast<double>(p.x);
    const double y = static_cast<double>(p.y);
    return x >= roi_.x_min && x <= roi_.x_max && y >= roi_.y_min && y <= roi_.y_max;
  }

  static bool point_in_polygon(
    double x, double y, const std::vector<std::pair<double,
    double>> & poly)
  {
    if (poly.size() < 3) {
      return false;
    }
    bool inside = false;
    std::size_t j = poly.size() - 1;
    for (std::size_t i = 0; i < poly.size(); ++i) {
      const double xi = poly[i].first;
      const double yi = poly[i].second;
      const double xj = poly[j].first;
      const double yj = poly[j].second;
      const bool intersect =
        ((yi > y) != (yj > y)) &&
        (x < (xj - xi) * (y - yi) / ((yj - yi) + std::numeric_limits<double>::epsilon()) + xi);
      if (intersect) {
        inside = !inside;
      }
      j = i;
    }
    return inside;
  }

  bool inside_self_mask(double x, double y) const
  {
    for (const auto & mask : masks_) {
      if (point_in_polygon(x, y, mask.polygon)) {
        return true;
      }
    }
    return false;
  }

  std::vector<FilterPoint> remove_isolated_points(const std::vector<FilterPoint> & pts) const
  {
    if (pts.empty()) {
      return {};
    }
    const double res = isolation_.grid_resolution > 0.0 ? isolation_.grid_resolution : 0.05;
    const double r = isolation_.neighbor_radius > 0.0 ? isolation_.neighbor_radius : 0.08;
    const double r_sq = r * r;
    const int cell_radius = std::max(1, static_cast<int>(std::ceil(r / res)));

    struct CellKey
    {
      int cx;
      int cy;
      bool operator==(const CellKey & o) const {return cx == o.cx && cy == o.cy;}
    };
    struct CellKeyHash
    {
      std::size_t operator()(const CellKey & k) const noexcept
      {
        return std::hash<int64_t>()(
          static_cast<int64_t>(k.cx) * 73856093LL ^
          static_cast<int64_t>(k.cy) * 19349663LL);
      }
    };

    std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHash> grid;
    grid.reserve(pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) {
      CellKey key{static_cast<int>(std::floor(pts[i].x / res)),
        static_cast<int>(std::floor(pts[i].y / res))};
      grid[key].push_back(i);
    }

    std::vector<FilterPoint> out;
    out.reserve(pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) {
      const double px = static_cast<double>(pts[i].x);
      const double py = static_cast<double>(pts[i].y);
      const double pr_sq = px * px + py * py;

      if (isolation_.skip_isolation_range > 0.0 &&
        pr_sq <= isolation_.skip_isolation_range * isolation_.skip_isolation_range)
      {
        out.push_back(pts[i]);
        continue;
      }

      const CellKey center{static_cast<int>(std::floor(px / res)),
        static_cast<int>(std::floor(py / res))};
      int neighbors = 0;
      for (int dx = -cell_radius; dx <= cell_radius && neighbors < isolation_.min_neighbors; ++dx) {
        for (int dy = -cell_radius; dy <= cell_radius && neighbors < isolation_.min_neighbors;
          ++dy)
        {
          const CellKey key{center.cx + dx, center.cy + dy};
          auto it = grid.find(key);
          if (it == grid.end()) {
            continue;
          }
          for (const auto j : it->second) {
            if (j == i) {
              continue;
            }
            const double ddx = static_cast<double>(pts[j].x) - px;
            const double ddy = static_cast<double>(pts[j].y) - py;
            if (ddx * ddx + ddy * ddy <= r_sq) {
              ++neighbors;
              if (neighbors >= isolation_.min_neighbors) {
                break;
              }
            }
          }
        }
      }
      if (neighbors >= isolation_.min_neighbors) {
        out.push_back(pts[i]);
      }
    }
    return out;
  }

  std::vector<FilterPoint> voxel_downsample(const std::vector<FilterPoint> & pts) const
  {
    if (pts.empty()) {
      return {};
    }
    const double vox = voxel_.voxel_size_xy > 0.0 ? voxel_.voxel_size_xy : 0.04;

    struct CellKey
    {
      int64_t cx;
      int64_t cy;
      bool operator==(const CellKey & o) const {return cx == o.cx && cy == o.cy;}
    };
    struct CellKeyHash
    {
      std::size_t operator()(const CellKey & k) const noexcept
      {
        return std::hash<int64_t>()(k.cx * 73856093LL ^ k.cy * 19349663LL);
      }
    };

    struct Best
    {
      std::size_t index;
      double dist_sq;
    };

    std::unordered_map<CellKey, Best, CellKeyHash> grid;
    grid.reserve(pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) {
      const CellKey key{static_cast<int64_t>(std::floor(pts[i].x / vox)),
        static_cast<int64_t>(std::floor(pts[i].y / vox))};
      const double dist_sq =
        static_cast<double>(pts[i].x) * static_cast<double>(pts[i].x) +
        static_cast<double>(pts[i].y) * static_cast<double>(pts[i].y);
      auto it = grid.find(key);
      if (it == grid.end()) {
        grid.emplace(key, Best{i, dist_sq});
      } else if (voxel_.keep_nearest) {
        if (dist_sq < it->second.dist_sq) {
          it->second = Best{i, dist_sq};
        }
      } else {
        if (dist_sq > it->second.dist_sq) {
          it->second = Best{i, dist_sq};
        }
      }
    }

    std::vector<FilterPoint> out;
    out.reserve(grid.size());
    for (const auto & kv : grid) {
      out.push_back(pts[kv.second.index]);
    }
    return out;
  }

  std::vector<FilterPoint> filter(const std::vector<FilterPoint> & input, FilterStats & stats) const
  {
    stats = FilterStats{};
    stats.input = input.size();

    std::vector<FilterPoint> after_finite_range;
    after_finite_range.reserve(input.size());
    for (const auto & p : input) {
      if (!is_finite(p)) {
        continue;
      }
      if (!pass_range_and_z(p)) {
        continue;
      }
      after_finite_range.push_back(p);
    }
    stats.after_finite_range = after_finite_range.size();

    std::vector<FilterPoint> after_roi;
    after_roi.reserve(after_finite_range.size());
    for (const auto & p : after_finite_range) {
      if (pass_roi(p)) {
        after_roi.push_back(p);
      }
    }
    stats.after_roi = after_roi.size();

    std::vector<FilterPoint> after_self_mask;
    after_self_mask.reserve(after_roi.size());
    for (const auto & p : after_roi) {
      if (self_mask_enabled_ &&
        inside_self_mask(static_cast<double>(p.x), static_cast<double>(p.y)))
      {
        continue;
      }
      after_self_mask.push_back(p);
    }
    stats.after_self_mask = after_self_mask.size();

    std::vector<FilterPoint> after_isolation;
    if (isolation_enabled_) {
      after_isolation = remove_isolated_points(after_self_mask);
    } else {
      after_isolation = after_self_mask;
    }
    stats.after_isolation = after_isolation.size();

    std::vector<FilterPoint> after_voxel;
    if (voxel_enabled_) {
      after_voxel = voxel_downsample(after_isolation);
    } else {
      after_voxel = after_isolation;
    }
    stats.after_voxel = after_voxel.size();

    return after_voxel;
  }

  static std::vector<PolygonMask> default_self_masks()
  {
    return {
      {"chassis",
        {{-0.45, -0.36}, {0.55, -0.36}, {0.55, 0.36}, {-0.45, 0.36}}},
      {"front_footrest",
        {{0.45, -0.30}, {0.85, -0.30}, {0.85, 0.30}, {0.45, 0.30}}},
    };
  }

private:
  RangeConfig range_;
  RoiConfig roi_;
  std::vector<PolygonMask> masks_;
  IsolationConfig isolation_;
  VoxelConfig voxel_;
  bool self_mask_enabled_ = true;
  bool isolation_enabled_ = true;
  bool voxel_enabled_ = true;
};

}  // namespace wheelchair_obstacle

#endif  // WHEELCHAIR_OBSTACLE__OBSTACLE_CLOUD_FILTER_CORE_HPP_
