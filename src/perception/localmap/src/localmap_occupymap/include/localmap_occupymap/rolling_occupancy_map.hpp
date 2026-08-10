// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#ifndef LOCALMAP_OCCUPYMAP__ROLLING_OCCUPANCY_MAP_HPP_
#define LOCALMAP_OCCUPYMAP__ROLLING_OCCUPANCY_MAP_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "localmap_occupymap/probability_grid.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace localmap_occupymap
{

struct OccupymapParams
{
  double map_extent_forward_m{4.0};
  double map_extent_backward_m{4.0};
  double map_extent_left_m{4.0};
  double map_extent_right_m{4.0};
  double observe_extent_forward_m{4.0};
  double observe_extent_backward_m{4.0};
  double observe_extent_left_m{3.0};
  double observe_extent_right_m{3.0};
  double resolution_m{0.05};
  double hit_probability{ProbabilityGrid::kDefaultHitProbability};
  double miss_probability{ProbabilityGrid::kDefaultMissProbability};
  double stationary_linear_thresh_mps{0.05};
  double stationary_angular_thresh_rps{0.05};
  double stationary_retention_sec{10.0};
  double min_z_m{-0.5};
  double max_z_m{2.0};
  double max_range_m{30.0};
  bool insert_free_space{true};
  std::string map_frame{"odom"};
  std::string robot_base_frame{"base_link"};
  std::string costmap_layer{"master"};
  double inflation_radius_m{0.55};
  double inscribed_radius_m{0.35};
  double cost_scaling_factor{10.0};
  // Vehicle footprint in base_link (origin at rear axle center).
  double vehicle_front_x_m{0.9};
  double vehicle_rear_x_m{-0.1};
  double vehicle_half_width_m{0.5};
  double blind_zone_retention_sec{10.0};
  bool track_unknown_space{false};
};

class RollingOccupancyMap
{
public:
  explicit RollingOccupancyMap(const OccupymapParams & params);

  void updateOdometry(const nav_msgs::msg::Odometry & odom);
  void insertPointCloud(
    const sensor_msgs::msg::PointCloud2 & cloud, const std::string & target_frame);

  nav_msgs::msg::OccupancyGrid getOccupancyGrid(const rclcpp::Time & stamp) const;
  nav2_msgs::msg::Costmap getCostmap(const rclcpp::Time & stamp) const;
  map_msgs::msg::OccupancyGridUpdate getOccupancyGridUpdate(const rclcpp::Time & stamp) const;
  void clearPendingUpdate();

  bool hasOdometry() const;
  bool isRobotStationary() const;
  std::string mapFrame() const { return params_.map_frame; }
  double inflationRadiusM() const { return params_.inflation_radius_m; }
  double costScalingFactor() const { return params_.cost_scaling_factor; }
  void setInflationParams(double inflation_radius_m, double cost_scaling_factor);

private:
  void updateRollingOrigin();
  static double yawFromQuaternion(
    double qx, double qy, double qz, double qw);

  OccupymapParams params_;
  mutable std::mutex mutex_;

  std::unique_ptr<ProbabilityGrid> grid_;
  bool has_odom_{false};
  bool robot_stationary_{false};
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_yaw_{0.0};
};

}  // namespace localmap_occupymap

#endif  // LOCALMAP_OCCUPYMAP__ROLLING_OCCUPANCY_MAP_HPP_
