// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#ifndef LOCALMAP_OCCUPYMAP__PROBABILITY_GRID_HPP_
#define LOCALMAP_OCCUPYMAP__PROBABILITY_GRID_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "localmap_occupymap/cost_value.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/time.hpp"

namespace localmap_occupymap
{

/// Cartographer-inspired 2D probability grid with log-odds Bayesian updates.
class ProbabilityGrid
{
public:
  static constexpr float kMinProbability = 0.1f;
  static constexpr float kMaxProbability = 0.9f;
  static constexpr float kDefaultHitProbability = 0.55f;
  static constexpr float kDefaultMissProbability = 0.49f;

  ProbabilityGrid(
    double resolution, int size_x, int size_y, double origin_x, double origin_y,
    float hit_probability = kDefaultHitProbability,
    float miss_probability = kDefaultMissProbability);

  void reset();
  void setOrigin(double origin_x, double origin_y);
  void shiftOrigin(double new_origin_x, double new_origin_y);

  double resolution() const { return resolution_; }
  int sizeX() const { return size_x_; }
  int sizeY() const { return size_y_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }

  bool worldToMap(double wx, double wy, int & mx, int & my) const;
  void mapToWorld(int mx, int my, double & wx, double & wy) const;

  void insertRangeData(
    double origin_x, double origin_y, const std::vector<double> & hit_x,
    const std::vector<double> & hit_y, double timestamp, bool insert_free_space);

  void setVehicleFootprint(
    double front_x, double rear_x, double half_width, double blind_zone_retention_sec);

  void setRobotPose(double x, double y, double yaw);

  void finishUpdate();

  nav_msgs::msg::OccupancyGrid toOccupancyGrid(
    const std::string & frame_id, const rclcpp::Time & stamp, double stationary_retention_sec,
    bool robot_stationary, double current_time, double inflation_radius_m,
    double inscribed_radius_m, bool track_unknown_space, double cost_scaling_factor) const;

  nav2_msgs::msg::Costmap toCostmap(
    const std::string & frame_id, const rclcpp::Time & stamp, double stationary_retention_sec,
    bool robot_stationary, double current_time, const std::string & layer,
    double inflation_radius_m, double inscribed_radius_m, bool track_unknown_space,
    double cost_scaling_factor) const;

  map_msgs::msg::OccupancyGridUpdate toOccupancyGridUpdate(
    const std::string & frame_id, const rclcpp::Time & stamp, double stationary_retention_sec,
    bool robot_stationary, double current_time, double inflation_radius_m,
    double inscribed_radius_m, bool track_unknown_space, double cost_scaling_factor) const;

  bool hasPendingUpdate() const { return has_pending_update_; }
  void clearPendingUpdate() { has_pending_update_ = false; }

  const std::vector<float> & logOdds() const { return log_odds_; }
  const std::vector<double> & lastHitTime() const { return last_hit_time_; }

private:
  static float clampProbability(float p);
  static float probabilityToLogOdds(float p);
  static float logOddsToProbability(float l);
  static int8_t probabilityToOccupancyValue(float p);
  std::uint8_t classifyNav2Cost(
    int idx, float probability, double wx, double wy, double stationary_retention_sec,
    bool robot_stationary, double current_time, bool track_unknown_space) const;
  bool isInBlindZone(double wx, double wy) const;
  void worldToRobotBase(double wx, double wy, double & bx, double & by) const;
  void fillGridOrigin(geometry_msgs::msg::Pose & origin) const;
  bool isOccupiedByHitProb(int idx) const;
  bool isBlindZoneRetentionActive(
    int idx, double wx, double wy, double current_time) const;
  void updateBlindZoneRetention(double timestamp);
  std::vector<std::uint8_t> buildInflatedCostGrid(
    double stationary_retention_sec, bool robot_stationary, double current_time,
    double inflation_radius_m, double inscribed_radius_m, bool track_unknown_space,
    double cost_scaling_factor) const;
  static std::uint8_t computeInflationCost(
    double dist_m, double inflation_radius_m, double inscribed_radius_m,
    double cost_scaling_factor);
  void expandUpdateBounds(int mx, int my, int inflation_cells);

  int index(int mx, int my) const;
  bool isInside(int mx, int my) const;

  void applyHit(int mx, int my, double timestamp);
  void applyMiss(int mx, int my, double wx, double wy, double current_time);
  void castRay(int x0, int y0, int x1, int y1, double current_time);

  static std::vector<std::vector<int>> bresenhamLine(int x0, int y0, int x1, int y1);

  double resolution_;
  int size_x_;
  int size_y_;
  double origin_x_;
  double origin_y_;

  std::vector<float> log_odds_;
  std::vector<double> last_hit_time_;
  std::vector<double> blind_zone_retention_time_;
  std::vector<uint16_t> update_marker_;

  float hit_log_odds_;
  float miss_log_odds_;
  double vehicle_front_x_{0.9};
  double vehicle_rear_x_{-0.1};
  double vehicle_half_width_{0.5};
  double blind_zone_retention_sec_{10.0};
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_yaw_{0.0};
  static constexpr uint16_t kUpdateMarker = 1u << 15;

  bool has_pending_update_{false};
  int update_x0_{0};
  int update_y0_{0};
  int update_xn_{0};
  int update_yn_{0};
};

}  // namespace localmap_occupymap

#endif  // LOCALMAP_OCCUPYMAP__PROBABILITY_GRID_HPP_
