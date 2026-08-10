// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "localmap_occupymap/rolling_occupancy_map.hpp"

#include <cmath>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace localmap_occupymap
{

RollingOccupancyMap::RollingOccupancyMap(const OccupymapParams & params)
: params_(params)
{
  const int size_x = static_cast<int>(std::ceil(
      (params_.map_extent_forward_m + params_.map_extent_backward_m) / params_.resolution_m));
  const int size_y = static_cast<int>(std::ceil(
      (params_.map_extent_left_m + params_.map_extent_right_m) / params_.resolution_m));

  grid_ = std::make_unique<ProbabilityGrid>(
    params_.resolution_m, size_x, size_y, 0.0, 0.0,
    static_cast<float>(params_.hit_probability),
    static_cast<float>(params_.miss_probability));
  grid_->setVehicleFootprint(
    params_.vehicle_front_x_m, params_.vehicle_rear_x_m, params_.vehicle_half_width_m,
    params_.blind_zone_retention_sec);
}

double RollingOccupancyMap::yawFromQuaternion(
  double qx, double qy, double qz, double qw)
{
  tf2::Quaternion q(qx, qy, qz, qw);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

void RollingOccupancyMap::updateRollingOrigin()
{
  const double map_width = params_.map_extent_forward_m + params_.map_extent_backward_m;
  const double map_height = params_.map_extent_left_m + params_.map_extent_right_m;
  double new_origin_x = robot_x_ - map_width / 2.0;
  double new_origin_y = robot_y_ - map_height / 2.0;
  new_origin_x =
    std::floor(new_origin_x / params_.resolution_m) * params_.resolution_m;
  new_origin_y =
    std::floor(new_origin_y / params_.resolution_m) * params_.resolution_m;

  if (std::abs(new_origin_x - grid_->originX()) > 1e-6 ||
    std::abs(new_origin_y - grid_->originY()) > 1e-6)
  {
    grid_->shiftOrigin(new_origin_x, new_origin_y);
  }
}

void RollingOccupancyMap::updateOdometry(const nav_msgs::msg::Odometry & odom)
{
  std::lock_guard<std::mutex> lock(mutex_);
  has_odom_ = true;

  robot_x_ = odom.pose.pose.position.x;
  robot_y_ = odom.pose.pose.position.y;
  robot_yaw_ = yawFromQuaternion(
    odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);

  const double linear_speed = std::hypot(
    odom.twist.twist.linear.x, odom.twist.twist.linear.y);
  const double angular_speed = std::abs(odom.twist.twist.angular.z);
  robot_stationary_ = linear_speed < params_.stationary_linear_thresh_mps &&
                      angular_speed < params_.stationary_angular_thresh_rps;

  grid_->setRobotPose(robot_x_, robot_y_, robot_yaw_);
  updateRollingOrigin();
}

void RollingOccupancyMap::insertPointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & target_frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_odom_) {
    return;
  }

  if (cloud.header.frame_id != target_frame) {
    return;
  }

  grid_->setRobotPose(robot_x_, robot_y_, robot_yaw_);
  updateRollingOrigin();

  const double timestamp = rclcpp::Time(cloud.header.stamp).seconds();
  const double cos_yaw = std::cos(robot_yaw_);
  const double sin_yaw = std::sin(robot_yaw_);

  std::vector<double> hit_x;
  std::vector<double> hit_y;
  hit_x.reserve(cloud.width * cloud.height);
  hit_y.reserve(cloud.width * cloud.height);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      continue;
    }
    if (*iter_z < params_.min_z_m || *iter_z > params_.max_z_m) {
      continue;
    }

    const double dx = *iter_x - robot_x_;
    const double dy = *iter_y - robot_y_;
    const double bx = cos_yaw * dx + sin_yaw * dy;
    const double by = -sin_yaw * dx + cos_yaw * dy;

    if (bx < -params_.observe_extent_backward_m || bx > params_.observe_extent_forward_m) {
      continue;
    }
    if (by < -params_.observe_extent_right_m || by > params_.observe_extent_left_m) {
      continue;
    }

    const double range = std::hypot(bx, by);
    if (range > params_.max_range_m) {
      continue;
    }

    const bool inside_vehicle =
      bx >= params_.vehicle_rear_x_m && bx <= params_.vehicle_front_x_m &&
      by >= -params_.vehicle_half_width_m && by <= params_.vehicle_half_width_m;
    if (inside_vehicle) {
      continue;
    }

    hit_x.push_back(*iter_x);
    hit_y.push_back(*iter_y);
  }

  grid_->insertRangeData(
    robot_x_, robot_y_, hit_x, hit_y, timestamp, params_.insert_free_space);
}

nav_msgs::msg::OccupancyGrid RollingOccupancyMap::getOccupancyGrid(
  const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double current_time = stamp.seconds();
  return grid_->toOccupancyGrid(
    params_.map_frame, stamp, params_.stationary_retention_sec, robot_stationary_,
    current_time, params_.inflation_radius_m, params_.inscribed_radius_m,
    params_.track_unknown_space, params_.cost_scaling_factor);
}

nav2_msgs::msg::Costmap RollingOccupancyMap::getCostmap(const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double current_time = stamp.seconds();
  return grid_->toCostmap(
    params_.map_frame, stamp, params_.stationary_retention_sec, robot_stationary_,
    current_time, params_.costmap_layer, params_.inflation_radius_m, params_.inscribed_radius_m,
    params_.track_unknown_space, params_.cost_scaling_factor);
}

map_msgs::msg::OccupancyGridUpdate RollingOccupancyMap::getOccupancyGridUpdate(
  const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double current_time = stamp.seconds();
  return grid_->toOccupancyGridUpdate(
    params_.map_frame, stamp, params_.stationary_retention_sec, robot_stationary_,
    current_time, params_.inflation_radius_m, params_.inscribed_radius_m,
    params_.track_unknown_space, params_.cost_scaling_factor);
}

void RollingOccupancyMap::clearPendingUpdate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  grid_->clearPendingUpdate();
}

bool RollingOccupancyMap::hasOdometry() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_odom_;
}

bool RollingOccupancyMap::isRobotStationary() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return robot_stationary_;
}

void RollingOccupancyMap::setInflationParams(
  double inflation_radius_m, double cost_scaling_factor)
{
  std::lock_guard<std::mutex> lock(mutex_);
  params_.inflation_radius_m = inflation_radius_m;
  params_.cost_scaling_factor = cost_scaling_factor;
}

}  // namespace localmap_occupymap
