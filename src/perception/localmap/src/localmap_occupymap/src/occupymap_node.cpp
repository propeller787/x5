// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "localmap_occupymap/occupymap_node.hpp"

#include <cmath>
#include <memory>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace localmap_occupymap
{

OccupymapNode::OccupymapNode(const rclcpp::NodeOptions & options)
: Node("occupymap_node", options)
{
  const auto params = loadParams();
  map_ = std::make_unique<RollingOccupancyMap>(params);

  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom/fused");
  cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/perception/obstacle_points");
  output_topic_ = declare_parameter<std::string>("output_topic", "/localmap/occupymap");
  output_raw_topic_ = declare_parameter<std::string>("output_raw_topic", "/localmap/occupymap_raw");
  output_update_topic_ = declare_parameter<std::string>(
    "output_update_topic", "/localmap/occupymap_updates");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  auto cloud_qos = rclcpp::SensorDataQoS().keep_last(5);
  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, odom_qos,
    std::bind(&OccupymapNode::onOdometry, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, cloud_qos,
    std::bind(&OccupymapNode::onObstaclePoints, this, std::placeholders::_1));

  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, costmap_qos);
  raw_map_pub_ = create_publisher<nav2_msgs::msg::Costmap>(output_raw_topic_, costmap_qos);
  map_update_pub_ = create_publisher<map_msgs::msg::OccupancyGridUpdate>(
    output_update_topic_, costmap_qos);

  RCLCPP_INFO(
    get_logger(),
    "occupymap_node started: odom=%s cloud=%s output=%s raw=%s updates=%s frame=%s "
    "extent=front%.1f/back%.1f/left%.1f/right%.1fm res=%.2fm retention=%.0fs publish=cloud",
    odom_topic_.c_str(), cloud_topic_.c_str(), output_topic_.c_str(), output_raw_topic_.c_str(),
    output_update_topic_.c_str(), params.map_frame.c_str(), params.map_extent_forward_m, params.map_extent_backward_m,
    params.map_extent_left_m, params.map_extent_right_m, params.resolution_m,
    params.stationary_retention_sec);

  param_cb_handle_ = add_on_set_parameters_callback(
    std::bind(&OccupymapNode::onSetParameters, this, std::placeholders::_1));
}

OccupymapParams OccupymapNode::loadParams()
{
  OccupymapParams params;
  params.map_extent_forward_m = declare_parameter<double>("map_extent_forward_m", 4.0);
  params.map_extent_backward_m = declare_parameter<double>("map_extent_backward_m", 4.0);
  params.map_extent_left_m = declare_parameter<double>("map_extent_left_m", 4.0);
  params.map_extent_right_m = declare_parameter<double>("map_extent_right_m", 4.0);
  params.observe_extent_forward_m =
    declare_parameter<double>("observe_extent_forward_m", params.map_extent_forward_m);
  params.observe_extent_backward_m =
    declare_parameter<double>("observe_extent_backward_m", params.map_extent_backward_m);
  params.observe_extent_left_m = declare_parameter<double>("observe_extent_left_m", 3.0);
  params.observe_extent_right_m = declare_parameter<double>("observe_extent_right_m", 3.0);
  params.resolution_m = declare_parameter<double>("resolution_m", 0.05);
  params.hit_probability = declare_parameter<double>("hit_probability", 0.55);
  params.miss_probability = declare_parameter<double>("miss_probability", 0.49);
  params.stationary_linear_thresh_mps =
    declare_parameter<double>("stationary_linear_thresh_mps", 0.05);
  params.stationary_angular_thresh_rps =
    declare_parameter<double>("stationary_angular_thresh_rps", 0.05);
  params.stationary_retention_sec = declare_parameter<double>("stationary_retention_sec", 10.0);
  params.min_z_m = declare_parameter<double>("min_z_m", -0.5);
  params.max_z_m = declare_parameter<double>("max_z_m", 2.0);
  params.max_range_m = declare_parameter<double>("max_range_m", 30.0);
  params.insert_free_space = declare_parameter<bool>("insert_free_space", true);
  params.map_frame = declare_parameter<std::string>("map_frame", "odom");
  params.robot_base_frame = declare_parameter<std::string>("robot_base_frame", "base_link");
  params.costmap_layer = declare_parameter<std::string>("costmap_layer", "master");
  params.inflation_radius_m = declare_parameter<double>("inflation_radius_m", 0.55);
  params.inscribed_radius_m = declare_parameter<double>("inscribed_radius_m", 0.35);
  params.cost_scaling_factor = declare_parameter<double>("cost_scaling_factor", 10.0);
  params.vehicle_front_x_m = declare_parameter<double>("vehicle_front_x_m", 0.9);
  params.vehicle_rear_x_m = declare_parameter<double>("vehicle_rear_x_m", -0.1);
  params.vehicle_half_width_m = declare_parameter<double>("vehicle_half_width_m", 0.5);
  params.blind_zone_retention_sec =
    declare_parameter<double>("blind_zone_retention_sec", 10.0);
  params.track_unknown_space = declare_parameter<bool>("track_unknown_space", false);
  return params;
}

rcl_interfaces::msg::SetParametersResult OccupymapNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  double radius = -1.0;
  double scaling = -1.0;
  for (const auto & p : parameters) {
    if (p.get_name() == "inflation_radius_m") {
      radius = p.as_double();
    } else if (p.get_name() == "cost_scaling_factor") {
      scaling = p.as_double();
    }
  }
  if (radius >= 0.0 || scaling >= 0.0) {
    if (radius < 0.0) {
      radius = map_->inflationRadiusM();
    }
    if (scaling < 0.0) {
      scaling = map_->costScalingFactor();
    }
    map_->setInflationParams(radius, scaling);
    RCLCPP_INFO(
      get_logger(), "occupymap inflation updated: radius=%.3f scale=%.1f",
      radius, scaling);
  }
  return result;
}

void OccupymapNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
  map_->updateOdometry(*msg);
}

bool OccupymapNode::transformCloudToMap(
  const sensor_msgs::msg::PointCloud2 & input,
  sensor_msgs::msg::PointCloud2 & output)
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      map_->mapFrame(), input.header.frame_id, input.header.stamp,
      rclcpp::Duration::from_seconds(0.1));
    tf2::doTransform(input, output, transform);
    output.header.frame_id = map_->mapFrame();
    return true;
  } catch (const tf2::TransformException & ex) {
    if (!latest_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TF transform failed: %s", ex.what());
      return false;
    }

    if (input.header.frame_id == map_->mapFrame()) {
      output = input;
      return true;
    }

    // Fallback: transform robot_base_frame points to map_frame via fused odometry.
    const auto & pose = latest_odom_->pose.pose;
    tf2::Quaternion q(
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    output = input;
    output.header.frame_id = map_->mapFrame();

    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");
    sensor_msgs::PointCloud2ConstIterator<float> in_x(input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(input, "z");

    for (; in_x != in_x.end(); ++in_x, ++in_y, ++in_z, ++out_x, ++out_y, ++out_z) {
      *out_x = pose.position.x + cos_yaw * *in_x - sin_yaw * *in_y;
      *out_y = pose.position.y + sin_yaw * *in_x + cos_yaw * *in_y;
      *out_z = *in_z;
    }
    return true;
  }
}

void OccupymapNode::onObstaclePoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!map_->hasOdometry()) {
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_in_map;
  if (!transformCloudToMap(*msg, cloud_in_map)) {
    return;
  }

  map_->insertPointCloud(cloud_in_map, map_->mapFrame());
  publishMap(rclcpp::Time(cloud_in_map.header.stamp));
}

void OccupymapNode::publishMap(const rclcpp::Time & stamp)
{
  if (!map_->hasOdometry()) {
    return;
  }
  map_pub_->publish(map_->getOccupancyGrid(stamp));
  raw_map_pub_->publish(map_->getCostmap(stamp));

  auto grid_update = map_->getOccupancyGridUpdate(stamp);
  if (!grid_update.data.empty()) {
    map_update_pub_->publish(grid_update);
    map_->clearPendingUpdate();
  }
}

}  // namespace localmap_occupymap
