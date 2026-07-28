// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#ifndef LOCALMAP_OCCUPYMAP__OCCUPYMAP_NODE_HPP_
#define LOCALMAP_OCCUPYMAP__OCCUPYMAP_NODE_HPP_

#include <memory>
#include <string>

#include "localmap_occupymap/rolling_occupancy_map.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace localmap_occupymap
{

class OccupymapNode : public rclcpp::Node
{
public:
  explicit OccupymapNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onObstaclePoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishMap(const rclcpp::Time & stamp);
  bool transformCloudToMap(
    const sensor_msgs::msg::PointCloud2 & input,
    sensor_msgs::msg::PointCloud2 & output);

  OccupymapParams loadParams();

  std::unique_ptr<RollingOccupancyMap> map_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr raw_map_pub_;
  rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr map_update_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string odom_topic_;
  std::string cloud_topic_;
  std::string output_topic_;
  std::string output_raw_topic_;
  std::string output_update_topic_;
};

}  // namespace localmap_occupymap

#endif  // LOCALMAP_OCCUPYMAP__OCCUPYMAP_NODE_HPP_
