/**
 * @file lidar_perception.h
 * @brief 多路雷达感知适配层
 *
 * 职责（对应架构 PerceptionAdapter）：
 * 1. 订阅左/右/后 LaserScan 与左/右倾斜 PointCloud2
 * 2. TF 变换到 base_link，ROI 裁剪
 * 3. 双通道输出：
 *    - pending_points_  → RollingLocalMap 落格（takePendingPoints 取走）
 *    - obstacle_points_ → 体素化 ObstacleField（Footprint 快路径）
 *
 * 与 ObstacleField 关系：双通道不互相覆盖，LocalMap 负责空间记忆，primitive 负责几何扫掠。
 */
#pragma once

#include "safety_node/core/avoidance_types.h"

#include <array>
#include <cstdint>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>
#include <tuple>
#include <vector>

struct PerceptionDebugPoint
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  uint8_t source_id = 0;
};

struct PerceptionHealth
{
  size_t fresh_source_count = 0;
  size_t received_source_count = 0;
  size_t fresh_horizontal_source_count = 0;
  size_t received_horizontal_source_count = 0;
  size_t fresh_tilt_source_count = 0;
  size_t received_tilt_source_count = 0;
};

struct PerceptionCacheStats
{
  size_t pending_points = 0;
  size_t obstacle_points = 0;
  size_t debug_points = 0;
};

/** 单路斜雷达调试快照（base_link） */
struct TiltSourceSnapshot
{
  std::string source_name;
  uint8_t source_id = 0;
  std::vector<ObstaclePoint> filtered_points;
  ObstacleField primitive_field;
};

class LidarPerception {
public:
  explicit LidarPerception(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);

  /** 取出并清空本周期待融合点云（供 LocalMap 10Hz 线程） */
  std::vector < std::tuple < double, double, double >> takePendingPoints();

  /** 从累积点云构建当前 ObstacleField（供 20Hz 控制环） */
  ObstacleField buildObstacleField(
    const AvoidanceParams & params,
    const rclcpp::Time & stamp) const;
  ObstacleField buildHorizontalObstacleField(
    const AvoidanceParams & params,
    const rclcpp::Time & stamp) const;
  ObstacleField buildTiltObstacleField(
    const AvoidanceParams & params,
    const rclcpp::Time & stamp) const;

  std::vector < PerceptionDebugPoint > takeDebugPoints();

  PerceptionHealth health(const rclcpp::Time & now, double timeout_s) const;
  PerceptionCacheStats cacheStats() const;

  /** 按 left_tilt / right_tilt 返回过滤点与体素 primitive，供 TiltObstacleScan 调试发布 */
  std::vector<TiltSourceSnapshot> snapshotTiltSources(
    const AvoidanceParams & params,
    const rclcpp::Time & stamp) const;

  /** 模式切换时清空缓存 */
  void clear();

private:
  bool acceptSourceMessage(
    const std::string & source_name,
    const builtin_interfaces::msg::Time & stamp);
  void markSourceTransformValid(
    const std::string & source_name,
    const rclcpp::Time & stamp);

  void processScan(
    const sensor_msgs::msg::LaserScan::SharedPtr & msg,
    const std::string & source_frame,
    const std::string & source_name);

  void processCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
    const std::string & default_frame,
    const std::string & source_name);

  void processTiltScanPoints(
    const builtin_interfaces::msg::Time & stamp,
    const std::string & source_name,
    const std::vector<std::tuple<double, double, double>> & base_link_points);

  bool acceptTiltBaseLinkPoint(double x, double y, double z) const;

  void appendBaseLinkPoints(
    const std::vector < std::tuple < double, double, double >> & pts,
    const std::string & source_name);
  bool rejectBaseLinkPoint(double x, double y, double z, bool cloud_source) const;

  bool lookupTransform(
    const std::string & source_frame,
    const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & transform) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr < tf2_ros::Buffer > tf_buffer_;
  std::shared_ptr < tf2_ros::TransformListener > tf_listener_;

  // 必须持有 shared_ptr，否则 create_subscription 临时对象析构后订阅会被销毁
  rclcpp::Subscription < sensor_msgs::msg::LaserScan > ::SharedPtr left_scan_sub_;
  rclcpp::Subscription < sensor_msgs::msg::LaserScan > ::SharedPtr right_scan_sub_;
  rclcpp::Subscription < sensor_msgs::msg::LaserScan > ::SharedPtr back_scan_sub_;
  rclcpp::Subscription < sensor_msgs::msg::PointCloud2 > ::SharedPtr left_tilt_cloud_sub_;
  rclcpp::Subscription < sensor_msgs::msg::PointCloud2 > ::SharedPtr right_tilt_cloud_sub_;
  rclcpp::SubscriptionBase::SharedPtr left_tilt_scan_sub_;
  rclcpp::SubscriptionBase::SharedPtr right_tilt_scan_sub_;

  double tilt_z_threshold_ = 0.27;    ///< 倾斜雷达 z 过滤阈值
  std::string tilt_input_type_ = "pointcloud2";

  bool debug_enabled_ = false;
  size_t debug_max_points_ = 30000;
  size_t max_points_per_source_ = 30000;
  size_t max_pending_points_ = 60000;
  bool local_map_enabled_ = false;
  bool tilt_enabled_ = true;
  bool tilt_feed_local_map_ = false;
  double tilt_max_range_m_ = 1.0;
  double tilt_min_forward_x_m_ = 0.0;
  double tilt_roi_x_min_m_ = -0.3;
  double tilt_roi_x_max_m_ = 1.0;
  double tilt_roi_y_abs_m_ = 0.45;
  double tilt_voxel_size_m_ = 0.08;
  size_t tilt_max_primitives_ = 48;
  double tf_lookup_timeout_s_ = 0.02;
  double perception_timeout_s_ = 0.35;
  bool self_filter_enabled_ = true;
  double self_filter_x_min_ = -0.36;
  double self_filter_x_max_ = 0.95;
  double self_filter_y_abs_ = 0.36;
  double self_filter_z_min_ = -0.15;
  double self_filter_z_max_ = 0.35;
  double mount_edge_reject_radius_m_ = 0.05;
  double ground_reject_below_z_m_ = -0.06;

  mutable std::mutex mutex_;
  std::vector < std::tuple < double, double, double >> pending_points_; ///< LocalMap 用
  std::vector < std::tuple < double, double, double >> obstacle_points_; ///< Footprint 用
  std::vector < PerceptionDebugPoint > debug_points_;
  std::vector < ObstaclePoint > horizontal_obstacle_points_;
  std::vector < ObstaclePoint > tilt_obstacle_points_;
  std::array < std::vector < std::tuple < double, double, double >>, 5 >
  latest_source_points_;
  std::array < rclcpp::Time, 5 > last_source_message_;
  std::array < rclcpp::Time, 5 > last_source_transform_;
  std::array < bool, 5 > source_received_ {{false, false, false, false, false}};
  std::array < bool, 5 > source_transform_valid_ {{false, false, false, false, false}};
};
