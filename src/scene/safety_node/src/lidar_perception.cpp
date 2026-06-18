/**
 * @file lidar_perception.cpp
 * @brief 雷达订阅、TF、ROI 与双通道点云缓存
 */
#include "safety_node/lidar_perception.h"

#include "safety_node/core/obstacle_field.h"
#include "safety_node/perception_utils.h"

#include "../../../common/utils/topic.h"

#include <interface/msg/tilt_obstacle_scan.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>

namespace
{
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

bool isTiltSourceId(uint8_t source_id)
{
  return source_id == PerceptionUtils::sourceId("left_tilt") ||
         source_id == PerceptionUtils::sourceId("right_tilt");
}

std::string sourceNameFromId(uint8_t source_id)
{
  switch (source_id) {
    case 1:
      return "right";
    case 2:
      return "back";
    case 3:
      return "left_tilt";
    case 4:
      return "right_tilt";
    case 0:
    default:
      return "left";
  }
}
} // namespace

LidarPerception::LidarPerception(
  const rclcpp::Node::SharedPtr & node,
  rclcpp::CallbackGroup::SharedPtr callback_group)
: node_(node)
{
  // 参数由 SafetyNode::declareAndLoadParams() 统一 declare，此处只读取
  tilt_z_threshold_ = node_->get_parameter("tilt_z_filter_threshold").as_double();
  debug_enabled_ = node_->get_parameter("debug.perception.enabled").as_bool();
  debug_max_points_ = static_cast<size_t>(std::max<int64_t>(
      1, node_->get_parameter("debug.perception.max_points").as_int()));
  max_points_per_source_ = static_cast<size_t>(std::max<int64_t>(
      1, node_->get_parameter("perception.max_points_per_source").as_int()));
  max_pending_points_ = static_cast<size_t>(std::max<int64_t>(
      1, node_->get_parameter("perception.max_pending_points").as_int()));
  local_map_enabled_ = node_->get_parameter("local_map.enabled").as_bool();
  auto get_tilt_b = [&](const char * name, bool fallback) {
      if (!node_->has_parameter(name)) {
        node_->declare_parameter<bool>(name, fallback);
      }
      return node_->get_parameter(name).as_bool();
    };
  auto get_tilt_d = [&](const char * name, double fallback) {
      if (!node_->has_parameter(name)) {
        node_->declare_parameter<double>(name, fallback);
      }
      return node_->get_parameter(name).as_double();
    };
  auto get_tilt_i = [&](const char * name, int64_t fallback) {
      if (!node_->has_parameter(name)) {
        node_->declare_parameter<int64_t>(name, fallback);
      }
      return node_->get_parameter(name).as_int();
    };
  auto get_tilt_s = [&](const char * name, const std::string & fallback) {
      if (!node_->has_parameter(name)) {
        node_->declare_parameter<std::string>(name, fallback);
      }
      return node_->get_parameter(name).as_string();
    };
  tilt_enabled_ = get_tilt_b("perception.tilt.enabled", true);
  tilt_input_type_ = get_tilt_s("perception.tilt.input_type", "pointcloud2");
  tilt_feed_local_map_ = get_tilt_b("perception.tilt.feed_local_map", false);
  tilt_max_range_m_ = get_tilt_d("perception.tilt.max_range_m", 1.0);
  tilt_min_forward_x_m_ = get_tilt_d("perception.tilt.min_forward_x_m", 0.0);
  tilt_roi_x_min_m_ = get_tilt_d("perception.tilt.roi_x_min_m", -0.3);
  tilt_roi_x_max_m_ = get_tilt_d("perception.tilt.roi_x_max_m", 1.0);
  tilt_roi_y_abs_m_ = get_tilt_d("perception.tilt.roi_y_abs_m", 0.45);
  tilt_z_threshold_ = get_tilt_d("perception.tilt.z_filter_threshold", 0.0);
  tilt_voxel_size_m_ = get_tilt_d("perception.tilt.voxel_size_m", 0.08);
  tilt_max_primitives_ = static_cast<size_t>(std::max<int64_t>(
      1, get_tilt_i("perception.tilt.max_primitives", 48)));
  tf_lookup_timeout_s_ = node_->get_parameter("input_health.tf_lookup_timeout_s").as_double();
  perception_timeout_s_ =
    node_->get_parameter("input_health.perception_timeout_s").as_double();
  self_filter_enabled_ = node_->get_parameter("perception.self_filter.enabled").as_bool();
  self_filter_x_min_ = node_->get_parameter("perception.self_filter.x_min_m").as_double();
  self_filter_x_max_ = node_->get_parameter("perception.self_filter.x_max_m").as_double();
  self_filter_y_abs_ = node_->get_parameter("perception.self_filter.y_abs_m").as_double();
  self_filter_z_min_ = node_->get_parameter("perception.self_filter.z_min_m").as_double();
  self_filter_z_max_ = node_->get_parameter("perception.self_filter.z_max_m").as_double();
  mount_edge_reject_radius_m_ =
    node_->get_parameter("perception.mount_edge_reject_radius_m").as_double();
  ground_reject_below_z_m_ =
    node_->get_parameter("perception.ground_reject_below_z_m").as_double();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group;

  // ── 水平雷达：LaserScan → 点 → base_link ──
  left_scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    leftLaserTopic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      if (!msg || !acceptSourceMessage("left", msg->header.stamp)) {
        return;
      }
      processScan(msg, "laser_frame_left", "left");
    }, sub_options);
  right_scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    rightLaserTopic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      if (!msg || !acceptSourceMessage("right", msg->header.stamp)) {
        return;
      }
      processScan(msg, "laser_frame_right", "right");
    }, sub_options);
  back_scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    backLaserTopic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      if (!msg || !acceptSourceMessage("back", msg->header.stamp)) {
        return;
      }
      processScan(msg, "laser_frame_back", "back");
    }, sub_options);

  // lidar_obstacle_groundmodel 以 RELIABLE 发布，需匹配 QoS
  const rclcpp::QoS tilt_cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  const rclcpp::QoS tilt_scan_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  const bool use_tilt_scan_input = (tilt_input_type_ == "tilt_scan");

  if (use_tilt_scan_input) {
    left_tilt_scan_sub_ = node_->create_subscription<interface::msg::TiltObstacleScan>(
      leftTiltScanTopic, tilt_scan_qos,
      [this](const interface::msg::TiltObstacleScan::SharedPtr msg) {
        if (!msg || !acceptSourceMessage("left_tilt", msg->header.stamp)) {
          return;
        }
        if (!tilt_enabled_) {
          return;
        }
        std::vector<std::tuple<double, double, double>> batch;
        batch.reserve(msg->filtered_points.size());
        for (const auto & pt : msg->filtered_points) {
          if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
          }
          batch.emplace_back(pt.x, pt.y, pt.z);
        }
        processTiltScanPoints(msg->header.stamp, "left_tilt", batch);
      }, sub_options);
    right_tilt_scan_sub_ = node_->create_subscription<interface::msg::TiltObstacleScan>(
      rightTiltScanTopic, tilt_scan_qos,
      [this](const interface::msg::TiltObstacleScan::SharedPtr msg) {
        if (!msg || !acceptSourceMessage("right_tilt", msg->header.stamp)) {
          return;
        }
        if (!tilt_enabled_) {
          return;
        }
        std::vector<std::tuple<double, double, double>> batch;
        batch.reserve(msg->filtered_points.size());
        for (const auto & pt : msg->filtered_points) {
          if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
          }
          batch.emplace_back(pt.x, pt.y, pt.z);
        }
        processTiltScanPoints(msg->header.stamp, "right_tilt", batch);
      }, sub_options);
  } else {
    // ── 倾斜雷达：PointCloud2（默认，兼容现有 bag / groundmodel） ──
    left_tilt_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      leftTiltPointTopice, tilt_cloud_qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!msg || !acceptSourceMessage("left_tilt", msg->header.stamp)) {
          return;
        }
        if (tilt_enabled_) {
          processCloud(msg, "left_tilt_rotate", "left_tilt");
        }
      }, sub_options);
    right_tilt_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      rightTiltPointTopice, tilt_cloud_qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!msg || !acceptSourceMessage("right_tilt", msg->header.stamp)) {
          return;
        }
        if (tilt_enabled_) {
          processCloud(msg, "right_tilt_rotate", "right_tilt");
        }
      }, sub_options);
  }
}

bool LidarPerception::acceptSourceMessage(
  const std::string & source_name,
  const builtin_interfaces::msg::Time & stamp)
{
  const size_t id = static_cast<size_t>(PerceptionUtils::sourceId(source_name));
  const rclcpp::Time message_stamp(stamp, node_->get_clock()->get_clock_type());
  const rclcpp::Time now = node_->now();
  const double age_s = (now - message_stamp).seconds();
  std::lock_guard<std::mutex> lock(mutex_);
  source_received_[id] = true;

  if (message_stamp.nanoseconds() <= 0 ||
    !std::isfinite(age_s) ||
    age_s < -0.05)
  {
    return false;
  }
  if (last_source_message_[id].nanoseconds() > 0 &&
    message_stamp <= last_source_message_[id])
  {
    return false;
  }

  last_source_message_[id] = message_stamp;
  return perception_timeout_s_ <= 0.0 || age_s <= perception_timeout_s_;
}

bool LidarPerception::rejectBaseLinkPoint(
  double x, double y, double z, bool cloud_source) const
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return true;
  }
  if (mount_edge_reject_radius_m_ > 0.0 &&
    std::hypot(x, y) <= mount_edge_reject_radius_m_)
  {
    return true;
  }
  if (cloud_source && z < ground_reject_below_z_m_) {
    return true;
  }
  if (!self_filter_enabled_) {
    return false;
  }
  return x >= self_filter_x_min_ && x <= self_filter_x_max_ &&
         std::abs(y) <= self_filter_y_abs_ &&
         z >= self_filter_z_min_ && z <= self_filter_z_max_;
}

void LidarPerception::markSourceTransformValid(
  const std::string & source_name,
  const rclcpp::Time & stamp)
{
  const size_t id = static_cast<size_t>(PerceptionUtils::sourceId(source_name));
  std::lock_guard<std::mutex> lock(mutex_);
  last_source_transform_[id] = stamp;
  source_transform_valid_[id] = true;
}

bool LidarPerception::lookupTransform(
  const std::string & source_frame,
  const rclcpp::Time & stamp,
  geometry_msgs::msg::TransformStamped & transform) const
{
  try {
    transform = tf_buffer_->lookupTransform(
      "base_link", source_frame, stamp, tf2::durationFromSec(tf_lookup_timeout_s_));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "TF failed (%s -> base_link): %s", source_frame.c_str(), ex.what());
    return false;
  }
}

void LidarPerception::appendBaseLinkPoints(
  const std::vector<std::tuple<double, double, double>> & pts,
  const std::string & source_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const uint8_t id = PerceptionUtils::sourceId(source_name);
  const size_t source_id = static_cast<size_t>(id);
  const bool tilt_source = isTiltSourceId(id);
  auto & source_points = latest_source_points_[source_id];
  if (pts.size() > max_points_per_source_) {
    source_points.assign(
      pts.end() - static_cast<std::ptrdiff_t>(max_points_per_source_), pts.end());
  } else {
    source_points = pts;
  }

  obstacle_points_.clear();
  horizontal_obstacle_points_.clear();
  tilt_obstacle_points_.clear();
  for (size_t sid = 0; sid < latest_source_points_.size(); ++sid) {
    const uint8_t cached_id = static_cast<uint8_t>(sid);
    const bool cached_tilt = isTiltSourceId(cached_id);
    auto & target = cached_tilt ? tilt_obstacle_points_ : horizontal_obstacle_points_;
    for (const auto & latest : latest_source_points_[sid]) {
      ObstaclePoint point;
      point.x = std::get<0>(latest);
      point.y = std::get<1>(latest);
      point.z = std::get<2>(latest);
      point.source = sourceNameFromId(cached_id);
      target.push_back(point);
      if (!cached_tilt) {
        obstacle_points_.push_back(latest);
      }
    }
  }

  if (local_map_enabled_ && (!tilt_source || tilt_feed_local_map_)) {
    pending_points_.insert(pending_points_.end(), pts.begin(), pts.end());
    if (pending_points_.size() > max_pending_points_) {
      pending_points_.erase(
        pending_points_.begin(),
        pending_points_.begin() +
        static_cast<std::ptrdiff_t>(pending_points_.size() - max_pending_points_));
    }
  }

  if (!debug_enabled_) {
    return;
  }

  if (pts.size() >= debug_max_points_) {
    debug_points_.clear();
    debug_points_.reserve(debug_max_points_);
    const auto first = pts.end() - static_cast<std::ptrdiff_t>(debug_max_points_);
    for (auto it = first; it != pts.end(); ++it) {
      debug_points_.push_back(
      {
        static_cast<float>(std::get<0>(*it)),
        static_cast<float>(std::get<1>(*it)),
        static_cast<float>(std::get<2>(*it)),
        id});
    }
    return;
  }

  const size_t required = debug_points_.size() + pts.size();
  if (required > debug_max_points_) {
    const size_t remove_count = required - debug_max_points_;
    debug_points_.erase(
      debug_points_.begin(),
      debug_points_.begin() + static_cast<std::ptrdiff_t>(remove_count));
  }
  debug_points_.reserve(std::min(debug_max_points_, required));
  for (const auto & pt : pts) {
    debug_points_.push_back(
    {
      static_cast<float>(std::get<0>(pt)),
      static_cast<float>(std::get<1>(pt)),
      static_cast<float>(std::get<2>(pt)),
      id});
  }
}

void LidarPerception::processScan(
  const sensor_msgs::msg::LaserScan::SharedPtr & msg,
  const std::string & source_frame,
  const std::string & source_name)
{
  if (!msg) {
    return;
  }

  const rclcpp::Time stamp(
    msg->header.stamp, node_->get_clock()->get_clock_type());
  const std::string frame =
    msg->header.frame_id.empty() ? source_frame : msg->header.frame_id;
  geometry_msgs::msg::TransformStamped transform;
  if (!lookupTransform(frame, stamp, transform)) {
    return;
  }
  markSourceTransformValid(source_name, stamp);

  if (msg->ranges.empty()) {
    appendBaseLinkPoints({}, source_name);
    return;
  }

  std::vector<std::tuple<double, double, double>> batch;
  batch.reserve(msg->ranges.size());

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const float range = msg->ranges[i];
    if (!PerceptionUtils::validRange(range, msg->range_min, msg->range_max)) {
      continue;
    }
    const float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
    const auto [lx, ly] = PerceptionUtils::polarToCartesian(range, angle);
    geometry_msgs::msg::PointStamped in;
    geometry_msgs::msg::PointStamped out;
    in.point.x = lx;
    in.point.y = ly;
    tf2::doTransform(in, out, transform);
    if (rejectBaseLinkPoint(out.point.x, out.point.y, out.point.z, false)) {
      continue;
    }
    if (!PerceptionUtils::inRoi(out.point.x, out.point.y)) {
      continue;
    }
    batch.emplace_back(out.point.x, out.point.y, out.point.z);
  }

  appendBaseLinkPoints(batch, source_name);
}

void LidarPerception::processCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
  const std::string & default_frame,
  const std::string & source_name)
{
  if (!msg) {
    return;
  }

  const std::string source_frame =
    msg->header.frame_id.empty() ? default_frame : msg->header.frame_id;
  const rclcpp::Time stamp(
    msg->header.stamp, node_->get_clock()->get_clock_type());
  geometry_msgs::msg::TransformStamped transform;
  if (!lookupTransform(source_frame, stamp, transform)) {
    return;
  }
  markSourceTransformValid(source_name, stamp);

  if (msg->data.empty()) {
    appendBaseLinkPoints({}, source_name);
    return;
  }

  PointCloud cloud;
  pcl::fromROSMsg(*msg, cloud);

  std::vector<std::tuple<double, double, double>> batch;
  batch.reserve(cloud.points.size());

  for (const auto & pt : cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    geometry_msgs::msg::PointStamped in;
    geometry_msgs::msg::PointStamped out;
    in.point.x = pt.x;
    in.point.y = pt.y;
    in.point.z = pt.z;
    tf2::doTransform(in, out, transform);
    if (!PerceptionUtils::validTiltZ(out.point.z, tilt_z_threshold_)) {
      continue;
    }
    if (rejectBaseLinkPoint(out.point.x, out.point.y, out.point.z, true)) {
      continue;
    }
    if (!acceptTiltBaseLinkPoint(out.point.x, out.point.y, out.point.z)) {
      continue;
    }
    batch.emplace_back(out.point.x, out.point.y, out.point.z);
  }

  appendBaseLinkPoints(batch, source_name);
}

bool LidarPerception::acceptTiltBaseLinkPoint(double x, double y, double z) const
{
  if (!PerceptionUtils::validTiltZ(z, tilt_z_threshold_)) {
    return false;
  }
  if (x < tilt_min_forward_x_m_ ||
    x < tilt_roi_x_min_m_ ||
    x > tilt_roi_x_max_m_ ||
    std::abs(y) > tilt_roi_y_abs_m_ ||
    (tilt_max_range_m_ > 0.0 && std::hypot(x, y) > tilt_max_range_m_))
  {
    return false;
  }
  return PerceptionUtils::inRoi(x, y);
}

void LidarPerception::processTiltScanPoints(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & source_name,
  const std::vector<std::tuple<double, double, double>> & base_link_points)
{
  const rclcpp::Time message_stamp(stamp, node_->get_clock()->get_clock_type());
  markSourceTransformValid(source_name, message_stamp);

  std::vector<std::tuple<double, double, double>> batch;
  batch.reserve(base_link_points.size());
  for (const auto & pt : base_link_points) {
    const double x = std::get<0>(pt);
    const double y = std::get<1>(pt);
    const double z = std::get<2>(pt);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (rejectBaseLinkPoint(x, y, z, true)) {
      continue;
    }
    if (!acceptTiltBaseLinkPoint(x, y, z)) {
      continue;
    }
    batch.emplace_back(x, y, z);
  }
  appendBaseLinkPoints(batch, source_name);
}

std::vector<std::tuple<double, double, double>> LidarPerception::takePendingPoints()
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto out = std::move(pending_points_);
  pending_points_.clear();
  return out;
}

ObstacleField LidarPerception::buildObstacleField(
  const AvoidanceParams & params,
  const rclcpp::Time & stamp) const
{
  return buildHorizontalObstacleField(params, stamp);
}

ObstacleField LidarPerception::buildHorizontalObstacleField(
  const AvoidanceParams & params,
  const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return ObstacleFieldBuilder::buildFromPoints(
    horizontal_obstacle_points_, stamp, params, 0.12, params.max_obstacle_primitives);
}

ObstacleField LidarPerception::buildTiltObstacleField(
  const AvoidanceParams & params,
  const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  AvoidanceParams tilt_params = params;
  tilt_params.footprint_obstacle_radius_default =
    std::max(0.0, params.tilt_ttc.obstacle_radius_m);
  return ObstacleFieldBuilder::buildFromPoints(
    tilt_obstacle_points_,
    stamp,
    tilt_params,
    tilt_voxel_size_m_,
    tilt_max_primitives_);
}

std::vector<PerceptionDebugPoint> LidarPerception::takeDebugPoints()
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto out = std::move(debug_points_);
  debug_points_.clear();
  return out;
}

PerceptionHealth LidarPerception::health(const rclcpp::Time & now, double timeout_s) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  PerceptionHealth result;
  for (size_t i = 0; i < source_received_.size(); ++i) {
    if (!source_received_[i]) {
      continue;
    }
    ++result.received_source_count;
    const bool tilt_source = isTiltSourceId(static_cast<uint8_t>(i));
    if (tilt_source) {
      ++result.received_tilt_source_count;
    } else {
      ++result.received_horizontal_source_count;
    }
    if (!source_transform_valid_[i]) {
      continue;
    }
    const double message_age_s = (now - last_source_message_[i]).seconds();
    const double transform_age_s = (now - last_source_transform_[i]).seconds();
    if (timeout_s <= 0.0 ||
      (message_age_s >= -0.05 && message_age_s <= timeout_s &&
      transform_age_s >= -0.05 && transform_age_s <= timeout_s))
    {
      ++result.fresh_source_count;
      if (tilt_source) {
        ++result.fresh_tilt_source_count;
      } else {
        ++result.fresh_horizontal_source_count;
      }
    }
  }
  return result;
}

PerceptionCacheStats LidarPerception::cacheStats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {
    pending_points_.size(),
    horizontal_obstacle_points_.size() + tilt_obstacle_points_.size(),
    debug_points_.size()};
}

std::vector<TiltSourceSnapshot> LidarPerception::snapshotTiltSources(
  const AvoidanceParams & params,
  const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TiltSourceSnapshot> snapshots;
  const std::array<std::string, 2> source_names = {"left_tilt", "right_tilt"};
  AvoidanceParams tilt_params = params;
  tilt_params.footprint_obstacle_radius_default =
    std::max(0.0, params.tilt_ttc.obstacle_radius_m);

  for (const auto & source_name : source_names) {
    TiltSourceSnapshot snapshot;
    snapshot.source_name = source_name;
    snapshot.source_id = PerceptionUtils::sourceId(source_name);
    for (const auto & point : tilt_obstacle_points_) {
      if (point.source == source_name) {
        snapshot.filtered_points.push_back(point);
      }
    }
    snapshot.primitive_field = ObstacleFieldBuilder::buildFromPoints(
      snapshot.filtered_points,
      stamp,
      tilt_params,
      tilt_voxel_size_m_,
      tilt_max_primitives_);
    snapshots.push_back(std::move(snapshot));
  }
  return snapshots;
}

void LidarPerception::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_points_.clear();
  obstacle_points_.clear();
  horizontal_obstacle_points_.clear();
  tilt_obstacle_points_.clear();
  debug_points_.clear();
  for (auto & points : latest_source_points_) {
    points.clear();
  }
  source_received_.fill(false);
  source_transform_valid_.fill(false);
}
