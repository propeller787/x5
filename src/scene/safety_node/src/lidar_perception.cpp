/**
 * @file lidar_perception.cpp
 * @brief 雷达订阅、TF、ROI 与双通道点云缓存
 */
#include "safety_node/lidar_perception.h"

#include "safety_node/core/obstacle_field.h"
#include "safety_node/perception_utils.h"

#include "../../../common/utils/topic.h"

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

/**
 * 从 TransformStamped 提取 3×3 旋转矩阵 + 平移向量。
 * 避免 tf2::doTransform 逐点构造 PointStamped 临时对象 + 虚函数调用开销。
 * 四元数 (qx, qy, qz, qw) → 旋转矩阵，标准公式。
 */
struct AffineTransform
{
  double r00, r01, r02;
  double r10, r11, r12;
  double r20, r21, r22;
  double tx, ty, tz;

  explicit AffineTransform(const geometry_msgs::msg::TransformStamped & t)
  {
    const auto & q = t.transform.rotation;
    const double qx = q.x, qy = q.y, qz = q.z, qw = q.w;
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
    r01 = 2.0 * (qx * qy - qz * qw);
    r02 = 2.0 * (qx * qz + qy * qw);
    r10 = 2.0 * (qx * qy + qz * qw);
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
    r12 = 2.0 * (qy * qz - qx * qw);
    r20 = 2.0 * (qx * qz - qy * qw);
    r21 = 2.0 * (qy * qz + qx * qw);
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
    tx = t.transform.translation.x;
    ty = t.transform.translation.y;
    tz = t.transform.translation.z;
  }

  inline void apply(
    double in_x, double in_y, double in_z,
    double & out_x, double & out_y, double & out_z) const
  {
    out_x = r00 * in_x + r01 * in_y + r02 * in_z + tx;
    out_y = r10 * in_x + r11 * in_y + r12 * in_z + ty;
    out_z = r20 * in_x + r21 * in_y + r22 * in_z + tz;
  }
};
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
  tilt_enabled_ = get_tilt_b("perception.tilt.enabled", true);
  horizontal_enabled_ = get_tilt_b("perception.horizontal.enabled", true);
  horizontal_left_enabled_ = get_tilt_b("perception.horizontal.left_enabled", true);
  horizontal_right_enabled_ = get_tilt_b("perception.horizontal.right_enabled", true);
  horizontal_back_enabled_ = get_tilt_b("perception.horizontal.back_enabled", true);
  horizontal_feed_local_map_ = get_tilt_b("perception.horizontal.feed_local_map", true);
  tilt_feed_local_map_ = get_tilt_b("perception.tilt.feed_local_map", false);
  tilt_treat_as_horizontal_ = get_tilt_b("perception.tilt.treat_as_horizontal", false);
  tilt_ground_reject_enabled_ = get_tilt_b("perception.tilt.ground_reject_enabled", true);
  tilt_z_filter_enabled_ = get_tilt_b("perception.tilt.z_filter_enabled", false);
  tilt_preserve_sensor_z_ = get_tilt_b("perception.tilt.preserve_sensor_z", true);
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
  self_filter_side_margin_ = get_tilt_d("perception.self_filter.side_margin_m", 0.02);

  // 感知 ROI 边界参数化（默认 [-2.5,6.5]×[-1.8,1.8]）
  roi_x_min_ = get_tilt_d("perception.roi.x_min_m", -2.5);
  roi_x_max_ = get_tilt_d("perception.roi.x_max_m", 6.5);
  roi_y_min_ = get_tilt_d("perception.roi.y_min_m", -1.8);
  roi_y_max_ = get_tilt_d("perception.roi.y_max_m", 1.8);

  // 自车过滤沿真实车体轮廓收窄（含车头两条斜边），使斜边外的障碍不被误当自车滤除。
  // footprint.* 由 SafetyNode 统一 declare；单元测试可能只声明部分参数，故用 declare-if-absent。
  {
    AvoidanceParams footprint_params;
    footprint_params.robot_length = get_tilt_d("footprint.length_m", 0.88);
    const double width_rear =
      get_tilt_d("footprint.width_rear_m", get_tilt_d("footprint.width_m", 0.70));
    footprint_params.robot_width_rear_m = width_rear;
    footprint_params.robot_width_front_m = get_tilt_d("footprint.width_front_m", width_rear);
    footprint_params.robot_rear_offset = get_tilt_d("footprint.rear_offset_m", 0.22);
    footprint_params.footprint_use_dual_trapezoid =
      get_tilt_b("footprint.use_dual_trapezoid", false);
    if (footprint_params.footprint_use_dual_trapezoid) {
      const auto load_segment = [&](const char * prefix, FootprintSegmentParams & segment) {
          segment.length_m = get_tilt_d((std::string(prefix) + ".length_m").c_str(), 0.0);
          segment.width_front_m = get_tilt_d((std::string(prefix) + ".width_front_m").c_str(), 0.0);
          segment.width_rear_m = get_tilt_d((std::string(prefix) + ".width_rear_m").c_str(), 0.0);
        };
      load_segment("footprint.front_segment", footprint_params.footprint_front_segment);
      load_segment("footprint.rear_segment", footprint_params.footprint_rear_segment);
      footprint_params.robot_width_rear_m = footprint_params.footprint_rear_segment.width_rear_m;
      footprint_params.robot_width_front_m = footprint_params.footprint_front_segment.width_front_m;
    }
    self_filter_footprint_.emplace(footprint_params);
  }

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
      if (!isHorizontalSourceEnabled("left")) {
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
      if (!isHorizontalSourceEnabled("right")) {
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
      if (!isHorizontalSourceEnabled("back")) {
        return;
      }
      processScan(msg, "laser_frame_back", "back");
    }, sub_options);

  // lidar_obstacle_groundmodel 以 RELIABLE 发布，需匹配 QoS
  const rclcpp::QoS tilt_cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

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
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "[LidarPerception] %s: receive_time - data_time = %.3f ms",
    source_name.c_str(), age_s * 1000.0);
  return perception_timeout_s_ <= 0.0 || age_s <= perception_timeout_s_;
}

bool LidarPerception::isHorizontalSourceEnabled(const std::string & source_name) const
{
  if (!horizontal_enabled_) {
    return false;
  }
  if (source_name == "left") {
    return horizontal_left_enabled_;
  }
  if (source_name == "right") {
    return horizontal_right_enabled_;
  }
  if (source_name == "back") {
    return horizontal_back_enabled_;
  }
  return false;
}

bool LidarPerception::isSourceEnabledForHealth(uint8_t source_id) const
{
  if (isTiltSourceId(source_id)) {
    return tilt_enabled_;
  }
  return isHorizontalSourceEnabled(sourceNameFromId(source_id));
}

bool LidarPerception::rejectBaseLinkPoint(
  double x, double y, double z, bool cloud_source, bool for_tilt) const
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return true;
  }
  if (mount_edge_reject_radius_m_ > 0.0 &&
    std::hypot(x, y) <= mount_edge_reject_radius_m_)
  {
    return true;
  }
  if (cloud_source &&
    (!for_tilt || tilt_ground_reject_enabled_) &&
    z < ground_reject_below_z_m_)
  {
    return true;
  }
  if (!self_filter_enabled_) {
    return false;
  }
  if (x < self_filter_x_min_ || x > self_filter_x_max_ ||
    z < self_filter_z_min_ || z > self_filter_z_max_)
  {
    return false;
  }
  // 横向上限沿真实车体轮廓收窄：车头两段斜边处半宽随之减小，斜边外的障碍不再被误当自车。
  // 收窄后的上限不会超过 y_abs 盒体，故只会比原来滤除更少的点，不会新增盲区。
  double y_limit = self_filter_y_abs_;
  if (self_filter_footprint_) {
    const double body_limit =
      self_filter_footprint_->physicalHalfWidthAtX(x) + std::max(0.0, self_filter_side_margin_);
    y_limit = std::min(y_limit, body_limit);
  }
  return std::abs(y) <= y_limit;
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

bool LidarPerception::acceptBaseLinkObstaclePoint(
  double x, double y, double z,
  bool cloud_source,
  bool for_tilt) const
{
  if (rejectBaseLinkPoint(x, y, z, cloud_source, for_tilt)) {
    return false;
  }
  return PerceptionUtils::inRoi(x, y, roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_);
}

bool LidarPerception::acceptTiltObstacleInBaseLink(double x, double y, double z) const
{
  if (!std::isfinite(z)) {
    return false;
  }
  if (tilt_z_filter_enabled_ &&
    !PerceptionUtils::validTiltZ(z, tilt_z_threshold_))
  {
    return false;
  }
  // 斜雷达为 PointCloud2 来源（cloud_source=true），for_tilt=true 使
  // rejectBaseLinkPoint 的地面过滤分支在 tilt_ground_reject_enabled_ 时生效。
  return acceptBaseLinkObstaclePoint(x, y, z, true, true);
}

void LidarPerception::appendBaseLinkPoints(
  const std::vector<std::tuple<double, double, double>> & pts,
  const std::string & source_name,
  const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const uint8_t id = PerceptionUtils::sourceId(source_name);
  const size_t source_id = static_cast<size_t>(id);
  const bool tilt_source = isTiltSourceId(id);
  // 点集与其帧时间戳必须原子提交，避免控制线程读到“新时间戳 + 上一帧点集”。
  latest_source_points_stamp_[source_id] = stamp;
  latest_source_points_valid_[source_id] = true;
  auto & source_points = latest_source_points_[source_id];
  if (pts.size() > max_points_per_source_) {
    // 均匀降采样：等间隔取样，保留全角度覆盖。
    // 旧实现取尾部会丢弃起始角度范围的点，可能在车体前方制造盲区。
    source_points.clear();
    source_points.reserve(max_points_per_source_);
    const size_t stride = pts.size() / max_points_per_source_;
    for (size_t i = 0; i < pts.size() && source_points.size() < max_points_per_source_; i += stride) {
      source_points.push_back(pts[i]);
    }
  } else {
    source_points = pts;
  }

  horizontal_obstacle_points_.clear();
  tilt_obstacle_points_.clear();
  for (size_t sid = 0; sid < latest_source_points_.size(); ++sid) {
    const uint8_t cached_id = static_cast<uint8_t>(sid);
    if (!isSourceEnabledForHealth(cached_id)) {
      continue;
    }
    // treat_as_horizontal 时斜雷达点并入水平数据流，参与 Footprint/制动包络主链。
    const bool cached_tilt = isTiltSourceId(cached_id) && !tilt_treat_as_horizontal_;
    auto & target = cached_tilt ? tilt_obstacle_points_ : horizontal_obstacle_points_;
    for (const auto & latest : latest_source_points_[sid]) {
      ObstaclePoint point;
      point.x = std::get<0>(latest);
      point.y = std::get<1>(latest);
      point.z = std::get<2>(latest);
      point.source = sourceNameFromId(cached_id);
      target.push_back(point);
    }
  }

  if (local_map_enabled_) {
    // treat_as_horizontal 时斜雷达按水平雷达的 feed 策略写入 LocalMap。
    const bool feed_as_tilt = tilt_source && !tilt_treat_as_horizontal_;
    const bool feed_pending = feed_as_tilt ? tilt_feed_local_map_ : horizontal_feed_local_map_;
    if (feed_pending) {
      pending_points_.insert(pending_points_.end(), pts.begin(), pts.end());
      if (pending_points_.size() > max_pending_points_) {
        pending_points_.erase(
          pending_points_.begin(),
          pending_points_.begin() +
          static_cast<std::ptrdiff_t>(pending_points_.size() - max_pending_points_));
      }
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
    appendBaseLinkPoints({}, source_name, stamp);
    return;
  }

  std::vector<std::tuple<double, double, double>> batch;
  batch.reserve(msg->ranges.size());

  // 批量仿射变换：预提取旋转矩阵+平移，逐点手动变换，避免每点构造 PointStamped + doTransform 虚函数调用
  const AffineTransform tf(transform);

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const float range = msg->ranges[i];
    if (!PerceptionUtils::validRange(range, msg->range_min, msg->range_max)) {
      continue;
    }
    const float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
    const auto [lx, ly] = PerceptionUtils::polarToCartesian(range, angle);
    double bx, by, bz;
    tf.apply(lx, ly, 0.0, bx, by, bz);
    if (!acceptBaseLinkObstaclePoint(bx, by, bz)) {
      continue;
    }
    batch.emplace_back(bx, by, bz);
  }

  appendBaseLinkPoints(batch, source_name, stamp);
}

void LidarPerception::processCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
  const std::string & default_frame,
  const std::string & source_name)
{
  if (!msg) {
    return;
  }

  const rclcpp::Time stamp(
    msg->header.stamp, node_->get_clock()->get_clock_type());
  const std::string frame =
    msg->header.frame_id.empty() ? default_frame : msg->header.frame_id;
  geometry_msgs::msg::TransformStamped transform;
  if (!lookupTransform(frame, stamp, transform)) {
    return;
  }
  markSourceTransformValid(source_name, stamp);

  if (msg->data.empty()) {
    appendBaseLinkPoints({}, source_name, stamp);
    return;
  }

  PointCloud cloud;
  pcl::fromROSMsg(*msg, cloud);

  std::vector<std::tuple<double, double, double>> batch;
  batch.reserve(cloud.points.size());

  // 批量仿射变换：预提取旋转矩阵+平移，逐点手动变换
  const AffineTransform tf(transform);

  for (const auto & pt : cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    double bx = 0.0;
    double by = 0.0;
    double bz = 0.0;
    tf.apply(pt.x, pt.y, pt.z, bx, by, bz);
    if (tilt_preserve_sensor_z_) {
      bz = pt.z;   // preserve_sensor_z：z 保留传感器 frame 原始值
    }
    if (!acceptTiltObstacleInBaseLink(bx, by, bz)) {
      continue;
    }
    batch.emplace_back(bx, by, bz);
  }

  appendBaseLinkPoints(batch, source_name, stamp);
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
  if (!horizontal_enabled_) {
    return ObstacleField{};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return ObstacleFieldBuilder::buildFromPoints(
    horizontal_obstacle_points_, stamp, params, 0.12, params.max_obstacle_primitives);
}

/**
 * 从 tilt_obstacle_points_ 构建斜雷达专用 ObstacleField。
 *
 * 供 AvoidanceController::evaluateTiltIntervention 使用：体素更细（tilt_voxel_size_m）、
 * 图元半径取自 tilt_ttc.obstacle_radius_m，与水平场分离体素化。
 */
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
    // 方案 4：per-source 启用位图先行填充——未收到消息的源也要标注启用状态，
    // 启用但从未新鲜的源同样构成降级输入（对应加电即缺失的失效模式）。
    if (i < result.source_enabled.size()) {
      result.source_enabled[i] = isSourceEnabledForHealth(static_cast<uint8_t>(i));
    }
    if (!source_received_[i]) {
      continue;
    }
    if (!isSourceEnabledForHealth(static_cast<uint8_t>(i))) {
      continue;
    }
    ++result.received_source_count;
    // treat_as_horizontal 时斜雷达计入水平雷达健康统计（满足 require_horizontal_perception）。
    const bool tilt_source =
      isTiltSourceId(static_cast<uint8_t>(i)) && !tilt_treat_as_horizontal_;
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
      if (i < result.source_fresh.size()) {
        result.source_fresh[i] = true;
      }
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
    const size_t source_id = static_cast<size_t>(snapshot.source_id);
    snapshot.frame_valid =
      tilt_enabled_ && latest_source_points_valid_[source_id];
    if (snapshot.frame_valid) {
      snapshot.frame_stamp_ns = latest_source_points_stamp_[source_id].nanoseconds();
      snapshot.frame_age_s =
        std::max(0.0, (stamp - latest_source_points_stamp_[source_id]).seconds());
    }
    // 直接按源读取最新完整帧。即使 treat_as_horizontal=true，低通专用快照仍只含斜雷达；
    // 不改变 horizontal_obstacle_points_ / tilt_obstacle_points_ 的既有控制语义。
    for (const auto & latest : latest_source_points_[source_id]) {
      ObstaclePoint point;
      point.x = std::get<0>(latest);
      point.y = std::get<1>(latest);
      point.z = std::get<2>(latest);
      point.source = source_name;
      snapshot.filtered_points.push_back(point);
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
  horizontal_obstacle_points_.clear();
  tilt_obstacle_points_.clear();
  debug_points_.clear();
  for (auto & points : latest_source_points_) {
    points.clear();
  }
  latest_source_points_valid_.fill(false);
  source_received_.fill(false);
  source_transform_valid_.fill(false);
}
