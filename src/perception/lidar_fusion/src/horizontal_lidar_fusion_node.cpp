#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
using LaserScan = sensor_msgs::msg::LaserScan;
using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;

constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

int64_t stampToNs(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<int64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time nsToStamp(int64_t stamp_ns)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(stamp_ns / kNanosecondsPerSecond);
  stamp.nanosec = static_cast<uint32_t>(stamp_ns % kNanosecondsPerSecond);
  if (stamp.nanosec >= static_cast<uint32_t>(kNanosecondsPerSecond)) {
    ++stamp.sec;
    stamp.nanosec -= static_cast<uint32_t>(kNanosecondsPerSecond);
  }
  return stamp;
}

int64_t secondsToNs(double seconds)
{
  return static_cast<int64_t>(std::llround(seconds * static_cast<double>(kNanosecondsPerSecond)));
}

bool isFinite(double value)
{
  return std::isfinite(value);
}

bool isValidQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return isFinite(q.x) && isFinite(q.y) && isFinite(q.z) && isFinite(q.w) &&
         norm_sq > 1.0e-6;
}

tf2::Quaternion normalized(const geometry_msgs::msg::Quaternion & q_msg)
{
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  q.normalize();
  return q;
}

}  // namespace

class HorizontalLidarFusionNode : public rclcpp::Node
{
public:
  HorizontalLidarFusionNode()
  : Node("horizontal_lidar_fusion"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    loadParameters();

    fused_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_cloud_topic_, rclcpp::SensorDataQoS());

    if (publish_debug_clouds_) {
      left_debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        left_debug_cloud_topic_, rclcpp::SensorDataQoS());
      right_debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        right_debug_cloud_topic_, rclcpp::SensorDataQoS());
      back_debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        back_debug_cloud_topic_, rclcpp::SensorDataQoS());
    }

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HorizontalLidarFusionNode::imuCallback, this, std::placeholders::_1));

    const auto qos = rclcpp::SensorDataQoS();
    left_scan_plain_sub_ = create_subscription<LaserScan>(
      left_scan_topic_, qos,
      [this](const LaserScan::ConstSharedPtr msg) {storeScan(left_slot_, msg);});
    right_scan_plain_sub_ = create_subscription<LaserScan>(
      right_scan_topic_, qos,
      [this](const LaserScan::ConstSharedPtr msg) {storeScan(right_slot_, msg);});
    back_scan_plain_sub_ = create_subscription<LaserScan>(
      back_scan_topic_, qos,
      [this](const LaserScan::ConstSharedPtr msg) {storeScan(back_slot_, msg);});

    const double fusion_period = 1.0 / std::max(1.0, fusion_frequency_);
    fusion_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(fusion_period)),
      std::bind(&HorizontalLidarFusionNode::onFusionTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "horizontal_lidar_fusion started: lidar %.1f Hz, imu %.1f Hz, "
      "fusion %.1f Hz, max_scan_age %.3f s, min_sources %d",
      lidar_frequency_, imu_frequency_, fusion_frequency_, max_scan_age_sec_,
      min_sources_);
  }

private:
  struct ImuSample
  {
    int64_t stamp_ns = 0;
    tf2::Quaternion orientation;
    std::string frame_id;
  };

  struct BuildStats
  {
    size_t valid_points = 0;
    size_t skipped_ranges = 0;
    size_t deskew_failures = 0;
    bool used_deskew = false;
  };

  struct ScanSlot
  {
    LaserScan::ConstSharedPtr scan;
    std::chrono::steady_clock::time_point recv{};
  };

  void loadParameters()
  {
    left_scan_topic_ = declare_parameter<std::string>("left_scan_topic", "/scan/left");
    right_scan_topic_ = declare_parameter<std::string>("right_scan_topic", "/scan/right");
    back_scan_topic_ = declare_parameter<std::string>("back_scan_topic", "/scan/back");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");

    output_cloud_topic_ = declare_parameter<std::string>(
      "output_cloud_topic", "/perception/lidar_fusion/points");
    left_debug_cloud_topic_ = declare_parameter<std::string>(
      "left_debug_cloud_topic", "/perception/lidar_fusion/debug/left");
    right_debug_cloud_topic_ = declare_parameter<std::string>(
      "right_debug_cloud_topic", "/perception/lidar_fusion/debug/right");
    back_debug_cloud_topic_ = declare_parameter<std::string>(
      "back_debug_cloud_topic", "/perception/lidar_fusion/debug/back");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");

    imu_frequency_ = declare_parameter<double>("imu_frequency", 100.0);
    lidar_frequency_ = declare_parameter<double>("lidar_frequency", 10.0);
    fallback_scan_time_sec_ = declare_parameter<double>("fallback_scan_time_sec", 0.1);
    if (fallback_scan_time_sec_ <= 0.0 && lidar_frequency_ > 0.0) {
      fallback_scan_time_sec_ = 1.0 / lidar_frequency_;
    }
    if (fallback_scan_time_sec_ <= 0.0) {
      fallback_scan_time_sec_ = 0.1;
    }

    // Retained for backward-compatible YAML (no longer used to gate fusion).
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 30);
    sync_slop_sec_ = declare_parameter<double>("sync_slop_sec", 0.08);
    sync_slop_ns_ = secondsToNs(sync_slop_sec_);

    // Timer-driven fusion: use the freshest available scan from each source so a
    // single lidar dropout degrades gracefully instead of stalling the whole
    // pipeline (the old hard 3-way ApproximateTime sync required all three).
    max_scan_age_sec_ = declare_parameter<double>("max_scan_age_sec", 0.20);
    min_sources_ = declare_parameter<int>("min_sources", 1);
    fusion_frequency_ = declare_parameter<double>(
      "fusion_frequency", lidar_frequency_ > 0.0 ? lidar_frequency_ : 10.0);
    if (min_sources_ < 1) {
      min_sources_ = 1;
    }
    if (min_sources_ > 3) {
      min_sources_ = 3;
    }
    if (fusion_frequency_ <= 0.0) {
      fusion_frequency_ = 10.0;
    }

    deskew_enabled_ = declare_parameter<bool>("deskew_enabled", true);
    drop_if_imu_missing_ = declare_parameter<bool>("drop_if_imu_missing", false);
    allow_imu_nearest_fallback_ = declare_parameter<bool>("allow_imu_nearest_fallback", true);
    max_imu_gap_sec_ = declare_parameter<double>("max_imu_gap_sec", 0.05);
    max_imu_gap_ns_ = secondsToNs(max_imu_gap_sec_);
    imu_buffer_sec_ = declare_parameter<double>("imu_buffer_sec", 3.0);
    imu_buffer_ns_ = secondsToNs(imu_buffer_sec_);

    publish_debug_clouds_ = declare_parameter<bool>("publish_debug_clouds", false);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.05);

    left_time_offset_ns_ = secondsToNs(declare_parameter<double>("left_time_offset_sec", 0.0));
    right_time_offset_ns_ = secondsToNs(declare_parameter<double>("right_time_offset_sec", 0.0));
    back_time_offset_ns_ = secondsToNs(declare_parameter<double>("back_time_offset_sec", 0.0));

    imu_frame_override_ = declare_parameter<std::string>("imu_frame_id", "");

    if (sync_queue_size_ < 3) {
      sync_queue_size_ = 3;
    }
    if (sync_slop_ns_ <= 0) {
      sync_slop_ns_ = secondsToNs(0.08);
    }
    if (max_imu_gap_ns_ <= 0) {
      max_imu_gap_ns_ = secondsToNs(0.05);
    }
    if (imu_buffer_ns_ <= 0) {
      imu_buffer_ns_ = secondsToNs(3.0);
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!msg || !isValidQuaternion(msg->orientation)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Drop invalid IMU orientation sample");
      return;
    }

    ImuSample sample;
    sample.stamp_ns = stampToNs(msg->header.stamp);
    sample.orientation = normalized(msg->orientation);
    sample.frame_id = imu_frame_override_.empty() ? msg->header.frame_id : imu_frame_override_;

    if (sample.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU frame_id is empty; deskew will assume IMU aligned with %s",
        target_frame_.c_str());
    }

    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (!imu_buffer_.empty() && sample.stamp_ns < imu_buffer_.back().stamp_ns) {
      auto it = std::upper_bound(
        imu_buffer_.begin(), imu_buffer_.end(), sample.stamp_ns,
        [](int64_t stamp_ns, const ImuSample & existing) {
          return stamp_ns < existing.stamp_ns;
        });
      imu_buffer_.insert(it, sample);
    } else {
      imu_buffer_.push_back(sample);
    }

    const int64_t keep_after = sample.stamp_ns - imu_buffer_ns_;
    while (!imu_buffer_.empty() && imu_buffer_.front().stamp_ns < keep_after) {
      imu_buffer_.pop_front();
    }
  }

  void storeScan(ScanSlot & slot, const LaserScan::ConstSharedPtr & msg)
  {
    if (!msg) {
      return;
    }
    std::lock_guard<std::mutex> lock(scan_mutex_);
    slot.scan = msg;
    slot.recv = std::chrono::steady_clock::now();
  }

  // Returns the stored scan if it arrived within max_scan_age_sec_, else null.
  // Caller must hold scan_mutex_. Freshness uses the monotonic reception clock
  // so it is immune to header-stamp / clock-source issues and detects dropouts.
  LaserScan::ConstSharedPtr freshScan(
    const ScanSlot & slot, std::chrono::steady_clock::time_point now_tp) const
  {
    if (!slot.scan) {
      return nullptr;
    }
    const double age = std::chrono::duration<double>(now_tp - slot.recv).count();
    if (age > max_scan_age_sec_) {
      return nullptr;
    }
    return slot.scan;
  }

  void onFusionTimer()
  {
    LaserScan::ConstSharedPtr left;
    LaserScan::ConstSharedPtr right;
    LaserScan::ConstSharedPtr back;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      const auto now_tp = std::chrono::steady_clock::now();
      left = freshScan(left_slot_, now_tp);
      right = freshScan(right_slot_, now_tp);
      back = freshScan(back_slot_, now_tp);
    }

    const int available = (left ? 1 : 0) + (right ? 1 : 0) + (back ? 1 : 0);
    if (available < min_sources_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Only %d/3 lidar scans fresh (< min_sources=%d), skip fusion "
        "[left=%s right=%s back=%s]",
        available, min_sources_,
        left ? "ok" : "stale", right ? "ok" : "stale", back ? "ok" : "stale");
      return;
    }
    if (available < 3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Degraded fusion with %d/3 lidars [left=%s right=%s back=%s]",
        available,
        left ? "ok" : "stale", right ? "ok" : "stale", back ? "ok" : "stale");
    }

    fuseScans(left, right, back);
  }

  // Fuse whichever of the three scans are provided (any may be null). Producing
  // an output from a partial set keeps obstacle coverage alive when one lidar
  // drops out.
  void fuseScans(
    const LaserScan::ConstSharedPtr left,
    const LaserScan::ConstSharedPtr right,
    const LaserScan::ConstSharedPtr back)
  {
    bool have_ref = false;
    int64_t reference_ns = 0;
    const auto consider = [&](const LaserScan::ConstSharedPtr & scan, int64_t offset) {
      if (!scan) {
        return;
      }
      const int64_t start_ns = stampToNs(scan->header.stamp) + offset;
      if (!have_ref || start_ns > reference_ns) {
        reference_ns = start_ns;
        have_ref = true;
      }
    };
    consider(left, left_time_offset_ns_);
    consider(right, right_time_offset_ns_);
    consider(back, back_time_offset_ns_);
    if (!have_ref) {
      return;
    }

    // Skip republishing when no source advanced since the last fusion (the
    // fusion timer can fire between scan arrivals).
    if (reference_ns == last_fused_ref_ns_) {
      return;
    }
    last_fused_ref_ns_ = reference_ns;

    const auto reference_orientation = deskew_enabled_ ? interpolateImu(reference_ns) : std::nullopt;
    if (deskew_enabled_ && !reference_orientation.has_value()) {
      ++missing_reference_imu_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "IMU data does not cover fusion reference time; deskew disabled for this group");
      if (drop_if_imu_missing_) {
        return;
      }
    }

    auto fused_cloud = std::make_shared<PointCloud>();
    fused_cloud->header.frame_id = target_frame_;
    fused_cloud->header.stamp = pcl_conversions::toPCL(nsToStamp(reference_ns));
    fused_cloud->is_dense = false;

    BuildStats left_stats;
    BuildStats right_stats;
    BuildStats back_stats;

    auto left_cloud = buildCloud(
      left, "left", left_time_offset_ns_, reference_ns, reference_orientation, left_stats);
    auto right_cloud = buildCloud(
      right, "right", right_time_offset_ns_, reference_ns, reference_orientation, right_stats);
    auto back_cloud = buildCloud(
      back, "back", back_time_offset_ns_, reference_ns, reference_orientation, back_stats);

    appendCloud(left_cloud, *fused_cloud);
    appendCloud(right_cloud, *fused_cloud);
    appendCloud(back_cloud, *fused_cloud);
    fused_cloud->width = static_cast<uint32_t>(fused_cloud->points.size());
    fused_cloud->height = 1;

    if (fused_cloud->points.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Fusion output is empty");
      return;
    }

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*fused_cloud, output_msg);
    output_msg.header.stamp = nsToStamp(reference_ns);
    output_msg.header.frame_id = target_frame_;
    fused_cloud_pub_->publish(output_msg);

    if (publish_debug_clouds_) {
      publishDebugCloud(left_cloud, reference_ns, left_debug_pub_);
      publishDebugCloud(right_cloud, reference_ns, right_debug_pub_);
      publishDebugCloud(back_cloud, reference_ns, back_debug_pub_);
    }

    const size_t deskew_failures =
      left_stats.deskew_failures + right_stats.deskew_failures + back_stats.deskew_failures;
    if (deskew_failures > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Deskew missed %zu points in latest fusion group", deskew_failures);
    }

    RCLCPP_DEBUG(
      get_logger(),
      "Fused points: %zu (left=%zu right=%zu back=%zu)",
      fused_cloud->points.size(), left_stats.valid_points,
      right_stats.valid_points, back_stats.valid_points);
  }

  PointCloud::Ptr buildCloud(
    const LaserScan::ConstSharedPtr & scan,
    const std::string & source_name,
    int64_t time_offset_ns,
    int64_t reference_ns,
    const std::optional<ImuSample> & reference_orientation,
    BuildStats & stats)
  {
    auto cloud = std::make_shared<PointCloud>();
    if (!scan || scan->ranges.empty()) {
      return cloud;
    }

    const std::string source_frame = scan->header.frame_id;
    if (source_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Scan %s has empty frame_id", source_name.c_str());
      return cloud;
    }

    const auto lidar_to_target = lookupTransformToTarget(source_frame, scan->header.stamp);
    if (!lidar_to_target.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Missing TF %s -> %s, skip %s scan",
        source_frame.c_str(), target_frame_.c_str(), source_name.c_str());
      return cloud;
    }

    std::optional<tf2::Quaternion> target_to_imu_rotation = getTargetToImuRotation(reference_orientation);
    const bool can_deskew =
      deskew_enabled_ && reference_orientation.has_value() && target_to_imu_rotation.has_value();

    const int64_t scan_start_ns = stampToNs(scan->header.stamp) + time_offset_ns;
    const int64_t point_dt_ns = estimatePointTimeIncrementNs(*scan);

    cloud->header.frame_id = target_frame_;
    cloud->header.stamp = pcl_conversions::toPCL(nsToStamp(reference_ns));
    cloud->is_dense = false;
    cloud->points.reserve(scan->ranges.size());

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      const float range = scan->ranges[i];
      if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
        ++stats.skipped_ranges;
        continue;
      }

      const double angle = static_cast<double>(scan->angle_min) +
        static_cast<double>(i) * static_cast<double>(scan->angle_increment);
      const tf2::Vector3 raw_point(
        static_cast<double>(range) * std::cos(angle),
        static_cast<double>(range) * std::sin(angle),
        0.0);

      tf2::Vector3 target_point = lidar_to_target.value() * raw_point;

      if (can_deskew) {
        const int64_t point_time_ns = scan_start_ns + static_cast<int64_t>(i) * point_dt_ns;
        const auto point_orientation = interpolateImu(point_time_ns);
        if (point_orientation.has_value()) {
          target_point = deskewPoint(
            target_point, point_orientation.value(), reference_orientation.value(),
            target_to_imu_rotation.value());
          stats.used_deskew = true;
        } else {
          ++stats.deskew_failures;
          if (drop_if_imu_missing_) {
            continue;
          }
        }
      }

      PointType point;
      point.x = static_cast<float>(target_point.x());
      point.y = static_cast<float>(target_point.y());
      point.z = static_cast<float>(target_point.z());
      if (i < scan->intensities.size() && std::isfinite(scan->intensities[i])) {
        point.intensity = scan->intensities[i];
      } else {
        point.intensity = 0.0F;
      }
      cloud->points.push_back(point);
      ++stats.valid_points;
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    return cloud;
  }

  int64_t estimatePointTimeIncrementNs(const LaserScan & scan) const
  {
    if (scan.time_increment > 0.0F && std::isfinite(scan.time_increment)) {
      return secondsToNs(scan.time_increment);
    }
    if (scan.scan_time > 0.0F && std::isfinite(scan.scan_time) && !scan.ranges.empty()) {
      return secondsToNs(static_cast<double>(scan.scan_time) /
        static_cast<double>(scan.ranges.size()));
    }
    return secondsToNs(fallback_scan_time_sec_ / static_cast<double>(scan.ranges.size()));
  }

  std::optional<tf2::Transform> lookupTransformToTarget(
    const std::string & source_frame,
    const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform_msg;
    try {
      transform_msg = tf_buffer_->lookupTransform(
        target_frame_, source_frame, rclcpp::Time(stamp),
        tf2::durationFromSec(transform_timeout_sec_));
    } catch (const tf2::TransformException &) {
      try {
        transform_msg = tf_buffer_->lookupTransform(
          target_frame_, source_frame, tf2::TimePointZero,
          tf2::durationFromSec(transform_timeout_sec_));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "TF lookup failed: %s", ex.what());
        return std::nullopt;
      }
    }

    tf2::Transform transform;
    tf2::fromMsg(transform_msg.transform, transform);
    return transform;
  }

  std::optional<ImuSample> interpolateImu(int64_t stamp_ns)
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (imu_buffer_.empty()) {
      return std::nullopt;
    }

    if (stamp_ns <= imu_buffer_.front().stamp_ns) {
      if (allow_imu_nearest_fallback_ &&
          std::llabs(imu_buffer_.front().stamp_ns - stamp_ns) <= max_imu_gap_ns_) {
        return imu_buffer_.front();
      }
      return std::nullopt;
    }

    if (stamp_ns >= imu_buffer_.back().stamp_ns) {
      if (allow_imu_nearest_fallback_ &&
          std::llabs(stamp_ns - imu_buffer_.back().stamp_ns) <= max_imu_gap_ns_) {
        return imu_buffer_.back();
      }
      return std::nullopt;
    }

    auto upper = std::lower_bound(
      imu_buffer_.begin(), imu_buffer_.end(), stamp_ns,
      [](const ImuSample & sample, int64_t target_stamp_ns) {
        return sample.stamp_ns < target_stamp_ns;
      });

    if (upper == imu_buffer_.end()) {
      return std::nullopt;
    }
    if (upper->stamp_ns == stamp_ns) {
      return *upper;
    }
    if (upper == imu_buffer_.begin()) {
      return std::nullopt;
    }

    const auto lower = std::prev(upper);
    const int64_t interval_ns = upper->stamp_ns - lower->stamp_ns;
    if (interval_ns <= 0 || interval_ns > max_imu_gap_ns_) {
      return std::nullopt;
    }

    const double ratio =
      static_cast<double>(stamp_ns - lower->stamp_ns) / static_cast<double>(interval_ns);

    ImuSample interpolated;
    interpolated.stamp_ns = stamp_ns;
    interpolated.orientation = lower->orientation.slerp(upper->orientation, ratio);
    interpolated.orientation.normalize();
    interpolated.frame_id = lower->frame_id.empty() ? upper->frame_id : lower->frame_id;
    return interpolated;
  }

  std::optional<tf2::Quaternion> getTargetToImuRotation(
    const std::optional<ImuSample> & reference_orientation)
  {
    if (!reference_orientation.has_value() || reference_orientation->frame_id.empty()) {
      tf2::Quaternion identity;
      identity.setRPY(0.0, 0.0, 0.0);
      return identity;
    }

    const std::string & imu_frame = reference_orientation->frame_id;
    if (imu_frame == target_frame_) {
      tf2::Quaternion identity;
      identity.setRPY(0.0, 0.0, 0.0);
      return identity;
    }

    try {
      const auto transform_msg = tf_buffer_->lookupTransform(
        target_frame_, imu_frame, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_sec_));
      tf2::Quaternion q_target_imu;
      tf2::fromMsg(transform_msg.transform.rotation, q_target_imu);
      q_target_imu.normalize();
      return q_target_imu;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Missing IMU static TF %s -> %s, assume aligned: %s",
        imu_frame.c_str(), target_frame_.c_str(), ex.what());
      tf2::Quaternion identity;
      identity.setRPY(0.0, 0.0, 0.0);
      return identity;
    }
  }

  tf2::Vector3 deskewPoint(
    const tf2::Vector3 & point,
    const ImuSample & point_orientation,
    const ImuSample & reference_orientation,
    const tf2::Quaternion & q_target_imu)
  {
    const tf2::Quaternion q_imu_relative =
      reference_orientation.orientation.inverse() * point_orientation.orientation;
    const tf2::Quaternion q_target_relative =
      q_target_imu * q_imu_relative * q_target_imu.inverse();
    return tf2::quatRotate(q_target_relative.normalized(), point);
  }

  void appendCloud(const PointCloud::Ptr & source, PointCloud & target)
  {
    if (!source || source->points.empty()) {
      return;
    }
    target.points.insert(target.points.end(), source->points.begin(), source->points.end());
  }

  void publishDebugCloud(
    const PointCloud::Ptr & cloud,
    int64_t stamp_ns,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
  {
    if (!publisher || !cloud || cloud->points.empty()) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = nsToStamp(stamp_ns);
    msg.header.frame_id = target_frame_;
    publisher->publish(msg);
  }

  std::string left_scan_topic_;
  std::string right_scan_topic_;
  std::string back_scan_topic_;
  std::string imu_topic_;
  std::string output_cloud_topic_;
  std::string left_debug_cloud_topic_;
  std::string right_debug_cloud_topic_;
  std::string back_debug_cloud_topic_;
  std::string target_frame_;
  std::string imu_frame_override_;

  double imu_frequency_ = 100.0;
  double lidar_frequency_ = 10.0;
  double fallback_scan_time_sec_ = 0.1;
  int sync_queue_size_ = 30;
  double sync_slop_sec_ = 0.08;
  int64_t sync_slop_ns_ = secondsToNs(0.08);
  double max_scan_age_sec_ = 0.20;
  int min_sources_ = 1;
  double fusion_frequency_ = 10.0;
  bool deskew_enabled_ = true;
  bool drop_if_imu_missing_ = false;
  bool allow_imu_nearest_fallback_ = true;
  double max_imu_gap_sec_ = 0.05;
  int64_t max_imu_gap_ns_ = secondsToNs(0.05);
  double imu_buffer_sec_ = 3.0;
  int64_t imu_buffer_ns_ = secondsToNs(3.0);
  bool publish_debug_clouds_ = false;
  double transform_timeout_sec_ = 0.05;
  int64_t left_time_offset_ns_ = 0;
  int64_t right_time_offset_ns_ = 0;
  int64_t back_time_offset_ns_ = 0;
  size_t missing_reference_imu_count_ = 0;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr back_debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Subscription<LaserScan>::SharedPtr left_scan_plain_sub_;
  rclcpp::Subscription<LaserScan>::SharedPtr right_scan_plain_sub_;
  rclcpp::Subscription<LaserScan>::SharedPtr back_scan_plain_sub_;
  rclcpp::TimerBase::SharedPtr fusion_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex scan_mutex_;
  ScanSlot left_slot_;
  ScanSlot right_slot_;
  ScanSlot back_slot_;
  int64_t last_fused_ref_ns_ = 0;

  std::mutex imu_mutex_;
  std::deque<ImuSample> imu_buffer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HorizontalLidarFusionNode>());
  rclcpp::shutdown();
  return 0;
}
