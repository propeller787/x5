#pragma once

#include <array>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace lidar_imu_fusion
{

enum class SensorId
{
  Left = 0,
  Right = 1,
  Back = 2,
};

struct TimedScanSet
{
  sensor_msgs::msg::LaserScan::SharedPtr left;
  sensor_msgs::msg::LaserScan::SharedPtr right;
  sensor_msgs::msg::LaserScan::SharedPtr back;
};

struct FusedPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  float intensity{0.0F};
};

// Result of appending one lidar's deskewed points to the fused cloud.
//   kOk                 - points appended (possibly with some beams un-deskewed)
//   kSensorUnavailable  - this lidar's transform could not be resolved; skip this
//                         sensor but keep the rest of the synchronized group.
//   kImuCoverageLost    - IMU coverage required by require_imu_for_publish_ was
//                         lost mid-scan; drop the whole synchronized group.
enum class AppendResult
{
  kOk = 0,
  kSensorUnavailable = 1,
  kImuCoverageLost = 2,
};

class ScanSynchronizer
{
public:
  void setMaxSlop(double max_slop_sec);
  void setQueueSize(std::size_t queue_size);
  void setDiagnostics(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);
  void push(SensorId sensor_id, sensor_msgs::msg::LaserScan::SharedPtr scan);
  std::optional<TimedScanSet> tryPop();

private:
  using ScanQueue = std::deque<sensor_msgs::msg::LaserScan::SharedPtr>;

  static int64_t stampNs(const sensor_msgs::msg::LaserScan::SharedPtr & scan);
  ScanQueue & queueFor(SensorId sensor_id);
  void trimQueue(ScanQueue & queue) const;
  std::size_t pruneStaleScans();

  ScanQueue left_queue_;
  ScanQueue right_queue_;
  ScanQueue back_queue_;
  int64_t max_slop_ns_{30000000};
  std::size_t queue_size_{10};
  rclcpp::Logger logger_{rclcpp::get_logger("scan_synchronizer")};
  rclcpp::Clock::SharedPtr clock_;
  bool diagnostics_enabled_{false};
};

class ImuBuffer
{
public:
  void setRetention(double retention_sec);
  void setMaxExtrapolation(double max_extrapolation_sec);
  void push(const rclcpp::Time & stamp, const tf2::Vector3 & angular_velocity);
  bool hasCoverage(const rclcpp::Time & from, const rclcpp::Time & to) const;
  bool integrateRotation(
    const rclcpp::Time & from,
    const rclcpp::Time & to,
    tf2::Quaternion * delta_rotation) const;
  bool empty() const;

private:
  struct ImuSample
  {
    rclcpp::Time stamp;
    tf2::Vector3 angular_velocity;
  };

  bool angularVelocityAt(const rclcpp::Time & stamp, tf2::Vector3 * angular_velocity) const;

  mutable std::mutex mutex_;
  std::deque<ImuSample> samples_;
  // Reused across integrateRotation calls (accessed under mutex_) to avoid a
  // heap allocation per lidar beam.
  mutable std::vector<int64_t> knots_buffer_;
  double retention_sec_{2.0};
  double max_extrapolation_sec_{0.03};
};

class ThreeLidarImuFusionNode : public rclcpp::Node
{
public:
  explicit ThreeLidarImuFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadParameters();
  tf2::Transform loadStaticTransform(const std::string & prefix);

  void handleScan(SensorId sensor_id, sensor_msgs::msg::LaserScan::SharedPtr scan);
  void handleImu(sensor_msgs::msg::Imu::SharedPtr imu);

  void enqueueSynchronizedScans(const TimedScanSet & scans);
  void drainPendingScans();
  bool imuCoversScans(const TimedScanSet & scans) const;
  void processSynchronizedScans(const TimedScanSet & scans);
  AppendResult appendDeskewedScanPoints(
    const sensor_msgs::msg::LaserScan & scan,
    SensorId sensor_id,
    const rclcpp::Time & fusion_time,
    std::vector<FusedPoint> * points);

  sensor_msgs::msg::LaserScan buildOutputScan(
    const std::vector<FusedPoint> & points,
    const rclcpp::Time & stamp,
    double scan_time,
    double angle_increment) const;

  rclcpp::Time scanStartTime(const sensor_msgs::msg::LaserScan & scan) const;
  rclcpp::Time scanEndTime(const sensor_msgs::msg::LaserScan & scan) const;
  rclcpp::Time earliestScanStartTime(const TimedScanSet & scans) const;
  rclcpp::Time fusionReferenceTime(const TimedScanSet & scans) const;
  double scanDuration(const sensor_msgs::msg::LaserScan & scan) const;
  double pointTimeIncrement(const sensor_msgs::msg::LaserScan & scan) const;
  double outputAngleIncrement(const TimedScanSet & scans) const;
  bool rangeIsValid(float range, const sensor_msgs::msg::LaserScan & scan) const;

  bool lidarTransform(
    SensorId sensor_id,
    const std::string & source_frame,
    tf2::Transform * transform);
  tf2::Vector3 imuAngularVelocityInTarget(const sensor_msgs::msg::Imu & imu);
  static const char * sensorName(SensorId sensor_id);
  static std::size_t sensorIndex(SensorId sensor_id);

  std::string left_scan_topic_{"/scan/left"};
  std::string right_scan_topic_{"/scan/right"};
  std::string back_scan_topic_{"/scan/back"};
  std::string imu_topic_{"/sensor/imu/data"};
  std::string output_scan_topic_{"/perception/lidar_fusion/scan"};
  std::string target_frame_{"base_link"};

  bool use_tf_transforms_{true};
  bool fallback_to_static_transforms_{true};
  bool enable_imu_deskew_{true};
  bool require_imu_for_publish_{false};
  bool publish_empty_scan_{false};

  double output_angle_min_{-3.14159265358979323846};
  double output_angle_max_{3.14159265358979323846};
  double output_angle_increment_{0.004363323129985824};
  double output_range_min_{0.05};
  double output_range_max_{10.0};
  double default_scan_time_{0.1};
  double max_sync_slop_sec_{0.03};
  double imu_queue_duration_sec_{2.0};
  double max_imu_extrapolation_sec_{0.03};
  std::size_t scan_queue_size_{10};
  std::size_t max_pending_scan_sets_{5};

  std::array<tf2::Transform, 3> static_lidar_transforms_;
  tf2::Transform static_imu_transform_;

  // Cached IMU->target rotation basis. The extrinsics are effectively constant
  // for a rigidly mounted IMU, so we refresh from TF only periodically instead
  // of on every 100 Hz IMU sample.
  tf2::Matrix3x3 cached_imu_basis_;
  bool imu_basis_cached_{false};
  std::size_t imu_basis_refresh_counter_{0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr left_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr right_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr back_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fused_scan_pub_;
  rclcpp::TimerBase::SharedPtr pending_scan_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  ScanSynchronizer scan_synchronizer_;
  ImuBuffer imu_buffer_;
  std::mutex scan_mutex_;
  std::deque<TimedScanSet> pending_scan_sets_;
  std::mutex pending_scan_mutex_;
};

}  // namespace lidar_imu_fusion
