/**
 * @file test_lidar_perception.cpp
 */
#include <gtest/gtest.h>

#include "safety_node/lidar_perception.h"

#include "../../../common/utils/topic.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{

class LidarHarness
{
public:
  struct HorizontalConfig
  {
    bool enabled = true;
    bool left_enabled = true;
    bool right_enabled = true;
    bool back_enabled = true;
    bool feed_local_map = true;
  };

  LidarHarness(const std::string & name, bool publish_transform)
  : LidarHarness(name, publish_transform, HorizontalConfig{})
  {
  }

  LidarHarness(
    const std::string & name,
    bool publish_transform,
    const HorizontalConfig & horizontal,
    bool self_filter_enabled = true)
  : node_(std::make_shared<rclcpp::Node>(name))
    , driver_(std::make_shared<rclcpp::Node>(name + "_driver"))
  {
    node_->declare_parameter<double>("tilt_z_filter_threshold", 0.27);
    node_->declare_parameter<bool>("debug.perception.enabled", true);
    node_->declare_parameter<int64_t>("debug.perception.max_points", 4);
    node_->declare_parameter<int64_t>("perception.max_points_per_source", 3);
    node_->declare_parameter<int64_t>("perception.max_pending_points", 5);
    node_->declare_parameter<bool>("perception.horizontal.enabled", horizontal.enabled);
    node_->declare_parameter<bool>("perception.horizontal.left_enabled", horizontal.left_enabled);
    node_->declare_parameter<bool>("perception.horizontal.right_enabled", horizontal.right_enabled);
    node_->declare_parameter<bool>("perception.horizontal.back_enabled", horizontal.back_enabled);
    node_->declare_parameter<bool>("perception.horizontal.feed_local_map", horizontal.feed_local_map);
    node_->declare_parameter<bool>("perception.tilt.enabled", true);
    node_->declare_parameter<std::string>("perception.tilt.coordinate_frame", "sensor");
    node_->declare_parameter<bool>("perception.tilt.feed_local_map", false);
    node_->declare_parameter<bool>("perception.tilt.ground_reject_enabled", true);
    node_->declare_parameter<bool>("perception.tilt.z_filter_enabled", false);
    node_->declare_parameter<bool>("perception.tilt.preserve_sensor_z", true);
    node_->declare_parameter<double>("perception.tilt.z_filter_threshold", 0.0);
    node_->declare_parameter<double>("perception.tilt.voxel_size_m", 0.08);
    node_->declare_parameter<int64_t>("perception.tilt.max_primitives", 48);
    node_->declare_parameter<bool>("perception.self_filter.enabled", self_filter_enabled);
    node_->declare_parameter<double>("perception.self_filter.x_min_m", -0.36);
    node_->declare_parameter<double>("perception.self_filter.x_max_m", 0.95);
    node_->declare_parameter<double>("perception.self_filter.y_abs_m", 0.36);
    node_->declare_parameter<double>("perception.self_filter.z_min_m", -0.15);
    node_->declare_parameter<double>("perception.self_filter.z_max_m", 0.35);
    node_->declare_parameter<double>("perception.mount_edge_reject_radius_m", 0.05);
    node_->declare_parameter<double>("perception.ground_reject_below_z_m", -0.06);
    node_->declare_parameter<bool>("local_map.enabled", true);
    node_->declare_parameter<double>("input_health.tf_lookup_timeout_s", 0.005);
    node_->declare_parameter<double>("input_health.perception_timeout_s", 0.12);

    perception_ = std::make_unique<LidarPerception>(node_);
    left_pub_ = driver_->create_publisher<sensor_msgs::msg::LaserScan>(
      leftLaserTopic, rclcpp::SensorDataQoS());
    right_pub_ = driver_->create_publisher<sensor_msgs::msg::LaserScan>(
      rightLaserTopic, rclcpp::SensorDataQoS());
    left_tilt_cloud_pub_ = driver_->create_publisher<sensor_msgs::msg::PointCloud2>(
      leftTiltPointTopice, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(driver_);
    executor_.add_node(node_);
    executor_.add_node(driver_);
    if (publish_transform) {
      broadcaster_->sendTransform(
        {
          transform("laser_frame_left"),
          transform("laser_frame_right"),
          transform("left_tilt_rotate"),
        });
    }
    spinFor(100ms, []() {});
  }

  void publishLeft(size_t point_count, const std::string & frame = "laser_frame_left")
  {
    publish(left_pub_, point_count, frame, nextAutoStamp());
  }

  void publishLeftAt(size_t point_count, const rclcpp::Time & stamp)
  {
    publish(left_pub_, point_count, "laser_frame_left", stamp);
  }

  void publishRight(size_t point_count)
  {
    publish(right_pub_, point_count, "laser_frame_right", nextAutoStamp());
  }

  void publishLeftTiltCloud(float x, float y, float z, const std::string & frame = "left_tilt_rotate")
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = nextAutoStamp();
    cloud.header.frame_id = frame;
    cloud.height = 1;
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    *iter_x = x;
    *iter_y = y;
    *iter_z = z;
    left_tilt_cloud_pub_->publish(cloud);
  }

  void publishLeftTiltTransform(double tx, double ty)
  {
    auto tf = transform("left_tilt_rotate");
    tf.transform.translation.x = tx;
    tf.transform.translation.y = ty;
    broadcaster_->sendTransform(tf);
  }

  template<typename PublishFn>
  void spinFor(std::chrono::milliseconds duration, PublishFn publish)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      publish();
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    executor_.spin_some();
  }

  LidarPerception & perception()
  {
    return *perception_;
  }

  rclcpp::Time now() const
  {
    return node_->now();
  }

private:
  rclcpp::Time nextAutoStamp()
  {
    rclcpp::Time stamp = driver_->now();
    if (last_auto_stamp_.nanoseconds() > 0 && stamp <= last_auto_stamp_) {
      stamp = last_auto_stamp_ + rclcpp::Duration::from_seconds(0.001);
    }
    last_auto_stamp_ = stamp;
    return stamp;
  }

  geometry_msgs::msg::TransformStamped transform(const std::string & child) const
  {
    geometry_msgs::msg::TransformStamped value;
    value.header.stamp = driver_->now();
    value.header.frame_id = "base_link";
    value.child_frame_id = child;
    value.transform.rotation.w = 1.0;
    return value;
  }

  void publish(
    const rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr & publisher,
    size_t point_count,
    const std::string & frame,
    const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = stamp;
    scan.header.frame_id = frame;
    scan.angle_min = -0.02f;
    scan.angle_increment = 0.01f;
    scan.angle_max =
      scan.angle_min + static_cast<float>(point_count) * scan.angle_increment;
    scan.range_min = 0.05f;
    scan.range_max = 20.0f;
    scan.ranges.assign(point_count, 2.0f);
    publisher->publish(scan);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr driver_;
  std::unique_ptr<LidarPerception> perception_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_tilt_cloud_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Time last_auto_stamp_ {0, 0, RCL_ROS_TIME};
};

}  // namespace

class LidarPerceptionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(LidarPerceptionTest, CachesRemainBoundedAndLatestFramesReplacePreviousData)
{
  LidarHarness harness("lidar_cache_bounds_test", true);
  harness.spinFor(200ms, [&]() {harness.publishLeft(10);});

  PerceptionCacheStats stats = harness.perception().cacheStats();
  EXPECT_EQ(stats.obstacle_points, 3u);
  EXPECT_EQ(stats.pending_points, 5u);
  EXPECT_EQ(stats.debug_points, 4u);

  harness.spinFor(200ms, [&]() {harness.publishRight(10);});
  stats = harness.perception().cacheStats();
  EXPECT_EQ(stats.obstacle_points, 6u);
  EXPECT_EQ(stats.pending_points, 5u);
  EXPECT_EQ(stats.debug_points, 4u);

  const auto pending = harness.perception().takePendingPoints();
  const auto debug = harness.perception().takeDebugPoints();
  EXPECT_EQ(pending.size(), 5u);
  EXPECT_EQ(debug.size(), 4u);
  EXPECT_EQ(harness.perception().cacheStats().pending_points, 0u);
  EXPECT_EQ(harness.perception().cacheStats().debug_points, 0u);

  harness.spinFor(120ms, [&]() {harness.publishLeft(1);});
  EXPECT_EQ(harness.perception().cacheStats().obstacle_points, 4u);

  harness.spinFor(120ms, [&]() {harness.publishLeft(0);});
  stats = harness.perception().cacheStats();
  EXPECT_EQ(stats.obstacle_points, 3u);
  const PerceptionHealth health = harness.perception().health(harness.now(), 1.0);
  EXPECT_EQ(health.received_source_count, 2u);
  EXPECT_EQ(health.fresh_source_count, 2u);
}

TEST_F(LidarPerceptionTest, MissingTransformIsReceivedButNeverReportedFresh)
{
  LidarHarness harness("lidar_missing_tf_health_test", false);
  harness.spinFor(
    200ms, [&]() {harness.publishLeft(1, "missing_laser_frame");});

  const PerceptionHealth health = harness.perception().health(harness.now(), 0.2);
  EXPECT_EQ(health.received_source_count, 1u);
  EXPECT_EQ(health.fresh_source_count, 0u);
  EXPECT_EQ(harness.perception().cacheStats().obstacle_points, 0u);
}

TEST_F(LidarPerceptionTest, RepeatedTimestampCannotKeepSourceFresh)
{
  LidarHarness harness("lidar_repeated_stamp_test", true);
  const rclcpp::Time frozen_stamp = harness.now();
  harness.spinFor(
    80ms, [&]() {harness.publishLeftAt(1, frozen_stamp);});
  EXPECT_EQ(harness.perception().health(harness.now(), 0.12).fresh_source_count, 1u);

  harness.spinFor(
    180ms, [&]() {harness.publishLeftAt(1, frozen_stamp);});
  const PerceptionHealth health = harness.perception().health(harness.now(), 0.12);
  EXPECT_EQ(health.received_source_count, 1u);
  EXPECT_EQ(health.fresh_source_count, 0u);
}

TEST_F(LidarPerceptionTest, StaleAndOutOfOrderFramesDoNotReplaceLatestCache)
{
  LidarHarness harness("lidar_timestamp_order_test", true);
  const rclcpp::Time fresh_stamp = harness.now();
  harness.spinFor(
    40ms, [&]() {harness.publishLeftAt(1, fresh_stamp);});
  ASSERT_EQ(harness.perception().cacheStats().obstacle_points, 1u);

  const rclcpp::Time older_stamp =
    fresh_stamp - rclcpp::Duration::from_seconds(0.02);
  harness.spinFor(
    40ms, [&]() {harness.publishLeftAt(3, older_stamp);});
  EXPECT_EQ(harness.perception().cacheStats().obstacle_points, 1u);

  const rclcpp::Time stale_stamp =
    harness.now() - rclcpp::Duration::from_seconds(1.0);
  harness.perception().clear();
  harness.spinFor(
    40ms, [&]() {harness.publishLeftAt(3, stale_stamp);});
  const PerceptionHealth health = harness.perception().health(harness.now(), 0.12);
  EXPECT_EQ(health.received_source_count, 1u);
  EXPECT_EQ(health.fresh_source_count, 0u);
  EXPECT_EQ(harness.perception().cacheStats().obstacle_points, 0u);
}

TEST_F(LidarPerceptionTest, DisabledHorizontalSourceIsIgnoredByCacheAndHealth)
{
  LidarHarness::HorizontalConfig horizontal;
  horizontal.left_enabled = false;
  LidarHarness harness("lidar_disabled_left_test", true, horizontal);
  harness.spinFor(200ms, [&]() {harness.publishLeft(10);});

  EXPECT_EQ(harness.perception().cacheStats().obstacle_points, 0u);
  const PerceptionHealth health = harness.perception().health(harness.now(), 1.0);
  EXPECT_EQ(health.received_source_count, 0u);
  EXPECT_EQ(health.fresh_horizontal_source_count, 0u);

  harness.spinFor(200ms, [&]() {harness.publishRight(10);});
  EXPECT_EQ(harness.perception().cacheStats().obstacle_points, 3u);
  const PerceptionHealth mixed = harness.perception().health(harness.now(), 1.0);
  EXPECT_EQ(mixed.received_source_count, 1u);
  EXPECT_EQ(mixed.fresh_horizontal_source_count, 1u);
}

TEST_F(LidarPerceptionTest, DisabledHorizontalFeedLocalMapSkipsPendingQueue)
{
  LidarHarness::HorizontalConfig horizontal;
  horizontal.feed_local_map = false;
  LidarHarness harness("lidar_no_local_map_feed_test", true, horizontal);
  harness.spinFor(200ms, [&]() {harness.publishLeft(10);});

  EXPECT_EQ(harness.perception().cacheStats().obstacle_points, 3u);
  EXPECT_EQ(harness.perception().cacheStats().pending_points, 0u);
}

TEST_F(LidarPerceptionTest, TiltCloudIsTransformedToBaseLinkViaTf)
{
  LidarHarness harness("lidar_tilt_tf_test", true, LidarHarness::HorizontalConfig{}, false);
  harness.publishLeftTiltCloud(0.50f, 0.0f, 0.0f);
  harness.spinFor(200ms, []() {});

  AvoidanceParams params;
  const ObstacleField field =
    harness.perception().buildTiltObstacleField(params, harness.now());
  ASSERT_EQ(field.obstacles.size(), 1u);
  EXPECT_NEAR(field.obstacles.front().x, 0.50, 1e-3);
  EXPECT_NEAR(field.obstacles.front().y, 0.0, 1e-3);

  harness.publishLeftTiltTransform(0.20, 0.10);
  harness.publishLeftTiltCloud(0.50f, 0.0f, 0.0f);
  harness.spinFor(200ms, []() {});

  const ObstacleField shifted =
    harness.perception().buildTiltObstacleField(params, harness.now());
  ASSERT_EQ(shifted.obstacles.size(), 1u);
  EXPECT_NEAR(shifted.obstacles.front().x, 0.70, 1e-3);
  EXPECT_NEAR(shifted.obstacles.front().y, 0.10, 1e-3);
}

TEST_F(LidarPerceptionTest, TiltCloudPreservesSensorZAfterTransform)
{
  LidarHarness harness("lidar_tilt_preserve_z_test", true, LidarHarness::HorizontalConfig{}, false);
  harness.publishLeftTiltTransform(0.20, 0.10);
  harness.publishLeftTiltCloud(0.50f, 0.0f, 0.27f);
  harness.spinFor(200ms, []() {});

  const auto debug_points = harness.perception().takeDebugPoints();
  ASSERT_EQ(debug_points.size(), 1u);
  EXPECT_NEAR(debug_points.front().z, 0.27f, 1e-3);
}

TEST_F(LidarPerceptionTest, TiltCloudWithoutTransformIsNotCached)
{
  LidarHarness harness("lidar_tilt_missing_tf_test", false);
  harness.publishLeftTiltCloud(0.50f, 0.0f, 0.0f);
  harness.spinFor(200ms, []() {});

  AvoidanceParams params;
  const ObstacleField field =
    harness.perception().buildTiltObstacleField(params, harness.now());
  EXPECT_TRUE(field.obstacles.empty());
  const PerceptionHealth health = harness.perception().health(harness.now(), 0.2);
  EXPECT_EQ(health.received_tilt_source_count, 1u);
  EXPECT_EQ(health.fresh_tilt_source_count, 0u);
}

TEST_F(LidarPerceptionTest, DisabledHorizontalMasterSwitchReturnsEmptyField)
{
  LidarHarness::HorizontalConfig horizontal;
  horizontal.enabled = false;
  LidarHarness harness("lidar_disabled_horizontal_test", true, horizontal);
  harness.spinFor(200ms, [&]() {harness.publishLeft(10);});

  AvoidanceParams params;
  const ObstacleField field =
    harness.perception().buildHorizontalObstacleField(params, harness.now());
  EXPECT_TRUE(field.obstacles.empty());
}
