/**
 * @file test_safety_node_io.cpp
 */
#include <gtest/gtest.h>

#include "safety_node/perception_utils.h"
#include "safety_node/safety_node.h"

#include "../../../common/utils/topic.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <interface/msg/joystick.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;

namespace
{

struct HarnessConfig
{
  std::string mode = "pass_through";
  bool local_map_enabled = false;
  bool debug_enabled = false;
  bool require_perception = true;
  bool publish_standard_transforms = true;
  bool self_filter_enabled = true;
  int min_fresh_sources = 1;
  double tf_lookup_timeout_s = 0.01;
  bool rebuild_map_each_frame = true;
};

std::string testJoystickTopic(const std::string & node_name)
{
  return "/" + node_name + "/joystick";
}

rclcpp::NodeOptions testOptions(const std::string & node_name, const HarnessConfig & config)
{
  rclcpp::NodeOptions options;
  options.arguments(
    {
      "--ros-args",
      "-r", "__node:=" + node_name,
      "-r", std::string(joystickTopic) + ":=" + testJoystickTopic(node_name),
    });
  options.parameter_overrides(
    {
      rclcpp::Parameter("control_rate_hz", 50.0),
      rclcpp::Parameter("mode.default", config.mode),
      rclcpp::Parameter("local_map.enabled", config.local_map_enabled),
      rclcpp::Parameter("local_map.update_rate_hz", 20.0),
      rclcpp::Parameter("local_map.publish_viz", config.local_map_enabled),
      rclcpp::Parameter("local_map.viz_rate_hz", 20.0),
      rclcpp::Parameter("local_map.rebuild_each_frame", config.rebuild_map_each_frame),
      rclcpp::Parameter("local_map.decay_halflife_s", 100.0),
      rclcpp::Parameter("local_map.max_hit_age_s", 100.0),
      rclcpp::Parameter("debug.perception.enabled", config.debug_enabled),
      rclcpp::Parameter("debug.perception.rate_hz", 20.0),
      rclcpp::Parameter("debug.perception.max_points", 1000),
      rclcpp::Parameter("perception.max_points_per_source", 1000),
      rclcpp::Parameter("perception.max_pending_points", 5000),
      rclcpp::Parameter("perception.self_filter.enabled", config.self_filter_enabled),
      rclcpp::Parameter("input_health.control_command_timeout_s", 0.12),
      rclcpp::Parameter("input_health.joystick_timeout_s", 0.12),
      rclcpp::Parameter("input_health.joystick_deadzone", 5),
      rclcpp::Parameter("input_health.joystick_reverse_threshold", 20),
      rclcpp::Parameter("input_health.wheel_feedback_timeout_s", 0.12),
      rclcpp::Parameter("input_health.require_wheel_feedback", true),
      rclcpp::Parameter("input_health.perception_timeout_s", 0.12),
      rclcpp::Parameter("input_health.tf_lookup_timeout_s", config.tf_lookup_timeout_s),
      rclcpp::Parameter(
        "input_health.min_fresh_perception_sources",
        config.min_fresh_sources),
      rclcpp::Parameter("input_health.require_perception", config.require_perception),
      rclcpp::Parameter("output.direction_reversal_fast_stop_enabled", true),
      rclcpp::Parameter("output.direction_reversal_stop_decel_m_s2", 2.5),
      rclcpp::Parameter("output.direction_reversal_stop_jerk_m_s3", 8.0),
      rclcpp::Parameter("output.direction_reversal_release_speed_m_s", 0.03),
      // 跳过本机 /opt/freego/user_config.yaml，避免开发机持久化状态干扰单测。
      rclcpp::Parameter("user_config_file", ""),
    });
  return options;
}

geometry_msgs::msg::TransformStamped identityTransform(
  const rclcpp::Time & stamp,
  const std::string & child_frame)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = "base_link";
  transform.child_frame_id = child_frame;
  transform.transform.rotation.w = 1.0;
  return transform;
}

sensor_msgs::msg::PointCloud2 makeCloud(
  const rclcpp::Time & stamp,
  const std::string & frame,
  float x,
  float y,
  float z)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
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
  return cloud;
}

bool hasTopicType(
  const std::map<std::string, std::vector<std::string>> & topics,
  const std::string & topic,
  const std::string & type)
{
  const auto it = topics.find(topic);
  if (it == topics.end()) {
    return false;
  }
  return std::find(it->second.begin(), it->second.end(), type) != it->second.end();
}

struct IoHarness
{
  std::shared_ptr<SafetyNode> safety;
  rclcpp::Node::SharedPtr driver;
  std::string joystick_topic;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_pub;
  rclcpp::Publisher<interface ::msg::Joystick>::SharedPtr joystick_pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr left_scan_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr right_scan_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr back_scan_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_cloud_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr output_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr debug_points_sub;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr markers_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr log_sub;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster;
  rclcpp::executors::SingleThreadedExecutor executor;
  geometry_msgs::msg::Twist last_output;
  sensor_msgs::msg::PointCloud2 last_debug_points;
  visualization_msgs::msg::MarkerArray last_markers;
  nav_msgs::msg::OccupancyGrid last_map;
  std::vector<std::string> logs;
  size_t output_count = 0;
  bool received_output = false;
  bool received_debug_points = false;
  bool received_markers = false;
  bool received_map = false;
  bool outputs_valid = true;
  bool auto_publish_joystick = true;
  double max_abs_linear = 0.0;
  double max_abs_angular = 0.0;

  IoHarness(const std::string & name, const HarnessConfig & config = HarnessConfig{})
  : safety(std::make_shared<SafetyNode>(testOptions(name, config)))
    , driver(std::make_shared<rclcpp::Node>(name + "_driver"))
    , joystick_topic(testJoystickTopic(name))
  {
    safety->attachComponents();
    control_pub = driver->create_publisher<geometry_msgs::msg::Twist>(handleControlTopic, 10);
    joystick_pub =
      driver->create_publisher<interface ::msg::Joystick>(joystick_topic, 10);
    wheel_pub =
      driver->create_publisher<geometry_msgs::msg::TwistStamped>(carWheelSpeedStaTopic, 10);
    left_scan_pub = driver->create_publisher<sensor_msgs::msg::LaserScan>(
      leftLaserTopic, rclcpp::SensorDataQoS());
    right_scan_pub = driver->create_publisher<sensor_msgs::msg::LaserScan>(
      rightLaserTopic, rclcpp::SensorDataQoS());
    back_scan_pub = driver->create_publisher<sensor_msgs::msg::LaserScan>(
      backLaserTopic, rclcpp::SensorDataQoS());
    const auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    left_cloud_pub =
      driver->create_publisher<sensor_msgs::msg::PointCloud2>(leftTiltPointTopice, cloud_qos);
    right_cloud_pub =
      driver->create_publisher<sensor_msgs::msg::PointCloud2>(rightTiltPointTopice, cloud_qos);

    output_sub = driver->create_subscription<geometry_msgs::msg::Twist>(
      nav2ControlTopic, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (msg) {
          last_output = *msg;
          received_output = true;
          ++output_count;
          outputs_valid =
          outputs_valid &&
          std::isfinite(msg->linear.x) &&
          std::isfinite(msg->linear.y) &&
          std::isfinite(msg->linear.z) &&
          std::isfinite(msg->angular.x) &&
          std::isfinite(msg->angular.y) &&
          std::isfinite(msg->angular.z) &&
          msg->linear.y == 0.0 &&
          msg->linear.z == 0.0 &&
          msg->angular.x == 0.0 &&
          msg->angular.y == 0.0;
          max_abs_linear = std::max(max_abs_linear, std::abs(msg->linear.x));
          max_abs_angular = std::max(max_abs_angular, std::abs(msg->angular.z));
        }
      });
    debug_points_sub = driver->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/safety/debug/perception_points", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (msg && !msg->data.empty()) {
          last_debug_points = *msg;
          received_debug_points = true;
        }
      });
    markers_sub = driver->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/safety/debug/perception_markers", 10,
      [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        if (msg && !msg->markers.empty()) {
          last_markers = *msg;
          received_markers = true;
        }
      });
    map_sub = driver->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/safety/local_map", 10,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (msg) {
          last_map = *msg;
          received_map = true;
        }
      });
    log_sub = driver->create_subscription<std_msgs::msg::String>(
      "/safety/intervention_log", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg) {
          logs.push_back(msg->data);
        }
      });

    tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(driver);
    executor.add_node(safety);
    executor.add_node(driver);

    if (config.publish_standard_transforms) {
      publishTransforms(
        {
          "laser_frame_left",
          "laser_frame_right",
          "laser_frame_back",
          "left_tilt_rotate",
          "right_tilt_rotate",
        });
    }
    runFor(100ms, []() {});
  }

  void publishTransforms(const std::vector<std::string> & frames)
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(frames.size());
    for (const auto & frame : frames) {
      transforms.push_back(identityTransform(driver->now(), frame));
    }
    tf_broadcaster->sendTransform(transforms);
  }

  void publishShiftedTransform(const std::string & frame, double x, double y)
  {
    auto transform = identityTransform(driver->now(), frame);
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    tf_broadcaster->sendTransform(transform);
  }

  void publishControl(double linear, double angular = 0.0)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = linear;
    command.angular.z = angular;
    publishControl(command);
  }

  void publishControl(const geometry_msgs::msg::Twist & command)
  {
    if (auto_publish_joystick) {
      const int16_t rocker_x =
        std::isfinite(command.linear.x) && std::abs(command.linear.x) > 1e-9 ?
        (command.linear.x > 0.0 ? 80 : -80) : 0;
      const int16_t rocker_y =
        std::isfinite(command.angular.z) && std::abs(command.angular.z) > 1e-9 ?
        (command.angular.z > 0.0 ? 80 : -80) : 0;
      publishJoystick(rocker_x, rocker_y);
    }
    control_pub->publish(command);
  }

  void publishJoystick(int16_t rocker_x, int16_t rocker_y)
  {
    interface ::msg::Joystick joystick;
    joystick.rocker_x = rocker_x;
    joystick.rocker_y = rocker_y;
    joystick_pub->publish(joystick);
  }

  void publishWheel(double speed)
  {
    publishWheelValues(speed, 0.0);
  }

  void publishWheelValues(double linear, double angular)
  {
    publishWheelValuesAt(linear, angular, driver->now());
  }

  void publishWheelValuesAt(double linear, double angular, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TwistStamped feedback;
    feedback.header.stamp = stamp;
    feedback.twist.linear.x = linear;
    feedback.twist.angular.z = angular;
    wheel_pub->publish(feedback);
  }

  void publishCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const std::string & frame,
    float x,
    float y,
    float z)
  {
    publisher->publish(makeCloud(driver->now(), frame, x, y, z));
  }

  void publishScan(
    const rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr & publisher,
    const std::string & frame,
    float range,
    float angle = 0.0f)
  {
    publishScanAt(publisher, frame, range, driver->now(), angle);
  }

  void publishScanAt(
    const rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr & publisher,
    const std::string & frame,
    float range,
    const rclcpp::Time & stamp,
    float angle = 0.0f)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = stamp;
    scan.header.frame_id = frame;
    scan.angle_min = angle;
    scan.angle_max = angle;
    scan.angle_increment = 0.01f;
    scan.range_min = 0.05f;
    scan.range_max = 20.0f;
    scan.ranges = {range};
    publisher->publish(scan);
  }

  void publishSensor(float range = 4.0f, const std::string & frame = "laser_frame_left")
  {
    publishScan(left_scan_pub, frame, range);
  }

  void publishFiveSensors()
  {
    publishScan(left_scan_pub, "laser_frame_left", 3.0f, 0.0f);
    publishScan(right_scan_pub, "laser_frame_right", 3.1f, 0.0f);
    publishScan(back_scan_pub, "laser_frame_back", 3.2f, 0.0f);
    left_cloud_pub->publish(makeCloud(driver->now(), "left_tilt_rotate", 0.98f, 0.0f, 0.0f));
    right_cloud_pub->publish(makeCloud(driver->now(), "right_tilt_rotate", 0.99f, 0.0f, 0.0f));
  }

  template<typename PublishFn>
  void runFor(std::chrono::milliseconds duration, PublishFn publish)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      publish();
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    executor.spin_some();
  }
};

std::set<std::array<uint8_t, 3>> pointCloudColors(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::set<std::array<uint8_t, 3>> colors;
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(cloud, "r");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(cloud, "g");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(cloud, "b");
  for (; iter_r != iter_r.end(); ++iter_r, ++iter_g, ++iter_b) {
    colors.insert({*iter_r, *iter_g, *iter_b});
  }
  return colors;
}

bool containsLog(const std::vector<std::string> & logs, const std::string & text)
{
  return std::any_of(
    logs.begin(), logs.end(),
    [&text](const std::string & log) {return log.find(text) != std::string::npos;});
}

bool containsExplainedSpeedIntervention(const std::vector<std::string> & logs)
{
  return std::any_of(
    logs.begin(), logs.end(),
    [](const std::string & log) {
      return log.find("\"speed_intervention_active\":true") != std::string::npos &&
      log.find("\"speed_intervention_reason\":\"") != std::string::npos &&
      log.find("\"speed_intervention_reason\":\"none\"") == std::string::npos;
    });
}

size_t residentBytes()
{
  std::ifstream statm("/proc/self/statm");
  size_t total_pages = 0;
  size_t resident_pages = 0;
  statm >> total_pages >> resident_pages;
  (void)total_pages;
  return resident_pages * static_cast<size_t>(sysconf(_SC_PAGESIZE));
}

}  // namespace

class SafetyNodeIoTest : public ::testing::Test
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

TEST_F(SafetyNodeIoTest, ControlTimeoutForcesZeroOutput)
{
  IoHarness harness("safety_node_control_timeout_test");
  harness.runFor(
    500ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
    });

  ASSERT_TRUE(harness.received_output);
  ASSERT_GT(harness.last_output.linear.x, 0.1);

  harness.runFor(350ms, [&]() {harness.publishWheel(0.8);});

  EXPECT_NEAR(harness.last_output.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(harness.last_output.angular.z, 0.0, 1e-9);
}

TEST_F(SafetyNodeIoTest, CenteredJoystickSmoothlyStopsDecayingControl)
{
  IoHarness harness("safety_node_joystick_neutral_test");
  harness.auto_publish_joystick = false;
  harness.runFor(
    600ms, [&]() {
      harness.publishJoystick(80, 0);
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
    });

  ASSERT_TRUE(harness.received_output);
  ASSERT_GT(harness.last_output.linear.x, 0.1);

  harness.runFor(
    80ms, [&]() {
      harness.publishJoystick(0, 0);
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
    });

  EXPECT_GT(harness.last_output.linear.x, 0.0);
  EXPECT_LT(harness.last_output.linear.x, 0.8);
  EXPECT_TRUE(containsLog(harness.logs, "\"speed_intervention_active\":true"));
  EXPECT_TRUE(
    containsLog(harness.logs, "\"speed_intervention_reason\":\"joystick_neutral_brake"));

  harness.runFor(
    1600ms, [&]() {
      harness.publishJoystick(0, 0);
      harness.publishControl(0.8);
      harness.publishWheel(0.0);
    });

  EXPECT_NEAR(harness.last_output.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(harness.last_output.angular.z, 0.0, 1e-9);
}

TEST_F(SafetyNodeIoTest, ReverseJoystickFastStopsResidualForwardCommand)
{
  IoHarness harness("safety_node_reverse_fast_stop_test");
  harness.auto_publish_joystick = false;
  harness.runFor(
    600ms, [&]() {
      harness.publishJoystick(80, 0);
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
    });

  ASSERT_TRUE(harness.received_output);
  const double forward_output = harness.last_output.linear.x;
  ASSERT_GT(forward_output, 0.1);

  harness.runFor(
    300ms, [&]() {
      harness.publishJoystick(-80, 0);
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
    });

  EXPECT_GE(harness.last_output.linear.x, 0.0);
  EXPECT_LT(harness.last_output.linear.x, forward_output - 1e-3);

  harness.runFor(
    1800ms, [&]() {
      harness.publishJoystick(-80, 0);
      harness.publishControl(0.8);
      harness.publishWheel(0.0);
    });

  EXPECT_NEAR(harness.last_output.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(harness.last_output.angular.z, 0.0, 1e-9);

  harness.runFor(
    600ms, [&]() {
      harness.publishJoystick(-80, 0);
      harness.publishControl(-0.4);
      harness.publishWheel(0.0);
    });

  EXPECT_LT(harness.last_output.linear.x, -0.01);
}

TEST_F(SafetyNodeIoTest, WheelTimeoutForcesZeroOutput)
{
  IoHarness harness("safety_node_wheel_timeout_test");
  harness.runFor(
    500ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
    });

  ASSERT_TRUE(harness.received_output);
  ASSERT_GT(harness.last_output.linear.x, 0.1);

  harness.runFor(350ms, [&]() {harness.publishControl(0.8);});

  EXPECT_NEAR(harness.last_output.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(harness.last_output.angular.z, 0.0, 1e-9);
}

TEST_F(SafetyNodeIoTest, PerceptionTimeoutForcesZeroOutputInAssistMode)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  IoHarness harness("safety_node_perception_timeout_test", config);
  harness.runFor(
    600ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
      harness.publishSensor();
    });

  ASSERT_TRUE(harness.received_output);
  ASSERT_GT(harness.last_output.linear.x, 0.1);

  harness.runFor(
    350ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
    });

  EXPECT_NEAR(harness.last_output.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(harness.last_output.angular.z, 0.0, 1e-9);
}

TEST_F(SafetyNodeIoTest, TopicTypesAndQosMatchInterfaceContract)
{
  HarnessConfig config;
  config.debug_enabled = true;
  config.local_map_enabled = true;
  IoHarness harness("safety_node_interface_test", config);
  harness.runFor(400ms, []() {});

  const auto topics = harness.driver->get_topic_names_and_types();
  EXPECT_TRUE(hasTopicType(topics, handleControlTopic, "geometry_msgs/msg/Twist"));
  EXPECT_TRUE(
    hasTopicType(
      topics, testJoystickTopic("safety_node_interface_test"), "interface/msg/Joystick"));
  EXPECT_TRUE(hasTopicType(topics, carWheelSpeedStaTopic, "geometry_msgs/msg/TwistStamped"));
  EXPECT_TRUE(hasTopicType(topics, leftLaserTopic, "sensor_msgs/msg/LaserScan"));
  EXPECT_TRUE(hasTopicType(topics, rightLaserTopic, "sensor_msgs/msg/LaserScan"));
  EXPECT_TRUE(hasTopicType(topics, backLaserTopic, "sensor_msgs/msg/LaserScan"));
  EXPECT_TRUE(hasTopicType(topics, leftTiltPointTopice, "sensor_msgs/msg/PointCloud2"));
  EXPECT_TRUE(hasTopicType(topics, rightTiltPointTopice, "sensor_msgs/msg/PointCloud2"));
  EXPECT_TRUE(hasTopicType(topics, nav2ControlTopic, "geometry_msgs/msg/Twist"));
  EXPECT_TRUE(
    hasTopicType(
      topics, "/safety/debug/perception_points", "sensor_msgs/msg/PointCloud2"));
  EXPECT_TRUE(
    hasTopicType(
      topics, "/safety/debug/perception_markers", "visualization_msgs/msg/MarkerArray"));
  EXPECT_TRUE(hasTopicType(topics, "/safety/local_map", "nav_msgs/msg/OccupancyGrid"));
  EXPECT_TRUE(hasTopicType(topics, "/safety/intervention_log", "std_msgs/msg/String"));
  // 对外状态/预警上报：类型须与 scene_node 逐字节同构，消费端（TFT / 灯效 / 语音 / 底盘）不做适配。
  EXPECT_TRUE(hasTopicType(topics, auxiliaryStatusTopic, "std_msgs/msg/Bool"));
  EXPECT_TRUE(hasTopicType(topics, cursorAlarmTopic, "std_msgs/msg/Int32"));
  EXPECT_TRUE(hasTopicType(topics, collisionWarningTopic, "interface/msg/CollisionWarning"));

  const auto scan_endpoints =
    harness.driver->get_subscriptions_info_by_topic(leftLaserTopic);
  const auto cloud_endpoints =
    harness.driver->get_subscriptions_info_by_topic(leftTiltPointTopice);
  ASSERT_FALSE(scan_endpoints.empty());
  ASSERT_FALSE(cloud_endpoints.empty());
  EXPECT_EQ(
    scan_endpoints.front().qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(
    cloud_endpoints.front().qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::Reliable);
}

TEST_F(SafetyNodeIoTest, FiveSensorSourcesAppearWithExpectedDebugColors)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  config.debug_enabled = true;
  config.min_fresh_sources = 5;
  IoHarness harness("safety_node_five_sensor_debug_test", config);
  harness.runFor(
    700ms, [&]() {
      harness.publishControl(0.4);
      harness.publishWheel(0.4);
      harness.publishFiveSensors();
    });

  ASSERT_TRUE(harness.received_debug_points);
  const auto colors = pointCloudColors(harness.last_debug_points);
  for (uint8_t source = 0; source < 5; ++source) {
    EXPECT_TRUE(colors.count(PerceptionUtils::debugColor(source)) > 0);
  }
  ASSERT_TRUE(harness.received_markers);
  std::set<std::string> namespaces;
  for (const auto & marker : harness.last_markers.markers) {
    namespaces.insert(marker.ns);
  }
  EXPECT_TRUE(namespaces.count("perception_roi") > 0);
  EXPECT_TRUE(namespaces.count("vehicle_footprint_margin") > 0);
  EXPECT_TRUE(namespaces.count("vehicle_footprint_physical") > 0);
  EXPECT_TRUE(namespaces.count("horizontal_obstacle_primitives") > 0);
  EXPECT_TRUE(namespaces.count("tilt_obstacle_primitives") > 0);
  EXPECT_TRUE(namespaces.count("published_command_trajectory") > 0);
  EXPECT_TRUE(namespaces.count("joystick_intent_trajectory") > 0);
}

TEST_F(SafetyNodeIoTest, MessageFrameIdOverridesFallbackAndTransformIsApplied)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  config.debug_enabled = true;
  config.publish_standard_transforms = false;
  IoHarness harness("safety_node_frame_override_test", config);
  harness.publishShiftedTransform("custom_scan_frame", 1.0, 0.25);
  harness.runFor(
    700ms, [&]() {
      harness.publishControl(0.4);
      harness.publishWheel(0.4);
      harness.publishSensor(3.0f, "custom_scan_frame");
    });

  ASSERT_TRUE(harness.received_debug_points);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(harness.last_debug_points, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(harness.last_debug_points, "y");
  ASSERT_NE(iter_x, iter_x.end());
  EXPECT_NEAR(*iter_x, 4.0, 1e-3);
  EXPECT_NEAR(*iter_y, 0.25, 1e-3);
  ASSERT_TRUE(harness.received_output);
  EXPECT_GT(harness.last_output.linear.x, 0.05);
}

TEST_F(SafetyNodeIoTest, MissingTfFailsSafeWithoutBlockingControlLoop)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  config.publish_standard_transforms = false;
  config.tf_lookup_timeout_s = 0.01;
  IoHarness harness("safety_node_missing_tf_test", config);
  const size_t initial_outputs = harness.output_count;
  harness.runFor(
    600ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
      harness.publishSensor(4.0f, "missing_scan_frame");
    });

  EXPECT_GE(harness.output_count - initial_outputs, 15u);
  EXPECT_NEAR(harness.last_output.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(harness.last_output.angular.z, 0.0, 1e-9);
}

TEST_F(SafetyNodeIoTest, DelayedTfRecoversAfterTransformBecomesAvailable)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  config.publish_standard_transforms = false;
  IoHarness harness("safety_node_delayed_tf_test", config);
  harness.runFor(
    250ms, [&]() {
      harness.publishControl(0.6);
      harness.publishWheel(0.6);
      harness.publishSensor(4.0f);
    });
  ASSERT_TRUE(harness.received_output);
  EXPECT_DOUBLE_EQ(harness.last_output.linear.x, 0.0);

  harness.publishTransforms({"laser_frame_left"});
  harness.runFor(
    600ms, [&]() {
      harness.publishControl(0.6);
      harness.publishWheel(0.6);
      harness.publishSensor(4.0f);
    });
  EXPECT_GT(harness.last_output.linear.x, 0.05);
}

TEST_F(SafetyNodeIoTest, NoObstacleCommandPassesAndOutputIsPlanarFinite)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  IoHarness harness("safety_node_no_obstacle_test", config);
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.8;
  command.linear.y = 7.0;
  command.linear.z = -3.0;
  command.angular.x = 4.0;
  command.angular.y = -5.0;
  command.angular.z = 0.2;
  harness.runFor(
    700ms, [&]() {
      harness.publishControl(command);
      harness.publishWheel(0.8);
      harness.publishSensor(4.0f);
    });

  ASSERT_TRUE(harness.received_output);
  EXPECT_GT(harness.last_output.linear.x, 0.05);
  EXPECT_TRUE(std::isfinite(harness.last_output.linear.x));
  EXPECT_TRUE(std::isfinite(harness.last_output.angular.z));
  EXPECT_DOUBLE_EQ(harness.last_output.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(harness.last_output.linear.z, 0.0);
  EXPECT_DOUBLE_EQ(harness.last_output.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(harness.last_output.angular.y, 0.0);
}

TEST_F(SafetyNodeIoTest, NonFiniteDesiredCommandFailsSafe)
{
  IoHarness harness("safety_node_nonfinite_command_test");
  geometry_msgs::msg::Twist command;
  command.linear.x = std::numeric_limits<double>::quiet_NaN();
  command.angular.z = std::numeric_limits<double>::infinity();
  harness.runFor(
    350ms, [&]() {
      harness.publishControl(command);
      harness.publishWheel(0.0);
    });

  ASSERT_TRUE(harness.received_output);
  EXPECT_DOUBLE_EQ(harness.last_output.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(harness.last_output.angular.z, 0.0);
}

TEST_F(SafetyNodeIoTest, NonFiniteWheelFeedbackFailsSafe)
{
  IoHarness harness("safety_node_nonfinite_wheel_test");
  harness.runFor(
    350ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheelValues(
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity());
    });

  ASSERT_TRUE(harness.received_output);
  EXPECT_DOUBLE_EQ(harness.last_output.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(harness.last_output.angular.z, 0.0);
  EXPECT_TRUE(harness.outputs_valid);
}

TEST_F(SafetyNodeIoTest, RepeatedSensorTimestampCannotKeepAssistActive)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  IoHarness harness("safety_node_frozen_sensor_stamp_test", config);
  const rclcpp::Time frozen_stamp = harness.driver->now();
  harness.runFor(
    350ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
      harness.publishScanAt(
        harness.left_scan_pub, "laser_frame_left", 4.0f, frozen_stamp);
    });

  ASSERT_TRUE(harness.received_output);
  EXPECT_DOUBLE_EQ(harness.last_output.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(harness.last_output.angular.z, 0.0);
}

TEST_F(SafetyNodeIoTest, SensorDelayThresholdAccepts100MsAndRejects200Ms)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  IoHarness harness("safety_node_sensor_delay_test", config);
  harness.runFor(
    500ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
      harness.publishScanAt(
        harness.left_scan_pub,
        "laser_frame_left",
        4.0f,
        harness.driver->now() - rclcpp::Duration::from_seconds(0.10));
    });
  ASSERT_TRUE(harness.received_output);
  EXPECT_GT(harness.last_output.linear.x, 0.05);

  harness.runFor(
    350ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
      harness.publishScanAt(
        harness.left_scan_pub,
        "laser_frame_left",
        4.0f,
        harness.driver->now() - rclcpp::Duration::from_seconds(0.20));
    });
  EXPECT_DOUBLE_EQ(harness.last_output.linear.x, 0.0);
}

TEST_F(SafetyNodeIoTest, OutOfOrderSensorFrameDoesNotOverrideLatestClearFrame)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  IoHarness harness("safety_node_out_of_order_sensor_test", config);
  harness.runFor(
    500ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
      harness.publishScan(harness.left_scan_pub, "laser_frame_left", 4.0f);
    });
  ASSERT_TRUE(harness.received_output);
  ASSERT_GT(harness.last_output.linear.x, 0.05);

  const rclcpp::Time older_near_stamp =
    harness.driver->now() - rclcpp::Duration::from_seconds(0.08);
  harness.runFor(
    60ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
      harness.publishScanAt(
        harness.left_scan_pub,
        "laser_frame_left",
        0.6f,
        older_near_stamp);
    });

  EXPECT_GT(harness.last_output.linear.x, 0.05);
}

TEST_F(SafetyNodeIoTest, FiftyPercentSensorPacketLossKeepsAssistActive)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  IoHarness harness("safety_node_packet_loss_test", config);
  harness.runFor(
    500ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
      harness.publishSensor(4.0f);
    });
  ASSERT_TRUE(harness.received_output);
  ASSERT_GT(harness.last_output.linear.x, 0.05);

  int tick = 0;
  harness.runFor(
    500ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
      if ((tick++ % 2) == 0) {
        harness.publishSensor(4.0f);
      }
    });

  ASSERT_TRUE(harness.received_output);
  EXPECT_GT(harness.last_output.linear.x, 0.05);
  EXPECT_TRUE(harness.outputs_valid);
}

TEST_F(SafetyNodeIoTest, RepeatedWheelTimestampCannotKeepNodeActive)
{
  IoHarness harness("safety_node_frozen_wheel_stamp_io_test");
  const rclcpp::Time frozen_stamp = harness.driver->now();
  harness.runFor(
    350ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheelValuesAt(0.8, 0.0, frozen_stamp);
    });

  ASSERT_TRUE(harness.received_output);
  EXPECT_DOUBLE_EQ(harness.last_output.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(harness.last_output.angular.z, 0.0);
}

TEST_F(SafetyNodeIoTest, NonFinitePointCloudInputDoesNotLeakToControlOutput)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  config.debug_enabled = true;
  IoHarness harness("safety_node_nonfinite_cloud_test", config);
  harness.runFor(
    600ms, [&]() {
      harness.publishControl(0.6);
      harness.publishWheel(0.6);
      harness.publishCloud(
        harness.left_cloud_pub,
        "left_tilt_rotate",
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::quiet_NaN(),
        0.0f);
    });

  ASSERT_TRUE(harness.received_output);
  EXPECT_TRUE(harness.outputs_valid);
  EXPECT_TRUE(std::isfinite(harness.last_output.linear.x));
  EXPECT_TRUE(std::isfinite(harness.last_output.angular.z));
}

TEST_F(SafetyNodeIoTest, TiltRadarEmergencyBypassesOutputSmoothing)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  config.self_filter_enabled = false;
  IoHarness harness("safety_node_tilt_hard_stop_test", config);
  harness.runFor(
    600ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
      harness.publishSensor(4.0f);
    });

  ASSERT_TRUE(harness.received_output);
  ASSERT_GT(harness.last_output.linear.x, 0.1);

  harness.runFor(
    120ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
      harness.publishSensor(4.0f);
      harness.publishCloud(
        harness.left_cloud_pub, "left_tilt_rotate", 0.08f, 0.0f, 0.0f);
    });

  EXPECT_DOUBLE_EQ(harness.last_output.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(harness.last_output.angular.z, 0.0);
  EXPECT_TRUE(containsLog(harness.logs, "\"tilt_intervention\":\"emergency\""));
}

TEST_F(SafetyNodeIoTest, SingleSensorLossKeepsOperatingWhenOneFreshSourceIsEnough)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  config.min_fresh_sources = 1;
  IoHarness harness("safety_node_single_sensor_loss_test", config);
  harness.runFor(
    300ms, [&]() {
      harness.publishControl(0.6);
      harness.publishWheel(0.6);
      harness.publishScan(harness.left_scan_pub, "laser_frame_left", 4.0f);
      harness.publishScan(harness.right_scan_pub, "laser_frame_right", 4.0f);
    });

  harness.runFor(
    350ms, [&]() {
      harness.publishControl(0.6);
      harness.publishWheel(0.6);
      harness.publishScan(harness.right_scan_pub, "laser_frame_right", 4.0f);
    });
  ASSERT_TRUE(harness.received_output);
  EXPECT_GT(harness.last_output.linear.x, 0.1);
}

TEST_F(SafetyNodeIoTest, FrontObstacleReducesForwardCommand)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  IoHarness harness("safety_node_front_obstacle_test", config);
  harness.runFor(
    500ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
      harness.publishSensor(0.8f);
    });

  ASSERT_TRUE(harness.received_output);
  EXPECT_LT(harness.last_output.linear.x, 0.1);
  EXPECT_TRUE(containsLog(harness.logs, "\"emergency_stop\":true"));
  EXPECT_TRUE(containsLog(harness.logs, "\"emergency_reason\":\""));
  EXPECT_TRUE(containsLog(harness.logs, "\"has_collision\":true"));
  EXPECT_TRUE(containsLog(harness.logs, "\"min_ttc\":"));
  EXPECT_TRUE(containsLog(harness.logs, "\"min_clearance\":"));
  EXPECT_TRUE(containsLog(harness.logs, "\"map_ttc\":null"));
  EXPECT_FALSE(containsLog(harness.logs, "\"map_ttc\":inf"));
  EXPECT_FALSE(containsLog(harness.logs, "\"min_ttc\":inf"));
  EXPECT_FALSE(containsLog(harness.logs, "\"min_clearance\":inf"));
  EXPECT_TRUE(containsExplainedSpeedIntervention(harness.logs));
  EXPECT_TRUE(containsLog(harness.logs, "\"speed_debug_joystick_linear_x\":"));
  EXPECT_TRUE(containsLog(harness.logs, "\"speed_debug_controller_linear_x\":"));
  EXPECT_TRUE(containsLog(harness.logs, "\"speed_debug_output_linear_x\":"));
  EXPECT_TRUE(containsLog(harness.logs, "\"speed_debug_reduction_m_s\":"));
}

TEST_F(SafetyNodeIoTest, RearObstacleReducesReverseCommand)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  IoHarness harness("safety_node_rear_obstacle_test", config);
  harness.runFor(
    500ms, [&]() {
      harness.publishControl(-0.6);
      harness.publishWheel(-0.6);
      harness.publishScan(
        harness.back_scan_pub, "laser_frame_back", 0.5f,
        static_cast<float>(3.14159265358979323846));
    });

  ASSERT_TRUE(harness.received_output);
  EXPECT_GT(harness.last_output.linear.x, -0.1);
}

TEST_F(SafetyNodeIoTest, LeftObstacleChangesUnsafeLeftTurn)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  IoHarness harness("safety_node_left_turn_obstacle_test", config);
  harness.runFor(
    600ms, [&]() {
      harness.publishControl(0.4, 1.0);
      harness.publishWheel(0.4);
      harness.publishScan(
        harness.left_scan_pub, "laser_frame_left", 0.55f,
        static_cast<float>(1.0));
    });

  ASSERT_TRUE(harness.received_output);
  EXPECT_TRUE(
    harness.last_output.linear.x < 0.35 ||
    harness.last_output.angular.z < 0.8);
}

TEST_F(SafetyNodeIoTest, HistoricalLocalMapObstacleUsesDistanceEnvelope)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  config.local_map_enabled = true;
  config.rebuild_map_each_frame = false;
  IoHarness harness("safety_node_map_history_test", config);

  harness.runFor(
    400ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
      harness.publishSensor(4.0f);
    });
  ASSERT_GT(harness.last_output.linear.x, 0.05);

  harness.runFor(
    250ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
      harness.publishSensor(1.0f);
    });
  harness.logs.clear();
  harness.runFor(
    350ms, [&]() {
      harness.publishControl(0.8);
      harness.publishWheel(0.8);
      harness.publishSensor(4.0f);
    });

  EXPECT_TRUE(harness.received_map);
  EXPECT_TRUE(containsLog(harness.logs, "\"braking_limited\":true"));
  EXPECT_LT(harness.last_output.linear.x, 0.2);
}

TEST_F(SafetyNodeIoTest, EstopForcesZeroWithinOneControlCycle)
{
  HarnessConfig config;
  config.mode = "estop";
  IoHarness harness("safety_node_estop_io_test", config);
  const size_t initial_outputs = harness.output_count;
  harness.runFor(
    80ms, [&]() {
      harness.publishControl(1.0, 1.0);
      harness.publishWheel(1.0);
    });

  ASSERT_GT(harness.output_count, initial_outputs);
  EXPECT_DOUBLE_EQ(harness.last_output.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(harness.last_output.angular.z, 0.0);
}

TEST_F(SafetyNodeIoTest, HighFrequencyFiveSensorInputRemainsStable)
{
  HarnessConfig config;
  config.mode = "safe_assist";
  config.local_map_enabled = true;
  config.debug_enabled = true;
  config.min_fresh_sources = 5;
  IoHarness harness("safety_node_high_frequency_test", config);

  const auto publish_all = [&]() {
      harness.publishControl(0.5, 0.2);
      harness.publishWheel(0.5);
      harness.publishFiveSensors();
    };
  harness.runFor(1s, publish_all);
  const size_t rss_before = residentBytes();
  const size_t outputs_before = harness.output_count;

  int soak_seconds = 2;
  if (const char * value = std::getenv("SAFETY_NODE_SOAK_SECONDS")) {
    soak_seconds = std::max(1, std::atoi(value));
  }
  harness.runFor(std::chrono::seconds(soak_seconds), publish_all);
  const size_t rss_after = residentBytes();

  EXPECT_TRUE(harness.outputs_valid);
  EXPECT_LE(harness.max_abs_linear, 2.222 + 1e-9);
  EXPECT_LE(harness.max_abs_angular, 3.0 + 1e-9);
  EXPECT_GE(
    harness.output_count - outputs_before,
    static_cast<size_t>(soak_seconds * 30));
  const size_t memory_growth =
    rss_after > rss_before ? rss_after - rss_before : 0u;
  EXPECT_LT(memory_growth, 64u * 1024u * 1024u);
}
