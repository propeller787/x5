#include <lidar_imu_fusion/lidar_imu_fusion_node.hpp>

#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace
{

sensor_msgs::msg::LaserScan::SharedPtr makeScan(int64_t stamp_ns)
{
  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  scan->header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
  return scan;
}

}  // namespace

TEST(ScanSynchronizer, MatchesThreeScansWithinSlop)
{
  lidar_imu_fusion::ScanSynchronizer synchronizer;
  synchronizer.setMaxSlop(0.03);
  synchronizer.setQueueSize(4);

  synchronizer.push(lidar_imu_fusion::SensorId::Left, makeScan(1000000000LL));
  synchronizer.push(lidar_imu_fusion::SensorId::Right, makeScan(1010000000LL));
  synchronizer.push(lidar_imu_fusion::SensorId::Back, makeScan(1020000000LL));

  auto result = synchronizer.tryPop();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(rclcpp::Time(result->left->header.stamp).nanoseconds(), 1000000000LL);
  EXPECT_EQ(rclcpp::Time(result->right->header.stamp).nanoseconds(), 1010000000LL);
  EXPECT_EQ(rclcpp::Time(result->back->header.stamp).nanoseconds(), 1020000000LL);
}

TEST(ScanSynchronizer, RejectsScansOutsideSlop)
{
  lidar_imu_fusion::ScanSynchronizer synchronizer;
  synchronizer.setMaxSlop(0.01);
  synchronizer.setQueueSize(4);

  synchronizer.push(lidar_imu_fusion::SensorId::Left, makeScan(1000000000LL));
  synchronizer.push(lidar_imu_fusion::SensorId::Right, makeScan(1030000000LL));
  synchronizer.push(lidar_imu_fusion::SensorId::Back, makeScan(1040000000LL));

  EXPECT_FALSE(synchronizer.tryPop().has_value());
}

TEST(ImuBuffer, ReportsCoverageInsideBufferedRange)
{
  lidar_imu_fusion::ImuBuffer buffer;
  buffer.setRetention(1.0);
  buffer.setMaxExtrapolation(0.01);

  buffer.push(rclcpp::Time(1000000000LL, RCL_ROS_TIME), tf2::Vector3(0.0, 0.0, 1.0));
  buffer.push(rclcpp::Time(1010000000LL, RCL_ROS_TIME), tf2::Vector3(0.0, 0.0, 1.0));

  EXPECT_TRUE(
    buffer.hasCoverage(
      rclcpp::Time(1000000000LL, RCL_ROS_TIME),
      rclcpp::Time(1010000000LL, RCL_ROS_TIME)));
  EXPECT_FALSE(
    buffer.hasCoverage(
      rclcpp::Time(900000000LL, RCL_ROS_TIME),
      rclcpp::Time(1010000000LL, RCL_ROS_TIME)));
}

TEST(ImuBuffer, IntegratesConstantYawRate)
{
  lidar_imu_fusion::ImuBuffer buffer;
  buffer.setRetention(1.0);
  buffer.setMaxExtrapolation(0.0);

  for (int i = 0; i <= 10; ++i) {
    buffer.push(
      rclcpp::Time(static_cast<int64_t>(i) * 10000000LL, RCL_ROS_TIME),
      tf2::Vector3(0.0, 0.0, 1.0));
  }

  tf2::Quaternion delta;
  ASSERT_TRUE(
    buffer.integrateRotation(
      rclcpp::Time(0LL, RCL_ROS_TIME),
      rclcpp::Time(100000000LL, RCL_ROS_TIME),
      &delta));

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(delta).getRPY(roll, pitch, yaw);

  EXPECT_NEAR(roll, 0.0, 1.0e-6);
  EXPECT_NEAR(pitch, 0.0, 1.0e-6);
  EXPECT_NEAR(yaw, 0.1, 1.0e-6);
}
