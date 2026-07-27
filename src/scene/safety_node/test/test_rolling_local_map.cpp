/**
 * @file test_rolling_local_map.cpp
 */
#include <gtest/gtest.h>

#include "safety_node/rolling_local_map.h"

#include <cmath>

TEST(RollingLocalMapTest, SingleHitIsLethalAfterInflation)
{
  LocalMapConfig cfg;
  cfg.width_m = 2.0;
  cfg.height_m = 2.0;
  cfg.resolution_m = 0.1;
  cfg.inflation_radius_m = 0.15;
  cfg.lethal_threshold = 0.65;
  cfg.rebuild_each_frame = true;

  RollingLocalMap map(cfg);
  const rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
  std::vector<std::tuple<double, double, double>> pts = {{0.5, 0.0, 0.0}};
  map.integratePoints(pts, stamp);
  map.finalizeFrame(stamp);

  EXPECT_TRUE(map.lethalQuery(0.5, 0.0, cfg.lethal_threshold));
  EXPECT_GT(map.queryCost(0.58, 0.0), 0.1f);
}

TEST(RollingLocalMapTest, EmptyMapIsNotLethal)
{
  LocalMapConfig cfg;
  RollingLocalMap map(cfg);
  const rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
  map.finalizeFrame(stamp);
  EXPECT_FALSE(map.lethalQuery(0.0, 0.0, cfg.lethal_threshold));
}

TEST(RollingLocalMapTest, RebuildClearsPreviousFrame)
{
  LocalMapConfig cfg;
  cfg.rebuild_each_frame = true;
  RollingLocalMap map(cfg);
  const rclcpp::Time t1(1, 0, RCL_ROS_TIME);
  map.integratePoints({{0.3, 0.0, 0.0}}, t1);
  map.finalizeFrame(t1);
  EXPECT_GT(map.queryCost(0.3, 0.0), 0.5f);

  const rclcpp::Time t2(2, 0, RCL_ROS_TIME);
  map.integratePoints({}, t2);
  map.finalizeFrame(t2);
  EXPECT_LT(map.queryCost(0.3, 0.0), 0.1f);
}

TEST(RollingLocalMapTest, NonRebuildMapDecaysOverTime)
{
  LocalMapConfig cfg;
  cfg.rebuild_each_frame = false;
  cfg.decay_halflife_s = 1.0;
  cfg.max_hit_age_s = 10.0;
  cfg.inflation_radius_m = 0.0;
  RollingLocalMap map(cfg);

  const rclcpp::Time t1(1, 0, RCL_ROS_TIME);
  map.integratePoints({{0.3, 0.0, 0.0}}, t1);
  map.finalizeFrame(t1);
  const float initial = map.queryCost(0.3, 0.0);

  const rclcpp::Time t2(2, 0, RCL_ROS_TIME);
  map.finalizeFrame(t2);
  EXPECT_NEAR(map.queryCost(0.3, 0.0), initial * 0.5f, 0.05f);
}

TEST(RollingLocalMapTest, NonRebuildMapPrunesExpiredCells)
{
  LocalMapConfig cfg;
  cfg.rebuild_each_frame = false;
  cfg.max_hit_age_s = 0.5;
  RollingLocalMap map(cfg);

  const rclcpp::Time t1(1, 0, RCL_ROS_TIME);
  map.integratePoints({{0.3, 0.0, 0.0}}, t1);
  map.finalizeFrame(t1);
  EXPECT_GT(map.queryCost(0.3, 0.0), 0.5f);

  map.finalizeFrame(rclcpp::Time(2, 0, RCL_ROS_TIME));
  EXPECT_FLOAT_EQ(map.queryCost(0.3, 0.0), 0.0f);
}

TEST(RollingLocalMapTest, TiltAndOutOfBoundsPointsAreIgnored)
{
  LocalMapConfig cfg;
  cfg.width_m = 2.0;
  cfg.height_m = 2.0;
  cfg.tilt_z_threshold = 0.2;
  RollingLocalMap map(cfg);
  const rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
  map.integratePoints({{0.0, 0.0, 0.3}, {2.0, 0.0, 0.0}}, stamp);
  map.finalizeFrame(stamp);
  EXPECT_FLOAT_EQ(map.queryCost(0.0, 0.0), 0.0f);
}

TEST(RollingLocalMapTest, UpdateResetPrimitiveIntegrationAndMessageExport)
{
  LocalMapConfig cfg;
  cfg.width_m = 2.0;
  cfg.height_m = 2.0;
  cfg.resolution_m = 0.1;
  RollingLocalMap map(cfg);
  cfg.observation_weight = 0.9;
  map.updateConfig(cfg);

  ObstaclePrimitive invalid;
  invalid.valid = false;
  ObstaclePrimitive outside;
  outside.x = 3.0;
  outside.valid = true;
  ObstaclePrimitive valid;
  valid.x = 0.2;
  valid.y = -0.2;
  valid.valid = true;
  ObstacleField field;
  field.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  field.obstacles = {invalid, outside, valid};
  map.integratePrimitives(field, 0.8);
  map.finalizeFrame(field.stamp);
  EXPECT_GT(map.queryCost(valid.x, valid.y), 0.5f);
  EXPECT_FLOAT_EQ(map.queryCost(5.0, 5.0), 0.0f);

  const auto message = map.toOccupancyGridMsg(field.stamp);
  EXPECT_EQ(message.header.frame_id, "base_link");
  EXPECT_EQ(message.data.size(), map.grid().probability.size());

  map.reset();
  EXPECT_FLOAT_EQ(map.queryCost(valid.x, valid.y), 0.0f);
}
