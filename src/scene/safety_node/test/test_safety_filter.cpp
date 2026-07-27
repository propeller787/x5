/**
 * @file test_safety_filter.cpp
 */
#include <gtest/gtest.h>

#include "safety_node/core/safety_filter.h"

namespace
{

TrajectoryRollout candidateAt(double v, double w, double x, double y, double t)
{
  TrajectoryRollout rollout;
  rollout.cmd.v = v;
  rollout.cmd.w = w;
  rollout.poses.push_back({x, y, 0.0, t});
  return rollout;
}

ObstacleField obstacleAtOrigin()
{
  ObstaclePrimitive obstacle;
  obstacle.x = 0.0;
  obstacle.y = 0.0;
  obstacle.radius = 0.05;
  obstacle.valid = true;
  obstacle.source = "test";
  ObstacleField field;
  field.obstacles.push_back(obstacle);
  return field;
}

}  // namespace

TEST(SafetyFilterTest, EmptyRolloutSetKeepsDesiredCommand)
{
  AvoidanceParams params;
  VehicleFootprint footprint(params);
  SweptFootprintChecker checker(params, footprint);
  SafetyFilter filter(params, checker);
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.7;
  command.angular.z = -0.3;

  const auto result = filter.selectSafeVelocity(command, {}, {}, 0.0, 0.0, 0.1);

  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.7);
  EXPECT_DOUBLE_EQ(result.safe_twist.angular.z, -0.3);
  EXPECT_EQ(result.selected_candidate_index, -1);
}

TEST(SafetyFilterTest, ChoosesClosestSafeCandidateAndAggregatesRejectedRisk)
{
  AvoidanceParams params;
  params.max_decel_m_s2 = 0.0;
  params.max_angular_decel_rad_s2 = 0.0;
  VehicleFootprint footprint(params);
  SweptFootprintChecker checker(params, footprint);
  SafetyFilter filter(params, checker);

  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  command.angular.z = 0.5;
  const std::vector<TrajectoryRollout> rollouts = {
    candidateAt(1.0, 0.5, 0.0, 0.0, 0.1),
    candidateAt(0.7, 0.4, -3.0, 0.0, 0.2),
    candidateAt(0.2, 0.0, -3.0, 0.0, 0.2),
  };

  const auto result = filter.selectSafeVelocity(
    command, rollouts, obstacleAtOrigin(), 0.0, 0.0, 0.0);

  EXPECT_EQ(result.selected_candidate_index, 1);
  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.7);
  EXPECT_DOUBLE_EQ(result.safe_twist.angular.z, 0.4);
  EXPECT_TRUE(result.risk.has_collision);
  EXPECT_NEAR(result.risk.min_ttc, 0.1, 1e-9);
}

TEST(SafetyFilterTest, AllUnsafeCandidatesPreferLongestPathThenLowestSpeed)
{
  AvoidanceParams params;
  params.max_decel_m_s2 = 0.0;
  params.max_angular_decel_rad_s2 = 0.0;
  VehicleFootprint footprint(params);
  SweptFootprintChecker checker(params, footprint);
  SafetyFilter filter(params, checker);
  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  const std::vector<TrajectoryRollout> rollouts = {
    candidateAt(0.8, 0.0, 0.0, 0.0, 0.1),
    candidateAt(0.4, 0.0, 0.0, 0.0, 0.5),
    candidateAt(0.2, 0.0, 0.0, 0.0, 0.5),
  };

  const auto result = filter.selectSafeVelocity(
    command, rollouts, obstacleAtOrigin(), 0.0, 0.0, 0.0);

  EXPECT_TRUE(result.all_candidates_collide);
  EXPECT_EQ(result.selected_candidate_index, 2);
  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.2);
}

TEST(SafetyFilterTest, AppliesRateLimitsAndAbsoluteClamps)
{
  AvoidanceParams params;
  params.max_decel_m_s2 = 1.0;
  params.max_recover_accel_m_s2 = 0.5;
  params.max_angular_decel_rad_s2 = 2.0;
  params.max_angular_recover_rad_s2 = 1.0;
  VehicleFootprint footprint(params);
  SweptFootprintChecker checker(params, footprint);
  SafetyFilter filter(params, checker);

  geometry_msgs::msg::Twist command;
  command.linear.x = -1.0;
  command.angular.z = -1.0;
  auto result = filter.selectSafeVelocity(
    command, {candidateAt(-1.0, -1.0, -3.0, 0.0, 0.0)}, {}, 1.0, 1.0, 0.1);
  EXPECT_NEAR(result.safe_twist.linear.x, 0.9, 1e-9);
  EXPECT_NEAR(result.safe_twist.angular.z, 0.8, 1e-9);

  command.linear.x = 1.0;
  command.angular.z = 1.0;
  result = filter.selectSafeVelocity(
    command, {candidateAt(1.0, 1.0, -3.0, 0.0, 0.0)}, {}, 0.0, 0.0, 0.1);
  EXPECT_NEAR(result.safe_twist.linear.x, 0.05, 1e-9);
  EXPECT_NEAR(result.safe_twist.angular.z, 0.1, 1e-9);

  params.max_decel_m_s2 = 0.0;
  params.max_angular_decel_rad_s2 = 0.0;
  params.absolute_max_linear_speed = 0.6;
  params.absolute_max_angular_speed = 0.7;
  filter.updateParams(params);
  result = filter.selectSafeVelocity(
    command, {candidateAt(3.0, -3.0, -3.0, 0.0, 0.0)}, {}, 0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.6);
  EXPECT_DOUBLE_EQ(result.safe_twist.angular.z, -0.7);
}
