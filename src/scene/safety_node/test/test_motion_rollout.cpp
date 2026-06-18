/**
 * @file test_motion_rollout.cpp
 */
#include <gtest/gtest.h>

#include "safety_node/core/motion_rollout.h"

TEST(MotionRolloutTest, ZeroCommandIncludesFeedbackBrakingDistance)
{
  AvoidanceParams params;
  params.rollout_horizon_s = 1.2;
  params.rollout_dt_s = 0.1;
  params.max_decel_m_s2 = 1.0;
  MotionRollout rollout(params);

  VelocityCandidate stop;
  const TrajectoryRollout trajectory = rollout.rolloutCandidate(stop, 1.0, 0.0);

  ASSERT_FALSE(trajectory.poses.empty());
  EXPECT_NEAR(trajectory.poses.back().x, 0.45, 1e-6);
  EXPECT_DOUBLE_EQ(trajectory.poses.back().y, 0.0);
}

TEST(MotionRolloutTest, ZeroFeedbackAndZeroCommandRemainStationary)
{
  MotionRollout rollout(AvoidanceParams{});

  geometry_msgs::msg::Twist stop;
  const auto trajectories = rollout.rolloutAll(stop, 0.0, 0.0);

  ASSERT_EQ(trajectories.size(), 6u);
  for (const auto & trajectory : trajectories) {
    ASSERT_FALSE(trajectory.poses.empty());
    EXPECT_DOUBLE_EQ(trajectory.poses.back().x, 0.0);
    EXPECT_DOUBLE_EQ(trajectory.poses.back().y, 0.0);
  }
}

TEST(MotionRolloutTest, PositionPredictionStartsFromWheelFeedback)
{
  AvoidanceParams params;
  params.rollout_horizon_s = 1.0;
  params.rollout_dt_s = 0.1;
  MotionRollout rollout(params);

  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  const auto trajectories = rollout.rolloutAll(command, 0.2, 0.0);

  ASSERT_FALSE(trajectories.empty());
  EXPECT_NEAR(trajectories.front().poses.back().x, 0.475, 1e-6);
}

TEST(MotionRolloutTest, StoppedWheelFeedbackDoesNotPredictInstantCommandSpeed)
{
  AvoidanceParams params;
  params.rollout_horizon_s = 1.0;
  params.rollout_dt_s = 0.1;
  MotionRollout rollout(params);

  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  const auto trajectories = rollout.rolloutAll(command, 0.0, 0.0);

  ASSERT_FALSE(trajectories.empty());
  EXPECT_NEAR(trajectories.front().poses.back().x, 0.275, 1e-6);
  EXPECT_LT(trajectories.front().poses.back().x, 1.0);
}

TEST(MotionRolloutTest, StoppedWheelFeedbackDoesNotPredictInstantCommandYawRate)
{
  AvoidanceParams params;
  params.rollout_horizon_s = 1.0;
  params.rollout_dt_s = 0.1;
  MotionRollout rollout(params);

  geometry_msgs::msg::Twist command;
  command.angular.z = 1.0;
  const auto trajectories = rollout.rolloutAll(command, 0.0, 0.0);

  ASSERT_FALSE(trajectories.empty());
  EXPECT_NEAR(trajectories.front().poses.back().theta, 0.55, 1e-6);
  EXPECT_LT(trajectories.front().poses.back().theta, 1.0);
}

TEST(MotionRolloutTest, ZeroCommandProducesOnlyStationaryCandidates)
{
  MotionRollout rollout(AvoidanceParams{});
  geometry_msgs::msg::Twist command;
  const auto candidates = rollout.sampleCandidates(command, 0.0, 0.0);
  ASSERT_FALSE(candidates.empty());
  for (const auto & candidate : candidates) {
    EXPECT_DOUBLE_EQ(candidate.v, 0.0);
    EXPECT_DOUBLE_EQ(candidate.w, 0.0);
  }
}
