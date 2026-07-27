#include "risk_assessment.h"

#include <gtest/gtest.h>

#include <map>
#include <vector>

namespace
{

ClusterInfo makeCluster(double x, double y, double radius)
{
  ClusterInfo cluster;
  cluster.center_x = x;
  cluster.center_y = y;
  cluster.center_z = 0.0;
  cluster.min_x = x - radius;
  cluster.max_x = x + radius;
  cluster.min_y = y - radius;
  cluster.max_y = y + radius;
  cluster.min_z = -radius;
  cluster.max_z = radius;
  cluster.point_count = 1;
  cluster.points = std::make_shared<PointCloud>();
  cluster.points->push_back(PointType(x, y, 0.0));
  return cluster;
}

scene_node::avoidance::RiskAssessmentParams makeParams()
{
  scene_node::avoidance::RiskAssessmentParams params;
  params.rollout.horizon_s = 1.0;
  params.rollout.dt_s = 0.1;
  params.rollout.speed_samples = 5;
  params.rollout.angular_samples = 3;
  params.footprint.front = 0.88;
  params.footprint.back = 0.25;
  params.footprint.half_width = 0.35;
  params.margins.front = 0.10;
  params.margins.back = 0.12;
  params.margins.side = 0.05;
  params.risk.emergency_ttc_s = 0.45;
  params.filter.emergency_ttc_s = params.risk.emergency_ttc_s;
  return params;
}

}  // namespace

TEST(SweptAvoidanceTest, ClearFieldKeepsDesiredCommand)
{
  const rclcpp::Time now(0, 0, RCL_ROS_TIME);
  scene_node::avoidance::RiskAssessment assessment;
  const scene_node::avoidance::MotionCommand desired{0.6, 0.2};
  const scene_node::avoidance::VehicleFeedback feedback{0.0, 0.0, now};

  const auto result = assessment.evaluate(
    desired,
    feedback,
    {},
    {},
    now,
    makeParams());

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_EQ(result.risk.level, scene_node::avoidance::RiskLevel::CLEAR);
  EXPECT_DOUBLE_EQ(result.selected_command.linear_x, desired.linear_x);
  EXPECT_DOUBLE_EQ(result.selected_command.angular_z, desired.angular_z);
  EXPECT_EQ(result.obstacle_count, 0u);
  EXPECT_GT(result.trajectory_count, 0u);
}

TEST(SweptAvoidanceTest, ObstacleInsideForwardFootprintCommandsStop)
{
  const rclcpp::Time now(0, 0, RCL_ROS_TIME);
  scene_node::avoidance::RiskAssessment assessment;
  const scene_node::avoidance::MotionCommand desired{0.6, 0.0};
  const scene_node::avoidance::VehicleFeedback feedback{0.0, 0.0, now};

  std::map<std::string, std::vector<ClusterInfo>> clusters;
  clusters["left"] = {makeCluster(0.5, 0.0, 0.05)};
  const std::map<std::string, rclcpp::Time> radar_times{{"left", now}};

  const auto result = assessment.evaluate(
    desired,
    feedback,
    clusters,
    radar_times,
    now,
    makeParams());

  EXPECT_TRUE(result.emergency_brake);
  EXPECT_TRUE(result.risk.has_collision);
  EXPECT_EQ(result.risk.level, scene_node::avoidance::RiskLevel::BRAKING);
  EXPECT_DOUBLE_EQ(result.selected_command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(result.selected_command.angular_z, 0.0);
  EXPECT_EQ(result.obstacle_count, 1u);
}
