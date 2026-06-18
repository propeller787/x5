/**
 * @file test_map_constraint_check.cpp
 */
#include <gtest/gtest.h>

#include "safety_node/core/motion_rollout.h"
#include "safety_node/map_constraint_check.h"

namespace
{

RollingLocalMapGrid makeMap(double resolution = 0.05)
{
  RollingLocalMapGrid map;
  map.cols = 120;
  map.rows = 120;
  map.resolution_m = resolution;
  map.origin_x_m = -3.0;
  map.origin_y_m = -3.0;
  map.probability.assign(map.cols * map.rows, 0.0f);
  map.inflated.assign(map.cols * map.rows, 0.0f);
  return map;
}

void setLethal(RollingLocalMapGrid & map, double x, double y)
{
  const int col = static_cast<int>(std::floor((x - map.origin_x_m) / map.resolution_m));
  const int row = static_cast<int>(std::floor((y - map.origin_y_m) / map.resolution_m));
  ASSERT_GE(col, 0);
  ASSERT_GE(row, 0);
  ASSERT_LT(col, static_cast<int>(map.cols));
  ASSERT_LT(row, static_cast<int>(map.rows));
  const size_t idx = static_cast<size_t>(row) * map.cols + static_cast<size_t>(col);
  map.probability[idx] = 0.9f;
  map.inflated[idx] = 0.9f;
}

AvoidanceParams productionFootprint()
{
  AvoidanceParams params;
  params.footprint_use_dual_trapezoid = true;
  params.robot_rear_offset = 0.23;
  params.footprint_rear_segment.length_m = 0.92;
  params.footprint_rear_segment.width_rear_m = 0.68;
  params.footprint_rear_segment.width_front_m = 0.62;
  params.footprint_front_segment.length_m = 0.25;
  params.footprint_front_segment.width_rear_m = 0.62;
  params.footprint_front_segment.width_front_m = 0.34;
  params.footprint_front_margin = 0.05;
  params.footprint_back_margin = 0.08;
  params.footprint_side_margin = 0.01;
  return params;
}

std::vector<PredictedPose> straightPoses(
  std::initializer_list<std::pair<double, double>> x_and_t)
{
  std::vector<PredictedPose> poses;
  for (const auto & item : x_and_t) {
    PredictedPose pose;
    pose.x = item.first;
    pose.t = item.second;
    poses.push_back(pose);
  }
  return poses;
}

}  // namespace

TEST(MapConstraintCheckTest, TrajectoryThroughOccupiedCellIsLethal)
{
  AvoidanceParams fp;
  MapConstraintConfig cfg;
  MapConstraintCheck checker(fp, cfg);

  RollingLocalMapGrid map;
  map.cols = 40;
  map.rows = 40;
  map.resolution_m = 0.1;
  map.origin_x_m = -2.0;
  map.origin_y_m = -2.0;
  map.probability.assign(map.cols * map.rows, 0.0f);
  map.inflated.assign(map.cols * map.rows, 0.0f);

  const int col = 20;
  const int row = 20;
  map.probability[static_cast<size_t>(row) * map.cols + static_cast<size_t>(col)] = 0.9f;
  map.inflated[static_cast<size_t>(row) * map.cols + static_cast<size_t>(col)] = 0.9f;

  std::vector<PredictedPose> poses;
  PredictedPose p;
  p.x = 0.0;
  p.y = 0.0;
  p.theta = 0.0;
  p.t = 0.0;
  poses.push_back(p);

  MapConstraintResult out;
  const bool lethal = checker.trajectoryLethal(map, poses, 0.65, false, out);
  EXPECT_TRUE(lethal);
  EXPECT_GE(out.p_max_on_path, 0.65);
}

TEST(MapConstraintCheckTest, TrajectoryAvoidsOccupiedCell)
{
  AvoidanceParams fp;
  MapConstraintCheck checker(fp, MapConstraintConfig{});

  RollingLocalMapGrid map;
  map.cols = 40;
  map.rows = 40;
  map.resolution_m = 0.1;
  map.origin_x_m = -2.0;
  map.origin_y_m = -2.0;
  map.probability.assign(map.cols * map.rows, 0.0f);
  map.inflated.assign(map.cols * map.rows, 0.0f);
  map.probability[static_cast<size_t>(20) * map.cols + 25] = 0.9f;

  std::vector<PredictedPose> poses;
  PredictedPose p;
  p.x = 0.0;
  p.y = 1.5;
  p.theta = 0.0;
  p.t = 0.0;
  poses.push_back(p);

  MapConstraintResult out;
  EXPECT_FALSE(checker.trajectoryLethal(map, poses, 0.65, true, out));
}

TEST(MapConstraintCheckTest, OccupiedFootprintInteriorIsLethal)
{
  AvoidanceParams fp;
  MapConstraintCheck checker(fp, MapConstraintConfig{});

  RollingLocalMapGrid map;
  map.cols = 40;
  map.rows = 40;
  map.resolution_m = 0.1;
  map.origin_x_m = -2.0;
  map.origin_y_m = -2.0;
  map.probability.assign(map.cols * map.rows, 0.0f);
  map.inflated.assign(map.cols * map.rows, 0.0f);
  map.probability[static_cast<size_t>(20) * map.cols + 20] = 0.9f;

  MapConstraintResult out;
  EXPECT_TRUE(
    checker.trajectoryLethal(
      map, {PredictedPose{}}, 0.65, true, out));
}

TEST(MapConstraintCheckTest, OutOfBoundsTrajectoryIsLethal)
{
  AvoidanceParams fp;
  MapConstraintCheck checker(fp, MapConstraintConfig{});
  RollingLocalMapGrid map;
  map.cols = 10;
  map.rows = 10;
  map.resolution_m = 0.1;
  map.origin_x_m = -0.5;
  map.origin_y_m = -0.5;
  map.probability.assign(100, 0.0f);
  map.inflated.assign(100, 0.0f);

  PredictedPose pose;
  pose.x = 2.0;
  MapConstraintResult out;
  EXPECT_TRUE(checker.trajectoryLethal(map, {pose}, 0.65, true, out));
}

TEST(MapConstraintCheckTest, InvalidAndTruncatedMapsFailSafe)
{
  AvoidanceParams fp;
  MapConstraintCheck checker(fp, MapConstraintConfig{});
  MapConstraintResult out;
  RollingLocalMapGrid invalid;
  EXPECT_TRUE(checker.trajectoryLethal(invalid, {PredictedPose{}}, 0.65, false, out));

  RollingLocalMapGrid truncated;
  truncated.cols = 2;
  truncated.rows = 2;
  truncated.resolution_m = 1.0;
  truncated.origin_x_m = -1.0;
  truncated.origin_y_m = -1.0;
  truncated.probability = {0.0f};
  truncated.inflated = {0.0f};
  EXPECT_TRUE(checker.trajectoryLethal(truncated, {PredictedPose{}}, 0.65, false, out));
}

TEST(MapConstraintCheckTest, UpdatesAndQueryWrapperUseNewFootprint)
{
  AvoidanceParams fp;
  MapConstraintCheck checker(fp, MapConstraintConfig{});
  fp.robot_width_rear_m = 1.0;
  fp.robot_width_front_m = 1.0;
  checker.updateFootprintParams(fp);
  MapConstraintConfig config;
  config.check_footprint_corners = false;
  checker.updateConfig(config);

  RollingLocalMapGrid map;
  map.cols = 40;
  map.rows = 40;
  map.resolution_m = 0.1;
  map.origin_x_m = -2.0;
  map.origin_y_m = -2.0;
  map.probability.assign(map.cols * map.rows, 0.0f);
  map.inflated.assign(map.cols * map.rows, 0.0f);
  const auto result = checker.queryTrajectory(map, {PredictedPose{}}, 0.65, true);
  EXPECT_FALSE(result.lethal);
  EXPECT_DOUBLE_EQ(result.p_max_on_path, 0.0);
}

TEST(MapConstraintCheckTest, ReverseCanClearInitialFrontLethalCell)
{
  MapConstraintCheck checker(productionFootprint(), MapConstraintConfig{});
  RollingLocalMapGrid map = makeMap();
  setLethal(map, 0.975, -0.15);

  MapConstraintResult out;
  EXPECT_TRUE(
    checker.trajectoryEscapesInitialLethal(
      map,
      straightPoses({{0.0, 0.0}, {-0.02, 0.1}, {-0.04, 0.2}, {-0.06, 0.3}}),
      0.65,
      true,
      0.35,
      out));
  EXPECT_TRUE(out.lethal);
  EXPECT_DOUBLE_EQ(out.first_lethal_t, 0.0);
}

TEST(MapConstraintCheckTest, ControlledRolloutClearsInitialFrontLethalCell)
{
  AvoidanceParams params = productionFootprint();
  params.max_decel_m_s2 = 0.8;
  params.max_recover_accel_m_s2 = 0.5;
  MotionRollout rollout(params);
  MapConstraintCheck checker(params, MapConstraintConfig{});
  RollingLocalMapGrid map = makeMap();
  setLethal(map, 0.975, -0.15);

  VelocityCandidate candidate;
  candidate.v = -0.12;
  const TrajectoryRollout trajectory =
    rollout.rolloutCandidate(candidate, 0.0, 0.0);
  MapConstraintResult out;
  EXPECT_TRUE(
    checker.trajectoryEscapesInitialLethal(
      map,
      trajectory.poses,
      0.65,
      true,
      0.80,
      out));
}

TEST(MapConstraintCheckTest, ReverseCannotEscapeRearLethalCell)
{
  MapConstraintCheck checker(productionFootprint(), MapConstraintConfig{});
  RollingLocalMapGrid map = makeMap();
  setLethal(map, -0.325, 0.0);

  MapConstraintResult out;
  EXPECT_FALSE(
    checker.trajectoryEscapesInitialLethal(
      map,
      straightPoses({{0.0, 0.0}, {-0.02, 0.1}, {-0.04, 0.2}, {-0.06, 0.3}}),
      0.65,
      true,
      0.35,
      out));
}

TEST(MapConstraintCheckTest, EscapeCannotEnterNewLethalCell)
{
  MapConstraintCheck checker(productionFootprint(), MapConstraintConfig{});
  RollingLocalMapGrid map = makeMap();
  setLethal(map, 0.975, -0.15);
  setLethal(map, -0.375, 0.0);

  MapConstraintResult out;
  EXPECT_FALSE(
    checker.trajectoryEscapesInitialLethal(
      map,
      straightPoses({{0.0, 0.0}, {-0.02, 0.1}, {-0.04, 0.2}, {-0.06, 0.3}}),
      0.65,
      true,
      0.35,
      out));
}
