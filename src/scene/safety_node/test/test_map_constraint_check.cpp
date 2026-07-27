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

// ── L2 单调退离逃逸放宽（tilt_on_7）──

// 回归护栏：逃逸窗过紧（前障要到 t≥0.1 才退出），默认（开关关）行为判否，与现状一致。
TEST(MapConstraintCheckTest, RetreatEscapeDisabledRejectsSlowClearOutsideWindow)
{
  MapConstraintConfig cfg;    // retreat_escape_enabled 默认 false
  MapConstraintCheck checker(productionFootprint(), cfg);
  RollingLocalMapGrid map = makeMap();
  setLethal(map, 0.975, -0.15);

  MapConstraintResult out;
  EXPECT_FALSE(
    checker.trajectoryEscapesInitialLethal(
      map,
      straightPoses({{0.0, 0.0}, {-0.02, 0.1}, {-0.04, 0.2}, {-0.06, 0.3}}),
      0.65,
      true,
      0.05,    // 窗 < 首个倒退位姿 t=0.1 → 慢清空落窗外
      out));
}

// 同场景 + 开关开启：单调退离（含慢清空）被接受 —— tilt_on_7 修复本体。
TEST(MapConstraintCheckTest, RetreatEscapeEnabledAcceptsSlowMonotonicRetreat)
{
  MapConstraintConfig cfg;
  cfg.retreat_escape_enabled = true;
  MapConstraintCheck checker(productionFootprint(), cfg);
  RollingLocalMapGrid map = makeMap();
  setLethal(map, 0.975, -0.15);

  MapConstraintResult out;
  EXPECT_TRUE(
    checker.trajectoryEscapesInitialLethal(
      map,
      straightPoses({{0.0, 0.0}, {-0.02, 0.1}, {-0.04, 0.2}, {-0.06, 0.3}}),
      0.65,
      true,
      0.05,
      out));
}

// 安全护栏①：开关开启也绝不放行「进入新 lethal」（后向新障）。
TEST(MapConstraintCheckTest, RetreatEscapeStillRejectsEnteringNewLethal)
{
  MapConstraintConfig cfg;
  cfg.retreat_escape_enabled = true;
  MapConstraintCheck checker(productionFootprint(), cfg);
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

// 安全护栏②：开关开启但车不动（无退离进展）仍判否 —— 只放行真正在退离的运动。
TEST(MapConstraintCheckTest, RetreatEscapeRejectsNoProgress)
{
  MapConstraintConfig cfg;
  cfg.retreat_escape_enabled = true;
  MapConstraintCheck checker(productionFootprint(), cfg);
  RollingLocalMapGrid map = makeMap();
  setLethal(map, 0.975, -0.15);

  MapConstraintResult out;
  EXPECT_FALSE(
    checker.trajectoryEscapesInitialLethal(
      map,
      straightPoses({{0.0, 0.0}, {0.0, 0.1}, {0.0, 0.2}, {0.0, 0.3}}),
      0.65,
      true,
      0.35,
      out));
}

// ── 原地转向豁免的 LocalMap 判据 trajectoryNoNewLethal（tilt_on_10）──

// 生成纯原地旋转位姿序列（x=y=0，theta 递增），供转向豁免复验。
static std::vector<PredictedPose> rotationPoses(std::initializer_list<double> thetas)
{
  std::vector<PredictedPose> poses;
  double t = 0.0;
  for (const double th : thetas) {
    PredictedPose pose;
    pose.theta = th;
    pose.t = t;
    t += 0.1;
    poses.push_back(pose);
  }
  return poses;
}

// 核心差异：与 RetreatEscapeRejectsNoProgress 同场景（起始 lethal、车不动/维持覆盖），
// trajectoryEscapesInitialLethal 因「无退离进展」判否，而 trajectoryNoNewLethal 判是——
// 这正是「接触/被 veto 态也允许只转向脱离」的判据基础（不要求净空改善，只要求不恶化）。
TEST(MapConstraintCheckTest, NoNewLethalAcceptsMaintainingInitialCoverage)
{
  MapConstraintCheck checker(productionFootprint(), MapConstraintConfig{});
  RollingLocalMapGrid map = makeMap();
  setLethal(map, 0.975, -0.15);

  MapConstraintResult out;
  EXPECT_TRUE(
    checker.trajectoryNoNewLethal(
      map,
      straightPoses({{0.0, 0.0}, {0.0, 0.1}, {0.0, 0.2}, {0.0, 0.3}}),
      0.65,
      true,
      out));
}

// 安全护栏：转向/运动踏入「起始姿势未占用」的新 lethal 栅格必被否决。
// 复用 EscapeCannotEnterNewLethalCell 场景：起始仅前障 lethal，倒退把后障（新 lethal）纳入覆盖。
TEST(MapConstraintCheckTest, NoNewLethalRejectsEnteringNewCell)
{
  MapConstraintCheck checker(productionFootprint(), MapConstraintConfig{});
  RollingLocalMapGrid map = makeMap();
  setLethal(map, 0.975, -0.15);
  setLethal(map, -0.375, 0.0);

  MapConstraintResult out;
  EXPECT_FALSE(
    checker.trajectoryNoNewLethal(
      map,
      straightPoses({{0.0, 0.0}, {-0.02, 0.1}, {-0.04, 0.2}, {-0.06, 0.3}}),
      0.65,
      true,
      out));
  EXPECT_TRUE(out.lethal);
}

// 起始不 lethal（空豁免集）时退化为「全程不得触任何 lethal」：纯净地图任意转向均安全。
TEST(MapConstraintCheckTest, NoNewLethalCleanMapRotationIsSafe)
{
  MapConstraintCheck checker(productionFootprint(), MapConstraintConfig{});
  RollingLocalMapGrid map = makeMap();

  MapConstraintResult out;
  EXPECT_TRUE(
    checker.trajectoryNoNewLethal(
      map,
      rotationPoses({0.0, 0.15, 0.30}),
      0.65,
      true,
      out));
}

// 单个 lethal 栅格落在旋转轴附近、被 footprint 全程覆盖：原地旋转不引入新 lethal → 放行。
TEST(MapConstraintCheckTest, NoNewLethalRotationKeepingSingleInteriorCellIsSafe)
{
  MapConstraintCheck checker(productionFootprint(), MapConstraintConfig{});
  RollingLocalMapGrid map = makeMap();
  setLethal(map, 0.0, 0.0);    // 车体内部、近旋转轴，旋转全程覆盖同一格

  MapConstraintResult out;
  EXPECT_TRUE(
    checker.trajectoryNoNewLethal(
      map,
      rotationPoses({0.0, 0.15, 0.30}),
      0.65,
      true,
      out));
}
