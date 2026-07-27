/**
 * @file test_retreat_escape.cpp
 * @brief 方案 2.4 单调退离逃逸放宽单测（tilt_on_7「前障贴身无法倒退」修复）
 *
 * trajectoryEscapesInitialLethal 原本要求「窗内完全清空初始 lethal」；
 * retreat_escape_enabled 后额外接受「不进任何新 lethal 且对初始 lethal 集
 * 单调退离」的轨迹。本文件验证：慢退被放行 / 关闭态拒绝 / 进入新 lethal
 * 恒拒绝 / 无进展恒拒绝。
 */
#include <gtest/gtest.h>

#include "safety_node/map_constraint_check.h"

namespace
{

/** 手工构建 40×40、5cm、原点 (-1,-1) 的栅格快照 */
RollingLocalMapGrid makeGrid()
{
  RollingLocalMapGrid grid;
  grid.cols = 40;
  grid.rows = 40;
  grid.resolution_m = 0.05;
  grid.origin_x_m = -1.0;
  grid.origin_y_m = -1.0;
  grid.lethal_threshold = 0.65;
  grid.probability.assign(40 * 40, 0.0f);
  grid.inflated.assign(40 * 40, 0.0f);
  return grid;
}

void markLethal(RollingLocalMapGrid & grid, double x, double y)
{
  const int col = static_cast<int>((x - grid.origin_x_m) / grid.resolution_m);
  const int row = static_cast<int>((y - grid.origin_y_m) / grid.resolution_m);
  ASSERT_GE(col, 0);
  ASSERT_GE(row, 0);
  ASSERT_LT(col, static_cast<int>(grid.cols));
  ASSERT_LT(row, static_cast<int>(grid.rows));
  grid.probability[static_cast<size_t>(row) * grid.cols + col] = 1.0f;
}

/** 沿 x 匀速后退的位姿序列（check_corners=false → 只采样中心点） */
std::vector<PredictedPose> retreatPoses(double from_x, double to_x, size_t steps)
{
  std::vector<PredictedPose> poses;
  for (size_t i = 0; i <= steps; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(steps);
    poses.push_back(
      {from_x + (to_x - from_x) * ratio, 0.0, 0.0,
        1.0 * ratio});   // 总时长 1.0 s
  }
  return poses;
}

MapConstraintCheck makeChecker(bool retreat_enabled)
{
  AvoidanceParams fp_params;
  MapConstraintConfig config;
  config.enabled = true;
  config.veto_on_lethal = true;
  config.check_footprint_corners = false;   // 中心采样，聚焦退离语义本身
  config.retreat_escape_enabled = retreat_enabled;
  config.retreat_min_progress_cells = 1.0;
  return MapConstraintCheck(fp_params, config);
}

}  // namespace

TEST(RetreatEscape, SlowRetreatAcceptedWhenEnabled)
{
  auto grid = makeGrid();
  markLethal(grid, 0.05, 0.0);

  // 起点在 lethal 内，慢退到 -0.5；逃逸窗 0.01 s 内清不掉（时间判据必然失败）
  const auto poses = retreatPoses(0.05, -0.5, 20);
  MapConstraintResult out;
  EXPECT_TRUE(
    makeChecker(true).trajectoryEscapesInitialLethal(
      grid, poses, 0.65, false, 0.01, out));
}

TEST(RetreatEscape, SlowRetreatRejectedWhenDisabled)
{
  auto grid = makeGrid();
  markLethal(grid, 0.05, 0.0);

  const auto poses = retreatPoses(0.05, -0.5, 20);
  MapConstraintResult out;
  EXPECT_FALSE(
    makeChecker(false).trajectoryEscapesInitialLethal(
      grid, poses, 0.65, false, 0.01, out));
}

TEST(RetreatEscape, FastClearInsideWindowStillAccepted)
{
  auto grid = makeGrid();
  markLethal(grid, 0.05, 0.0);

  // 窗 2.0 s 足够大：走原「窗内完全清空」路径，与开关无关
  const auto poses = retreatPoses(0.05, -0.5, 20);
  MapConstraintResult out;
  EXPECT_TRUE(
    makeChecker(false).trajectoryEscapesInitialLethal(
      grid, poses, 0.65, false, 2.0, out));
}

TEST(RetreatEscape, EnteringNewLethalAlwaysRejected)
{
  auto grid = makeGrid();
  markLethal(grid, 0.05, 0.0);
  markLethal(grid, -0.30, 0.0);   // 退路上的新 lethal

  const auto poses = retreatPoses(0.05, -0.5, 20);
  MapConstraintResult out;
  EXPECT_FALSE(
    makeChecker(true).trajectoryEscapesInitialLethal(
      grid, poses, 0.65, false, 0.01, out));
}

TEST(RetreatEscape, NoProgressRejected)
{
  auto grid = makeGrid();
  markLethal(grid, 0.05, 0.0);

  // 全程停在初始 lethal 内：单调但零进展 → 拒绝
  const auto poses = retreatPoses(0.05, 0.05, 10);
  MapConstraintResult out;
  EXPECT_FALSE(
    makeChecker(true).trajectoryEscapesInitialLethal(
      grid, poses, 0.65, false, 0.01, out));
}

TEST(RetreatEscape, NotStartingInLethalReturnsFalse)
{
  auto grid = makeGrid();
  markLethal(grid, 0.5, 0.5);   // 与轨迹无关的远处 lethal

  const auto poses = retreatPoses(0.05, -0.5, 10);
  MapConstraintResult out;
  EXPECT_FALSE(
    makeChecker(true).trajectoryEscapesInitialLethal(
      grid, poses, 0.65, false, 0.01, out));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
