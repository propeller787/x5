/**
 * @file test_swept_cull.cpp
 * @brief 方案 8 扫掠预裁剪等价性测试
 *
 * 对随机障碍场比较「预裁剪开（band=1.0）」与「关（band=0）」两个检查器：
 * 碰撞/TTC/硬停路径距离必须逐位一致；min_distance 在带宽以内逐位一致，
 * 带宽以外允许被裁剪为 +inf（其值不参与任何控制决策，校验器保证带宽 ≥ 阈值）。
 */
#include <gtest/gtest.h>

#include "safety_node/core/avoidance_types.h"
#include "safety_node/core/swept_footprint_checker.h"
#include "safety_node/core/vehicle_footprint.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace
{

TrajectoryRollout straightTrajectory(double length_m, size_t steps)
{
  TrajectoryRollout rollout;
  rollout.cmd.v = 1.0;
  rollout.cmd.w = 0.0;
  for (size_t i = 0; i <= steps; ++i) {
    const double s = length_m * static_cast<double>(i) / static_cast<double>(steps);
    rollout.poses.push_back({s, 0.0, 0.0, s / 1.0});
  }
  return rollout;
}

TrajectoryRollout arcTrajectory(double v, double w, double horizon_s, size_t steps)
{
  TrajectoryRollout rollout;
  rollout.cmd.v = v;
  rollout.cmd.w = w;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  rollout.poses.push_back({x, y, theta, 0.0});
  const double dt = horizon_s / static_cast<double>(steps);
  for (size_t i = 1; i <= steps; ++i) {
    x += v * std::cos(theta) * dt;
    y += v * std::sin(theta) * dt;
    theta += w * dt;
    rollout.poses.push_back({x, y, theta, dt * static_cast<double>(i)});
  }
  return rollout;
}

ObstacleField randomField(std::mt19937 & rng, size_t count)
{
  std::uniform_real_distribution<double> dist_x(-4.0, 6.0);
  std::uniform_real_distribution<double> dist_y(-3.0, 3.0);
  ObstacleField field;
  for (size_t i = 0; i < count; ++i) {
    ObstaclePrimitive obs;
    obs.x = dist_x(rng);
    obs.y = dist_y(rng);
    obs.radius = 0.01;
    obs.valid = true;
    obs.source = "test";
    field.obstacles.push_back(obs);
  }
  return field;
}

void expectEquivalent(
  const CollisionCheckResult & base,
  const CollisionCheckResult & culled,
  double band)
{
  ASSERT_EQ(base.collision, culled.collision);
  if (base.collision) {
    ASSERT_DOUBLE_EQ(base.ttc, culled.ttc);
    ASSERT_DOUBLE_EQ(base.collision_path_distance_m, culled.collision_path_distance_m);
    ASSERT_DOUBLE_EQ(base.collision_x, culled.collision_x);
    ASSERT_DOUBLE_EQ(base.collision_y, culled.collision_y);
  }
  ASSERT_EQ(
    std::isfinite(base.hard_stop_path_distance_m),
    std::isfinite(culled.hard_stop_path_distance_m));
  if (std::isfinite(base.hard_stop_path_distance_m)) {
    ASSERT_DOUBLE_EQ(base.hard_stop_path_distance_m, culled.hard_stop_path_distance_m);
  }
  // 带宽以内的净空必须逐位一致；带宽以外允许被裁剪为 +inf
  const double base_clamped = std::min(base.min_distance, band);
  const double culled_clamped = std::min(culled.min_distance, band);
  ASSERT_NEAR(base_clamped, culled_clamped, 1e-9);
  ASSERT_DOUBLE_EQ(base.path_length_m, culled.path_length_m);
}

}  // namespace

TEST(SweptCull, RandomFieldsEquivalentOnStraightTrajectory)
{
  AvoidanceParams base_params;
  base_params.swept_cull_band_m = 0.0;    // 关闭 = 旧行为
  AvoidanceParams cull_params = base_params;
  cull_params.swept_cull_band_m = 1.0;

  VehicleFootprint base_footprint(base_params);
  VehicleFootprint cull_footprint(cull_params);
  SweptFootprintChecker base_checker(base_params, base_footprint);
  SweptFootprintChecker cull_checker(cull_params, cull_footprint);

  std::mt19937 rng(42U);
  const TrajectoryRollout trajectory = straightTrajectory(1.6, 8);
  for (int scenario = 0; scenario < 50; ++scenario) {
    const ObstacleField field = randomField(rng, 200);
    const auto base = base_checker.checkTrajectory(trajectory, field);
    const auto culled = cull_checker.checkTrajectory(trajectory, field);
    expectEquivalent(base, culled, 1.0);
  }
}

TEST(SweptCull, RandomFieldsEquivalentOnArcTrajectory)
{
  AvoidanceParams base_params;
  base_params.swept_cull_band_m = 0.0;
  AvoidanceParams cull_params = base_params;
  cull_params.swept_cull_band_m = 1.0;

  VehicleFootprint base_footprint(base_params);
  VehicleFootprint cull_footprint(cull_params);
  SweptFootprintChecker base_checker(base_params, base_footprint);
  SweptFootprintChecker cull_checker(cull_params, cull_footprint);

  std::mt19937 rng(20260714U);
  const TrajectoryRollout trajectory = arcTrajectory(0.8, 0.9, 0.8, 8);
  for (int scenario = 0; scenario < 50; ++scenario) {
    const ObstacleField field = randomField(rng, 200);
    const auto base = base_checker.checkTrajectory(trajectory, field);
    const auto culled = cull_checker.checkTrajectory(trajectory, field);
    expectEquivalent(base, culled, 1.0);
  }
}

TEST(SweptCull, FarObstacleCulledNearObstacleKept)
{
  AvoidanceParams params;
  params.swept_cull_band_m = 1.0;
  VehicleFootprint footprint(params);
  SweptFootprintChecker checker(params, footprint);

  const TrajectoryRollout trajectory = straightTrajectory(1.0, 5);
  ObstacleField field;
  ObstaclePrimitive near_obs;
  near_obs.x = 0.9;
  near_obs.y = 0.4;
  near_obs.radius = 0.01;
  near_obs.valid = true;
  field.obstacles.push_back(near_obs);
  ObstaclePrimitive far_obs;
  far_obs.x = 5.5;
  far_obs.y = 2.9;
  far_obs.radius = 0.01;
  far_obs.valid = true;
  field.obstacles.push_back(far_obs);

  const auto result = checker.checkTrajectory(trajectory, field);
  // 近障净空如实给出；远障被裁剪后不得把 min_distance 拉到远障的值
  ASSERT_TRUE(std::isfinite(result.min_distance));
  ASSERT_LT(result.min_distance, 1.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
