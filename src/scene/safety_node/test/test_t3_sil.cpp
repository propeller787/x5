#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <utility>
#include <vector>

#include "safety_node/avoidance_controller.h"
#include "safety_node/core/motion_rollout.h"
#include "safety_node/core/swept_footprint_checker.h"
#include "safety_node/core/vehicle_footprint.h"
#include "safety_node/mode_supervisor.h"

namespace
{

constexpr double kDt = 0.025;

ObstacleField makeField(
  const std::vector<std::pair<double, double>> & points,
  double radius = 0.08)
{
  ObstacleField field;
  field.obstacles.reserve(points.size());
  for (const auto & point : points) {
    ObstaclePrimitive primitive;
    primitive.x = point.first;
    primitive.y = point.second;
    primitive.radius = radius;
    primitive.source = "t3_sil";
    primitive.valid = true;
    field.obstacles.push_back(primitive);
  }
  return field;
}

void appendLine(
  std::vector<std::pair<double, double>> & points,
  double x0,
  double y0,
  double x1,
  double y1,
  std::size_t count)
{
  if (count == 0U) {
    return;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const double ratio =
      count == 1U ? 0.0 : static_cast<double>(i) / static_cast<double>(count - 1U);
    points.emplace_back(x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio);
  }
}

AvoidanceParams defaultAvoidanceParams()
{
  AvoidanceParams params;
  params.rollout_horizon_s = 2.5;
  params.rollout_dt_s = 0.05;
  params.rollout_speed_samples = 9;
  params.rollout_angular_samples = 13;
  params.max_decel_m_s2 = 100.0;
  params.max_recover_accel_m_s2 = 100.0;
  params.max_angular_decel_rad_s2 = 100.0;
  params.max_angular_recover_rad_s2 = 100.0;
  params.emergency_ttc_s = 0.45;
  params.hard_stop_distance_m = 0.08;
  return params;
}

RollingLocalMapGrid boundedEmptyMap()
{
  RollingLocalMapGrid map;
  map.cols = 36;
  map.rows = 40;
  map.resolution_m = 0.05;
  map.origin_x_m = -0.9;
  map.origin_y_m = -1.0;
  map.lethal_threshold = 0.65;
  map.probability.assign(map.cols * map.rows, 0.0f);
  map.inflated.assign(map.cols * map.rows, 0.0f);
  return map;
}

class SilScenarioRunner
{
public:
  explicit SilScenarioRunner(
    const AvoidanceParams & params = defaultAvoidanceParams(),
    const MapConstraintConfig & map_config = MapConstraintConfig{})
  : params_(params),
    footprint_(params_),
    checker_(params_, footprint_),
    rollout_(params_),
    controller_(params_, map_config)
  {
  }

  AvoidanceResult evaluate(
    double desired_linear,
    double desired_angular,
    const ObstacleField & field,
    double feedback_linear = 0.0,
    double feedback_angular = 0.0,
    const RollingLocalMapGrid * map = nullptr,
    SafetyMode mode = SafetyMode::SAFE_ASSIST,
    double previous_linear = 0.0,
    double previous_angular = 0.0) const
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = desired_linear;
    command.angular.z = desired_angular;
    AvoidanceContext context;
    context.safety_mode = mode;
    return controller_.evaluate(
      command,
      feedback_linear,
      feedback_angular,
      field,
      map,
      ModeSupervisor(mode).profile(),
      previous_linear,
      previous_angular,
      kDt,
      context);
  }

  void expectFiniteAndBounded(const AvoidanceResult & result) const
  {
    EXPECT_TRUE(std::isfinite(result.safe_twist.linear.x));
    EXPECT_TRUE(std::isfinite(result.safe_twist.angular.z));
    EXPECT_LE(
      std::abs(result.safe_twist.linear.x),
      params_.absolute_max_linear_speed + 1e-9);
    EXPECT_LE(
      std::abs(result.safe_twist.angular.z),
      params_.absolute_max_angular_speed + 1e-9);
  }

  void expectOutputTrajectorySafe(
    const AvoidanceResult & result,
    const ObstacleField & field,
    double feedback_linear = 0.0,
    double feedback_angular = 0.0) const
  {
    const VelocityCandidate final_command {
      result.safe_twist.linear.x,
      result.safe_twist.angular.z};
    const double initial_linear =
      (feedback_linear * final_command.v<0.0 ||
      std::abs(feedback_linear)> std::abs(final_command.v)) ?
      feedback_linear : final_command.v;
    const double initial_angular =
      (feedback_angular * final_command.w<0.0 ||
      std::abs(feedback_angular)> std::abs(final_command.w)) ?
      feedback_angular : final_command.w;
    const TrajectoryRollout trajectory =
      rollout_.rolloutCandidate(final_command, initial_linear, initial_angular);
    const CollisionCheckResult check = checker_.checkTrajectory(trajectory, field);
    if (check.collision && result.risk.braking_limited) {
      EXPECT_TRUE(brakingEnvelopeCanStop(result.risk))
        << "distance envelope cannot stop output v=" << final_command.v
        << " available=" << result.risk.braking_available_distance_m
        << " stopping=" << result.risk.braking_stopping_distance_m;
      return;
    }
    EXPECT_FALSE(check.collision)
      << "unsafe output v=" << final_command.v << " w=" << final_command.w
      << " clearance=" << check.min_distance << " ttc=" << check.ttc;
  }

private:
  AvoidanceParams params_;
  mutable VehicleFootprint footprint_;
  mutable SweptFootprintChecker checker_;
  MotionRollout rollout_;
  AvoidanceController controller_;
};

TEST(T3SilTest, Sim001OpenSpaceSupportsPlanarCommands)
{
  const SilScenarioRunner runner;
  const ObstacleField empty;
  for (const auto & command : std::vector<std::pair<double, double>>{
      {0.8, 0.0}, {-0.5, 0.0}, {0.4, 0.8}, {0.0, 1.2}})
  {
    const auto result = runner.evaluate(command.first, command.second, empty);
    runner.expectFiniteAndBounded(result);
    runner.expectOutputTrajectorySafe(result, empty);
    EXPECT_FALSE(result.emergency_brake);
  }
}

TEST(T3SilTest, Sim002To004WallsAndDiagonalObstaclesRemainSafe)
{
  const SilScenarioRunner runner;
  std::vector<std::pair<double, double>> front_points;
  appendLine(front_points, 1.05, -0.9, 1.05, 0.9, 19);
  const ObstacleField front_wall = makeField(front_points);

  std::vector<std::pair<double, double>> rear_points;
  appendLine(rear_points, -0.75, -0.8, -0.75, 0.8, 17);
  const ObstacleField rear_wall = makeField(rear_points);
  const ObstacleField diagonal = makeField(
    {
      {0.95, 0.25}, {1.05, 0.35}, {1.15, 0.45}, {1.25, 0.55}});

  const auto front = runner.evaluate(1.2, 0.0, front_wall);
  runner.expectFiniteAndBounded(front);
  runner.expectOutputTrajectorySafe(front, front_wall);
  EXPECT_LT(front.safe_twist.linear.x, 1.2);

  const auto rear = runner.evaluate(-0.8, 0.0, rear_wall);
  runner.expectFiniteAndBounded(rear);
  runner.expectOutputTrajectorySafe(rear, rear_wall);
  EXPECT_GT(rear.safe_twist.linear.x, -0.8);

  const auto diagonal_result = runner.evaluate(0.9, 0.6, diagonal);
  runner.expectFiniteAndBounded(diagonal_result);
  runner.expectOutputTrajectorySafe(diagonal_result, diagonal);
}

TEST(T3SilTest, Sim005To010SideCorridorAndTurningScenariosRemainSafe)
{
  const SilScenarioRunner runner;
  for (const auto & scenario :
    std::vector<std::pair<ObstacleField, std::pair<double, double>>>{
      {makeField({{0.10, 0.58}}), {0.0, 1.2}},
      {makeField({{0.10, -0.58}}), {0.0, -1.2}},
      {makeField({{0.55, 0.50}, {0.72, 0.62}}), {0.8, 0.9}}})
  {
    const auto result =
      runner.evaluate(scenario.second.first, scenario.second.second, scenario.first);
    runner.expectFiniteAndBounded(result);
    runner.expectOutputTrajectorySafe(result, scenario.first);
  }

  std::vector<std::pair<double, double>> corridor_points;
  appendLine(corridor_points, -0.2, 0.65, 2.8, 0.65, 31);
  appendLine(corridor_points, -0.2, -0.65, 2.8, -0.65, 31);
  const ObstacleField corridor = makeField(corridor_points, 0.06);
  const auto corridor_result = runner.evaluate(0.8, 0.0, corridor);
  runner.expectFiniteAndBounded(corridor_result);
  runner.expectOutputTrajectorySafe(corridor_result, corridor);

  std::vector<std::pair<double, double>> one_side_points;
  appendLine(one_side_points, 0.85, 0.10, 1.35, 0.55, 8);
  const ObstacleField one_side = makeField(one_side_points);
  const auto one_side_result = runner.evaluate(0.9, 0.0, one_side);
  runner.expectFiniteAndBounded(one_side_result);
  runner.expectOutputTrajectorySafe(one_side_result, one_side);

  std::vector<std::pair<double, double>> blocked_points;
  appendLine(blocked_points, 0.98, -0.75, 0.98, 0.75, 17);
  const ObstacleField blocked = makeField(blocked_points);
  const auto blocked_result = runner.evaluate(1.0, 0.0, blocked);
  runner.expectFiniteAndBounded(blocked_result);
  runner.expectOutputTrajectorySafe(blocked_result, blocked);
  EXPECT_LT(blocked_result.safe_twist.linear.x, 1.0);
}

TEST(T3SilTest, Sim020NarrowCorridorAllowsFourCentimetersPerSide)
{
  const SilScenarioRunner runner;
  std::vector<std::pair<double, double>> corridor_points;
  appendLine(corridor_points, -0.2, 0.39, 2.8, 0.39, 31);
  appendLine(corridor_points, -0.2, -0.39, 2.8, -0.39, 31);
  const ObstacleField corridor = makeField(corridor_points, 0.01);

  const auto result = runner.evaluate(
    0.2, 0.0, corridor, 0.0, 0.0, nullptr, SafetyMode::NARROW);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_GT(result.safe_twist.linear.x, 0.0);
  EXPECT_GE(result.risk.min_clearance, 0.015);
}

TEST(T3SilTest, Sim020NarrowCorridorRejectsLessThanConfiguredClearance)
{
  const SilScenarioRunner runner;
  std::vector<std::pair<double, double>> corridor_points;
  appendLine(corridor_points, -0.2, 0.384, 2.8, 0.384, 31);
  appendLine(corridor_points, -0.2, -0.384, 2.8, -0.384, 31);
  const ObstacleField corridor = makeField(corridor_points, 0.01);

  const auto result = runner.evaluate(
    0.2, 0.0, corridor, 0.0, 0.0, nullptr, SafetyMode::NARROW);

  EXPECT_TRUE(result.emergency_brake);
  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.0);
  EXPECT_EQ(result.emergency_reason, EmergencyStopReason::HARD_STOP_DISTANCE);
}

TEST(T3SilTest, Sim021RightFrontLaserSurfaceRemainsPassableInNarrowMode)
{
  const SilScenarioRunner runner;
  const ObstacleField right_front_surface =
    makeField({{0.683, -0.408}}, 0.01);

  for (const auto & command : std::vector<std::pair<double, double>>{
      {0.08, 0.0}, {-0.08, 0.0}, {-0.05, 0.15}})
  {
    const auto result = runner.evaluate(
      command.first, command.second, right_front_surface,
      0.0, 0.0, nullptr, SafetyMode::NARROW);

    EXPECT_FALSE(result.emergency_brake);
    EXPECT_FALSE(result.recovery_active);
    EXPECT_NE(result.hybrid_log.final_candidate, -1);
    EXPECT_GT(result.risk.min_clearance, 0.015);
  }
}

TEST(T3SilTest, Sim011To014DynamicAndMultiSensorScenariosRemainSafe)
{
  const SilScenarioRunner runner;
  const ObstacleField empty;
  const ObstacleField sudden = makeField({{1.0, 0.0}, {1.0, 0.2}, {1.0, -0.2}});

  const auto before = runner.evaluate(1.0, 0.0, empty);
  runner.expectOutputTrajectorySafe(before, empty);
  const auto appeared = runner.evaluate(
    1.0, 0.0, sudden, 0.4, 0.0, nullptr,
    SafetyMode::SAFE_ASSIST,
    before.safe_twist.linear.x,
    before.safe_twist.angular.z);
  runner.expectFiniteAndBounded(appeared);
  runner.expectOutputTrajectorySafe(appeared, sudden, 0.4, 0.0);
  EXPECT_LE(appeared.safe_twist.linear.x, before.safe_twist.linear.x);

  const auto disappeared = runner.evaluate(
    1.0, 0.0, empty, 0.0, 0.0, nullptr,
    SafetyMode::SAFE_ASSIST,
    appeared.safe_twist.linear.x,
    appeared.safe_twist.angular.z);
  runner.expectFiniteAndBounded(disappeared);
  runner.expectOutputTrajectorySafe(disappeared, empty);

  for (double crossing_y : {0.9, 0.6, 0.3, 0.0, -0.3, -0.6, -0.9}) {
    const ObstacleField crossing = makeField({{1.1, crossing_y}});
    const auto result = runner.evaluate(0.9, 0.0, crossing);
    runner.expectFiniteAndBounded(result);
    runner.expectOutputTrajectorySafe(result, crossing);
  }

  const ObstacleField overlapping = makeField(
    {
      {1.05, 0.0}, {1.05, 0.0}, {1.04, 0.01}, {1.06, -0.01},
      {1.05, 0.0}, {1.05, 0.0}});
  const auto overlapping_result = runner.evaluate(1.0, 0.0, overlapping);
  runner.expectFiniteAndBounded(overlapping_result);
  runner.expectOutputTrajectorySafe(overlapping_result, overlapping);
}

TEST(T3SilTest, Sim015MapBoundaryConstrainsOutput)
{
  MapConstraintConfig map_config;
  map_config.enabled = true;
  map_config.veto_on_lethal = true;
  const SilScenarioRunner runner(defaultAvoidanceParams(), map_config);
  const RollingLocalMapGrid map = boundedEmptyMap();
  const ObstacleField empty;

  const auto result = runner.evaluate(
    1.0, 0.0, empty, 1.0, 0.0, &map,
    SafetyMode::SAFE_ASSIST, 1.0, 0.0);
  runner.expectFiniteAndBounded(result);
  EXPECT_TRUE(result.emergency_brake);
  EXPECT_EQ(
    result.emergency_reason,
    EmergencyStopReason::BRAKING_DISTANCE_INSUFFICIENT);
  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.0);
}

TEST(T3SilTest, RepeatedScenarioIsDeterministicForOneHundredRuns)
{
  const SilScenarioRunner runner;
  const ObstacleField field = makeField(
    {
      {1.05, -0.25}, {1.05, 0.0}, {1.05, 0.25}, {1.35, 0.55}});
  const auto reference = runner.evaluate(1.1, 0.25, field);
  runner.expectFiniteAndBounded(reference);
  runner.expectOutputTrajectorySafe(reference, field);

  for (int repeat = 0; repeat < 100; ++repeat) {
    const auto result = runner.evaluate(1.1, 0.25, field);
    EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, reference.safe_twist.linear.x);
    EXPECT_DOUBLE_EQ(result.safe_twist.angular.z, reference.safe_twist.angular.z);
    EXPECT_EQ(result.emergency_brake, reference.emergency_brake);
    EXPECT_EQ(
      result.hybrid_log.final_candidate,
      reference.hybrid_log.final_candidate);
  }
}

TEST(T3SilTest, CriticalParameterSweepRemainsFiniteBoundedAndSafe)
{
  for (double horizon : {1.0, 2.5, 4.0}) {
    for (double rollout_dt : {0.025, 0.05, 0.10}) {
      AvoidanceParams params = defaultAvoidanceParams();
      params.rollout_horizon_s = horizon;
      params.rollout_dt_s = rollout_dt;
      const SilScenarioRunner runner(params);
      for (double desired_linear : {-0.8, 0.0, 0.4, 1.2, 2.0}) {
        for (double desired_angular : {-2.0, -0.6, 0.0, 0.6, 2.0}) {
          const double obstacle_x = desired_linear < 0.0 ? -0.90 : 1.10;
          const ObstacleField field = makeField(
            {
              {obstacle_x, -0.25}, {obstacle_x, 0.0}, {obstacle_x, 0.25}});
          const auto result = runner.evaluate(desired_linear, desired_angular, field);
          runner.expectFiniteAndBounded(result);
          runner.expectOutputTrajectorySafe(result, field);
        }
      }
    }
  }
}

TEST(T3SilTest, ControllerEvaluationP99IsBelowControlPeriod)
{
  const SilScenarioRunner runner;
  std::vector<std::pair<double, double>> points;
  appendLine(points, 1.2, -1.0, 1.2, 1.0, 41);
  appendLine(points, -0.5, 0.7, 2.5, 0.7, 31);
  const ObstacleField field = makeField(points);

  std::vector<double> elapsed_ms;
  elapsed_ms.reserve(300);
  for (int iteration = 0; iteration < 300; ++iteration) {
    const auto start = std::chrono::steady_clock::now();
    const auto result = runner.evaluate(1.0, 0.3, field);
    const auto finish = std::chrono::steady_clock::now();
    runner.expectFiniteAndBounded(result);
    elapsed_ms.push_back(
      std::chrono::duration<double, std::milli>(finish - start).count());
  }

  std::sort(elapsed_ms.begin(), elapsed_ms.end());
  const std::size_t p99_index =
    static_cast<std::size_t>(std::ceil(0.99 * elapsed_ms.size())) - 1U;
  const double p99_ms = elapsed_ms.at(p99_index);
  const double average_ms =
    std::accumulate(elapsed_ms.begin(), elapsed_ms.end(), 0.0) /
    static_cast<double>(elapsed_ms.size());
  EXPECT_LT(average_ms, 25.0);
  EXPECT_LT(p99_ms, 40.0);
}

}  // namespace
