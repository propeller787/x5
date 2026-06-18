#include <gtest/gtest.h>

#include "safety_node/core/emergency_brake.h"
#include "safety_node/core/motion_rollout.h"
#include "safety_node/core/swept_footprint_checker.h"
#include "safety_node/core/vehicle_footprint.h"
#include "safety_node/mode_supervisor.h"

#include <cmath>
#include <limits>

namespace
{

ObstacleField singleObstacle(double x, double y, double radius = 0.05)
{
  ObstaclePrimitive obstacle;
  obstacle.x = x;
  obstacle.y = y;
  obstacle.radius = radius;
  obstacle.valid = true;
  ObstacleField field;
  field.obstacles.push_back(obstacle);
  return field;
}

}  // namespace

TEST(VehicleFootprintTest, InflatedBoundsMatchConfiguredGeometry)
{
  AvoidanceParams params;
  VehicleFootprint footprint(params);
  const FootprintPolygon physical = footprint.physicalPolygon();
  const FootprintRect physical_bounds = footprint.physicalRect();
  const FootprintRect inflated_bounds = footprint.inflatedRect();

  EXPECT_NEAR(physical.vertices[0].first, -0.22, 1e-9);
  EXPECT_NEAR(physical.vertices[1].first, 0.66, 1e-9);
  EXPECT_NEAR(physical.vertices[0].second, 0.35, 1e-9);
  EXPECT_NEAR(physical.vertices[3].second, -0.35, 1e-9);

  EXPECT_NEAR(physical_bounds.x_min, -0.22, 1e-9);
  EXPECT_NEAR(physical_bounds.x_max, 0.66, 1e-9);
  EXPECT_NEAR(inflated_bounds.x_min, -0.34, 1e-9);
  EXPECT_NEAR(inflated_bounds.x_max, 0.76, 1e-9);
  EXPECT_NEAR(inflated_bounds.y_min, -0.40, 1e-9);
  EXPECT_NEAR(inflated_bounds.y_max, 0.40, 1e-9);
}

TEST(VehicleFootprintTest, DualTrapezoidPhysicalVerticesMatchConfiguredGeometry)
{
  AvoidanceParams params;
  params.footprint_use_dual_trapezoid = true;
  params.robot_rear_offset = 0.23;
  params.footprint_rear_segment.length_m = 0.92;
  params.footprint_rear_segment.width_front_m = 0.62;
  params.footprint_rear_segment.width_rear_m = 0.68;
  params.footprint_front_segment.length_m = 0.25;
  params.footprint_front_segment.width_front_m = 0.34;
  params.footprint_front_segment.width_rear_m = 0.62;
  params.footprint_front_margin = 0.05;
  params.footprint_back_margin = 0.08;
  params.footprint_side_margin = 0.01;
  VehicleFootprint footprint(params);
  const FootprintPolygon physical = footprint.physicalPolygon();
  const FootprintRect physical_bounds = footprint.physicalRect();
  const FootprintRect inflated_bounds = footprint.inflatedRect();

  EXPECT_EQ(physical.vertices.size(), 6U);
  EXPECT_NEAR(physical.vertices[0].first, -0.23, 1e-9);
  EXPECT_NEAR(physical.vertices[1].first, 0.69, 1e-9);
  EXPECT_NEAR(physical.vertices[2].first, 0.94, 1e-9);
  EXPECT_NEAR(physical.vertices[0].second, 0.34, 1e-9);
  EXPECT_NEAR(physical.vertices[2].second, 0.17, 1e-9);
  EXPECT_NEAR(physical.vertices[3].second, -0.17, 1e-9);
  EXPECT_NEAR(physical_bounds.x_min, -0.23, 1e-9);
  EXPECT_NEAR(physical_bounds.x_max, 0.94, 1e-9);
  EXPECT_NEAR(physical_bounds.y_min, -0.34, 1e-9);
  EXPECT_NEAR(physical_bounds.y_max, 0.34, 1e-9);
  EXPECT_NEAR(inflated_bounds.x_min, -0.31, 1e-9);
  EXPECT_NEAR(inflated_bounds.x_max, 0.99, 1e-9);
  // 斜边法向膨胀在分节角点处 y 外扩略大于 side_margin。
  EXPECT_NEAR(inflated_bounds.y_min, -0.35, 0.005);
  EXPECT_NEAR(inflated_bounds.y_max, 0.35, 0.005);

  EXPECT_FALSE(VehicleFootprint::circleIntersectsPolygon(0.94, 0.18, 0.0, physical));
  EXPECT_TRUE(VehicleFootprint::circleContainedInPolygon(0.94, 0.16, 0.0, physical));
  EXPECT_TRUE(VehicleFootprint::circleContainedInPolygon(-0.23, 0.34, 0.0, physical));
}

TEST(VehicleFootprintTest, TrapezoidFrontIsNarrowerThanRear)
{
  AvoidanceParams params;
  params.footprint_use_dual_trapezoid = false;
  params.robot_length = 1.12;
  params.robot_rear_offset = 0.23;
  params.robot_width_rear_m = 0.68;
  params.robot_width_front_m = 0.62;
  VehicleFootprint footprint(params);
  const FootprintPolygon physical = footprint.physicalPolygon();

  EXPECT_NEAR(physical.vertices[0].first, -0.23, 1e-9);
  EXPECT_NEAR(physical.vertices[1].first, 0.89, 1e-9);
  EXPECT_NEAR(physical.vertices[0].second, 0.34, 1e-9);
  EXPECT_NEAR(physical.vertices[1].second, 0.31, 1e-9);
  EXPECT_NEAR(physical.vertices[2].second, -0.31, 1e-9);

  EXPECT_FALSE(VehicleFootprint::circleIntersectsPolygon(0.89, 0.33, 0.0, physical));
  EXPECT_TRUE(VehicleFootprint::circleContainedInPolygon(0.89, 0.30, 0.0, physical));
  EXPECT_TRUE(VehicleFootprint::circleContainedInPolygon(-0.23, 0.34, 0.0, physical));
}

TEST(VehicleFootprintTest, PhysicalPolygonExcludesMarginOnlyOverlap)
{
  AvoidanceParams params;
  params.footprint_use_dual_trapezoid = false;
  VehicleFootprint footprint(params);
  const FootprintPolygon physical = footprint.physicalPolygon();
  const FootprintPolygon inflated = footprint.inflatedPolygon();

  EXPECT_TRUE(VehicleFootprint::circleIntersectsPolygon(0.72, 0.0, 0.08, inflated));
  EXPECT_TRUE(VehicleFootprint::circleIntersectsPolygon(0.72, 0.0, 0.08, physical));
  EXPECT_TRUE(VehicleFootprint::circleIntersectsPolygon(0.60, 0.0, 0.08, physical));
  EXPECT_TRUE(VehicleFootprint::circleIntersectsPolygon(0.55, 0.0, 0.08, physical));
  EXPECT_FALSE(VehicleFootprint::circleIntersectsPolygon(0.72, 0.0, 0.01, physical));
  EXPECT_TRUE(VehicleFootprint::circleContainedInPolygon(0.72, 0.0, 0.01, inflated));
}

TEST(VehicleFootprintTest, PointInsideOutsideAndBoundary)
{
  const FootprintRect rect {-1.0, 1.0, -0.5, 0.5};
  EXPECT_TRUE(VehicleFootprint::pointInRect(0.0, 0.0, 0.0, rect));
  EXPECT_TRUE(VehicleFootprint::pointInRect(1.0, 0.0, 0.0, rect));
  EXPECT_FALSE(VehicleFootprint::pointInRect(1.01, 0.0, 0.0, rect));
  EXPECT_LT(VehicleFootprint::clearanceToRect(0.0, 0.0, 0.0, rect), 0.0);
  EXPECT_NEAR(VehicleFootprint::clearanceToRect(1.5, 0.0, 0.0, rect), 0.5, 1e-9);
  EXPECT_NEAR(
    VehicleFootprint::clearanceToRect(1.5, 1.0, 0.0, rect),
    std::hypot(0.5, 0.5), 1e-9);
  EXPECT_FALSE(VehicleFootprint::pointInRect(-1.1, 0.0, 0.0, rect));
  EXPECT_FALSE(VehicleFootprint::pointInRect(0.0, 0.6, 0.0, rect));
}

TEST(VehicleFootprintTest, UpdatedGeometryIsUsed)
{
  AvoidanceParams params;
  VehicleFootprint footprint(params);
  params.robot_length = 1.2;
  params.robot_width_rear_m = 0.8;
  params.robot_width_front_m = 0.8;
  footprint.updateParams(params);
  const auto rect = footprint.inflatedRect();
  EXPECT_NEAR(rect.x_max, 1.08, 1e-9);
  EXPECT_NEAR(rect.y_max, 0.45, 1e-9);
}

TEST(MotionRolloutGeometryTest, ForwardBackwardAndArcTrajectories)
{
  AvoidanceParams params;
  params.rollout_horizon_s = 1.0;
  params.rollout_dt_s = 0.1;
  MotionRollout rollout(params);

  const auto forward = rollout.rolloutCandidate({1.0, 0.0});
  const auto backward = rollout.rolloutCandidate({-1.0, 0.0});
  const auto left = rollout.rolloutCandidate({1.0, 1.0});
  const auto right = rollout.rolloutCandidate({1.0, -1.0});

  EXPECT_NEAR(forward.poses.back().x, 1.0, 1e-9);
  EXPECT_NEAR(backward.poses.back().x, -1.0, 1e-9);
  EXPECT_GT(left.poses.back().y, 0.0);
  EXPECT_GT(left.poses.back().theta, 0.0);
  EXPECT_LT(right.poses.back().y, 0.0);
  EXPECT_LT(right.poses.back().theta, 0.0);
}

TEST(MotionRolloutGeometryTest, InPlaceRotationAccumulatesHeading)
{
  AvoidanceParams params;
  params.rollout_horizon_s = 1.0;
  params.rollout_dt_s = 0.1;
  MotionRollout rollout(params);
  const auto trajectory = rollout.rolloutCandidate({0.0, 1.0});

  EXPECT_NEAR(trajectory.poses.back().x, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.poses.back().y, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.poses.back().theta, 1.0, 1e-9);
}

TEST(SweptFootprintCheckerTest, DetectsForwardAndBackwardCollision)
{
  AvoidanceParams params;
  VehicleFootprint footprint(params);
  SweptFootprintChecker checker(params, footprint);
  MotionRollout rollout(params);

  const auto forward = checker.checkTrajectory(
    rollout.rolloutCandidate({0.5, 0.0}), singleObstacle(1.0, 0.0));
  const auto backward = checker.checkTrajectory(
    rollout.rolloutCandidate({-0.5, 0.0}), singleObstacle(-0.8, 0.0));

  EXPECT_TRUE(forward.collision);
  EXPECT_TRUE(backward.collision);
  EXPECT_EQ(
    deriveCollisionDirection(forward.collision_x, forward.collision_y),
    CollisionDirection::FORWARD);
  EXPECT_EQ(
    deriveCollisionDirection(backward.collision_x, backward.collision_y),
    CollisionDirection::BACKWARD);
}

TEST(SweptFootprintCheckerTest, DetectsLeftAndRightCollision)
{
  AvoidanceParams params;
  VehicleFootprint footprint(params);
  SweptFootprintChecker checker(params, footprint);
  MotionRollout rollout(params);
  const auto stationary = rollout.rolloutCandidate({0.0, 0.0});

  const auto left = checker.checkTrajectory(stationary, singleObstacle(0.0, 0.44));
  const auto right = checker.checkTrajectory(stationary, singleObstacle(0.0, -0.44));

  EXPECT_TRUE(left.collision);
  EXPECT_TRUE(right.collision);
  EXPECT_EQ(deriveCollisionDirection(0.0, 0.44), CollisionDirection::LEFT);
  EXPECT_EQ(deriveCollisionDirection(0.0, -0.44), CollisionDirection::RIGHT);
}

TEST(SweptFootprintCheckerTest, IgnoresInvalidAndReportsClearTrajectory)
{
  AvoidanceParams params;
  VehicleFootprint footprint(params);
  SweptFootprintChecker checker(params, footprint);
  params.footprint_side_margin = 0.1;
  checker.updateParams(params);
  MotionRollout rollout(params);
  ObstaclePrimitive invalid;
  invalid.valid = false;
  ObstaclePrimitive far;
  far.x = 4.0;
  far.valid = true;
  ObstacleField field;
  field.obstacles = {invalid, far};

  const auto result = checker.checkTrajectory(
    rollout.rolloutCandidate({0.2, 0.0}), field);
  EXPECT_FALSE(result.collision);
  EXPECT_TRUE(std::isinf(result.ttc));
  EXPECT_TRUE(std::isfinite(result.min_distance));
}

TEST(SweptFootprintCheckerTest, ReportsDistanceToHardStopAndCollision)
{
  AvoidanceParams params;
  params.hard_stop_distance_m = 0.12;
  VehicleFootprint footprint(params);
  SweptFootprintChecker checker(params, footprint);
  TrajectoryRollout trajectory;
  trajectory.poses = {
    {0.0, 0.0, 0.0, 0.0},
    {0.1, 0.0, 0.0, 0.1},
    {0.2, 0.0, 0.0, 0.2},
  };

  const auto result = checker.checkTrajectory(
    trajectory, singleObstacle(1.01, 0.0, 0.05));

  EXPECT_NEAR(result.path_length_m, 0.2, 1e-9);
  EXPECT_NEAR(result.hard_stop_path_distance_m, 0.1, 1e-9);
  EXPECT_NEAR(result.collision_path_distance_m, 0.2, 1e-9);
}

TEST(AvoidanceUtilsTest, DirectionsLateralDistancesAndInterventionLevels)
{
  EXPECT_EQ(deriveCollisionDirection(0.0, 0.0), CollisionDirection::FORWARD);
  EXPECT_EQ(deriveCollisionDirection(1.0, 1.0), CollisionDirection::FORWARD);
  EXPECT_EQ(deriveCollisionDirection(-1.0, 0.2), CollisionDirection::BACKWARD);
  EXPECT_EQ(deriveCollisionDirection(0.2, 1.0), CollisionDirection::LEFT);
  EXPECT_EQ(deriveCollisionDirection(0.2, -1.0), CollisionDirection::RIGHT);

  ObstaclePrimitive invalid;
  invalid.y = 0.1;
  invalid.valid = false;
  ObstaclePrimitive left;
  left.y = 0.5;
  left.radius = 0.1;
  left.valid = true;
  ObstaclePrimitive right;
  right.y = -0.7;
  right.radius = 0.1;
  right.valid = true;
  double left_distance = 0.0;
  double right_distance = 0.0;
  ObstacleField lateral_field;
  lateral_field.obstacles = {invalid, left, right};
  estimateLateralDistances(lateral_field, left_distance, right_distance);
  EXPECT_NEAR(left_distance, 0.4, 1e-9);
  EXPECT_NEAR(right_distance, 0.6, 1e-9);

  AvoidanceParams params;
  const std::array<std::pair<std::pair<double, double>, CollisionDirection>, 4> cases = {{
    {{1.0, 0.0}, CollisionDirection::FORWARD},
    {{-1.0, 0.0}, CollisionDirection::BACKWARD},
    {{0.0, 1.0}, CollisionDirection::LEFT},
    {{0.0, -1.0}, CollisionDirection::RIGHT},
  }};
  for (const auto & item : cases) {
    CollisionRisk emergency;
    emergency.has_collision = true;
    emergency.worst.collision = true;
    emergency.worst.collision_x = item.first.first;
    emergency.worst.collision_y = item.first.second;
    emergency.min_ttc = 0.1;
    emergency.min_clearance = 1.0;
    fillInterventionLevels(emergency, params);
    EXPECT_EQ(emergency.levels.primary, item.second);

    CollisionRisk decel = emergency;
    decel.min_ttc = 0.8;
    decel.levels = {};
    fillInterventionLevels(decel, params);
    EXPECT_EQ(decel.levels.primary, item.second);
  }
}

TEST(EmergencyBrakeTest, DirectionAwareClearanceAllowsReleaseWhenStopped)
{
  AvoidanceParams params;
  params.hard_stop_distance_m = 0.12;
  params.hard_stop_release_distance_m = 0.20;
  EmergencyBrake brake(params);

  CollisionRisk risk;
  risk.min_clearance = 0.08;
  EmergencyBrakeContext context;
  context.vehicle_stopped = true;
  context.direction_command_active = true;
  context.directional_clearance_valid = true;
  context.directional_clearance = 0.30;

  EXPECT_FALSE(brake.shouldEmergencyBrake(risk, &context));
  EXPECT_FALSE(brake.update(risk, 0.025, &context));
}

TEST(EmergencyBrakeTest, UsesReleaseHysteresisAndClearHold)
{
  AvoidanceParams params;
  params.hard_stop_distance_m = 0.12;
  params.hard_stop_release_distance_m = 0.20;
  params.emergency_ttc_s = 0.45;
  params.emergency_ttc_release_s = 0.70;
  params.emergency_clear_hold_s = 0.30;
  EmergencyBrake brake(params);

  CollisionRisk danger;
  danger.min_clearance = 0.11;
  EXPECT_TRUE(brake.shouldEmergencyBrake(danger));
  EXPECT_TRUE(brake.update(danger, 0.025));
  EXPECT_TRUE(brake.latched());

  CollisionRisk still_near;
  still_near.min_clearance = 0.13;
  still_near.min_ttc = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(brake.shouldEmergencyBrake(still_near));
  EXPECT_TRUE(brake.update(still_near, 0.20));
  EXPECT_TRUE(brake.latched());

  CollisionRisk clear;
  clear.min_clearance = 0.25;
  clear.min_ttc = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(brake.update(clear, 0.10));
  EXPECT_TRUE(brake.latched());
  EXPECT_TRUE(brake.update(clear, 0.19));
  EXPECT_TRUE(brake.latched());
  EXPECT_FALSE(brake.update(clear, 0.01));
  EXPECT_FALSE(brake.latched());
}

TEST(EmergencyBrakeTest, TtcIsDiagnosticWhenDisabled)
{
  AvoidanceParams params;
  params.use_ttc_emergency_stop = false;
  EmergencyBrake brake(params);
  CollisionRisk risk;
  risk.has_collision = true;
  risk.min_ttc = 0.05;
  risk.min_clearance = 1.0;

  EXPECT_FALSE(brake.shouldEmergencyBrake(risk));

  params.use_ttc_emergency_stop = true;
  brake.updateParams(params);
  EXPECT_TRUE(brake.shouldEmergencyBrake(risk));
}

TEST(EmergencyBrakeTest, IgnoresHardStopClearanceWhenDisabled)
{
  AvoidanceParams params;
  params.hard_stop_clearance_enabled = false;
  params.hard_stop_distance_m = 0.12;
  EmergencyBrake brake(params);
  CollisionRisk risk;
  risk.min_clearance = 0.05;
  EXPECT_FALSE(brake.shouldEmergencyBrake(risk));
}

TEST(HardStopClearanceTest, FootprintCandidateAndBrakingEnvelopeRespectEnableFlag)
{
  AvoidanceParams enabled;
  enabled.hard_stop_clearance_enabled = true;
  enabled.hard_stop_distance_m = 0.12;
  EXPECT_FALSE(footprintCandidateClearanceOk(0.05, enabled));
  EXPECT_TRUE(footprintCandidateClearanceOk(0.20, enabled));

  AvoidanceParams disabled = enabled;
  disabled.hard_stop_clearance_enabled = false;
  EXPECT_TRUE(footprintCandidateClearanceOk(0.05, disabled));

  CollisionCheckResult check;
  check.hard_stop_path_distance_m = 1.0;
  check.collision_path_distance_m = 1.5;
  EXPECT_DOUBLE_EQ(brakingEnvelopeAvailableDistance(check, enabled), 1.0);
  EXPECT_DOUBLE_EQ(brakingEnvelopeAvailableDistance(check, disabled), 1.5);
}

TEST(ModeSupervisorTest, ProfilesEnableExpectedSubsystems)
{
  const ModeProfile pass = ModeSupervisor(SafetyMode::PASS_THROUGH).profile();
  const ModeProfile assist = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ModeProfile narrow = ModeSupervisor(SafetyMode::NARROW).profile();
  const ModeProfile cruise = ModeSupervisor(SafetyMode::CRUISE).profile();
  const ModeProfile estop = ModeSupervisor(SafetyMode::ESTOP).profile();

  EXPECT_FALSE(pass.footprint_selector_enabled);
  EXPECT_FALSE(pass.emergency_stop_enabled);
  EXPECT_TRUE(assist.footprint_selector_enabled);
  EXPECT_TRUE(assist.map_constraint_enabled);
  EXPECT_TRUE(narrow.avoidance_overrides_enabled);
  EXPECT_DOUBLE_EQ(narrow.footprint_side_margin_m, 0.01);
  EXPECT_DOUBLE_EQ(narrow.obstacle_radius_m, 0.01);
  EXPECT_DOUBLE_EQ(narrow.hard_stop_distance_m, 0.015);
  EXPECT_DOUBLE_EQ(narrow.hard_stop_release_distance_m, 0.02);
  EXPECT_DOUBLE_EQ(narrow.inflation_radius_m, 0.01);
  EXPECT_DOUBLE_EQ(narrow.lethal_threshold, assist.lethal_threshold);
  EXPECT_TRUE(cruise.footprint_selector_enabled);
  EXPECT_FALSE(estop.footprint_selector_enabled);
  EXPECT_TRUE(estop.emergency_stop_enabled);

  ModeSupervisor supervisor(SafetyMode::PASS_THROUGH);
  EXPECT_FALSE(supervisor.setMode(SafetyMode::PASS_THROUGH));
  EXPECT_TRUE(supervisor.setModeFromString("cruise"));
  EXPECT_EQ(supervisor.mode(), SafetyMode::CRUISE);
  EXPECT_TRUE(supervisor.onModeChanged(SafetyMode::CRUISE, SafetyMode::NARROW));
  EXPECT_FALSE(supervisor.onModeChanged(SafetyMode::CRUISE, SafetyMode::CRUISE));
  EXPECT_EQ(ModeSupervisor::modeToString(SafetyMode::CRUISE), "cruise");
  EXPECT_EQ(ModeSupervisor::modeToString(SafetyMode::NARROW), "narrow");
  EXPECT_EQ(ModeSupervisor::modeToString(SafetyMode::ESTOP), "estop");

  const AvoidanceParams active =
    applyModeProfile(AvoidanceParams{}, narrow);
  EXPECT_DOUBLE_EQ(active.footprint_side_margin, 0.01);
  EXPECT_DOUBLE_EQ(active.footprint_obstacle_radius_default, 0.01);
  EXPECT_DOUBLE_EQ(active.hard_stop_distance_m, 0.015);
  EXPECT_DOUBLE_EQ(active.hard_stop_release_distance_m, 0.02);
}
