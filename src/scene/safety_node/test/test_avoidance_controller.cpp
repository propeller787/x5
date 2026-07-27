/**
 * @file test_avoidance_controller.cpp
 */
#include <gtest/gtest.h>

#include "safety_node/avoidance_controller.h"
#include "safety_node/core/vehicle_footprint.h"
#include "safety_node/output_shaper.h"

#include <cmath>
#include <initializer_list>

namespace
{

AvoidanceResult evaluate(
  AvoidanceController & controller,
  const geometry_msgs::msg::Twist & cmd,
  const ModeProfile & profile,
  SafetyMode mode,
  const RollingLocalMapGrid * map = nullptr,
  const ObstacleField & field = {},
  double prev_linear = 0.0,
  double prev_angular = 0.0,
  double fb_linear = 0.0,
  double fb_angular = 0.0,
  const ObstacleField * tilt_field = nullptr)
{
  AvoidanceContext context;
  context.safety_mode = mode;
  return controller.evaluate(
    cmd,
    fb_linear,
    fb_angular,
    field,
    map,
    profile,
    prev_linear,
    prev_angular,
    0.025,
    context,
    tilt_field);
}

RollingLocalMapGrid fullyLethalMap()
{
  RollingLocalMapGrid map;
  map.cols = 100;
  map.rows = 100;
  map.resolution_m = 0.1;
  map.origin_x_m = -5.0;
  map.origin_y_m = -5.0;
  map.probability.assign(map.cols * map.rows, 1.0f);
  map.inflated.assign(map.cols * map.rows, 1.0f);
  return map;
}

RollingLocalMapGrid mapWithLethalCell(double x, double y)
{
  RollingLocalMapGrid map;
  map.cols = 120;
  map.rows = 120;
  map.resolution_m = 0.05;
  map.origin_x_m = -3.0;
  map.origin_y_m = -3.0;
  map.probability.assign(map.cols * map.rows, 0.0f);
  map.inflated.assign(map.cols * map.rows, 0.0f);
  const auto add_cell = [&](double cell_x, double cell_y, float cost = 0.9f) {
      const int col = static_cast<int>(
        std::floor((cell_x - map.origin_x_m) / map.resolution_m));
      const int row = static_cast<int>(
        std::floor((cell_y - map.origin_y_m) / map.resolution_m));
      const size_t idx = static_cast<size_t>(row) * map.cols + static_cast<size_t>(col);
      map.probability[idx] = cost;
      map.inflated[idx] = cost;
    };
  add_cell(x, y);
  return map;
}

RollingLocalMapGrid sim002Map()
{
  RollingLocalMapGrid map;
  map.cols = 120;
  map.rows = 120;
  map.resolution_m = 0.05;
  map.origin_x_m = -3.0;
  map.origin_y_m = -3.0;
  map.probability.assign(map.cols * map.rows, 0.0f);
  map.inflated.assign(map.cols * map.rows, 0.0f);
  const auto add_cell = [&](double x, double y) {
      const int col = static_cast<int>(
        std::floor((x - map.origin_x_m) / map.resolution_m));
      const int row = static_cast<int>(
        std::floor((y - map.origin_y_m) / map.resolution_m));
      const size_t idx = static_cast<size_t>(row) * map.cols + static_cast<size_t>(col);
      map.probability[idx] = 0.9f;
      map.inflated[idx] = 0.9f;
    };
  for (const auto & cell : {
      std::pair<double, double>{0.975, -0.175},
      {1.025, -0.175},
      {1.075, -0.175},
      {0.975, -0.125},
      {1.025, -0.125},
      {1.025, 0.125},
      {1.025, 0.175},
      {-0.675, 0.325},
      {-0.625, 0.325},
      {-0.625, 0.375},
      {-0.575, 0.425},
      {-0.575, 0.475},
      {-0.525, 0.475}})
  {
    add_cell(cell.first, cell.second);
  }
  return map;
}

ObstacleField obstacleAhead(double x)
{
  ObstaclePrimitive obstacle;
  obstacle.x = x;
  obstacle.y = 0.0;
  obstacle.radius = 0.08;
  obstacle.valid = true;

  ObstacleField field;
  field.obstacles.push_back(obstacle);
  return field;
}

// Places a side obstacle at a requested clearance from the inflated footprint.
ObstaclePrimitive sideObstacleAtInflatedClearance(
  const AvoidanceParams & params,
  double clearance_m)
{
  VehicleFootprint footprint(params);
  const FootprintPolygon polygon = footprint.inflatedPolygon();

  ObstaclePrimitive obstacle;
  obstacle.x = 0.0;
  obstacle.radius = 0.01;
  obstacle.valid = true;

  double low_y = 0.0;
  double high_y = 2.0;
  for (int i = 0; i < 64; ++i) {
    const double y = 0.5 * (low_y + high_y);
    const double clearance =
      VehicleFootprint::clearanceToPolygon(obstacle.x, y, obstacle.radius, polygon);
    if (clearance < clearance_m) {
      low_y = y;
    } else {
      high_y = y;
    }
  }
  obstacle.y = 0.5 * (low_y + high_y);
  return obstacle;
}

/** 障碍中心落在物理 Footprint 内（不含 margin），用于侵入/recovery 测试 */
ObstacleField obstacleInsidePhysicalBody(double x = 0.05)
{
  return obstacleAhead(x);
}

ObstacleField tiltObstacleAhead(double x)
{
  ObstaclePrimitive obstacle;
  obstacle.x = x;
  obstacle.y = 0.0;
  obstacle.radius = 0.01;
  obstacle.valid = true;
  obstacle.source = "left_tilt";

  ObstacleField field;
  field.obstacles.push_back(obstacle);
  return field;
}

ObstacleField obstaclesAt(std::initializer_list<double> xs)
{
  ObstacleField field;
  for (const double x : xs) {
    ObstaclePrimitive obstacle;
    obstacle.x = x;
    obstacle.y = 0.0;
    obstacle.radius = 0.08;
    obstacle.valid = true;
    field.obstacles.push_back(obstacle);
  }
  return field;
}

ObstacleField frontInflatedFootprintObstacle()
{
  ObstaclePrimitive obstacle;
  obstacle.x = 0.985;
  obstacle.y = 0.0;
  obstacle.radius = 0.01;
  obstacle.valid = true;

  ObstacleField field;
  field.obstacles.push_back(obstacle);
  return field;
}

AvoidanceParams simVehicleFootprintParams()
{
  AvoidanceParams params;
  params.robot_length = 1.12;
  params.robot_width_rear_m = 0.68;
  params.robot_width_front_m = 0.62;
  params.robot_rear_offset = 0.23;
  params.footprint_front_margin = 0.05;
  params.footprint_back_margin = 0.08;
  params.footprint_side_margin = 0.01;
  params.hard_stop_distance_m = 0.015;
  params.hard_stop_release_distance_m = 0.017;
  params.recovery_lookahead_s = 0.35;
  return params;
}

AvoidanceParams productionFootprintParams()
{
  AvoidanceParams params = simVehicleFootprintParams();
  params.footprint_use_dual_trapezoid = true;
  params.footprint_rear_segment.length_m = 0.92;
  params.footprint_rear_segment.width_rear_m = 0.68;
  params.footprint_rear_segment.width_front_m = 0.62;
  params.footprint_front_segment.length_m = 0.25;
  params.footprint_front_segment.width_rear_m = 0.62;
  params.footprint_front_segment.width_front_m = 0.34;
  params.braking_envelope_enabled = true;
  params.max_decel_m_s2 = 0.8;
  params.max_recover_accel_m_s2 = 0.5;
  params.recovery_max_linear_speed = 0.12;
  params.recovery_max_angular_speed = 0.35;
  return params;
}

/** 左后角贴膨胀框、未浸入物理车体，类似 SIM-036 */
ObstacleField leftRearProximityObstacle()
{
  ObstacleField field;
  ObstaclePrimitive obstacle;
  obstacle.x = -0.36;
  obstacle.y = -0.28;
  obstacle.radius = 0.01;
  obstacle.valid = true;
  field.obstacles.push_back(obstacle);
  return field;
}

AvoidanceParams tiltTtcTestParams()
{
  AvoidanceParams params;
  params.robot_length = 0.05;
  params.robot_width_rear_m = 0.20;
  params.robot_width_front_m = 0.20;
  params.robot_rear_offset = 0.0;
  params.footprint_front_margin = 0.0;
  params.footprint_back_margin = 0.0;
  params.footprint_side_margin = 0.0;
  params.footprint_obstacle_radius_default = 0.01;
  params.braking_envelope_enabled = false;
  params.candidate_speed_up_hold_s = 0.0;
  params.max_decel_m_s2 = 100.0;
  params.max_recover_accel_m_s2 = 100.0;
  params.max_angular_decel_rad_s2 = 100.0;
  params.max_angular_recover_rad_s2 = 100.0;
  params.tilt_ttc.enabled = true;
  params.tilt_ttc.horizon_s = 0.10;
  params.tilt_ttc.max_probe_distance_m = 1.0;
  params.tilt_ttc.hard_stop_distance_m = 0.12;
  params.tilt_ttc.limiting_distance_m = 1.0;
  params.tilt_ttc.release_distance_m = 1.05;
  params.tilt_ttc.distance_emergency_m = 0.50;
  params.tilt_ttc.emergency_clear_hold_s = 0.0;
  params.tilt_ttc.obstacle_radius_m = 0.01;
  params.braking_comfort_decel_m_s2 = 0.60;
  params.braking_max_jerk_m_s3 = 2.5;
  params.braking_reaction_delay_s = 0.10;
  params.max_decel_m_s2 = 0.8;
  return params;
}

}  // namespace

TEST(AvoidanceControllerTest, PassThroughClampsAbsoluteSpeed)
{
  AvoidanceParams params;
  params.absolute_max_linear_speed = 1.0;
  params.absolute_max_angular_speed = 2.0;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 3.0;
  cmd.angular.z = -4.0;

  const ModeProfile profile = ModeSupervisor(SafetyMode::PASS_THROUGH).profile();
  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::PASS_THROUGH);

  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(result.safe_twist.angular.z, -2.0);
  EXPECT_FALSE(result.emergency_brake);
}

TEST(AvoidanceControllerTest, EstopForcesImmediateZeroCommand)
{
  AvoidanceController controller(AvoidanceParams{}, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.8;
  cmd.angular.z = 0.4;

  const ModeProfile profile = ModeSupervisor(SafetyMode::ESTOP).profile();
  const AvoidanceResult result = evaluate(controller, cmd, profile, SafetyMode::ESTOP);

  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.safe_twist.angular.z, 0.0);
  EXPECT_TRUE(result.emergency_brake);
  EXPECT_EQ(result.emergency_reason, EmergencyStopReason::MODE_ESTOP);
  EXPECT_TRUE(result.risk.emergency);
}

TEST(AvoidanceControllerTest, MapVetoCannotFallBackToMapBlindVelocity)
{
  MapConstraintConfig map_config;
  map_config.enabled = true;
  map_config.veto_on_lethal = true;
  AvoidanceController controller(AvoidanceParams{}, map_config);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.8;
  const RollingLocalMapGrid map = fullyLethalMap();
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();

  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, &map);

  EXPECT_TRUE(result.hybrid_log.map_veto_applied);
  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.safe_twist.angular.z, 0.0);
  EXPECT_TRUE(result.emergency_brake);
  EXPECT_EQ(result.emergency_reason, EmergencyStopReason::MAP_VETO);
}

TEST(AvoidanceControllerTest, PinnedEscapeDisabledKeepsMapVetoHardStop)
{
  // 回归护栏：pinned_escape 默认关闭时，夹死死状态行为与既有 MAP_VETO 硬停一致。
  MapConstraintConfig map_config;
  map_config.enabled = true;
  map_config.veto_on_lethal = true;
  AvoidanceParams params;
  ASSERT_FALSE(params.pinned_escape.enabled);
  AvoidanceController controller(params, map_config);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.4;
  cmd.angular.z = 0.6;    // 驾驶员持续给令
  const RollingLocalMapGrid map = fullyLethalMap();
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();

  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, &map);

  EXPECT_TRUE(result.hybrid_log.map_veto_applied);
  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.safe_twist.angular.z, 0.0);
  EXPECT_TRUE(result.emergency_brake);
  EXPECT_EQ(result.emergency_reason, EmergencyStopReason::MAP_VETO);
  EXPECT_NE(result.recovery_state, RecoveryState::PINNED_ESCAPE);
}

TEST(AvoidanceControllerTest, PinnedEscapeNeverFabricatesMotionWhenFullyBoxed)
{
  // 开启但四周皆 lethal：无任何 map-clear 微动作，即使去抖满足也保持零速硬停，
  // 证明脱困层绝不凭空造出不安全运动，只在存在经复验的安全微动作时才放行。
  MapConstraintConfig map_config;
  map_config.enabled = true;
  map_config.veto_on_lethal = true;
  AvoidanceParams params;
  params.pinned_escape.enabled = true;
  params.pinned_escape.confirm_s = 0.05;    // ~2 帧即确认
  AvoidanceController controller(params, map_config);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.angular.z = 0.6;    // 驾驶员持续给左转令
  const RollingLocalMapGrid map = fullyLethalMap();
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();

  AvoidanceResult result;
  for (int i = 0; i < 12; ++i) {    // 超过 confirm_s 去抖窗
    result = evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, &map);
  }

  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.safe_twist.angular.z, 0.0);
  EXPECT_TRUE(result.emergency_brake);
  EXPECT_EQ(result.emergency_reason, EmergencyStopReason::MAP_VETO);
  EXPECT_NE(result.recovery_state, RecoveryState::PINNED_ESCAPE);
}

TEST(AvoidanceControllerTest, InitialFrontMapOccupancyAllowsControlledReverseEscape)
{
  AvoidanceParams params = productionFootprintParams();
  MapConstraintConfig map_config;
  map_config.enabled = true;
  map_config.veto_on_lethal = true;
  AvoidanceController controller(params, map_config);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.5;
  cmd.angular.z = 0.2;
  const RollingLocalMapGrid map = sim002Map();
  const ModeProfile profile = ModeSupervisor(SafetyMode::NARROW).profile();
  const ObstacleField field = frontInflatedFootprintObstacle();

  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::NARROW, &map, field);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_LT(result.safe_twist.linear.x, 0.0);
  EXPECT_LE(
    std::abs(result.safe_twist.linear.x),
    params.recovery_max_linear_speed + 1e-9);
  EXPECT_GE(result.hybrid_log.final_candidate, 0);
}

TEST(AvoidanceControllerTest, ControlledReverseEscapeContinuesWhileAlreadyMoving)
{
  AvoidanceParams params = productionFootprintParams();
  MapConstraintConfig map_config;
  map_config.enabled = true;
  map_config.veto_on_lethal = true;
  AvoidanceController controller(params, map_config);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.5;
  cmd.angular.z = 0.2;
  const RollingLocalMapGrid map = sim002Map();
  const ModeProfile profile = ModeSupervisor(SafetyMode::NARROW).profile();
  const ObstacleField field = frontInflatedFootprintObstacle();

  const AvoidanceResult result = evaluate(
    controller,
    cmd,
    profile,
    SafetyMode::NARROW,
    &map,
    field,
    -0.08,
    0.0,
    -0.04,
    0.0);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_LT(result.safe_twist.linear.x, 0.0);
  EXPECT_LE(
    std::abs(result.safe_twist.linear.x),
    params.recovery_max_linear_speed + 1e-9);
  EXPECT_GE(result.hybrid_log.final_candidate, 0);
}

TEST(AvoidanceControllerTest, InitialRearMapOccupancyStillBlocksReverse)
{
  AvoidanceParams params = productionFootprintParams();
  MapConstraintConfig map_config;
  map_config.enabled = true;
  map_config.veto_on_lethal = true;
  AvoidanceController controller(params, map_config);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.5;
  const RollingLocalMapGrid map = mapWithLethalCell(-0.325, 0.0);
  const ModeProfile profile = ModeSupervisor(SafetyMode::NARROW).profile();

  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::NARROW, &map);

  EXPECT_TRUE(result.emergency_brake);
  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.0);
  EXPECT_EQ(result.emergency_reason, EmergencyStopReason::MAP_VETO);
}

TEST(AvoidanceControllerTest, ProjectedCollisionUsesDistanceLimitWithoutEmergencyStop)
{
  AvoidanceParams params;
  params.max_recover_accel_m_s2 = 100.0;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = obstacleAhead(1.75);

  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);

  EXPECT_GT(result.safe_twist.linear.x, 0.0);
  EXPECT_LT(result.safe_twist.linear.x, cmd.linear.x);
  EXPECT_FALSE(result.emergency_brake);
  EXPECT_TRUE(result.risk.has_collision);
  EXPECT_TRUE(result.risk.braking_limited);
  EXPECT_TRUE(brakingEnvelopeCanStop(result.risk));
}

TEST(AvoidanceControllerTest, CandidateBelowHardStopClearanceIsRejectedBeforeOutput)
{
  AvoidanceParams params;
  params.braking_envelope_enabled = false;
  params.rollout_horizon_s = 0.10;
  params.rollout_dt_s = 0.10;
  params.max_decel_m_s2 = 100.0;
  params.max_recover_accel_m_s2 = 100.0;
  params.hard_stop_distance_m = 0.05;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  ObstacleField field;
  ObstaclePrimitive obstacle;
  obstacle.x = 0.91;
  obstacle.y = 0.0;
  obstacle.radius = 0.01;
  obstacle.valid = true;
  field.obstacles.push_back(obstacle);

  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_EQ(result.hybrid_log.final_candidate, 1);
  EXPECT_NEAR(result.safe_twist.linear.x, 0.8, 1e-9);
  EXPECT_GT(result.risk.min_clearance, params.hard_stop_distance_m);
}

TEST(AvoidanceControllerTest, DisabledIntrusionCheckSkipsIntrusionRecovery)
{
  AvoidanceParams params;
  params.recovery_intrusion_confirm_s = 0.0;
  params.footprint_intrusion_check_enabled = false;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = obstacleInsidePhysicalBody();

  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);

  EXPECT_FALSE(result.recovery_active);
}

TEST(AvoidanceControllerTest, InsideFootprintObstacleAllowsSlowEscapeAway)
{
  AvoidanceParams params;
  params.recovery_intrusion_confirm_s = 0.0;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = obstacleInsidePhysicalBody();

  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);

  EXPECT_TRUE(result.recovery_active);
  EXPECT_FALSE(result.emergency_brake);
  EXPECT_NEAR(result.safe_twist.linear.x, -params.recovery_max_linear_speed, 1e-9);
  EXPECT_TRUE(result.risk.has_collision);
}

TEST(AvoidanceControllerTest, StoppedProximityObstacleTriggersRecovery)
{
  AvoidanceParams params;
  params.recovery_intrusion_confirm_s = 0.0;
  params.hard_stop_distance_m = 0.12;
  AvoidanceController controller(params, MapConstraintConfig{});

  ObstacleField field;
  ObstaclePrimitive obstacle;
  obstacle.x = 0.88;
  obstacle.y = 0.0;
  obstacle.radius = 0.01;
  obstacle.valid = true;
  field.obstacles.push_back(obstacle);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field, 0.0, 0.0, 0.0, 0.0);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_LT(result.safe_twist.linear.x, 0.0);
}

TEST(AvoidanceControllerTest, StoppedProximityObstacleUsesRecoveryWhenNoSafeCandidate)
{
  AvoidanceParams params;
  params.recovery_intrusion_confirm_s = 0.0;
  params.hard_stop_distance_m = 0.12;
  params.recovery_auto_select_enabled = true;
  AvoidanceController controller(params, MapConstraintConfig{});

  ObstacleField field;
  ObstaclePrimitive obstacle;
  obstacle.x = 0.88;
  obstacle.y = 0.0;
  obstacle.radius = 0.01;
  obstacle.valid = true;
  field.obstacles.push_back(obstacle);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field, 0.0, 0.0, 0.0, 0.0);

  EXPECT_TRUE(result.recovery_active);
  EXPECT_FALSE(result.emergency_brake);
  EXPECT_LT(result.safe_twist.linear.x, 0.0);
  EXPECT_EQ(result.recovery_state, RecoveryState::ACTIVE);
}

TEST(AvoidanceControllerTest, MovingProximityObstacleDoesNotTriggerRecovery)
{
  AvoidanceParams params;
  params.recovery_intrusion_confirm_s = 0.0;
  AvoidanceController controller(params, MapConstraintConfig{});

  ObstacleField field;
  ObstaclePrimitive obstacle;
  obstacle.x = 0.88;
  obstacle.y = 0.0;
  obstacle.radius = 0.01;
  obstacle.valid = true;
  field.obstacles.push_back(obstacle);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const AvoidanceResult result =
    evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field,
    0.2, 0.0, 0.2, 0.0);

  EXPECT_FALSE(result.recovery_active);
  EXPECT_NE(result.recovery_state, RecoveryState::CONFIRMING);
  EXPECT_NE(result.recovery_state, RecoveryState::BLOCKED);
}

TEST(AvoidanceControllerTest, LeftRearProximityAllowsForwardEscape)
{
  AvoidanceParams params = simVehicleFootprintParams();
  params.recovery_mode_enabled = true;
  params.braking_envelope_enabled = false;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.5;
  const ModeProfile profile = ModeSupervisor(SafetyMode::NARROW).profile();
  const AvoidanceResult result = evaluate(
    controller, cmd, profile, SafetyMode::NARROW, nullptr, leftRearProximityObstacle(),
    0.0, 0.0, 0.0, 0.0);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_GT(result.safe_twist.linear.x, 0.01);
  EXPECT_GE(result.hybrid_log.final_candidate, 0);
}

TEST(AvoidanceControllerTest, ForwardEscapeWorksAfterProximityIdleStop)
{
  AvoidanceParams params = simVehicleFootprintParams();
  params.recovery_intrusion_confirm_s = 0.0;
  params.recovery_timeout_s = 0.10;
  params.braking_envelope_enabled = false;
  AvoidanceController controller(params, MapConstraintConfig{});

  const ModeProfile profile = ModeSupervisor(SafetyMode::NARROW).profile();
  const ObstacleField field = leftRearProximityObstacle();

  geometry_msgs::msg::Twist idle;
  for (int i = 0; i < 8; ++i) {
    evaluate(
      controller, idle, profile, SafetyMode::NARROW, nullptr, field,
      0.0, 0.0, 0.0, 0.0);
  }

  geometry_msgs::msg::Twist forward;
  forward.linear.x = 0.5;
  const AvoidanceResult result = evaluate(
    controller, forward, profile, SafetyMode::NARROW, nullptr, field,
    0.0, 0.0, 0.0, 0.0);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_GT(result.safe_twist.linear.x, 0.01);
}

TEST(AvoidanceControllerTest, DirectionAwareReleaseAllowsBackwardWhenFrontBlocked)
{
  AvoidanceParams params;
  params.recovery_mode_enabled = false;
  params.braking_envelope_enabled = false;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.5;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  ObstacleField field;
  ObstaclePrimitive obstacle;
  obstacle.x = 0.88;
  obstacle.y = 0.0;
  obstacle.radius = 0.01;
  obstacle.valid = true;
  field.obstacles.push_back(obstacle);
  const AvoidanceResult result =
    evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field,
    0.0, 0.0, 0.0, 0.0);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_LT(result.safe_twist.linear.x, 0.0);
}

TEST(AvoidanceControllerTest, BrakingSpeedLimitChangesAreRateLimited)
{
  AvoidanceParams params;
  params.braking_lookahead_m = 3.0;
  params.braking_path_step_m = 0.025;
  params.max_decel_m_s2 = 0.8;
  params.max_recover_accel_m_s2 = 0.5;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField far_field = obstacleAhead(2.0);
  const AvoidanceResult far =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, far_field);
  const AvoidanceResult near =
    evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, obstacleAhead(1.2),
    far.safe_twist.linear.x, 0.0, far.safe_twist.linear.x, 0.0);

  ASSERT_TRUE(far.risk.braking_limited);
  ASSERT_TRUE(near.risk.braking_limited);
  EXPECT_LT(near.risk.braking_speed_limit_m_s, far.risk.braking_speed_limit_m_s);
  EXPECT_GT(
    near.risk.braking_speed_limit_m_s,
    far.risk.braking_speed_limit_m_s - params.max_decel_m_s2 * 0.025 - 1e-6);
}

TEST(AvoidanceControllerTest, InsideFootprintObstacleAutoSelectsEscapeAwayFromRequestedMotion)
{
  AvoidanceParams params;
  params.recovery_intrusion_confirm_s = 0.0;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = obstacleInsidePhysicalBody();

  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);

  EXPECT_TRUE(result.recovery_active);
  EXPECT_FALSE(result.emergency_brake);
  EXPECT_LT(result.safe_twist.linear.x, 0.0);
  EXPECT_EQ(result.recovery_state, RecoveryState::ACTIVE);
}

TEST(AvoidanceControllerTest, RecoveryRejectsEscapeIntoExternalObstacle)
{
  AvoidanceParams params;
  params.recovery_intrusion_confirm_s = 0.0;
  params.recovery_auto_select_enabled = false;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = obstaclesAt({0.55, -0.45});

  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);

  EXPECT_FALSE(result.recovery_active);
  EXPECT_TRUE(result.emergency_brake);
  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.0);
}

TEST(AvoidanceControllerTest, IntrusionMustPersistBeforeRecoveryStarts)
{
  AvoidanceParams params;
  params.recovery_intrusion_confirm_s = 0.10;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = obstacleInsidePhysicalBody();

  for (int i = 0; i < 3; ++i) {
    const AvoidanceResult result =
      evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);
    EXPECT_TRUE(result.emergency_brake);
    EXPECT_FALSE(result.recovery_active);
    EXPECT_EQ(result.recovery_state, RecoveryState::CONFIRMING);
  }

  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);
  EXPECT_TRUE(result.recovery_active);
  EXPECT_FALSE(result.emergency_brake);
  EXPECT_EQ(result.recovery_state, RecoveryState::ACTIVE);
}

TEST(AvoidanceControllerTest, RecoveryStopsAndLatchesWhenClearanceMakesNoProgress)
{
  AvoidanceParams params;
  params.recovery_intrusion_confirm_s = 0.0;
  params.recovery_motion_grace_s = 0.05;
  params.recovery_no_progress_timeout_s = 0.10;
  params.recovery_timeout_s = 1.0;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = obstacleInsidePhysicalBody();

  AvoidanceResult result;
  for (int i = 0; i < 8; ++i) {
    result = evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);
  }

  EXPECT_FALSE(result.recovery_active);
  EXPECT_TRUE(result.emergency_brake);
  EXPECT_EQ(result.recovery_state, RecoveryState::ABORTED_NO_PROGRESS);

  const AvoidanceResult latched =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);
  EXPECT_TRUE(latched.emergency_brake);
  EXPECT_EQ(latched.recovery_state, RecoveryState::ABORTED_NO_PROGRESS);
}

TEST(AvoidanceControllerTest, RecoveryStopsWhenWheelFeedbackDoesNotRespond)
{
  AvoidanceParams params;
  params.recovery_intrusion_confirm_s = 0.0;
  params.recovery_motion_grace_s = 0.05;
  params.recovery_no_progress_timeout_s = 10.0;
  params.recovery_feedback_stall_timeout_s = 0.10;
  params.recovery_timeout_s = 1.0;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = obstacleInsidePhysicalBody();

  AvoidanceResult result;
  for (int i = 0; i < 8; ++i) {
    result = evaluate(
      controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field,
      0.0, 0.0, 0.0, 0.0);
  }

  EXPECT_FALSE(result.recovery_active);
  EXPECT_TRUE(result.emergency_brake);
  EXPECT_EQ(result.recovery_state, RecoveryState::ABORTED_FEEDBACK_STALL);
}

TEST(AvoidanceControllerTest, RecoveryCandidateIsHeldDuringMinimumStableWindow)
{
  AvoidanceParams params;
  params.recovery_intrusion_confirm_s = 0.0;
  params.recovery_candidate_hold_s = 0.30;
  params.recovery_motion_grace_s = 10.0;
  AvoidanceController controller(params, MapConstraintConfig{});
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = obstacleInsidePhysicalBody();

  geometry_msgs::msg::Twist first_cmd;
  first_cmd.linear.x = 0.8;
  const AvoidanceResult first =
    evaluate(controller, first_cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);
  ASSERT_TRUE(first.recovery_active);

  geometry_msgs::msg::Twist changed_cmd;
  changed_cmd.linear.x = -0.8;
  changed_cmd.angular.z = 0.8;
  const AvoidanceResult held =
    evaluate(controller, changed_cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);

  ASSERT_TRUE(held.recovery_active);
  EXPECT_DOUBLE_EQ(held.safe_twist.linear.x, first.safe_twist.linear.x);
  EXPECT_DOUBLE_EQ(held.safe_twist.angular.z, first.safe_twist.angular.z);
}

TEST(AvoidanceControllerTest, RecoveryTimeoutStopsPersistentIntrusion)
{
  AvoidanceParams params;
  params.recovery_intrusion_confirm_s = 0.0;
  params.recovery_motion_grace_s = 10.0;
  params.recovery_timeout_s = 0.10;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = obstacleInsidePhysicalBody();

  AvoidanceResult result;
  for (int i = 0; i < 4; ++i) {
    result = evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);
  }

  EXPECT_FALSE(result.recovery_active);
  EXPECT_TRUE(result.emergency_brake);
  EXPECT_EQ(result.recovery_state, RecoveryState::ABORTED_TIMEOUT);
}

// 前撞后障碍进入物理车体前部（默认物理前缘 0.66 m 内），driver 后退摇杆。
ObstacleField frontInsideBodyObstacle()
{
  ObstaclePrimitive obstacle;
  obstacle.x = 0.60;
  obstacle.y = 0.0;
  obstacle.radius = 0.05;
  obstacle.valid = true;
  ObstacleField field;
  field.obstacles.push_back(obstacle);
  return field;
}

// L1 定向脱困释放阀回归：复刻 bag「前向撞车后后退摇杆车辆无法行动」。
// 前障进入物理车体 → intrusion.active → recovery 计时超时 → ABORTED_TIMEOUT 自锁。
// 旧行为对所有轴无条件清零并被自锁永久钉死（连方向掩码放行的后退也锁死，deadlock）。
// 修复后：driver 后退方向未被掩码封锁 → 放行受控倒车逃逸，车得以退出侵入。
TEST(AvoidanceControllerTest, DirectionalEscapeReleasesReverseAfterIntrusionAbort)
{
  AvoidanceParams params;
  params.direction_mask_enabled = true;
  params.recovery_intrusion_confirm_s = 0.0;
  params.recovery_motion_grace_s = 10.0;   // 隔离无进展/停滞判据，仅测超时自锁
  params.recovery_timeout_s = 0.10;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.8;                      // driver 持续后退摇杆
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = frontInsideBodyObstacle();

  AvoidanceResult result;
  for (int i = 0; i < 8; ++i) {
    result = evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);
  }

  // 自锁后不再被钉死：放行掩码未封锁的后退方向受控逃逸。
  EXPECT_TRUE(result.recovery_active);
  EXPECT_FALSE(result.emergency_brake);
  EXPECT_EQ(result.recovery_state, RecoveryState::DIRECTIONAL_ESCAPE);
  EXPECT_LT(result.safe_twist.linear.x, 0.0);
  EXPECT_NEAR(result.safe_twist.linear.x, -params.recovery_max_linear_speed, 1e-9);
}

// 关闭方向解耦时维持旧行为：超时自锁后仍硬停（回归保护，证明差异确由释放阀带来）。
TEST(AvoidanceControllerTest, IntrusionAbortStaysLatchedWithoutDirectionMask)
{
  AvoidanceParams params;
  params.direction_mask_enabled = false;
  params.recovery_intrusion_confirm_s = 0.0;
  params.recovery_motion_grace_s = 10.0;
  params.recovery_timeout_s = 0.10;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = frontInsideBodyObstacle();

  AvoidanceResult result;
  for (int i = 0; i < 8; ++i) {
    result = evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);
  }

  EXPECT_FALSE(result.recovery_active);
  EXPECT_TRUE(result.emergency_brake);
  EXPECT_EQ(result.recovery_state, RecoveryState::ABORTED_TIMEOUT);
  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.0);
}

// 后退方向被掩码判威胁（后方也有近障）时释放阀不放行，仍硬停——脱困释放不牺牲安全。
TEST(AvoidanceControllerTest, DirectionalEscapeStillStopsWhenReverseThreatened)
{
  AvoidanceParams params;
  params.direction_mask_enabled = true;
  params.recovery_intrusion_confirm_s = 0.0;
  params.recovery_motion_grace_s = 10.0;
  params.recovery_timeout_s = 0.10;
  params.recovery_auto_select_enabled = false;   // 仅评估 driver 后退候选，隔离自动转向脱困
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  ObstacleField field = frontInsideBodyObstacle();     // 前障 → intrusion.active
  ObstaclePrimitive rear;                              // 后障 → 后退被判向物理接触逼近
  rear.x = -0.30;                                      // 复用 direction_mask 单测已验证的后中障位（backward 封锁）
  rear.y = 0.0;
  rear.radius = 0.01;
  rear.valid = true;
  field.obstacles.push_back(rear);

  AvoidanceResult result;
  for (int i = 0; i < 8; ++i) {
    result = evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);
  }

  EXPECT_FALSE(result.recovery_active);
  EXPECT_TRUE(result.emergency_brake);
  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.0);
}

// 转弯净空护栏回归：复刻 bag「转弯加速撞车」——转弯逼近侧向近障时前向护栏失效。
// 护栏在 |w| 显著且 omni 净空收缩时按可停净空钳制线速度，使兜底来得及。
AvoidanceParams turnGuardParams()
{
  AvoidanceParams params;
  params.turn_clearance_guard_enabled = true;
  params.turn_clearance_guard_min_angular_rad_s = 0.5;
  params.turn_clearance_guard_band_m = 0.6;
  params.braking_comfort_decel_m_s2 = 0.6;
  params.hard_stop_distance_m = 0.05;
  params.braking_envelope_enabled = false;
  params.braking_lookahead_m = 0.0;   // 探针仅取当前位姿 → 曲线扫掠净空 = 当前净空，判据确定
  params.max_decel_m_s2 = 100.0;
  params.max_recover_accel_m_s2 = 100.0;
  params.max_angular_decel_rad_s2 = 100.0;
  params.max_angular_recover_rad_s2 = 100.0;
  return params;
}

TEST(AvoidanceControllerTest, TurnClearanceGuardCapsSpeedDuringTurnNearObstacle)
{
  AvoidanceParams params = turnGuardParams();
  AvoidanceController controller(params, MapConstraintConfig{});
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  cmd.angular.z = 0.8;   // |w| ≥ 门控 → 转弯

  ObstacleField field;
  field.obstacles.push_back(sideObstacleAtInflatedClearance(params, 0.35));

  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field, 1.0, 0.8, 1.0, 0.8);

  // v ≤ √(2·0.6·(0.35−0.05)) = 0.6
  ASSERT_TRUE(std::isfinite(result.hybrid_log.turn_guard_speed_cap_m_s));
  EXPECT_NEAR(result.hybrid_log.turn_guard_speed_cap_m_s, 0.6, 0.03);
  EXPECT_LT(result.hybrid_log.turn_guard_speed_cap_m_s, cmd.linear.x);
  EXPECT_LE(result.safe_twist.linear.x, result.hybrid_log.turn_guard_speed_cap_m_s + 1e-6);
}

TEST(AvoidanceControllerTest, TurnClearanceGuardInertWhenDisabled)
{
  AvoidanceParams params = turnGuardParams();
  params.turn_clearance_guard_enabled = false;   // 结构体/默认关闭 → 不改变既有行为
  AvoidanceController controller(params, MapConstraintConfig{});
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  cmd.angular.z = 0.8;

  ObstacleField field;
  field.obstacles.push_back(sideObstacleAtInflatedClearance(params, 0.35));

  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field, 1.0, 0.8, 1.0, 0.8);

  EXPECT_FALSE(std::isfinite(result.hybrid_log.turn_guard_speed_cap_m_s));
}

TEST(AvoidanceControllerTest, TurnClearanceGuardInertOnStraightLine)
{
  AvoidanceParams params = turnGuardParams();
  AvoidanceController controller(params, MapConstraintConfig{});
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  cmd.angular.z = 0.0;   // 直线：|w| < 门控 → 护栏完全不介入

  ObstacleField field;
  field.obstacles.push_back(sideObstacleAtInflatedClearance(params, 0.35));

  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field, 1.0, 0.0, 1.0, 0.0);

  EXPECT_FALSE(std::isfinite(result.hybrid_log.turn_guard_speed_cap_m_s));
}

TEST(AvoidanceControllerTest, RecoveryCandidateStillHonorsMapVeto)
{
  AvoidanceParams params;
  params.recovery_intrusion_confirm_s = 0.0;
  MapConstraintConfig map_config;
  map_config.enabled = true;
  map_config.veto_on_lethal = true;
  AvoidanceController controller(params, map_config);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = obstacleInsidePhysicalBody();
  const RollingLocalMapGrid map = fullyLethalMap();

  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, &map, field);

  EXPECT_FALSE(result.recovery_active);
  EXPECT_TRUE(result.emergency_brake);
  EXPECT_EQ(result.recovery_state, RecoveryState::BLOCKED);
}

TEST(AvoidanceControllerTest, CandidateSpeedUpRequiresStableHold)
{
  AvoidanceParams params;
  params.max_decel_m_s2 = 100.0;
  params.max_recover_accel_m_s2 = 100.0;
  params.candidate_speed_up_hold_s = 0.10;
  AvoidanceController controller(params, MapConstraintConfig{});

  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.2;

  AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST);
  EXPECT_NEAR(result.safe_twist.linear.x, 0.2, 1e-9);
  EXPECT_FALSE(result.emergency_brake);

  cmd.linear.x = 0.8;
  for (int i = 0; i < 3; ++i) {
    result = evaluate(
      controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, {}, result.safe_twist.linear.x);
    EXPECT_NEAR(result.safe_twist.linear.x, 0.2, 1e-9);
    EXPECT_FALSE(result.emergency_brake);
  }

  result = evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, {}, result.safe_twist.linear.x);
  EXPECT_NEAR(result.safe_twist.linear.x, 0.8, 1e-9);
  EXPECT_FALSE(result.emergency_brake);
}

TEST(AvoidanceControllerTest, TiltRadarLimitsForwardSpeed)
{
  AvoidanceParams params = tiltTtcTestParams();
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.7;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField tilt_field = tiltObstacleAhead(0.60);

  const AvoidanceResult result = evaluate(
    controller,
    cmd,
    profile,
    SafetyMode::SAFE_ASSIST,
    nullptr,
    {},
    0.7,
    0.0,
    0.7,
    0.0,
    &tilt_field);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_EQ(result.hybrid_log.tilt_intervention, "limiting");
  EXPECT_EQ(result.hybrid_log.tilt_collision_source, "left_tilt");
  EXPECT_LT(result.safe_twist.linear.x, cmd.linear.x);
  EXPECT_GE(result.safe_twist.linear.x, 0.0);
}

TEST(AvoidanceControllerTest, TiltRadarHardDistanceTriggersTiltEmergency)
{
  AvoidanceParams params = tiltTtcTestParams();
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.2;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField tilt_field = tiltObstacleAhead(0.08);

  const AvoidanceResult result = evaluate(
    controller,
    cmd,
    profile,
    SafetyMode::SAFE_ASSIST,
    nullptr,
    {},
    0.2,
    0.0,
    0.2,
    0.0,
    &tilt_field);

  EXPECT_TRUE(result.emergency_brake);
  EXPECT_EQ(result.emergency_reason, EmergencyStopReason::TILT_NEAR_DISTANCE);
  EXPECT_EQ(result.hybrid_log.tilt_intervention, "emergency");
  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.0);
}

TEST(AvoidanceControllerTest, TiltRadarLateDetectionAtEightKmhTriggersEmergency)
{
  AvoidanceParams params = tiltTtcTestParams();
  params.absolute_max_linear_speed = 2.222;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 2.222;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField tilt_field = tiltObstacleAhead(0.51);

  const AvoidanceResult result = evaluate(
    controller,
    cmd,
    profile,
    SafetyMode::SAFE_ASSIST,
    nullptr,
    {},
    2.222,
    0.0,
    2.222,
    0.0,
    &tilt_field);

  EXPECT_TRUE(result.emergency_brake);
  EXPECT_EQ(result.emergency_reason, EmergencyStopReason::TILT_NEAR_DISTANCE);
  EXPECT_EQ(result.hybrid_log.tilt_intervention, "emergency");
  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.0);
}

TEST(AvoidanceControllerTest, TiltRadarDistanceWindowLimitsBeforeNearestStop)
{
  AvoidanceParams params = tiltTtcTestParams();
  params.absolute_max_linear_speed = 2.222;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 2.222;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField tilt_field = tiltObstacleAhead(0.70);

  const AvoidanceResult result = evaluate(
    controller,
    cmd,
    profile,
    SafetyMode::SAFE_ASSIST,
    nullptr,
    {},
    2.222,
    0.0,
    2.222,
    0.0,
    &tilt_field);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_EQ(result.emergency_reason, EmergencyStopReason::NONE);
  EXPECT_EQ(result.hybrid_log.tilt_intervention, "limiting");
  EXPECT_GT(result.hybrid_log.tilt_min_distance_m, params.tilt_ttc.distance_emergency_m);
  EXPECT_LT(result.safe_twist.linear.x, cmd.linear.x);
  EXPECT_GT(result.safe_twist.linear.x, 0.0);
}

TEST(AvoidanceControllerTest, TiltRadarSmoothLimitsFromOneMeterZone)
{
  AvoidanceParams params = tiltTtcTestParams();
  params.absolute_max_linear_speed = 2.222;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 2.222;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField tilt_field = tiltObstacleAhead(0.80);

  const AvoidanceResult result = evaluate(
    controller,
    cmd,
    profile,
    SafetyMode::SAFE_ASSIST,
    nullptr,
    {},
    0.5,
    0.0,
    0.5,
    0.0,
    &tilt_field);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_EQ(result.hybrid_log.tilt_intervention, "limiting");
  EXPECT_LT(result.safe_twist.linear.x, cmd.linear.x);
  EXPECT_GT(result.safe_twist.linear.x, 0.0);
}

TEST(AvoidanceControllerTest, TiltRadarUsesFrontClearanceForSim016Point)
{
  AvoidanceParams params = productionFootprintParams();
  params.horizontal_avoidance_enabled = false;
  params.braking_envelope_enabled = false;
  params.candidate_speed_up_hold_s = 0.0;
  params.tilt_ttc.enabled = true;
  params.tilt_ttc.horizon_s = 0.5;
  params.tilt_ttc.max_probe_distance_m = 0.7;
  params.tilt_ttc.hard_stop_distance_m = 0.12;
  params.tilt_ttc.limiting_distance_m = 0.7;
  params.tilt_ttc.release_distance_m = 0.75;
  params.tilt_ttc.distance_emergency_m = 0.45;
  params.tilt_ttc.emergency_clear_hold_s = 0.0;
  params.tilt_ttc.obstacle_radius_m = 0.01;
  params.tilt_ttc.limiting_ttc_s = 0.55;
  params.tilt_ttc.emergency_ttc_s = 0.35;
  params.tilt_ttc.emergency_ttc_release_s = 0.50;
  params.tilt_ttc.min_speed_for_ttc_m_s = 0.25;
  params.braking_comfort_decel_m_s2 = 0.60;
  params.braking_max_jerk_m_s3 = 2.5;
  params.braking_reaction_delay_s = 0.10;
  params.absolute_max_linear_speed = 2.222;
  params.max_decel_m_s2 = 100.0;
  params.max_recover_accel_m_s2 = 100.0;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.93;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField tilt_field = tiltObstacleAhead(1.443);
  const double expected_clearance =
    1.443 - 0.01 - VehicleFootprint(params).inflatedRect().x_max;

  const AvoidanceResult result = evaluate(
    controller,
    cmd,
    profile,
    SafetyMode::SAFE_ASSIST,
    nullptr,
    {},
    cmd.linear.x,
    0.0,
    cmd.linear.x,
    0.0,
    &tilt_field);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_EQ(result.emergency_reason, EmergencyStopReason::NONE);
  EXPECT_TRUE(std::isfinite(result.hybrid_log.tilt_speed_limit_m_s));
  EXPECT_EQ(result.hybrid_log.tilt_intervention, "limiting");
  EXPECT_NEAR(result.hybrid_log.tilt_min_distance_m, expected_clearance, 1e-9);
  EXPECT_LT(result.hybrid_log.tilt_min_distance_m, params.tilt_ttc.limiting_distance_m);
  EXPECT_LT(result.safe_twist.linear.x, cmd.linear.x);
  EXPECT_GT(result.safe_twist.linear.x, 0.0);
}

TEST(AvoidanceControllerTest, RateLimitedFinalCommandUsesBrakingDistanceProof)
{
  AvoidanceController controller(AvoidanceParams{}, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = obstacleAhead(1.75);

  const AvoidanceResult result = evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field, 1.0, 0.0, 1.0);

  EXPECT_NEAR(result.safe_twist.linear.x, 0.97, 1e-9);
  EXPECT_FALSE(result.emergency_brake);
  EXPECT_EQ(result.emergency_reason, EmergencyStopReason::NONE);
  EXPECT_TRUE(result.risk.braking_limited);
  EXPECT_TRUE(brakingEnvelopeCanStop(result.risk));
}

TEST(AvoidanceControllerTest, BrakingEnvelopeCapsForwardSpeedFromPathDistance)
{
  AvoidanceParams params;
  params.braking_lookahead_m = 3.0;
  params.braking_path_step_m = 0.025;
  params.braking_comfort_decel_m_s2 = 0.60;
  params.braking_max_jerk_m_s3 = 2.5;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const AvoidanceResult result = evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, obstacleAhead(2.0));

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_TRUE(result.risk.braking_limited);
  EXPECT_NEAR(result.risk.braking_available_distance_m, 1.05, 0.03);
  EXPECT_GT(result.risk.braking_speed_limit_m_s, 0.0);
  EXPECT_LT(result.risk.braking_speed_limit_m_s, cmd.linear.x);
  EXPECT_LT(result.safe_twist.linear.x, cmd.linear.x);
}

TEST(AvoidanceControllerTest, BrakingEnvelopeIgnoresReleasedSideClearanceCap)
{
  AvoidanceParams params;
  params.braking_lookahead_m = 3.0;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 2.0;
  ObstacleField field = obstacleAhead(2.8);
  field.obstacles.push_back(
    sideObstacleAtInflatedClearance(params, params.hard_stop_release_distance_m + 0.05));

  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);

  ASSERT_TRUE(result.risk.braking_limited);
  ASSERT_TRUE(std::isfinite(result.risk.braking_available_distance_m));
  ASSERT_TRUE(std::isfinite(result.risk.braking_effective_remaining_m));
  EXPECT_NEAR(
    result.risk.braking_effective_remaining_m,
    result.risk.braking_available_distance_m,
    1e-9);
  EXPECT_GT(result.risk.braking_speed_limit_m_s, 0.5);
}

TEST(AvoidanceControllerTest, BrakingEnvelopeStillCapsInsideHardStopReleaseBand)
{
  AvoidanceParams params;
  params.braking_lookahead_m = 3.0;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 2.0;
  ObstacleField field = obstacleAhead(2.8);
  field.obstacles.push_back(
    sideObstacleAtInflatedClearance(params, params.hard_stop_distance_m + 0.001));

  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);

  ASSERT_TRUE(result.risk.braking_limited);
  ASSERT_TRUE(std::isfinite(result.risk.braking_available_distance_m));
  EXPECT_TRUE(result.risk.braking_at_min_stop);
  EXPECT_LT(result.risk.braking_effective_remaining_m, 0.005);
  EXPECT_NEAR(result.safe_twist.linear.x, 0.0, 1e-9);
}

TEST(AvoidanceControllerTest, BrakingEnvelopeSupportsReverseMotion)
{
  AvoidanceParams params;
  params.braking_lookahead_m = 3.0;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = obstacleAhead(-1.2);
  const AvoidanceResult result =
    evaluate(controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_TRUE(result.risk.braking_limited);
  EXPECT_LT(result.safe_twist.linear.x, 0.0);
  EXPECT_LT(std::abs(result.risk.braking_speed_limit_m_s), std::abs(cmd.linear.x));
}

TEST(AvoidanceControllerTest, InsufficientPhysicalStoppingDistanceTriggersHardStop)
{
  AvoidanceParams params;
  params.braking_lookahead_m = 3.0;
  params.min_distance_braking_enabled = false;
  params.braking_suppress_distance_emergency = false;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const AvoidanceResult result = evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, obstacleAhead(1.4),
    0.8, 0.0, 0.8, 0.0);

  EXPECT_TRUE(result.emergency_brake);
  EXPECT_EQ(
    result.emergency_reason,
    EmergencyStopReason::BRAKING_DISTANCE_INSUFFICIENT);
  EXPECT_DOUBLE_EQ(result.safe_twist.linear.x, 0.0);
  EXPECT_GT(
    result.risk.braking_stopping_distance_m,
    result.risk.braking_available_distance_m);
}

TEST(AvoidanceControllerTest, DistanceEnvelopeAndJerkShaperStopBeforeHardBoundary)
{
  AvoidanceParams params;
  params.rollout_horizon_s = 0.8;
  params.max_decel_m_s2 = 0.8;
  params.max_recover_accel_m_s2 = 0.5;
  params.braking_lookahead_m = 3.0;
  params.braking_path_step_m = 0.025;
  params.braking_reaction_delay_s = 0.10;
  params.braking_comfort_decel_m_s2 = 0.60;
  params.braking_max_jerk_m_s3 = 2.5;
  AvoidanceController controller(params, MapConstraintConfig{});
  OutputShaper shaper;
  shaper.setEmergencyReleaseDelay(0.0);
  shaper.setMotionLimits(
    params.max_recover_accel_m_s2,
    params.max_decel_m_s2,
    params.braking_max_jerk_m_s3);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.8;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  constexpr double obstacle_world_x = 2.0;
  double vehicle_x = 0.0;
  double velocity = 0.0;
  bool braking_started = false;
  bool stopped = false;

  for (int i = 0; i < 600; ++i) {
    const ObstacleField field = obstacleAhead(obstacle_world_x - vehicle_x);
    const AvoidanceResult result = evaluate(
      controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field,
      velocity, 0.0, velocity, 0.0);
    ASSERT_FALSE(result.emergency_brake)
      << "step=" << i
      << " x=" << vehicle_x
      << " v=" << velocity
      << " available=" << result.risk.braking_available_distance_m
      << " stopping=" << result.risk.braking_stopping_distance_m
      << " reason=" << static_cast<int>(result.emergency_reason);
    braking_started = braking_started || result.risk.braking_limited;
    const auto output = shaper.shape(result.safe_twist, 0.025);
    velocity = output.linear.x;
    vehicle_x += velocity * 0.025;
    if (braking_started && std::abs(velocity) < 0.002) {
      stopped = true;
      break;
    }
  }

  const double hard_boundary_x =
    obstacle_world_x -
    (params.robot_length - params.robot_rear_offset +
    params.footprint_front_margin + 0.08 + params.hard_stop_distance_m);
  EXPECT_TRUE(braking_started);
  EXPECT_TRUE(stopped);
  EXPECT_LE(vehicle_x, hard_boundary_x + params.braking_path_step_m);
  EXPECT_GE(vehicle_x, hard_boundary_x - 0.05);
}

TEST(AvoidanceControllerTest, MinDistanceBrakingSuppressesDistanceEmergencyLatch)
{
  AvoidanceParams params;
  params.braking_lookahead_m = 3.0;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const AvoidanceResult result = evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, obstacleAhead(1.4),
    0.8, 0.0, 0.8, 0.0);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_TRUE(result.risk.braking_limited);
  EXPECT_LT(result.safe_twist.linear.x, 0.8);
}

TEST(AvoidanceControllerTest, MinDistanceBrakingUsesClearanceRemainingForSpeedCap)
{
  AvoidanceParams params;
  params.hard_stop_distance_m = 0.12;
  params.braking_lookahead_m = 3.0;
  params.braking_path_step_m = 0.025;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const AvoidanceResult result = evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, obstacleAhead(1.05),
    0.0, 0.0, 0.0, 0.0);

  EXPECT_FALSE(result.emergency_brake);
  if (result.risk.min_clearance <= params.hard_stop_distance_m + 1e-3) {
    EXPECT_NEAR(result.safe_twist.linear.x, 0.0, 1e-3);
    EXPECT_TRUE(result.risk.braking_at_min_stop || result.hybrid_log.hard_stop_reached_latched);
  }
}

TEST(AvoidanceControllerTest, ForwardStopHoldBlocksRepushBeforeHardStopBoundary)
{
  AvoidanceParams params;
  params.hard_stop_distance_m = 0.12;
  params.braking_lookahead_m = 3.0;
  params.forward_stop_hold_confirm_s = 0.0;
  params.forward_stop_hold_remaining_m = 0.08;
  params.forward_stop_hold_speed_limit_m_s = 0.20;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = obstacleAhead(1.02);

  const AvoidanceResult result = evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field,
    0.0, 0.0, 0.0, 0.0);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_FALSE(result.recovery_active);
  EXPECT_TRUE(result.hybrid_log.forward_stop_hold_active);
  EXPECT_FALSE(result.risk.braking_at_min_stop);
  EXPECT_GT(
    result.risk.braking_effective_remaining_m,
    params.braking_at_stop_distance_eps_m);
  EXPECT_NEAR(result.safe_twist.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(result.safe_twist.angular.z, 0.0, 1e-9);
}

TEST(AvoidanceControllerTest, ForwardStopHoldReleasesForReverseEscape)
{
  AvoidanceParams params;
  params.hard_stop_distance_m = 0.12;
  params.braking_lookahead_m = 3.0;
  params.forward_stop_hold_confirm_s = 0.0;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist forward;
  forward.linear.x = 1.0;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField field = obstacleAhead(1.02);

  const AvoidanceResult held = evaluate(
    controller, forward, profile, SafetyMode::SAFE_ASSIST, nullptr, field,
    0.0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(held.hybrid_log.forward_stop_hold_active);

  geometry_msgs::msg::Twist reverse;
  reverse.linear.x = -0.5;
  const AvoidanceResult released = evaluate(
    controller, reverse, profile, SafetyMode::SAFE_ASSIST, nullptr, field,
    held.safe_twist.linear.x, 0.0, 0.0, 0.0);

  EXPECT_FALSE(released.hybrid_log.forward_stop_hold_active);
  EXPECT_FALSE(released.emergency_brake);
  EXPECT_LT(released.safe_twist.linear.x, 0.0);
}

TEST(AvoidanceControllerTest, MinDistanceBrakingBlocksForwardReaccelAfterHardStop)
{
  AvoidanceParams params;
  params.hard_stop_distance_m = 0.12;
  params.braking_lookahead_m = 3.0;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const double obstacle_x =
    params.robot_length - params.robot_rear_offset +
    params.footprint_front_margin + 0.08 + params.hard_stop_distance_m;
  const ObstacleField field = obstacleAhead(obstacle_x);

  const AvoidanceResult stopped = evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field,
    0.0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(
    stopped.risk.braking_at_min_stop ||
    stopped.hybrid_log.hard_stop_reached_latched);

  const AvoidanceResult retry = evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field,
    0.0, 0.0, 0.0, 0.0);

  EXPECT_FALSE(retry.emergency_brake);
  EXPECT_NEAR(retry.safe_twist.linear.x, 0.0, 1e-3);
  EXPECT_LE(retry.risk.braking_speed_limit_m_s, 1e-3);
}

// 回归：硬停自锁后前向净空恢复为「非有限」（前方彻底畅通，或 direction_mask 启用时
// forward_clearance_m=+inf）必须解除自锁。修复前 release 判据要求 std::isfinite，+inf
// 判假 → 只有倒车能解锁，驾驶员持续前推被永久钉死，表现为前推蠕行/卡顿
// （对应 bags/卡顿/speed_test_tilt_on*）。
TEST(AvoidanceControllerTest, HardStopReleasesWhenForwardPathBecomesClear)
{
  AvoidanceParams params;
  params.hard_stop_distance_m = 0.12;
  params.braking_lookahead_m = 3.0;
  AvoidanceController controller(params, MapConstraintConfig{});

  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  geometry_msgs::msg::Twist forward;
  forward.linear.x = 1.0;

  // 1) 正前方近障触发 hard_stop 自锁（与上一用例同法）。
  const double obstacle_x =
    params.robot_length - params.robot_rear_offset +
    params.footprint_front_margin + 0.08 + params.hard_stop_distance_m;
  const ObstacleField front = obstacleAhead(obstacle_x);
  const AvoidanceResult latched = evaluate(
    controller, forward, profile, SafetyMode::SAFE_ASSIST, nullptr, front,
    0.0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(
    latched.risk.braking_at_min_stop ||
    latched.hybrid_log.hard_stop_reached_latched);

  // 2) 前方彻底清空（空障碍场 → currentInflatedClearance = +inf），driver 持续前推。
  const ObstacleField clear;
  AvoidanceResult result = latched;
  for (int i = 0; i < 100; ++i) {
    result = evaluate(
      controller, forward, profile, SafetyMode::SAFE_ASSIST, nullptr, clear,
      result.safe_twist.linear.x, 0.0, result.safe_twist.linear.x, 0.0);
  }

  EXPECT_FALSE(result.hybrid_log.hard_stop_reached_latched);
  EXPECT_FALSE(result.emergency_brake);
  EXPECT_GT(result.safe_twist.linear.x, 0.1);
}

TEST(AvoidanceControllerTest, MinDistanceBrakingSuppressesDistanceEmergency)
{
  AvoidanceParams params;
  params.braking_lookahead_m = 3.0;
  params.min_distance_braking_enabled = true;
  params.braking_suppress_distance_emergency = true;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const AvoidanceResult result = evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, obstacleAhead(1.4),
    0.8, 0.0, 0.8, 0.0);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_TRUE(result.risk.braking_limited);
  EXPECT_LE(result.safe_twist.linear.x, 0.8);
}

TEST(AvoidanceControllerTest, MinDistanceBrakingZerosWhenClearanceAtHardStop)
{
  AvoidanceParams params;
  params.hard_stop_distance_m = 0.12;
  params.braking_at_stop_distance_eps_m = 0.0125;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const double obstacle_x =
    params.robot_length - params.robot_rear_offset +
    params.footprint_front_margin + 0.08 + params.hard_stop_distance_m;
  const ObstacleField field = obstacleAhead(obstacle_x);

  const AvoidanceResult result = evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field,
    0.0, 0.0, 0.0, 0.0);

  EXPECT_FALSE(result.emergency_brake);
  EXPECT_NEAR(result.safe_twist.linear.x, 0.0, 1e-3);
  EXPECT_TRUE(result.risk.braking_at_min_stop);
}

// ── P0-B 停稳看护 ──
// 生产双梯形物理车头 x≈0.94，障碍半径 0.08 → 物理净空 ≈ obstacle_x − 1.02。

TEST(AvoidanceControllerTest, StandstillGuardLatchesWhenStoppedObstacleBreachesFloor)
{
  AvoidanceParams params = productionFootprintParams();
  params.standstill_guard_enabled = true;
  params.standstill_guard_band_m = 0.12;
  params.standstill_guard_floor_m = 0.03;
  params.standstill_guard_approach_rate_m_s = 0.05;
  params.standstill_guard_confirm_s = 0.10;
  params.standstill_guard_release_m = 0.15;
  AvoidanceController controller(params, MapConstraintConfig{});

  geometry_msgs::msg::Twist cmd;   // 停稳、无前进/后退命令
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();

  // 第 1 帧净空 ~0.05（> floor），仅建立净空历史，不锁存。
  const AvoidanceResult first = evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr,
    obstacleAhead(1.07), 0.0, 0.0, 0.0, 0.0);
  EXPECT_NE(first.emergency_reason, EmergencyStopReason::STANDSTILL_GUARD);

  // 第 2 帧净空 ~0.01（刚跌破 floor 0.03）→ 动态切入，立即锁存急停保护车体。
  const AvoidanceResult second = evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr,
    obstacleAhead(1.03), 0.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(second.emergency_brake);
  EXPECT_EQ(second.emergency_reason, EmergencyStopReason::STANDSTILL_GUARD);
  EXPECT_TRUE(second.hybrid_log.standstill_guard_active);
  EXPECT_DOUBLE_EQ(second.safe_twist.linear.x, 0.0);
}

TEST(AvoidanceControllerTest, StandstillGuardYieldsToReverseIntent)
{
  AvoidanceParams params = productionFootprintParams();
  AvoidanceController controller(params, MapConstraintConfig{});
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();

  geometry_msgs::msg::Twist stopped;
  evaluate(
    controller, stopped, profile, SafetyMode::SAFE_ASSIST, nullptr,
    obstacleAhead(1.07), 0.0, 0.0, 0.0, 0.0);   // 建立净空历史

  // 同样的逼近，但驾驶员命令倒车 → 看护让位，不由停稳看护接管（供退让/脱困）。
  geometry_msgs::msg::Twist reverse;
  reverse.linear.x = -0.3;
  const AvoidanceResult result = evaluate(
    controller, reverse, profile, SafetyMode::SAFE_ASSIST, nullptr,
    obstacleAhead(1.03), 0.0, 0.0, 0.0, 0.0);
  EXPECT_NE(result.emergency_reason, EmergencyStopReason::STANDSTILL_GUARD);
  EXPECT_FALSE(result.hybrid_log.standstill_guard_active);
}

TEST(AvoidanceControllerTest, StandstillGuardIgnoresStaticObstacle)
{
  AvoidanceParams params = productionFootprintParams();
  AvoidanceController controller(params, MapConstraintConfig{});
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();

  // 净空恒定 ~0.05 的静止障碍（贴障但不移动）：净空不缩小、未跌破 floor → 不触发看护。
  geometry_msgs::msg::Twist cmd;
  AvoidanceResult result;
  for (int i = 0; i < 10; ++i) {
    result = evaluate(
      controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr,
      obstacleAhead(1.07), 0.0, 0.0, 0.0, 0.0);
  }
  EXPECT_NE(result.emergency_reason, EmergencyStopReason::STANDSTILL_GUARD);
  EXPECT_FALSE(result.hybrid_log.standstill_guard_active);
}

// 复现 bag「由摇杆下发的速度过大，侧边距离还有很多但车辆动作也抑制」：
// 小线速度 + 大角速度 = 极小转弯半径（v/w≈0.13 m）。旧 buildDistanceProbe 沿固定 5 m 弧长
// 积分会原地打转数圈，把车后障碍误当作前向 ~0.025 m 即时硬停，speed_limit=0 且转向被同比清零，
// 整车冻结。探针航向截断（braking_probe_max_heading_rad）后前向走廊空旷，车辆不再被冻结。
TEST(AvoidanceControllerTest, TightTurnNotFrozenByRearObstacle)
{
  AvoidanceParams params = productionFootprintParams();
  params.direction_mask_enabled = true;      // 生产由 YAML 开启
  params.shared_control.enabled = true;       // 启用转向保真
  params.shared_control.preserve_steering = true;
  AvoidanceController controller(params, MapConstraintConfig{});
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();

  // 最近障碍在车后，前方完全空旷（对应点云真值 nearest≈(-0.565,-0.254)）。
  ObstacleField field;
  ObstaclePrimitive rear;
  rear.x = -0.565;
  rear.y = -0.254;
  rear.radius = 0.02;
  rear.valid = true;
  field.obstacles.push_back(rear);

  // 小前进 + 大左转（转弯半径 ~0.13 m）。prev/fb 取命令值，隔离加减速斜率的影响。
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.145;
  cmd.angular.z = 1.136;
  const AvoidanceResult result = evaluate(
    controller, cmd, profile, SafetyMode::SAFE_ASSIST, nullptr, field,
    0.145, 1.136, 0.145, 1.136);

  // 冻结病态已消除：不再有前向保持锁存/急停，线角至少一轴保有明显运动。
  EXPECT_FALSE(result.emergency_brake);
  EXPECT_FALSE(result.hybrid_log.forward_stop_hold_active);
  EXPECT_GT(
    std::abs(result.safe_twist.linear.x) + std::abs(result.safe_twist.angular.z),
    0.1);
  EXPECT_GT(result.safe_twist.linear.x, 0.0);   // 前向走廊空旷，不被车后障碍钉死
}
