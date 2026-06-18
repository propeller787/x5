/**
 * @file test_avoidance_controller.cpp
 */
#include <gtest/gtest.h>

#include "safety_node/avoidance_controller.h"
#include "safety_node/output_shaper.h"

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
  params.tilt_ttc.limiting_distance_m = 0.35;
  params.tilt_ttc.release_distance_m = 0.40;
  params.tilt_ttc.limiting_ttc_s = 0.55;
  params.tilt_ttc.emergency_ttc_s = 0.35;
  params.tilt_ttc.emergency_ttc_release_s = 0.50;
  params.tilt_ttc.emergency_clear_hold_s = 0.0;
  params.tilt_ttc.min_speed_for_ttc_m_s = 0.25;
  params.tilt_ttc.obstacle_radius_m = 0.01;
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
  cmd.linear.x = 0.5;
  const ModeProfile profile = ModeSupervisor(SafetyMode::SAFE_ASSIST).profile();
  const ObstacleField tilt_field = tiltObstacleAhead(0.30);

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
