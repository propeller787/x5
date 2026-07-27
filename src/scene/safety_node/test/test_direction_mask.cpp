/**
 * @file test_direction_mask.cpp
 * @brief L1 方向解耦掩码测试
 *
 * 覆盖：
 *  1. 符号净空与威胁归因谓词（含旋转扫掠判别用例：同侧障碍因前后位置不同
 *     而封锁相反转向——扇区规则无法正确处理，探针必须处理）
 *  2. SweptFootprintChecker 近障方向豁免（开/关对照）
 *  3. 控制器级表驱动矩阵：2 区域（物理车体内 / 物理外虚拟车体内）×
 *     4 障碍方位 × 4 单向操作
 *  4. 组合指令分量裁剪、急停/看护锁存后切换非受阻方向释放
 *
 * 默认车体几何（AvoidanceParams 默认值）：
 *   物理框 x∈[−0.22, 0.66]，y∈[−0.35, 0.35]
 *   膨胀框 x∈[−0.34, 0.76]，y∈[−0.40, 0.40]，hard_stop 0.12
 */
#include <gtest/gtest.h>

#include "safety_node/avoidance_controller.h"
#include "safety_node/core/direction_mask.h"
#include "safety_node/core/motion_rollout.h"
#include "safety_node/core/swept_footprint_checker.h"
#include "safety_node/core/vehicle_footprint.h"
#include "safety_node/mode_supervisor.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace
{

constexpr double kDt = 0.025;

ObstaclePrimitive makeObstacle(double x, double y, double radius = 0.01)
{
  ObstaclePrimitive obstacle;
  obstacle.x = x;
  obstacle.y = y;
  obstacle.radius = radius;
  obstacle.valid = true;
  return obstacle;
}

ObstacleField singleObstacleField(double x, double y, double radius = 0.01)
{
  ObstacleField field;
  field.obstacles.push_back(makeObstacle(x, y, radius));
  return field;
}

geometry_msgs::msg::Twist makeCmd(double v, double w)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = v;
  cmd.angular.z = w;
  return cmd;
}

/** 以固定 dt 连续评估 n 周期（反馈恒 0，prev 用上周期输出），返回末周期结果 */
AvoidanceResult runCycles(
  AvoidanceController & controller,
  const geometry_msgs::msg::Twist & cmd,
  const ObstacleField & field,
  int cycles,
  double * prev_linear_io = nullptr,
  double * prev_angular_io = nullptr)
{
  ModeProfile profile;
  AvoidanceContext context;
  context.safety_mode = SafetyMode::SAFE_ASSIST;
  double prev_linear = prev_linear_io != nullptr ? *prev_linear_io : 0.0;
  double prev_angular = prev_angular_io != nullptr ? *prev_angular_io : 0.0;
  AvoidanceResult result;
  for (int i = 0; i < cycles; ++i) {
    result = controller.evaluate(
      cmd, 0.0, 0.0, field, nullptr, profile,
      prev_linear, prev_angular, kDt, context, nullptr);
    prev_linear = result.safe_twist.linear.x;
    prev_angular = result.safe_twist.angular.z;
  }
  if (prev_linear_io != nullptr) {
    *prev_linear_io = prev_linear;
  }
  if (prev_angular_io != nullptr) {
    *prev_angular_io = prev_angular;
  }
  return result;
}

}  // namespace

// ────────────────────────────────────────────────────────────────────────────
// 1. 符号净空与威胁归因谓词
// ────────────────────────────────────────────────────────────────────────────

TEST(DirectionMaskPredicateTest, SignedClearanceMonotoneAcrossBoundary)
{
  AvoidanceParams params;
  VehicleFootprint footprint(params);
  const FootprintPolygon physical = footprint.physicalPolygon();

  // 前缘 x=0.66：外 0.10 → +0.10；贴边 → 0；内 0.06 → −0.06（单调跨界）
  EXPECT_NEAR(
    VehicleFootprint::signedClearanceToPolygon(0.76, 0.0, 0.0, physical), 0.10, 1e-9);
  EXPECT_NEAR(
    VehicleFootprint::signedClearanceToPolygon(0.66, 0.0, 0.0, physical), 0.0, 1e-9);
  EXPECT_NEAR(
    VehicleFootprint::signedClearanceToPolygon(0.60, 0.0, 0.0, physical), -0.06, 1e-9);
}

TEST(DirectionMaskPredicateTest, NearObstacleEligibility)
{
  AvoidanceParams params;
  VehicleFootprint footprint(params);
  const FootprintPolygon inflated = footprint.inflatedPolygon();

  // 膨胀体内（虚拟车体内）与硬停带内均为近障；远处不是
  EXPECT_TRUE(
    direction_mask::isNearObstacle(
      makeObstacle(0.70, 0.0), inflated, params.hard_stop_distance_m));
  EXPECT_TRUE(
    direction_mask::isNearObstacle(
      makeObstacle(0.85, 0.0), inflated, params.hard_stop_distance_m));
  EXPECT_FALSE(
    direction_mask::isNearObstacle(
      makeObstacle(2.0, 0.0), inflated, params.hard_stop_distance_m));
}

TEST(DirectionMaskPredicateTest, LinearProbeThreatAttribution)
{
  AvoidanceParams params;
  VehicleFootprint footprint(params);
  const FootprintPolygon physical = footprint.physicalPolygon();
  const auto forward = direction_mask::linearProbePoses(
    1.0, params.direction_mask_probe_distance_m, params.direction_mask_probe_step_m);
  const auto backward = direction_mask::linearProbePoses(
    -1.0, params.direction_mask_probe_distance_m, params.direction_mask_probe_step_m);
  const auto threatened = [&](const ObstaclePrimitive & obs,
      const std::vector<PredictedPose> & probe) {
      return direction_mask::obstacleThreatenedAlongPoses(
        obs, probe, physical,
        params.direction_mask_contact_buffer_m,
        params.direction_mask_approach_eps_m);
    };

  const ObstaclePrimitive front = makeObstacle(0.70, 0.0);
  EXPECT_TRUE(threatened(front, forward));
  EXPECT_FALSE(threatened(front, backward));

  const ObstaclePrimitive rear = makeObstacle(-0.30, 0.0);
  EXPECT_FALSE(threatened(rear, forward));
  EXPECT_TRUE(threatened(rear, backward));

  // 侧向障碍与平移平行掠过：净空恒定，前后均不威胁
  const ObstaclePrimitive side = makeObstacle(0.20, 0.39);
  EXPECT_FALSE(threatened(side, forward));
  EXPECT_FALSE(threatened(side, backward));
}

TEST(DirectionMaskPredicateTest, RotationSwingAttributionBeatsSectorLogic)
{
  AvoidanceParams params;
  VehicleFootprint footprint(params);
  const FootprintPolygon physical = footprint.physicalPolygon();
  const auto left_probe = direction_mask::angularProbePoses(
    1.0, params.direction_mask_probe_angle_rad, params.direction_mask_probe_step_rad);
  const auto right_probe = direction_mask::angularProbePoses(
    -1.0, params.direction_mask_probe_angle_rad, params.direction_mask_probe_step_rad);
  const auto threatened = [&](const ObstaclePrimitive & obs,
      const std::vector<PredictedPose> & probe) {
      return direction_mask::obstacleThreatenedAlongPoses(
        obs, probe, physical,
        params.direction_mask_contact_buffer_m,
        params.direction_mask_approach_eps_m);
    };

  // 同为「左侧」障碍，前后位置不同 → 封锁相反的转向轴（扇区规则的判别用例）：
  // 左前侧腹：左转（前半向左扫）威胁，右转远离
  const ObstaclePrimitive left_front = makeObstacle(0.55, 0.39);
  EXPECT_TRUE(threatened(left_front, left_probe));
  EXPECT_FALSE(threatened(left_front, right_probe));

  // 左后侧腹：右转（后半向左扫）威胁，左转远离
  const ObstaclePrimitive left_rear = makeObstacle(-0.10, 0.39);
  EXPECT_FALSE(threatened(left_rear, left_probe));
  EXPECT_TRUE(threatened(left_rear, right_probe));

  // 右侧镜像
  const ObstaclePrimitive right_front = makeObstacle(0.55, -0.39);
  EXPECT_TRUE(threatened(right_front, right_probe));
  EXPECT_FALSE(threatened(right_front, left_probe));

  const ObstaclePrimitive right_rear = makeObstacle(-0.10, -0.39);
  EXPECT_FALSE(threatened(right_rear, right_probe));
  EXPECT_TRUE(threatened(right_rear, left_probe));
}

TEST(DirectionMaskPredicateTest, ComputeMaskPerAxis)
{
  AvoidanceParams params;
  params.direction_mask_enabled = true;
  VehicleFootprint footprint(params);
  const FootprintPolygon physical = footprint.physicalPolygon();
  const FootprintPolygon inflated = footprint.inflatedPolygon();
  const auto mask_of = [&](const ObstacleField & field) {
      return direction_mask::computeDirectionBlockMask(field, physical, inflated, params);
    };

  const DirectionBlockMask front = mask_of(singleObstacleField(0.70, 0.0));
  EXPECT_TRUE(front.forward);
  EXPECT_FALSE(front.backward);
  EXPECT_TRUE(std::isfinite(front.forward_clearance_m));

  const DirectionBlockMask rear = mask_of(singleObstacleField(-0.30, 0.0));
  EXPECT_TRUE(rear.backward);
  EXPECT_FALSE(rear.forward);
  // 后中障碍距物理后缘 0.08 m：0.3 rad 内的角扫掠不足以触及 → 左右均放行
  EXPECT_FALSE(rear.left);
  EXPECT_FALSE(rear.right);

  const DirectionBlockMask left_front = mask_of(singleObstacleField(0.55, 0.39));
  EXPECT_TRUE(left_front.left);
  EXPECT_FALSE(left_front.right);
  EXPECT_FALSE(left_front.forward);
  EXPECT_FALSE(left_front.backward);

  const DirectionBlockMask left_rear = mask_of(singleObstacleField(-0.10, 0.39));
  EXPECT_TRUE(left_rear.right);
  EXPECT_FALSE(left_rear.left);

  // 远障不参与掩码
  const DirectionBlockMask far_field = mask_of(singleObstacleField(2.0, 0.0));
  EXPECT_FALSE(far_field.any());

  // 总开关关闭 → 恒空掩码
  AvoidanceParams disabled = params;
  disabled.direction_mask_enabled = false;
  const DirectionBlockMask off = direction_mask::computeDirectionBlockMask(
    singleObstacleField(0.70, 0.0), physical, inflated, disabled);
  EXPECT_FALSE(off.any());
}

// ────────────────────────────────────────────────────────────────────────────
// 2. SweptFootprintChecker 近障方向豁免
// ────────────────────────────────────────────────────────────────────────────

TEST(DirectionMaskCheckerTest, ReverseTrajectoryExemptsFrontNearObstacle)
{
  AvoidanceParams params;
  params.direction_mask_enabled = true;
  VehicleFootprint footprint(params);
  SweptFootprintChecker checker(params, footprint);
  MotionRollout rollout(params);
  const ObstacleField field = singleObstacleField(0.70, 0.0);  // 虚拟车体内前障

  VelocityCandidate reverse;
  reverse.v = -0.3;
  const CollisionCheckResult exempt =
    checker.checkTrajectory(rollout.rolloutCandidate(reverse), field);
  EXPECT_FALSE(exempt.collision);
  EXPECT_GT(exempt.min_distance, params.hard_stop_distance_m);
  EXPECT_FALSE(std::isfinite(exempt.hard_stop_path_distance_m));

  // 旧行为对照：关闭后同一倒车轨迹在 t=0 即碰撞（障碍在膨胀体内）
  AvoidanceParams legacy = params;
  legacy.direction_mask_enabled = false;
  VehicleFootprint legacy_footprint(legacy);
  SweptFootprintChecker legacy_checker(legacy, legacy_footprint);
  const CollisionCheckResult blocked =
    legacy_checker.checkTrajectory(rollout.rolloutCandidate(reverse), field);
  EXPECT_TRUE(blocked.collision);
  EXPECT_NEAR(blocked.ttc, 0.0, 1e-9);
}

TEST(DirectionMaskCheckerTest, ApproachingTrajectoryStillCollides)
{
  AvoidanceParams params;
  params.direction_mask_enabled = true;
  VehicleFootprint footprint(params);
  SweptFootprintChecker checker(params, footprint);
  MotionRollout rollout(params);
  const ObstacleField field = singleObstacleField(0.70, 0.0);

  VelocityCandidate forward;
  forward.v = 0.3;
  const CollisionCheckResult check =
    checker.checkTrajectory(rollout.rolloutCandidate(forward), field);
  EXPECT_TRUE(check.collision);  // 朝物理接触推进的近障不豁免
}

TEST(DirectionMaskCheckerTest, SideNearObstacleExemptForForwardTrajectory)
{
  AvoidanceParams params;
  params.direction_mask_enabled = true;
  VehicleFootprint footprint(params);
  SweptFootprintChecker checker(params, footprint);
  MotionRollout rollout(params);
  const ObstacleField field = singleObstacleField(0.20, 0.39);  // 左侧腹近障

  VelocityCandidate forward;
  forward.v = 0.3;
  const CollisionCheckResult check =
    checker.checkTrajectory(rollout.rolloutCandidate(forward), field);
  EXPECT_FALSE(check.collision);
  EXPECT_GT(check.min_distance, params.hard_stop_distance_m);
  EXPECT_FALSE(std::isfinite(check.hard_stop_path_distance_m));
}

// ────────────────────────────────────────────────────────────────────────────
// 3. 控制器级表驱动矩阵
// ────────────────────────────────────────────────────────────────────────────

namespace
{

enum class Op { kForward, kBackward, kLeft, kRight };
enum class Expect { kAllowed, kBlocked, kSkip };

struct MatrixCase
{
  const char * name;
  double ox;
  double oy;
  Expect forward;
  Expect backward;
  Expect left;
  Expect right;
};

geometry_msgs::msg::Twist opCommand(Op op)
{
  switch (op) {
    case Op::kForward: return makeCmd(0.5, 0.0);
    case Op::kBackward: return makeCmd(-0.5, 0.0);
    case Op::kLeft: return makeCmd(0.0, 0.8);
    default: return makeCmd(0.0, -0.8);
  }
}

void checkOp(const MatrixCase & row, Op op, Expect expect)
{
  if (expect == Expect::kSkip) {
    return;
  }
  AvoidanceParams params;
  params.direction_mask_enabled = true;
  MapConstraintConfig map_config;
  AvoidanceController controller(params, map_config);
  const ObstacleField field = singleObstacleField(row.ox, row.oy);
  const AvoidanceResult result =
    runCycles(controller, opCommand(op), field, 16);
  const double v = result.safe_twist.linear.x;
  const double w = result.safe_twist.angular.z;
  const std::string label =
    std::string(row.name) + " op=" + std::to_string(static_cast<int>(op));

  switch (op) {
    case Op::kForward:
      if (expect == Expect::kAllowed) {
        EXPECT_GT(v, 0.005) << label;
      } else {
        EXPECT_LE(v, 1e-6) << label;
      }
      break;
    case Op::kBackward:
      if (expect == Expect::kAllowed) {
        EXPECT_LT(v, -0.005) << label;
      } else {
        EXPECT_GE(v, -1e-6) << label;
      }
      break;
    case Op::kLeft:
      if (expect == Expect::kAllowed) {
        EXPECT_GT(w, 0.005) << label;
      } else {
        EXPECT_LE(w, 1e-6) << label;
      }
      break;
    case Op::kRight:
      if (expect == Expect::kAllowed) {
        EXPECT_LT(w, -0.005) << label;
      } else {
        EXPECT_GE(w, -1e-6) << label;
      }
      break;
  }
}

void runMatrix(const std::vector<MatrixCase> & rows)
{
  for (const auto & row : rows) {
    checkOp(row, Op::kForward, row.forward);
    checkOp(row, Op::kBackward, row.backward);
    checkOp(row, Op::kLeft, row.left);
    checkOp(row, Op::kRight, row.right);
  }
}

}  // namespace

TEST(DirectionMaskControllerMatrixTest, VirtualBodyZone)
{
  // 障碍位于物理车体外、膨胀虚拟车体内。
  // 说明：前中/后中障碍的旋转判定按真实扫掠几何——前中 0.03 m 净空时原地转向
  // 会真实触碰（前缘角扫掠），诚实封锁；后中 0.08 m 净空扫不到，放行。
  runMatrix(
  {
    //  name              ox     oy     F                  B                  L                  R
    {"virtual-front", 0.70, 0.0,
      Expect::kBlocked, Expect::kAllowed, Expect::kBlocked, Expect::kBlocked},
    {"virtual-rear", -0.30, 0.0,
      Expect::kAllowed, Expect::kBlocked, Expect::kAllowed, Expect::kAllowed},
    {"virtual-left-front", 0.55, 0.39,
      Expect::kAllowed, Expect::kAllowed, Expect::kBlocked, Expect::kAllowed},
    {"virtual-right-front", 0.55, -0.39,
      Expect::kAllowed, Expect::kAllowed, Expect::kAllowed, Expect::kBlocked},
    // 旋转扫掠判别行：左后侧腹障碍封右转、放左转（右后镜像）
    {"virtual-left-rear", -0.10, 0.39,
      Expect::kAllowed, Expect::kAllowed, Expect::kAllowed, Expect::kBlocked},
    {"virtual-right-rear", -0.10, -0.39,
      Expect::kAllowed, Expect::kAllowed, Expect::kBlocked, Expect::kAllowed},
  });
}

TEST(DirectionMaskControllerMatrixTest, PhysicalBodyZone)
{
  // 障碍浅侵入物理车体（脱困 recovery 生效区）。
  // 只断言与障碍方位强相关的轴：受威胁方向不得产生正向运动，
  // 逃逸方向须在确认期（0.10 s）后跟随驾驶员指令输出。
  // 其余轴由 recovery 自主选择（可能输出其他方向的自动脱困），不做断言。
  runMatrix(
  {
    {"physical-front", 0.655, 0.0,
      Expect::kBlocked, Expect::kAllowed, Expect::kSkip, Expect::kSkip},
    {"physical-rear", -0.215, 0.0,
      Expect::kAllowed, Expect::kBlocked, Expect::kSkip, Expect::kSkip},
    {"physical-left", 0.20, 0.345,
      Expect::kSkip, Expect::kSkip, Expect::kBlocked, Expect::kAllowed},
    {"physical-right", 0.20, -0.345,
      Expect::kSkip, Expect::kSkip, Expect::kAllowed, Expect::kBlocked},
  });
}

// ────────────────────────────────────────────────────────────────────────────
// 4. 组合指令与锁存释放
// ────────────────────────────────────────────────────────────────────────────

TEST(DirectionMaskCombinedCommandTest, LeftObstacleClipsOnlySteering)
{
  // 左前侧腹近障 + 前进左转：只清左转，保留前进
  AvoidanceParams params;
  params.direction_mask_enabled = true;
  MapConstraintConfig map_config;
  AvoidanceController controller(params, map_config);
  const ObstacleField field = singleObstacleField(0.55, 0.39);
  const AvoidanceResult result =
    runCycles(controller, makeCmd(0.5, 0.8), field, 16);
  EXPECT_GT(result.safe_twist.linear.x, 0.005);
  EXPECT_LE(result.safe_twist.angular.z, 1e-6);
  EXPECT_TRUE(result.hybrid_log.direction_mask_left);
  EXPECT_FALSE(result.hybrid_log.direction_mask_forward);
}

TEST(DirectionMaskCombinedCommandTest, RearObstacleClipsOnlyReverse)
{
  // 后中近障 + 后退右转：只清后退，保留右转
  AvoidanceParams params;
  params.direction_mask_enabled = true;
  MapConstraintConfig map_config;
  AvoidanceController controller(params, map_config);
  const ObstacleField field = singleObstacleField(-0.30, 0.0);
  const AvoidanceResult result =
    runCycles(controller, makeCmd(-0.5, -0.8), field, 16);
  EXPECT_GE(result.safe_twist.linear.x, -1e-6);
  EXPECT_LT(result.safe_twist.angular.z, -0.005);
  EXPECT_TRUE(result.hybrid_log.direction_mask_backward);
  EXPECT_FALSE(result.hybrid_log.direction_mask_right);
}

TEST(DirectionMaskCombinedCommandTest, FrontCenterBlocksBothAxes)
{
  // 前中近障（0.03 m 物理净空）+ 前进左转：前进与旋转均真实冲突 → 全零
  AvoidanceParams params;
  params.direction_mask_enabled = true;
  MapConstraintConfig map_config;
  AvoidanceController controller(params, map_config);
  const ObstacleField field = singleObstacleField(0.70, 0.0);
  const AvoidanceResult result =
    runCycles(controller, makeCmd(0.5, 0.8), field, 16);
  EXPECT_LE(std::abs(result.safe_twist.linear.x), 1e-6);
  EXPECT_LE(std::abs(result.safe_twist.angular.z), 1e-6);
}

TEST(DirectionMaskLatchTest, SwitchToUnblockedDirectionReleases)
{
  // 前障封前进（锁存急停/看护）→ 切换倒车 → 定向释放后倒车恢复
  AvoidanceParams params;
  params.direction_mask_enabled = true;
  MapConstraintConfig map_config;
  AvoidanceController controller(params, map_config);
  const ObstacleField field = singleObstacleField(0.70, 0.0);

  double prev_linear = 0.0;
  double prev_angular = 0.0;
  const AvoidanceResult held =
    runCycles(controller, makeCmd(0.5, 0.0), field, 8, &prev_linear, &prev_angular);
  EXPECT_LE(held.safe_twist.linear.x, 1e-6);

  const AvoidanceResult released =
    runCycles(controller, makeCmd(-0.5, 0.0), field, 20, &prev_linear, &prev_angular);
  EXPECT_LT(released.safe_twist.linear.x, -0.005);
}

TEST(DirectionMaskStandstillGuardTest, GuardLatchesThenReleasesForSafeRotation)
{
  // 左侧障碍逼近停稳车辆 → 看护锁存零速；驾驶员右转（未受威胁轴）→ 释放并转动。
  // 旧行为仅倒车可释放，右转会被全向看护误锁。
  AvoidanceParams params;
  params.direction_mask_enabled = true;
  MapConstraintConfig map_config;
  AvoidanceController controller(params, map_config);

  double prev_linear = 0.0;
  double prev_angular = 0.0;
  AvoidanceResult result;
  double y = 0.46;
  for (int i = 0; i < 14; ++i) {
    y -= 0.004;  // 0.16 m/s 逼近速率，高于 approach_rate 阈值
    result = runCycles(
      controller, makeCmd(0.0, 0.0), singleObstacleField(0.20, y), 1,
      &prev_linear, &prev_angular);
  }
  EXPECT_TRUE(result.hybrid_log.standstill_guard_active);
  EXPECT_LE(std::abs(result.safe_twist.linear.x), 1e-6);
  EXPECT_LE(std::abs(result.safe_twist.angular.z), 1e-6);

  const AvoidanceResult released = runCycles(
    controller, makeCmd(0.0, -0.8), singleObstacleField(0.20, y), 16,
    &prev_linear, &prev_angular);
  EXPECT_FALSE(released.hybrid_log.standstill_guard_active);
  EXPECT_LT(released.safe_twist.angular.z, -0.005);
}
