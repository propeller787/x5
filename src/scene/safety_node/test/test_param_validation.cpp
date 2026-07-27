/**
 * @file test_param_validation.cpp
 * @brief 方案 9 参数校验器单测：每条 FATAL 规则一个反例 + 默认配置通过
 */
#include <gtest/gtest.h>

#include "safety_node/safety_params.h"

#include <algorithm>
#include <string>
#include <vector>

namespace
{

bool hasIssue(
  const std::vector<ParamIssue> & issues,
  ParamIssue::Level level,
  const std::string & keyword)
{
  return std::any_of(
    issues.begin(), issues.end(),
    [&](const ParamIssue & issue) {
      return issue.level == level &&
             issue.message.find(keyword) != std::string::npos;
    });
}

bool hasAnyFatal(const std::vector<ParamIssue> & issues)
{
  return std::any_of(
    issues.begin(), issues.end(),
    [](const ParamIssue & issue) {return issue.level == ParamIssue::FATAL;});
}

}  // namespace

TEST(ParamValidation, DefaultParamsPass)
{
  SafetyNodeParams params;
  const auto issues = validateParams(params);
  EXPECT_FALSE(hasAnyFatal(issues));
}

TEST(ParamValidation, SchemaVersionMismatchIsFatal)
{
  SafetyNodeParams params;
  params.config_schema_version =
    SafetyNodeParams::kSupportedConfigSchemaVersion + 1;
  EXPECT_TRUE(
    hasIssue(validateParams(params), ParamIssue::FATAL, "config_schema_version"));
}

TEST(ParamValidation, HardStopReleaseBelowHardStopIsFatal)
{
  SafetyNodeParams params;
  params.avoidance.hard_stop_distance_m = 0.05;
  params.avoidance.hard_stop_release_distance_m = 0.04;
  EXPECT_TRUE(
    hasIssue(
      validateParams(params), ParamIssue::FATAL, "hard_stop_release_distance_m"));
}

TEST(ParamValidation, NarrowHardStopReleaseBelowHardStopIsFatal)
{
  SafetyNodeParams params;
  params.narrow_passage.hard_stop_distance_m = 0.03;
  params.narrow_passage.hard_stop_release_distance_m = 0.02;
  EXPECT_TRUE(
    hasIssue(validateParams(params), ParamIssue::FATAL, "mode.narrow"));
}

TEST(ParamValidation, BrakingLookaheadTooShortIsFatal)
{
  SafetyNodeParams params;
  // 最高速 2.222 m/s 的完整停车距离约 4.6 m，0.5 m 前视必然不足
  params.avoidance.braking_lookahead_m = 0.5;
  EXPECT_TRUE(
    hasIssue(validateParams(params), ParamIssue::FATAL, "braking_lookahead_m"));
}

TEST(ParamValidation, DirectionMaskProbeShorterThanGuardBandIsFatal)
{
  SafetyNodeParams params;
  params.avoidance.direction_mask_enabled = true;
  params.avoidance.direction_mask_probe_distance_m = 0.05;
  params.avoidance.standstill_guard_band_m = 0.12;
  EXPECT_TRUE(
    hasIssue(validateParams(params), ParamIssue::FATAL, "probe_distance_m"));
}

TEST(ParamValidation, MapExemptRadiusTooSmallIsFatal)
{
  SafetyNodeParams params;
  params.avoidance.direction_mask_enabled = true;
  params.local_map.inflation_radius_m = 0.20;
  params.local_map.resolution_m = 0.05;
  params.avoidance.direction_mask_map_exempt_radius_m = 0.22;   // < 0.25
  EXPECT_TRUE(
    hasIssue(validateParams(params), ParamIssue::FATAL, "map_exempt_radius_m"));
}

TEST(ParamValidation, SweptCullBandBelowDecisionThresholdIsFatal)
{
  SafetyNodeParams params;
  params.avoidance.turn_clearance_guard_enabled = true;
  params.avoidance.turn_clearance_guard_band_m = 0.6;
  params.avoidance.swept_cull_band_m = 0.4;   // < 0.6 会改变转弯护栏决策
  EXPECT_TRUE(
    hasIssue(validateParams(params), ParamIssue::FATAL, "swept_cull_band_m"));
}

TEST(ParamValidation, SweptCullDisabledSkipsCheck)
{
  SafetyNodeParams params;
  params.avoidance.turn_clearance_guard_enabled = true;
  params.avoidance.swept_cull_band_m = 0.0;   // 关闭预裁剪 → 不校验带宽
  EXPECT_FALSE(hasAnyFatal(validateParams(params)));
}

TEST(ParamValidation, DualTrapezoidWidthMismatchIsFatal)
{
  SafetyNodeParams params;
  params.avoidance.footprint_use_dual_trapezoid = true;
  params.avoidance.footprint_front_segment.width_rear_m = 0.62;
  params.avoidance.footprint_rear_segment.width_front_m = 0.60;
  EXPECT_TRUE(
    hasIssue(validateParams(params), ParamIssue::FATAL, "双梯形"));
}

TEST(ParamValidation, InflationBelowResolutionIsWarn)
{
  SafetyNodeParams params;
  params.local_map.inflation_radius_m = 0.01;
  params.local_map.resolution_m = 0.05;
  const auto issues = validateParams(params);
  EXPECT_TRUE(hasIssue(issues, ParamIssue::WARN, "inflation_radius_m"));
  EXPECT_FALSE(hasAnyFatal(issues));
}

TEST(ParamValidation, MemoryWithoutEgoCompensationIsWarn)
{
  SafetyNodeParams params;
  params.local_map.rebuild_each_frame = false;
  params.local_map.ego_motion_compensation = false;
  EXPECT_TRUE(
    hasIssue(validateParams(params), ParamIssue::WARN, "rebuild_each_frame"));
}

TEST(ParamValidation, ShadowModeIsWarnNotFatal)
{
  SafetyNodeParams params;
  params.avoidance.sudden_obstacle.enabled = true;
  params.avoidance.sudden_obstacle.shadow_mode = true;
  const auto issues = validateParams(params);
  EXPECT_TRUE(hasIssue(issues, ParamIssue::WARN, "sudden_obstacle"));
  EXPECT_FALSE(hasAnyFatal(issues));
}

// ── 前向近障低通刹停 ──

TEST(ParamValidation, FrontLowpassStopDisabledSkipsValidation)
{
  SafetyNodeParams params;
  params.front_obstacle_lowpass_stop.enabled = false;
  // 关闭时任何取值都不参与决策，不该拒启
  params.front_obstacle_lowpass_stop.release_distance_m = 0.0;
  params.front_obstacle_lowpass_stop.trigger_distance_m = -1.0;
  EXPECT_FALSE(hasAnyFatal(validateParams(params)));
}

// 注意：结构体默认 hard_stop_distance_m=0.12，而生产 yaml 实配为 0.03。
// 本层的告警规则与 hard_stop 耦合，故按实配取值验证，别照搬结构体默认值。
TEST(ParamValidation, FrontLowpassStopDefaultsPassWhenEnabled)
{
  SafetyNodeParams params;
  params.avoidance.hard_stop_distance_m = 0.03;    // 生产实配
  params.front_obstacle_lowpass_stop.enabled = true;
  const auto issues = validateParams(params);
  EXPECT_FALSE(hasAnyFatal(issues));
  EXPECT_FALSE(hasIssue(issues, ParamIssue::WARN, "front_obstacle_lowpass_stop"));
}

TEST(ParamValidation, FrontLowpassStopRejectsUnknownSource)
{
  SafetyNodeParams params;
  params.front_obstacle_lowpass_stop.enabled = true;
  params.front_obstacle_lowpass_stop.obstacle_source = "all";
  EXPECT_TRUE(
    hasIssue(validateParams(params), ParamIssue::FATAL, "source"));
}

TEST(ParamValidation, TiltOnlyFrontLowpassRequiresTiltPerception)
{
  SafetyNodeParams params;
  params.front_obstacle_lowpass_stop.enabled = true;
  params.front_obstacle_lowpass_stop.obstacle_source = "tilt_only";
  params.tilt_perception.enabled = false;
  EXPECT_TRUE(
    hasIssue(validateParams(params), ParamIssue::FATAL, "perception.tilt.enabled"));
}

TEST(ParamValidation, FrontLowpassStopRejectsInvalidPredictionWindow)
{
  SafetyNodeParams params;
  params.front_obstacle_lowpass_stop.enabled = true;
  params.front_obstacle_lowpass_stop.stale_prediction_max_s = 0.0;
  EXPECT_TRUE(
    hasIssue(validateParams(params), ParamIssue::FATAL, "stale_prediction_max_s"));

  params.front_obstacle_lowpass_stop.stale_prediction_max_s = 0.20;
  params.front_obstacle_lowpass_stop.prediction_uncertainty_margin_m = -0.01;
  EXPECT_TRUE(
    hasIssue(validateParams(params), ParamIssue::FATAL, "prediction_uncertainty_margin_m"));
}

TEST(ParamValidation, FrontLowpassStopReleaseNotAboveTriggerIsFatal)
{
  SafetyNodeParams params;
  params.front_obstacle_lowpass_stop.enabled = true;
  params.front_obstacle_lowpass_stop.release_distance_m =
    params.front_obstacle_lowpass_stop.trigger_distance_m;
  EXPECT_TRUE(
    hasIssue(validateParams(params), ParamIssue::FATAL, "release_distance_m"));
}

TEST(ParamValidation, FrontLowpassStopInvertedTimeConstantsAreFatal)
{
  SafetyNodeParams params;
  params.front_obstacle_lowpass_stop.enabled = true;
  params.front_obstacle_lowpass_stop.filter_time_constant_s = 0.05;
  params.front_obstacle_lowpass_stop.min_time_constant_s = 0.20;
  EXPECT_TRUE(
    hasIssue(validateParams(params), ParamIssue::FATAL, "filter_time_constant_s"));
}

TEST(ParamValidation, FrontLowpassStopStopBufferBeyondTriggerIsFatal)
{
  SafetyNodeParams params;
  params.front_obstacle_lowpass_stop.enabled = true;
  params.front_obstacle_lowpass_stop.stop_buffer_m = 0.20;
  params.front_obstacle_lowpass_stop.trigger_distance_m = 0.16;
  EXPECT_TRUE(
    hasIssue(validateParams(params), ParamIssue::FATAL, "stop_buffer_m"));
}

// 触发边界所需 τ 已低于下限 → 连最急的低通都停不进余量
TEST(ParamValidation, FrontLowpassStopUnstoppableAtTriggerIsWarn)
{
  SafetyNodeParams params;
  params.avoidance.hard_stop_distance_m = 0.03;
  params.front_obstacle_lowpass_stop.enabled = true;
  // budget = 0.16 - 0.08 = 0.08；τ_needed = 0.08/2.0 = 0.04 < min 0.08
  params.front_obstacle_lowpass_stop.trigger_speed_m_s = 2.0;
  const auto issues = validateParams(params);
  EXPECT_TRUE(hasIssue(issues, ParamIssue::WARN, "min_time_constant_s"));
  EXPECT_FALSE(hasAnyFatal(issues));
}

// 默认边界：τ_needed = 0.08/0.35 = 0.229 > min 0.08 → 停得住，不该告警
TEST(ParamValidation, FrontLowpassStopStoppableAtTriggerIsNotWarn)
{
  SafetyNodeParams params;
  params.avoidance.hard_stop_distance_m = 0.03;
  params.front_obstacle_lowpass_stop.enabled = true;
  EXPECT_FALSE(
    hasIssue(validateParams(params), ParamIssue::WARN, "min_time_constant_s"));
}

TEST(ParamValidation, FrontLowpassStopTriggerInsideHardStopIsWarn)
{
  SafetyNodeParams params;
  params.avoidance.hard_stop_distance_m = 0.12;
  params.front_obstacle_lowpass_stop.enabled = true;
  params.front_obstacle_lowpass_stop.trigger_distance_m = 0.10;
  params.front_obstacle_lowpass_stop.stop_buffer_m = 0.02;
  const auto issues = validateParams(params);
  EXPECT_TRUE(hasIssue(issues, ParamIssue::WARN, "触发区已落在急停区内"));
  EXPECT_FALSE(hasAnyFatal(issues));
}

// 低通瞄准点若在硬停线之内，急停会先触发，本层等于白配
TEST(ParamValidation, FrontLowpassStopBufferInsideHardStopIsWarn)
{
  SafetyNodeParams params;
  params.avoidance.hard_stop_distance_m = 0.12;
  params.front_obstacle_lowpass_stop.enabled = true;
  params.front_obstacle_lowpass_stop.stop_buffer_m = 0.05;
  const auto issues = validateParams(params);
  EXPECT_TRUE(hasIssue(issues, ParamIssue::WARN, "瞄准点在硬停线之内"));
  EXPECT_FALSE(hasAnyFatal(issues));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
