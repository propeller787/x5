/**
 * @file safety_params.cpp
 * @brief ROS 参数 → SafetyNodeParams 映射
 *
 * 参数键与 config/safety_node_params.yaml 一一对应，
 * 使用 declare_parameter + get_parameter 支持 launch 覆盖。
 */
#include "safety_node/safety_params.h"

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

SafetyNodeParams SafetyNodeParams::loadFromNode(rclcpp::Node & node)
{
  SafetyNodeParams p;

  if (!node.has_parameter("control_rate_hz")) {
    node.declare_parameter<double>("control_rate_hz", 20.0);
  }
  p.control_rate_hz = node.get_parameter("control_rate_hz").as_double();

  auto get_d = [&](const char * name, double def) {
      if (!node.has_parameter(name)) {
        node.declare_parameter<double>(name, def);
      }
      return node.get_parameter(name).as_double();
    };
  auto get_i = [&](const char * name, int def) {
      if (!node.has_parameter(name)) {
        node.declare_parameter<int>(name, def);
      }
      return node.get_parameter(name).as_int();
    };
  auto get_b = [&](const char * name, bool def) {
      if (!node.has_parameter(name)) {
        node.declare_parameter<bool>(name, def);
      }
      return node.get_parameter(name).as_bool();
    };
  auto get_s = [&](const char * name, const std::string & def) {
      if (!node.has_parameter(name)) {
        node.declare_parameter<std::string>(name, def);
      }
      return node.get_parameter(name).as_string();
    };

  // ── 方案 9.3 配置版本 / 方案 7 执行器开关 ──
  p.config_schema_version =
    get_i("config_schema_version", kSupportedConfigSchemaVersion);
  p.multithreaded_executor = get_b("runtime.multithreaded_executor", false);

  p.control_command_timeout_s = get_d("input_health.control_command_timeout_s", 0.25);
  p.joystick_timeout_s = get_d("input_health.joystick_timeout_s", 0.25);
  p.joystick_deadzone = get_i("input_health.joystick_deadzone", 5);
  p.joystick_reverse_threshold = get_i("input_health.joystick_reverse_threshold", 20);
  p.wheel_feedback_timeout_s = get_d("input_health.wheel_feedback_timeout_s", 0.25);
  p.require_wheel_feedback = get_b("input_health.require_wheel_feedback", true);
  p.car_info_timeout_s = get_d("input_health.car_info_timeout_s", 1.0);
  p.perception_timeout_s = get_d("input_health.perception_timeout_s", 0.35);
  p.tf_lookup_timeout_s = get_d("input_health.tf_lookup_timeout_s", 0.02);
  p.min_fresh_perception_sources = get_i("input_health.min_fresh_perception_sources", 1);
  p.require_perception = get_b("input_health.require_perception", true);
  p.min_fresh_horizontal_sources = get_i("input_health.min_fresh_horizontal_sources", 1);
  p.require_horizontal_perception = get_b("input_health.require_horizontal_perception", true);
  p.require_tilt_perception = get_b("input_health.require_tilt_perception", false);
  p.emergency_release_delay_s = get_d("output.emergency_release_delay_s", 0.15);
  p.neutral_stop_filter_time_constant_s =
    get_d("output.neutral_stop_filter_time_constant_s", 0.30);
  p.direction_reversal_fast_stop_enabled =
    get_b("output.direction_reversal_fast_stop_enabled", true);
  p.direction_reversal_stop_decel_m_s2 =
    get_d("output.direction_reversal_stop_decel_m_s2", 2.5);
  p.direction_reversal_stop_jerk_m_s3 =
    get_d("output.direction_reversal_stop_jerk_m_s3", 8.0);
  p.direction_reversal_release_speed_m_s =
    get_d("output.direction_reversal_release_speed_m_s", 0.03);

  // ── 前向近障低通刹停（启动级开关，默认关）──
  {
    const char * kPrefix = "output.front_obstacle_lowpass_stop.";
    auto key = [&](const char * leaf) {
        return std::string(kPrefix) + leaf;
      };
    auto & f = p.front_obstacle_lowpass_stop;
    f.enabled = get_b(key("enabled").c_str(), false);
    f.obstacle_source = get_s(key("source").c_str(), "horizontal");
    f.trigger_distance_m = get_d(key("trigger_distance_m").c_str(), 0.16);
    f.trigger_speed_m_s = get_d(key("trigger_speed_m_s").c_str(), 0.35);
    f.filter_time_constant_s = get_d(key("filter_time_constant_s").c_str(), 0.30);
    f.min_time_constant_s = get_d(key("min_time_constant_s").c_str(), 0.08);
    f.stop_buffer_m = get_d(key("stop_buffer_m").c_str(), 0.08);
    f.confirm_frames = get_i(key("confirm_frames").c_str(), 2);
    f.release_distance_m = get_d(key("release_distance_m").c_str(), 0.25);
    f.clear_hold_s = get_d(key("clear_hold_s").c_str(), 0.15);
    f.standstill_release_speed_eps_m_s =
      get_d(key("standstill_release_speed_eps_m_s").c_str(), 0.03);
    f.preserve_steering = get_b(key("preserve_steering").c_str(), true);
    f.override_map_veto = get_b(key("override_map_veto").c_str(), true);
    f.stale_prediction_enabled =
      get_b(key("stale_prediction_enabled").c_str(), true);
    f.stale_prediction_max_s =
      get_d(key("stale_prediction_max_s").c_str(), 0.20);
    f.prediction_uncertainty_margin_m =
      get_d(key("prediction_uncertainty_margin_m").c_str(), 0.01);
  }

  // ── 水平 / 斜雷达感知 ──
  p.horizontal_perception.enabled = get_b("perception.horizontal.enabled", true);
  p.horizontal_perception.left_enabled = get_b("perception.horizontal.left_enabled", true);
  p.horizontal_perception.right_enabled = get_b("perception.horizontal.right_enabled", true);
  p.horizontal_perception.back_enabled = get_b("perception.horizontal.back_enabled", true);
  p.horizontal_perception.feed_local_map = get_b("perception.horizontal.feed_local_map", true);

  p.tilt_perception.coordinate_frame =
    get_s("perception.tilt.coordinate_frame", "sensor");
  p.tilt_perception.publish_debug_scan = get_b("perception.tilt.publish_debug_scan", true);
  p.tilt_perception.enabled = get_b("perception.tilt.enabled", true);
  p.tilt_perception.feed_local_map = get_b("perception.tilt.feed_local_map", false);
  p.tilt_perception.ground_reject_enabled =
    get_b("perception.tilt.ground_reject_enabled", true);
  p.tilt_perception.z_filter_enabled =
    get_b("perception.tilt.z_filter_enabled", false);
  p.tilt_perception.preserve_sensor_z =
    get_b("perception.tilt.preserve_sensor_z", true);
  p.tilt_perception.z_filter_threshold =
    get_d("perception.tilt.z_filter_threshold", 0.0);
  p.tilt_perception.voxel_size_m = get_d("perception.tilt.voxel_size_m", 0.08);
  p.tilt_perception.max_primitives = get_i("perception.tilt.max_primitives", 48);

  // 感知 ROI 边界（参数化替代硬编码，默认 [-2.5,6.5]×[-1.8,1.8]）
  p.perception.x_min_m = get_d("perception.roi.x_min_m", -2.5);
  p.perception.x_max_m = get_d("perception.roi.x_max_m", 6.5);
  p.perception.y_min_m = get_d("perception.roi.y_min_m", -1.8);
  p.perception.y_max_m = get_d("perception.roi.y_max_m", 1.8);

  p.avoidance.rollout_horizon_s = get_d("avoidance.horizon_s", 1.2);
  p.avoidance.rollout_dt_s = get_d("avoidance.dt_s", 0.1);
  p.avoidance.rollout_speed_samples = get_i("avoidance.speed_samples", 6);
  p.avoidance.rollout_angular_samples = get_i("avoidance.angular_samples", 5);
  p.avoidance.use_ttc_emergency_stop =
    get_b("avoidance.use_ttc_emergency_stop", false);
  p.avoidance.emergency_ttc_s = get_d("avoidance.emergency_ttc_s", 0.45);
  p.avoidance.hard_stop_clearance_enabled =
    get_b("avoidance.hard_stop_clearance_enabled", true);
  p.avoidance.hard_stop_distance_m = get_d("avoidance.hard_stop_distance_m", 0.12);
  p.avoidance.emergency_ttc_release_s = get_d("avoidance.emergency_ttc_release_s", 0.70);
  p.avoidance.hard_stop_release_distance_m =
    get_d("avoidance.hard_stop_release_distance_m", 0.20);
  p.avoidance.emergency_clear_hold_s = get_d("avoidance.emergency_clear_hold_s", 0.15);
  p.avoidance.safety_filter_angular_weight = get_d("avoidance.safety_filter_angular_weight", 0.30);
  p.avoidance.max_decel_m_s2 = get_d("avoidance.max_decel_m_s2", 1.2);
  p.avoidance.max_recover_accel_m_s2 = get_d("avoidance.max_recover_accel_m_s2", 0.5);
  p.avoidance.max_angular_decel_rad_s2 =
    get_d("avoidance.max_angular_decel_rad_s2", 2.0);
  p.avoidance.max_angular_recover_rad_s2 =
    get_d("avoidance.max_angular_recover_rad_s2", 1.0);
  p.avoidance.braking_envelope_enabled =
    get_b("avoidance.braking_envelope_enabled", true);
  p.avoidance.braking_use_local_map =
    get_b("avoidance.braking_use_local_map", true);
  p.avoidance.braking_lookahead_m =
    get_d("avoidance.braking_lookahead_m", 5.0);
  p.avoidance.braking_path_step_m =
    get_d("avoidance.braking_path_step_m", 0.025);
  p.avoidance.braking_probe_max_heading_rad =
    get_d("avoidance.braking_probe_max_heading_rad", 1.5708);
  p.avoidance.braking_reaction_delay_s =
    get_d("avoidance.braking_reaction_delay_s", 0.10);
  p.avoidance.braking_comfort_decel_m_s2 =
    get_d("avoidance.braking_comfort_decel_m_s2", 0.60);
  p.avoidance.braking_max_jerk_m_s3 =
    get_d("avoidance.braking_max_jerk_m_s3", 2.5);
  p.avoidance.braking_stop_buffer_m =
    get_d("avoidance.braking_stop_buffer_m", 0.02);
  p.avoidance.braking_obstacle_closing_reserve_m_s =
    get_d("avoidance.braking_obstacle_closing_reserve_m_s", 0.0);
  p.avoidance.min_distance_braking_enabled =
    get_b("avoidance.min_distance_braking_enabled", true);
  p.avoidance.braking_exact_stop_enabled =
    get_b("avoidance.braking_exact_stop_enabled", true);
  p.avoidance.braking_at_stop_distance_eps_m =
    get_d("avoidance.braking_at_stop_distance_eps_m", 0.0125);
  p.avoidance.braking_no_forward_reaccel_at_hard_stop =
    get_b("avoidance.braking_no_forward_reaccel_at_hard_stop", true);
  p.avoidance.braking_suppress_distance_emergency =
    get_b("avoidance.braking_suppress_distance_emergency", true);
  p.avoidance.forward_stop_hold_enabled =
    get_b("avoidance.forward_stop_hold.enabled", true);
  p.avoidance.forward_stop_hold_confirm_s =
    get_d("avoidance.forward_stop_hold.confirm_s", 0.10);
  p.avoidance.forward_stop_hold_feedback_speed_m_s =
    get_d("avoidance.forward_stop_hold.feedback_speed_m_s", 0.03);
  p.avoidance.forward_stop_hold_speed_limit_m_s =
    get_d("avoidance.forward_stop_hold.speed_limit_m_s", 0.20);
  p.avoidance.forward_stop_hold_remaining_m =
    get_d("avoidance.forward_stop_hold.remaining_m", 0.08);
  p.avoidance.forward_stop_hold_release_distance_m =
    get_d("avoidance.forward_stop_hold.release_distance_m", 0.20);
  // ── P0-B 停稳看护 ──
  p.avoidance.standstill_guard_enabled =
    get_b("avoidance.standstill_guard.enabled", true);
  p.avoidance.standstill_guard_band_m =
    get_d("avoidance.standstill_guard.band_m", 0.12);
  p.avoidance.standstill_guard_floor_m =
    get_d("avoidance.standstill_guard.floor_m", 0.03);
  p.avoidance.standstill_guard_approach_rate_m_s =
    get_d("avoidance.standstill_guard.approach_rate_m_s", 0.05);
  p.avoidance.standstill_guard_confirm_s =
    get_d("avoidance.standstill_guard.confirm_s", 0.10);
  p.avoidance.standstill_guard_release_m =
    get_d("avoidance.standstill_guard.release_m", 0.15);
  // ── L1 方向解耦掩码（direction mask，见 direction_mask.h） ──
  p.avoidance.direction_mask_enabled =
    get_b("avoidance.direction_mask.enabled", false);
  p.avoidance.direction_mask_probe_distance_m =
    get_d("avoidance.direction_mask.probe_distance_m", 0.15);
  p.avoidance.direction_mask_probe_angle_rad =
    get_d("avoidance.direction_mask.probe_angle_rad", 0.30);
  p.avoidance.direction_mask_probe_step_m =
    get_d("avoidance.direction_mask.probe_step_m", 0.03);
  p.avoidance.direction_mask_probe_step_rad =
    get_d("avoidance.direction_mask.probe_step_rad", 0.06);
  p.avoidance.direction_mask_contact_buffer_m =
    get_d("avoidance.direction_mask.contact_buffer_m", 0.03);
  p.avoidance.direction_mask_approach_eps_m =
    get_d("avoidance.direction_mask.approach_eps_m", 0.005);
  p.avoidance.direction_mask_map_exempt_radius_m =
    get_d("avoidance.direction_mask.map_exempt_radius_m", 0.22);
  // ── 转弯净空护栏（turn clearance guard，见 avoidance_types.h） ──
  p.avoidance.turn_clearance_guard_enabled =
    get_b("avoidance.turn_clearance_guard.enabled", false);
  p.avoidance.turn_clearance_guard_min_angular_rad_s =
    get_d("avoidance.turn_clearance_guard.min_angular_rad_s", 0.5);
  p.avoidance.turn_clearance_guard_min_curvature_1_m =
    get_d("avoidance.turn_clearance_guard.min_curvature_1_m", 0.0);
  p.avoidance.turn_clearance_guard_band_m =
    get_d("avoidance.turn_clearance_guard.band_m", 0.6);
  p.avoidance.turn_speed_envelope_enabled =
    get_b("avoidance.turn_speed_envelope.enabled", false);
  p.avoidance.turn_envelope_edge_radius_m =
    get_d("avoidance.turn_speed_envelope.edge_radius_m", 0.655);
  p.avoidance.turn_envelope_max_edge_speed_m_s =
    get_d("avoidance.turn_speed_envelope.max_edge_speed_m_s", 2.222);
  p.avoidance.turn_envelope_max_lat_accel_m_s2 =
    get_d("avoidance.turn_speed_envelope.max_lat_accel_m_s2", 1.2);
  // ── 急速转向线速度硬帽（rapid steer cap，见 core/avoidance_types.h） ──
  p.avoidance.rapid_steer_cap_enabled =
    get_b("avoidance.rapid_steer_cap.enabled", false);
  p.avoidance.rapid_steer_trigger_angular_rad_s =
    get_d("avoidance.rapid_steer_cap.trigger_angular_rad_s", 0.6);
  p.avoidance.rapid_steer_max_linear_m_s =
    get_d("avoidance.rapid_steer_cap.max_linear_m_s", 0.6);
  // ── L2 被困脱困（pinned escape，见 avoidance_types.h） ──
  p.avoidance.pinned_escape.enabled =
    get_b("avoidance.pinned_escape.enabled", false);
  p.avoidance.pinned_escape.shadow_mode =
    get_b("avoidance.pinned_escape.shadow_mode", true);
  p.avoidance.pinned_escape.confirm_s =
    get_d("avoidance.pinned_escape.confirm_s", 0.20);
  p.avoidance.pinned_escape.creep_linear_m_s =
    get_d("avoidance.pinned_escape.creep_linear_m_s", 0.06);
  p.avoidance.pinned_escape.creep_angular_rad_s =
    get_d("avoidance.pinned_escape.creep_angular_rad_s", 0.25);
  p.avoidance.pinned_escape.escape_margin_m =
    get_d("avoidance.pinned_escape.escape_margin_m", 0.03);
  p.avoidance.pinned_escape.min_clearance_gain_m =
    get_d("avoidance.pinned_escape.min_clearance_gain_m", 0.005);
  p.avoidance.pinned_escape.standstill_speed_eps_m_s =
    get_d("avoidance.pinned_escape.standstill_speed_eps_m_s", 0.03);
  // ── 原地转向豁免（rotation escape，见 core/avoidance_types.h） ──
  p.avoidance.rotation_escape.enabled =
    get_b("avoidance.rotation_escape.enabled", false);
  p.avoidance.rotation_escape.min_angular_rad_s =
    get_d("avoidance.rotation_escape.min_angular_rad_s", 0.10);
  p.avoidance.rotation_escape.max_angular_rad_s =
    get_d("avoidance.rotation_escape.max_angular_rad_s", 0.60);
  p.avoidance.rotation_escape.probe_angle_rad =
    get_d("avoidance.rotation_escape.probe_angle_rad", 0.35);
  p.avoidance.rotation_escape.probe_step_rad =
    get_d("avoidance.rotation_escape.probe_step_rad", 0.05);
  // ── 斜雷达前向近障软限速（见 core/front_tilt_speed_cap.h） ──
  p.avoidance.front_tilt_speed_cap.enabled =
    get_b("avoidance.front_tilt_speed_cap.enabled", false);
  p.avoidance.front_tilt_speed_cap.corridor_margin_m =
    get_d("avoidance.front_tilt_speed_cap.corridor_margin_m", 0.30);
  p.avoidance.front_tilt_speed_cap.trigger_distance_m =
    get_d("avoidance.front_tilt_speed_cap.trigger_distance_m", 2.0);
  p.avoidance.front_tilt_speed_cap.release_distance_m =
    get_d("avoidance.front_tilt_speed_cap.release_distance_m", 2.3);
  p.avoidance.front_tilt_speed_cap.speed_cap_m_s =
    get_d("avoidance.front_tilt_speed_cap.speed_cap_m_s", 1.25);
  p.avoidance.front_tilt_speed_cap.confirm_frames =
    get_i("avoidance.front_tilt_speed_cap.confirm_frames", 2);
  p.avoidance.front_tilt_speed_cap.clear_hold_s =
    get_d("avoidance.front_tilt_speed_cap.clear_hold_s", 0.15);
  // ── L0-EB 突发障碍动态紧急制动（见 SUDDEN_BRAKE_PLAN.md） ──
  p.avoidance.sudden_obstacle.enabled =
    get_b("avoidance.sudden_obstacle.enabled", false);
  p.avoidance.sudden_obstacle.shadow_mode =
    get_b("avoidance.sudden_obstacle.shadow_mode", true);
  p.avoidance.sudden_obstacle.emergency_decel_m_s2 =
    get_d("avoidance.sudden_obstacle.emergency_decel_m_s2", 2.0);
  p.avoidance.sudden_obstacle.comfort_decel_m_s2 =
    get_d("avoidance.sudden_obstacle.comfort_decel_m_s2", 0.6);
  p.avoidance.sudden_obstacle.t_react_s =
    get_d("avoidance.sudden_obstacle.t_react_s", 0.30);
  p.avoidance.sudden_obstacle.emergency_distance_m =
    get_d("avoidance.sudden_obstacle.emergency_distance_m", 2.0);
  p.avoidance.sudden_obstacle.min_forward_speed_m_s =
    get_d("avoidance.sudden_obstacle.min_forward_speed_m_s", 0.15);
  p.avoidance.sudden_obstacle.confirm_frames =
    get_i("avoidance.sudden_obstacle.confirm_frames", 2);
  p.avoidance.sudden_obstacle.a_req_filter_alpha =
    get_d("avoidance.sudden_obstacle.a_req_filter_alpha", 0.5);
  p.avoidance.sudden_obstacle.d_eff_floor_m =
    get_d("avoidance.sudden_obstacle.d_eff_floor_m", 0.03);
  p.avoidance.sudden_obstacle.corridor_margin_m =
    get_d("avoidance.sudden_obstacle.corridor_margin_m", 0.05);
  p.avoidance.sudden_obstacle.release_distance_m =
    get_d("avoidance.sudden_obstacle.release_distance_m", 0.30);
  p.avoidance.sudden_obstacle.clear_hold_s =
    get_d("avoidance.sudden_obstacle.clear_hold_s", 0.20);
  p.avoidance.horizontal_avoidance_enabled =
    get_b("avoidance.horizontal_avoidance.enabled", true);
  // ── 斜雷达近距 TTC 通道【deprecated，方案 2.5】──
  // 已被 treat_as_horizontal 并入水平主链替代。代码默认值改为 false（安全侧）：
  // 缺参数文件启动时废弃通道不得自动开启。物理删除见 PRODUCTION_HARDENING_PLAN.md。
  p.avoidance.tilt_ttc.enabled = get_b("avoidance.tilt_ttc.enabled", false);
  p.avoidance.tilt_ttc.horizon_s = get_d("avoidance.tilt_ttc.horizon_s", 0.5);
  p.avoidance.tilt_ttc.max_probe_distance_m =
    get_d("avoidance.tilt_ttc.max_probe_distance_m", 1.0);
  p.avoidance.tilt_ttc.hard_stop_distance_m =
    get_d("avoidance.tilt_ttc.hard_stop_distance_m", 0.12);
  p.avoidance.tilt_ttc.limiting_distance_m =
    get_d("avoidance.tilt_ttc.limiting_distance_m", 1.0);
  p.avoidance.tilt_ttc.release_distance_m =
    get_d("avoidance.tilt_ttc.release_distance_m", 1.05);
  p.avoidance.tilt_ttc.distance_emergency_m =
    get_d("avoidance.tilt_ttc.distance_emergency_m", 0.50);
  p.avoidance.tilt_ttc.limiting_ttc_s =
    get_d("avoidance.tilt_ttc.limiting_ttc_s", 0.55);
  p.avoidance.tilt_ttc.emergency_ttc_s =
    get_d("avoidance.tilt_ttc.emergency_ttc_s", 0.35);
  p.avoidance.tilt_ttc.emergency_ttc_release_s =
    get_d("avoidance.tilt_ttc.emergency_ttc_release_s", 0.50);
  p.avoidance.tilt_ttc.emergency_clear_hold_s =
    get_d("avoidance.tilt_ttc.emergency_clear_hold_s", 0.15);
  p.avoidance.tilt_ttc.min_speed_for_ttc_m_s =
    get_d("avoidance.tilt_ttc.min_speed_for_ttc_m_s", 0.25);
  p.avoidance.tilt_ttc.obstacle_radius_m =
    get_d("avoidance.tilt_ttc.obstacle_radius_m", 0.01);
  p.avoidance.tilt_ttc.use_command_velocity_for_ttc =
    get_b("avoidance.tilt_ttc.use_command_velocity_for_ttc", true);
  // ── P0 精准停车：近距指数律 + 脉冲补偿 ──
  p.avoidance.precise_stop.enabled = get_b("avoidance.precise_stop.enabled", false);
  p.avoidance.precise_stop.near_zone_m =
    get_d("avoidance.precise_stop.near_zone_m", 0.35);
  p.avoidance.precise_stop.exp_lambda_m =
    get_d("avoidance.precise_stop.exp_lambda_m", 0.08);
  p.avoidance.precise_stop.near_speed_scale_m_s =
    get_d("avoidance.precise_stop.near_speed_scale_m_s", 0.45);
  p.avoidance.precise_stop.stop_tolerance_m =
    get_d("avoidance.precise_stop.stop_tolerance_m", 0.015);
  p.avoidance.precise_stop.pulse_enabled =
    get_b("avoidance.precise_stop.pulse.enabled", true);
  p.avoidance.precise_stop.pulse_creep_speed_m_s =
    get_d("avoidance.precise_stop.pulse.creep_speed_m_s", 0.08);
  p.avoidance.precise_stop.pulse_max_duration_s =
    get_d("avoidance.precise_stop.pulse.max_duration_s", 0.25);
  p.avoidance.precise_stop.pulse_stall_speed_eps_m_s =
    get_d("avoidance.precise_stop.pulse.stall_speed_eps_m_s", 0.02);
  p.avoidance.precise_stop.pulse_stall_hold_s =
    get_d("avoidance.precise_stop.pulse.stall_hold_s", 0.20);
  p.avoidance.precise_stop.pulse_cooldown_s =
    get_d("avoidance.precise_stop.pulse.cooldown_s", 0.20);

  // ── P1 人机共享控制：连续接管 α + 转向保真 ──
  p.avoidance.shared_control.enabled = get_b("avoidance.shared_control.enabled", false);
  p.avoidance.shared_control.preserve_steering =
    get_b("avoidance.shared_control.preserve_steering", true);
  p.avoidance.shared_control.alpha_ttc_zero_s =
    get_d("avoidance.shared_control.alpha_ttc_zero_s", 1.5);
  p.avoidance.shared_control.alpha_ttc_full_s =
    get_d("avoidance.shared_control.alpha_ttc_full_s", 0.45);
  p.avoidance.shared_control.alpha_clearance_zero_m =
    get_d("avoidance.shared_control.alpha_clearance_zero_m", 0.30);
  p.avoidance.shared_control.alpha_clearance_full_m =
    get_d("avoidance.shared_control.alpha_clearance_full_m", 0.05);
  p.avoidance.shared_control.alpha_release_rate_per_s =
    get_d("avoidance.shared_control.alpha_release_rate_per_s", 2.0);

  // ── P2 窄道通过：走廊提取 + Bézier 中线导引 ──
  p.avoidance.narrow_planner.enabled =
    get_b("avoidance.narrow_passage_planner.enabled", false);
  p.avoidance.narrow_planner.auto_detect_enabled =
    get_b("avoidance.narrow_passage_planner.auto_detect_enabled", true);
  p.avoidance.narrow_planner.look_ahead_m =
    get_d("avoidance.narrow_passage_planner.look_ahead_m", 2.0);
  p.avoidance.narrow_planner.roi_half_width_m =
    get_d("avoidance.narrow_passage_planner.roi_half_width_m", 1.2);
  p.avoidance.narrow_planner.bin_size_m =
    get_d("avoidance.narrow_passage_planner.bin_size_m", 0.15);
  p.avoidance.narrow_planner.min_corridor_len_m =
    get_d("avoidance.narrow_passage_planner.min_corridor_len_m", 0.45);
  p.avoidance.narrow_planner.min_side_margin_m =
    get_d("avoidance.narrow_passage_planner.min_side_margin_m", 0.03);
  p.avoidance.narrow_planner.narrow_width_max_m =
    get_d("avoidance.narrow_passage_planner.narrow_width_max_m", 0.60);
  p.avoidance.narrow_planner.speed_per_clearance =
    get_d("avoidance.narrow_passage_planner.speed_per_clearance", 3.0);
  p.avoidance.narrow_planner.min_pass_speed_m_s =
    get_d("avoidance.narrow_passage_planner.min_pass_speed_m_s", 0.15);
  p.avoidance.narrow_planner.max_curvature_1_m =
    get_d("avoidance.narrow_passage_planner.max_curvature_1_m", 1.5);
  p.avoidance.narrow_planner.cost_bias =
    get_d("avoidance.narrow_passage_planner.cost_bias", 0.15);

  // ── P0-A' 动态障碍逼近速度估计（方案 5，见 closing_speed_estimator.h） ──
  p.avoidance.closing_speed.enabled = get_b("avoidance.closing_speed.enabled", false);
  p.avoidance.closing_speed.filter_alpha =
    get_d("avoidance.closing_speed.filter_alpha", 0.4);
  p.avoidance.closing_speed.confirm_frames =
    get_i("avoidance.closing_speed.confirm_frames", 3);
  p.avoidance.closing_speed.max_rate_m_s =
    get_d("avoidance.closing_speed.max_rate_m_s", 2.0);
  p.avoidance.closing_speed.jump_reset_m =
    get_d("avoidance.closing_speed.jump_reset_m", 0.30);

  // ── 方案 8 扫掠预裁剪 / 方案 1 影子仲裁 ──
  p.avoidance.swept_cull_band_m = get_d("avoidance.swept_cull_band_m", 1.0);
  p.avoidance.arbiter_shadow_enabled =
    get_b("avoidance.arbiter_shadow.enabled", true);

  p.avoidance.candidate_speed_up_hold_s = get_d("avoidance.candidate_speed_up_hold_s", 0.30);
  p.avoidance.recovery_mode_enabled = get_b("avoidance.recovery_mode_enabled", true);
  p.avoidance.recovery_max_linear_speed =
    get_d("avoidance.recovery_max_linear_speed", 0.12);
  p.avoidance.recovery_max_angular_speed =
    get_d("avoidance.recovery_max_angular_speed", 0.35);
  p.avoidance.recovery_lookahead_s = get_d("avoidance.recovery_lookahead_s", 0.35);
  p.avoidance.recovery_min_command = get_d("avoidance.recovery_min_command", 0.02);
  p.avoidance.standstill_speed_eps_m_s =
    get_d("avoidance.standstill_speed_eps_m_s", 0.02);
  p.avoidance.recovery_min_clearance_improvement_m =
    get_d("avoidance.recovery_min_clearance_improvement_m", 0.005);
  p.avoidance.recovery_max_transient_clearance_loss_m =
    get_d("avoidance.recovery_max_transient_clearance_loss_m", 0.001);
  p.avoidance.recovery_auto_select_enabled =
    get_b("avoidance.recovery_auto_select_enabled", true);
  p.avoidance.recovery_intrusion_confirm_s =
    get_d("avoidance.recovery_intrusion_confirm_s", 0.10);
  p.avoidance.recovery_candidate_hold_s =
    get_d("avoidance.recovery_candidate_hold_s", 0.30);
  p.avoidance.recovery_timeout_s =
    get_d("avoidance.recovery_timeout_s", 1.50);
  p.avoidance.recovery_no_progress_timeout_s =
    get_d("avoidance.recovery_no_progress_timeout_s", 0.50);
  p.avoidance.recovery_motion_grace_s =
    get_d("avoidance.recovery_motion_grace_s", 0.25);
  p.avoidance.recovery_feedback_stall_timeout_s =
    get_d("avoidance.recovery_feedback_stall_timeout_s", 0.35);
  p.avoidance.recovery_feedback_linear_min_m_s =
    get_d("avoidance.recovery_feedback_linear_min_m_s", 0.01);
  p.avoidance.recovery_feedback_angular_min_rad_s =
    get_d("avoidance.recovery_feedback_angular_min_rad_s", 0.03);

  // ── 车体 footprint 几何 ──
  p.avoidance.robot_length = get_d("footprint.length_m", 0.88);
  const double width_rear = get_d("footprint.width_rear_m", get_d("footprint.width_m", 0.70));
  p.avoidance.robot_width_rear_m = width_rear;
  p.avoidance.robot_width_front_m = get_d("footprint.width_front_m", width_rear);
  p.avoidance.robot_rear_offset = get_d("footprint.rear_offset_m", 0.22);
  p.avoidance.footprint_use_dual_trapezoid =
    get_b("footprint.use_dual_trapezoid", false);
  if (p.avoidance.footprint_use_dual_trapezoid) {
    auto load_segment = [&](const char * prefix, FootprintSegmentParams & segment) {
        segment.length_m = get_d((std::string(prefix) + ".length_m").c_str(), 0.0);
        segment.width_front_m =
          get_d((std::string(prefix) + ".width_front_m").c_str(), 0.0);
        segment.width_rear_m =
          get_d((std::string(prefix) + ".width_rear_m").c_str(), 0.0);
      };
    load_segment("footprint.front_segment", p.avoidance.footprint_front_segment);
    load_segment("footprint.rear_segment", p.avoidance.footprint_rear_segment);
    p.avoidance.robot_length =
      p.avoidance.footprint_rear_segment.length_m +
      p.avoidance.footprint_front_segment.length_m;
    p.avoidance.robot_width_rear_m = p.avoidance.footprint_rear_segment.width_rear_m;
    p.avoidance.robot_width_front_m =
      p.avoidance.footprint_front_segment.width_front_m;
  }
  p.avoidance.footprint_front_margin = get_d("footprint.front_margin_m", 0.10);
  p.avoidance.footprint_back_margin = get_d("footprint.back_margin_m", 0.12);
  p.avoidance.footprint_side_margin = get_d("footprint.side_margin_m", 0.05);
  p.avoidance.footprint_obstacle_radius_default =
    get_d("footprint.obstacle_radius_default", 0.01);
  p.avoidance.footprint_intrusion_check_enabled =
    get_b("footprint.intrusion_check_enabled", true);
  p.avoidance.max_obstacle_primitives = 128;
  p.avoidance.absolute_max_linear_speed = get_d("avoidance.absolute_max_linear_speed", 2.222);
  p.avoidance.absolute_max_angular_speed = get_d("avoidance.absolute_max_angular_speed", 3.0);

  // ── Rolling LocalMap ──
  p.local_map.enabled = get_b("local_map.enabled", true);
  p.local_map.width_m = get_d("local_map.width_m", 6.0);
  p.local_map.height_m = get_d("local_map.height_m", 6.0);
  p.local_map.resolution_m = get_d("local_map.resolution_m", 0.05);
  p.local_map.inflation_radius_m = get_d("local_map.inflation_radius_m", 0.15);
  p.local_map.lethal_threshold = get_d("local_map.lethal_threshold", 0.65);
  p.local_map.decay_halflife_s = get_d("local_map.decay_halflife_s", 0.35);
  p.local_map.max_hit_age_s = get_d("local_map.max_hit_age_s", 0.8);
  p.local_map.observation_weight = get_d("local_map.observation_weight", 0.85);
  p.local_map.rebuild_each_frame = get_b("local_map.rebuild_each_frame", true);
  // 方案 6A：自运动补偿（仅 rebuild_each_frame=false 时有意义，见 rolling_local_map.h）
  p.local_map.ego_motion_compensation =
    get_b("local_map.ego_motion_compensation", false);
  p.local_map.rotation_decay_threshold_rad =
    get_d("local_map.rotation_decay_threshold_rad", 0.20);
  p.local_map.rotation_decay_factor =
    get_d("local_map.rotation_decay_factor", 0.5);
  p.local_map.tilt_z_threshold = get_d("local_map.tilt_z_threshold", 0.5);
  p.local_map.integrate_primitive_centers = get_b("local_map.integrate_primitives", false);
  p.local_map.primitive_observation_weight = get_d("local_map.primitive_observation_weight", 0.6);

  // ── Map 硬约束 ──
  p.map_constraint.enabled = get_b("map_constraint.enabled", true);
  p.map_constraint.veto_on_lethal = get_b("map_constraint.veto_on_lethal", true);
  p.map_constraint.check_footprint_corners = get_b("map_constraint.check_footprint_corners", true);
  p.map_constraint.retreat_escape_enabled =
    get_b("map_constraint.retreat_escape_enabled", false);
  p.map_constraint.retreat_min_progress_cells =
    get_d("map_constraint.retreat_min_progress_cells", 1.0);
  p.narrow_passage.footprint_side_margin_m =
    get_d("mode.narrow.footprint_side_margin_m", 0.01);
  p.narrow_passage.obstacle_radius_m =
    get_d("mode.narrow.obstacle_radius_m", 0.01);
  p.narrow_passage.hard_stop_distance_m =
    get_d("mode.narrow.hard_stop_distance_m", 0.015);
  p.narrow_passage.hard_stop_release_distance_m =
    get_d("mode.narrow.hard_stop_release_distance_m", 0.02);
  p.narrow_passage.local_map_inflation_radius_m =
    get_d("mode.narrow.local_map_inflation_radius_m", 0.01);
  p.narrow_passage.lethal_threshold =
    get_d("mode.narrow.lethal_threshold", 0.65);

  // 巡航（定速保持）模式参数
  p.cruise.angular_speed_threshold_rad_s =
    get_d("cruise.angular_speed_threshold_rad_s", 0.5);
  p.cruise.speed_recovery_rate_m_s = get_d("cruise.speed_recovery_rate_m_s", 0.05);
  p.cruise.speed_timeout_s = get_d("cruise.speed_timeout_s", 8.0);
  p.cruise.obstacle_zero_speed_eps_m_s =
    get_d("cruise.obstacle_zero_speed_eps_m_s", 0.02);

  // ── 对外状态/预警上报（reporting:，见 safety_params.h 的 ReportingConfig） ──
  p.reporting.auxiliary_status_enabled =
    get_b("reporting.auxiliary_status.enabled", true);
  p.reporting.auxiliary_status_rate_hz =
    get_d("reporting.auxiliary_status.rate_hz", 1.0);
  p.reporting.cruise_status_enabled = get_b("reporting.cruise_status.enabled", true);
  p.reporting.collision_warning.enabled =
    get_b("reporting.collision_warning.enabled", true);
  p.reporting.collision_warning.repeat_interval_s =
    get_d("reporting.collision_warning.repeat_interval_s", 1.0);
  p.reporting.collision_warning.report_emergency_fallback =
    get_b("reporting.collision_warning.report_emergency_fallback", true);

  // ── 方案 4 传感器降级矩阵（degradation:，见 degradation_policy.h） ──
  p.degradation.enabled = get_b("degradation.enabled", false);
  auto load_rule = [&](const char * prefix, DegradationRule & rule) {
      const std::string base(prefix);
      rule.block_forward = get_b((base + ".block_forward").c_str(), false);
      rule.block_backward = get_b((base + ".block_backward").c_str(), false);
      rule.block_left = get_b((base + ".block_left").c_str(), false);
      rule.block_right = get_b((base + ".block_right").c_str(), false);
      rule.speed_cap_m_s = get_d((base + ".speed_cap_m_s").c_str(), 0.0);
    };
  load_rule("degradation.left", p.degradation.per_source[kSourceLeft]);
  load_rule("degradation.right", p.degradation.per_source[kSourceRight]);
  load_rule("degradation.back", p.degradation.per_source[kSourceBack]);
  load_rule("degradation.left_tilt", p.degradation.per_source[kSourceLeftTilt]);
  load_rule("degradation.right_tilt", p.degradation.per_source[kSourceRightTilt]);
  load_rule("degradation.forward_pair_lost", p.degradation.forward_pair_lost);

  // 缺省回退到安全模式；narrow/pass_through/estop 已关闭，仅 safe_assist / cruise 可运行时切换。
  p.default_mode = parseSafetyMode(get_s("mode.default", "safe_assist"));

  return p;
}

namespace
{

/** 与 avoidance_controller.cpp 同式的 jerk 受限停车距离（校验器专用副本） */
double validatorStoppingDistance(
  double speed, double decel, double jerk, double reaction_delay)
{
  const double v = std::max(0.0, speed);
  const double a = std::max(1e-3, decel);
  const double j = std::max(1e-3, jerk);
  double distance = v * std::max(0.0, reaction_delay);
  const double ramp_time = a / j;
  const double ramp_speed_drop = 0.5 * j * ramp_time * ramp_time;
  if (v <= ramp_speed_drop) {
    const double stop_time = std::sqrt(2.0 * v / j);
    return distance + (2.0 / 3.0) * v * stop_time;
  }
  const double speed_after_ramp = v - ramp_speed_drop;
  distance += v * ramp_time - j * ramp_time * ramp_time * ramp_time / 6.0;
  distance += speed_after_ramp * speed_after_ramp / (2.0 * a);
  return distance;
}

}  // namespace

std::vector<ParamIssue> validateParams(const SafetyNodeParams & p)
{
  std::vector<ParamIssue> issues;
  auto fatal = [&](const std::string & msg) {
      issues.push_back({ParamIssue::FATAL, msg});
    };
  auto warn = [&](const std::string & msg) {
      issues.push_back({ParamIssue::WARN, msg});
    };
  auto fmt = [](const char * text, double a, double b) {
      std::ostringstream oss;
      oss << text << " (" << a << " vs " << b << ")";
      return oss.str();
    };

  // ── 配置版本 ──
  if (p.config_schema_version != SafetyNodeParams::kSupportedConfigSchemaVersion) {
    fatal(fmt(
        "config_schema_version 与代码支持版本不一致，请核对配置文件",
        p.config_schema_version, SafetyNodeParams::kSupportedConfigSchemaVersion));
  }

  // ── 范围类 ──
  if (!(p.control_rate_hz >= 1.0)) {
    fatal(fmt("control_rate_hz 必须 >= 1", p.control_rate_hz, 1.0));
  }
  if (!(p.local_map.resolution_m > 0.0)) {
    fatal(fmt("local_map.resolution_m 必须 > 0", p.local_map.resolution_m, 0.0));
  }
  if (!(p.local_map.width_m > 0.0) || !(p.local_map.height_m > 0.0)) {
    fatal("local_map.width_m / height_m 必须 > 0");
  }
  if (!(p.avoidance.rollout_horizon_s > 0.0) || !(p.avoidance.rollout_dt_s > 0.0)) {
    fatal("avoidance.horizon_s / dt_s 必须 > 0");
  }
  if (p.avoidance.rollout_speed_samples < 1 || p.avoidance.rollout_angular_samples < 1) {
    fatal("avoidance.speed_samples / angular_samples 必须 >= 1");
  }
  if (!(p.avoidance.absolute_max_linear_speed > 0.0) ||
    !(p.avoidance.absolute_max_angular_speed > 0.0))
  {
    fatal("avoidance.absolute_max_linear/angular_speed 必须 > 0");
  }
  if (p.avoidance.hard_stop_distance_m < 0.0 ||
    p.avoidance.braking_path_step_m <= 0.0 ||
    p.avoidance.braking_comfort_decel_m_s2 <= 0.0 ||
    p.avoidance.braking_max_jerk_m_s3 <= 0.0)
  {
    fatal("制动包络参数（hard_stop/path_step/comfort_decel/max_jerk）超出合法范围");
  }

  // ── 前向近障低通刹停（仅在 enabled 时校验，关闭时任何取值都不参与决策）──
  if (p.front_obstacle_lowpass_stop.enabled) {
    const auto & f = p.front_obstacle_lowpass_stop;
    if (f.obstacle_source != "horizontal" && f.obstacle_source != "tilt_only") {
      fatal("output.front_obstacle_lowpass_stop.source 仅支持 horizontal / tilt_only");
    }
    if (f.obstacle_source == "tilt_only" && !p.tilt_perception.enabled) {
      fatal(
        "output.front_obstacle_lowpass_stop.source=tilt_only 时 "
        "perception.tilt.enabled 必须为 true");
    }
    if (!(f.stale_prediction_max_s > 0.0)) {
      fatal("output.front_obstacle_lowpass_stop.stale_prediction_max_s 必须 > 0");
    }
    if (f.prediction_uncertainty_margin_m < 0.0) {
      fatal(
        "output.front_obstacle_lowpass_stop.prediction_uncertainty_margin_m 必须 >= 0");
    }
    if (!(f.trigger_distance_m > 0.0) || !(f.trigger_speed_m_s > 0.0)) {
      fatal("output.front_obstacle_lowpass_stop.trigger_distance_m / trigger_speed_m_s 必须 > 0");
    }
    if (!(f.min_time_constant_s > 0.0) ||
      f.filter_time_constant_s < f.min_time_constant_s)
    {
      fatal(fmt(
          "output.front_obstacle_lowpass_stop.filter_time_constant_s 必须 >= "
          "min_time_constant_s > 0（动态 τ 的取值区间）",
          f.filter_time_constant_s, f.min_time_constant_s));
    }
    if (f.release_distance_m <= f.trigger_distance_m) {
      fatal(fmt(
          "output.front_obstacle_lowpass_stop.release_distance_m 必须 > trigger_distance_m"
          "（释放滞回，否则锁存在阈值附近抖动）",
          f.release_distance_m, f.trigger_distance_m));
    }
    if (f.stop_buffer_m < 0.0 || f.stop_buffer_m >= f.trigger_distance_m) {
      fatal(fmt(
          "output.front_obstacle_lowpass_stop.stop_buffer_m 必须 ∈ [0, trigger_distance_m)"
          "（否则触发瞬间反算余量即为负，τ 恒被钳到下限）",
          f.stop_buffer_m, f.trigger_distance_m));
    }
    if (f.confirm_frames < 1) {
      fatal("output.front_obstacle_lowpass_stop.confirm_frames 必须 >= 1");
    }
    if (f.clear_hold_s < 0.0) {
      fatal("output.front_obstacle_lowpass_stop.clear_hold_s 必须 >= 0");
    }
    // 低通滑行距离闭式解 d = τ·v。动态 τ 会按剩余余量收紧到 (净空−stop_buffer)/v，
    // 但收紧有下限 min_time_constant_s。若在刚好触发的边界上，所需 τ 已经低于该下限，
    // 说明连最急的低通都停不进预留余量——触发瞬间就注定吃掉 stop_buffer。
    // 注意 trigger_speed_m_s 只是触发下限，实际入场更快时靠 emergency_brake 门控让位给急停。
    const double budget = f.trigger_distance_m - f.stop_buffer_m;
    const double tau_needed = budget / std::max(1e-3, f.trigger_speed_m_s);
    if (tau_needed < f.min_time_constant_s) {
      warn(fmt(
          "output.front_obstacle_lowpass_stop：触发边界所需时间常数已低于 min_time_constant_s，"
          "低通停不进 trigger_distance_m - stop_buffer_m 的余量；"
          "建议调低 trigger_speed_m_s / stop_buffer_m，或调高 trigger_distance_m",
          tau_needed, f.min_time_constant_s));
    }
    if (f.trigger_distance_m <= p.avoidance.hard_stop_distance_m) {
      warn(fmt(
          "output.front_obstacle_lowpass_stop.trigger_distance_m 未超出 "
          "avoidance.hard_stop_distance_m，触发区已落在急停区内，本层几乎不会生效"
          "（急停帧不接管）",
          f.trigger_distance_m, p.avoidance.hard_stop_distance_m));
    }
    if (f.stop_buffer_m < p.avoidance.hard_stop_distance_m) {
      warn(fmt(
          "output.front_obstacle_lowpass_stop.stop_buffer_m 小于 "
          "avoidance.hard_stop_distance_m：低通瞄准点在硬停线之内，急停会先于本层触发",
          f.stop_buffer_m, p.avoidance.hard_stop_distance_m));
    }
    if (f.standstill_release_speed_eps_m_s < 0.0) {
      fatal("output.front_obstacle_lowpass_stop.standstill_release_speed_eps_m_s 必须 >= 0");
    }
  }

  // ── 一致性类（把 yaml 注释里的约束代码化，方案 9.1） ──
  if (p.avoidance.hard_stop_release_distance_m < p.avoidance.hard_stop_distance_m) {
    fatal(fmt(
        "avoidance.hard_stop_release_distance_m 必须 >= hard_stop_distance_m（释放滞回）",
        p.avoidance.hard_stop_release_distance_m, p.avoidance.hard_stop_distance_m));
  }
  if (p.narrow_passage.hard_stop_release_distance_m <
    p.narrow_passage.hard_stop_distance_m)
  {
    fatal(fmt(
        "mode.narrow.hard_stop_release_distance_m 必须 >= hard_stop_distance_m",
        p.narrow_passage.hard_stop_release_distance_m,
        p.narrow_passage.hard_stop_distance_m));
  }
  if (p.avoidance.braking_envelope_enabled) {
    const double full_speed_stop = validatorStoppingDistance(
      p.avoidance.absolute_max_linear_speed,
      p.avoidance.braking_comfort_decel_m_s2,
      p.avoidance.braking_max_jerk_m_s3,
      p.avoidance.braking_reaction_delay_s);
    if (p.avoidance.braking_lookahead_m < full_speed_stop) {
      fatal(fmt(
          "avoidance.braking_lookahead_m 未覆盖最高速完整停车距离，"
          "高速接近时减速区不足", p.avoidance.braking_lookahead_m, full_speed_stop));
    }
  }
  if (p.avoidance.direction_mask_enabled &&
    p.avoidance.direction_mask_probe_distance_m < p.avoidance.standstill_guard_band_m)
  {
    fatal(fmt(
        "direction_mask.probe_distance_m 必须 >= standstill_guard.band_m（覆盖看护带内近障）",
        p.avoidance.direction_mask_probe_distance_m,
        p.avoidance.standstill_guard_band_m));
  }
  if (p.avoidance.direction_mask_enabled &&
    p.avoidance.direction_mask_map_exempt_radius_m <
    p.local_map.inflation_radius_m + p.local_map.resolution_m)
  {
    fatal(fmt(
        "direction_mask.map_exempt_radius_m 必须 >= 地图膨胀半径 + 分辨率"
        "（豁免圆须覆盖近障膨胀栅格）",
        p.avoidance.direction_mask_map_exempt_radius_m,
        p.local_map.inflation_radius_m + p.local_map.resolution_m));
  }
  if (p.avoidance.precise_stop.enabled &&
    p.avoidance.precise_stop.stop_tolerance_m <
    p.avoidance.braking_at_stop_distance_eps_m)
  {
    fatal(fmt(
        "precise_stop.stop_tolerance_m 必须 >= braking_at_stop_distance_eps_m",
        p.avoidance.precise_stop.stop_tolerance_m,
        p.avoidance.braking_at_stop_distance_eps_m));
  }
  if (p.avoidance.forward_stop_hold_enabled &&
    p.avoidance.forward_stop_hold_release_distance_m <
    p.avoidance.hard_stop_release_distance_m)
  {
    fatal(fmt(
        "forward_stop_hold.release_distance_m 必须 >= hard_stop_release_distance_m",
        p.avoidance.forward_stop_hold_release_distance_m,
        p.avoidance.hard_stop_release_distance_m));
  }
  if (p.avoidance.footprint_use_dual_trapezoid &&
    std::abs(
      p.avoidance.footprint_rear_segment.width_front_m -
      p.avoidance.footprint_front_segment.width_rear_m) > 1e-6)
  {
    fatal(fmt(
        "双梯形 footprint 分节线宽度不连续：rear_segment.width_front_m 须等于 "
        "front_segment.width_rear_m",
        p.avoidance.footprint_rear_segment.width_front_m,
        p.avoidance.footprint_front_segment.width_rear_m));
  }
  // 方案 8 预裁剪带宽须覆盖一切净空判据阈值，否则会改变控制决策
  if (p.avoidance.swept_cull_band_m > 0.0) {
    const double max_decision_clearance = std::max(
      {p.avoidance.hard_stop_release_distance_m,
        p.avoidance.comfort_clearance,
        p.avoidance.shared_control.alpha_clearance_zero_m,
        p.avoidance.turn_clearance_guard_enabled ?
        p.avoidance.turn_clearance_guard_band_m : 0.0,
        p.avoidance.standstill_guard_release_m});
    if (p.avoidance.swept_cull_band_m < max_decision_clearance) {
      fatal(fmt(
          "avoidance.swept_cull_band_m 小于最大净空判据阈值，预裁剪会改变控制决策",
          p.avoidance.swept_cull_band_m, max_decision_clearance));
    }
  }

  // ── WARN 类 ──
  if (p.local_map.inflation_radius_m > 0.0 &&
    p.local_map.inflation_radius_m < p.local_map.resolution_m)
  {
    warn(fmt(
        "local_map.inflation_radius_m < resolution_m：膨胀退化为占据格自身拷贝",
        p.local_map.inflation_radius_m, p.local_map.resolution_m));
  }
  if (p.local_map.ego_motion_compensation && p.local_map.rebuild_each_frame) {
    warn("local_map.ego_motion_compensation=true 但 rebuild_each_frame=true：补偿无效果"
      "（每帧重建天然无错位），启用跨帧记忆需同时置 rebuild_each_frame=false");
  }
  if (!p.local_map.rebuild_each_frame && !p.local_map.ego_motion_compensation) {
    warn("rebuild_each_frame=false 且未开启自运动补偿：车动后历史栅格会整体错位，"
      "跨帧记忆几何不正确（见 PRODUCTION_HARDENING_PLAN.md 方案 6）");
  }
  if (p.avoidance.sudden_obstacle.enabled && p.avoidance.sudden_obstacle.shadow_mode) {
    warn("sudden_obstacle 处于影子模式：只记日志不实际急停（灰度期正常，转正须置 shadow_mode=false）");
  }
  if (p.avoidance.pinned_escape.enabled && p.avoidance.pinned_escape.shadow_mode) {
    warn("pinned_escape 处于影子模式：只记日志不放行脱困（灰度期正常）");
  }
  if (p.avoidance.rapid_steer_cap_enabled) {
    if (p.avoidance.rapid_steer_trigger_angular_rad_s <= 0.0) {
      warn("rapid_steer_cap.enabled 但 trigger_angular_rad_s ≤0（未配置），本层不会介入任何转向");
    }
    if (p.avoidance.rapid_steer_max_linear_m_s <= 0.0) {
      warn("rapid_steer_cap.max_linear_m_s ≤0：急速转向期间线速度将被钳为 0（只能原地转）");
    }
  }
  if (p.avoidance.rotation_escape.enabled) {
    if (p.avoidance.rotation_escape.max_angular_rad_s <= 0.0 ||
      p.avoidance.rotation_escape.probe_angle_rad <= 0.0)
    {
      warn("rotation_escape.enabled 但 max_angular_rad_s / probe_angle_rad ≤0，本层实际不会放行任何转向");
    }
    if (p.avoidance.rotation_escape.probe_step_rad < 0.0)
    {
      warn("rotation_escape.probe_step_rad <0：将回退使用 direction_mask_probe_step_rad");
    }
    warn("rotation_escape 已启用：急停全停帧下会放行「可证明不恶化」的纯原地转向脱困（上车前须完成 bag 组合回放验证）");
  }
  if (p.avoidance.front_tilt_speed_cap.enabled) {
    if (!(p.avoidance.front_tilt_speed_cap.speed_cap_m_s > 0.0)) {
      warn("front_tilt_speed_cap.speed_cap_m_s ≤0：前向探到障碍时线速度将被钳为 0（等价刹停，非软限速）");
    }
    if (p.avoidance.front_tilt_speed_cap.release_distance_m <=
      p.avoidance.front_tilt_speed_cap.trigger_distance_m)
    {
      warn("front_tilt_speed_cap.release_distance_m ≤ trigger_distance_m：滞回带非正，限速会在触发边界反复启停");
    }
    if (p.avoidance.front_tilt_speed_cap.corridor_margin_m < 0.0) {
      warn("front_tilt_speed_cap.corridor_margin_m <0：将按 0 处理（走廊退化为车体半宽）");
    }
    warn("front_tilt_speed_cap 已启用：斜雷达前向走廊探到障碍将把前向速度上限钳到 speed_cap_m_s（上车前须回放确认斜雷达在该距离覆盖可靠）");
  }

  // ── 对外上报（reporting:） ──
  if (p.reporting.auxiliary_status_enabled && !(p.reporting.auxiliary_status_rate_hz > 0.0)) {
    fatal(fmt(
        "reporting.auxiliary_status.rate_hz 必须 > 0",
        p.reporting.auxiliary_status_rate_hz, 0.0));
  }
  if (!(p.cruise.obstacle_zero_speed_eps_m_s >= 0.0)) {
    fatal(fmt(
        "cruise.obstacle_zero_speed_eps_m_s 必须 >= 0",
        p.cruise.obstacle_zero_speed_eps_m_s, 0.0));
  }
  if (p.reporting.collision_warning.enabled &&
    !(p.reporting.collision_warning.repeat_interval_s > 0.0))
  {
    warn(fmt(
        "reporting.collision_warning.repeat_interval_s ≤0：同向同级预警每个控制周期都会重发",
        p.reporting.collision_warning.repeat_interval_s, 0.0));
  }

  // ── 跨参数依赖校验（参数间隐式关系，调参时容易遗漏）──

  // 依赖 1：tilt_ttc.hard_stop_distance_m 与 avoidance.hard_stop_distance_m
  // 两者默认值恰好都是 0.12，但分属不同子结构体，调参时若只改一个会导致
  // 前向与斜雷达硬停线不一致。
  if (p.avoidance.tilt_ttc.enabled &&
    std::abs(p.avoidance.tilt_ttc.hard_stop_distance_m -
             p.avoidance.hard_stop_distance_m) > 1e-6)
  {
    warn(fmt(
        "tilt_ttc.hard_stop_distance_m 与 avoidance.hard_stop_distance_m 不一致："
        "前向与斜雷达硬停线不齐，确认是否有意（否则应设为相同值）",
        p.avoidance.tilt_ttc.hard_stop_distance_m,
        p.avoidance.hard_stop_distance_m));
  }

  // 依赖 2：standstill_guard_band_m 应 >= hard_stop_distance_m
  // 否则停稳看护的评估窗口落在硬停区内，看护永远在急停之后才介入（逻辑冗余）。
  if (p.avoidance.standstill_guard_enabled &&
    p.avoidance.standstill_guard_band_m > 0.0 &&
    p.avoidance.standstill_guard_band_m < p.avoidance.hard_stop_distance_m)
  {
    warn(fmt(
        "standstill_guard.band_m < hard_stop_distance_m：停稳看护窗口落在硬停区内，"
        "看护介入时急停已触发（逻辑冗余），建议 band_m >= hard_stop_distance_m",
        p.avoidance.standstill_guard_band_m,
        p.avoidance.hard_stop_distance_m));
  }

  // 依赖 3：sudden_obstacle.emergency_decel 应 > braking_comfort_decel
  // 两者接近时突发障碍检测层与制动包络层功能重叠。
  if (p.avoidance.sudden_obstacle.enabled &&
    p.avoidance.sudden_obstacle.emergency_decel_m_s2 <=
      p.avoidance.braking_comfort_decel_m_s2)
  {
    warn(fmt(
        "sudden_obstacle.emergency_decel_m_s2 <= braking_comfort_decel_m_s2："
        "突发障碍检测层与制动包络层功能重叠，emergency_decel 应显著大于 comfort_decel",
        p.avoidance.sudden_obstacle.emergency_decel_m_s2,
        p.avoidance.braking_comfort_decel_m_s2));
  }

  // 依赖 4：pinned_escape.creep_linear_m_s 应 <= recovery_max_linear_speed
  // 否则脱困候选会被钳位导致行为异常。
  if (p.avoidance.pinned_escape.enabled &&
    p.avoidance.pinned_escape.creep_linear_m_s >
      p.avoidance.recovery_max_linear_speed)
  {
    warn(fmt(
        "pinned_escape.creep_linear_m_s > recovery_max_linear_speed："
        "脱困蠕行速度会被脱困最大速度钳位，行为异常",
        p.avoidance.pinned_escape.creep_linear_m_s,
        p.avoidance.recovery_max_linear_speed));
  }

  // 依赖 5：pinned_escape.escape_margin_m 实际取 max(本值, hard_stop_distance_m)
  // 调参者若不知此取 max 逻辑，可能误以为设 0.03 就是实际地板。
  if (p.avoidance.pinned_escape.enabled &&
    p.avoidance.pinned_escape.escape_margin_m <
      p.avoidance.hard_stop_distance_m)
  {
    warn(fmt(
        "pinned_escape.escape_margin_m < hard_stop_distance_m："
        "实际地板会被提升到 hard_stop_distance_m，设的值不生效",
        p.avoidance.pinned_escape.escape_margin_m,
        p.avoidance.hard_stop_distance_m));
  }

  // 依赖 6：turn_envelope_edge_radius_m 应与 footprint 几何尺寸匹配
  // 注释说"默认取角点 sqrt((L/2)²+(W/2)²)"，但改 footprint 后此值不会自动更新。
  {
    const double half_len =
      (p.avoidance.footprint_front_margin +
       p.avoidance.footprint_back_margin) * 0.5;
    const double half_wid = p.avoidance.footprint_side_margin;
    const double expected_radius = std::sqrt(half_len * half_len + half_wid * half_wid);
    if (std::abs(p.avoidance.turn_envelope_edge_radius_m - expected_radius) > 0.05)
    {
      warn(fmt(
          "turn_envelope_edge_radius_m 与 footprint 几何推算值偏差 > 5cm："
          "修改 footprint 尺寸后须同步更新此参数",
          p.avoidance.turn_envelope_edge_radius_m, expected_radius));
    }
  }

  // 依赖 7：tilt_ttc.max_probe_distance_m 应 >= tilt_ttc.hard_stop_distance_m
  // 否则斜雷达探针长度不足以覆盖硬停区，TTC 评估在硬停区内无数据。
  if (p.avoidance.tilt_ttc.enabled &&
    p.avoidance.tilt_ttc.max_probe_distance_m <
      p.avoidance.tilt_ttc.hard_stop_distance_m)
  {
    warn(fmt(
        "tilt_ttc.max_probe_distance_m < hard_stop_distance_m："
        "斜雷达探针长度不足以覆盖硬停区，TTC 评估在硬停区内无数据",
        p.avoidance.tilt_ttc.max_probe_distance_m,
        p.avoidance.tilt_ttc.hard_stop_distance_m));
  }

  // 依赖 8：narrow_passage_planner.roi_half_width_m 应 <= 感知 ROI y 范围
  // 否则窄道提取会处理被全局 ROI 裁掉的点（感知 ROI 参数化后此处可精确校验）。
  if (p.avoidance.narrow_planner.enabled &&
    p.avoidance.narrow_planner.roi_half_width_m > p.perception.y_max_m)
  {
    warn(fmt(
        "narrow_passage_planner.roi_half_width_m > perception.roi.y_max_m："
        "窄道 ROI 超出全局感知 ROI，窄道提取会处理不存在的点",
        p.avoidance.narrow_planner.roi_half_width_m,
        p.perception.y_max_m));
  }

  // 依赖 9：recovery_min_command 与 forward_stop_hold_feedback_speed_m_s
  // 两者定义了"驾驶员在给令"和"已停稳"的门控，若差距过小会导致脱困启动与停稳保持不一致。
  if (p.avoidance.forward_stop_hold_enabled &&
    p.avoidance.forward_stop_hold_feedback_speed_m_s <
      p.avoidance.recovery_min_command)
  {
    warn(fmt(
        "forward_stop_hold_feedback_speed_m_s < recovery_min_command："
        "停稳保持门控低于脱困启动门控，调参时注意两者一致性",
        p.avoidance.forward_stop_hold_feedback_speed_m_s,
        p.avoidance.recovery_min_command));
  }

  // 依赖 10：direction_mask_approach_eps_m 应 <= direction_mask_contact_buffer_m
  // approach_eps 用于判定"是否在逼近"，contact_buffer 用于判定"是否在接触缓冲内"，
  // approach_eps > contact_buffer 会导致"已判定逼近但不在缓冲内"的逻辑矛盾。
  if (p.avoidance.direction_mask_enabled &&
    p.avoidance.direction_mask_approach_eps_m >
      p.avoidance.direction_mask_contact_buffer_m)
  {
    warn(fmt(
        "direction_mask_approach_eps_m > contact_buffer_m："
        "逼近判定门限大于接触缓冲，可能导致逻辑矛盾",
        p.avoidance.direction_mask_approach_eps_m,
        p.avoidance.direction_mask_contact_buffer_m));
  }

  return issues;
}

void validateParamsOrThrow(const SafetyNodeParams & params, const rclcpp::Logger & logger)
{
  const auto issues = validateParams(params);
  bool has_fatal = false;
  std::ostringstream fatal_msgs;
  for (const auto & issue : issues) {
    if (issue.level == ParamIssue::FATAL) {
      has_fatal = true;
      RCLCPP_ERROR(logger, "[参数校验][FATAL] %s", issue.message.c_str());
      fatal_msgs << "\n  - " << issue.message;
    } else {
      RCLCPP_WARN(logger, "[参数校验][WARN] %s", issue.message.c_str());
    }
  }
  if (has_fatal) {
    throw std::runtime_error("safety_node 参数校验失败，拒绝启动:" + fatal_msgs.str());
  }
}
