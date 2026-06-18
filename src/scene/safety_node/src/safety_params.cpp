/**
 * @file safety_params.cpp
 * @brief ROS 参数 → SafetyNodeParams 映射
 *
 * 参数键与 config/safety_node_params.yaml 一一对应，
 * 使用 declare_parameter + get_parameter 支持 launch 覆盖。
 */
#include "safety_node/safety_params.h"

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

  p.control_command_timeout_s = get_d("input_health.control_command_timeout_s", 0.25);
  p.joystick_timeout_s = get_d("input_health.joystick_timeout_s", 0.25);
  p.joystick_deadzone = get_i("input_health.joystick_deadzone", 5);
  p.wheel_feedback_timeout_s = get_d("input_health.wheel_feedback_timeout_s", 0.25);
  p.require_wheel_feedback = get_b("input_health.require_wheel_feedback", true);
  p.perception_timeout_s = get_d("input_health.perception_timeout_s", 0.35);
  p.tf_lookup_timeout_s = get_d("input_health.tf_lookup_timeout_s", 0.02);
  p.min_fresh_perception_sources = get_i("input_health.min_fresh_perception_sources", 1);
  p.require_perception = get_b("input_health.require_perception", true);
  p.min_fresh_horizontal_sources = get_i("input_health.min_fresh_horizontal_sources", 1);
  p.require_tilt_perception = get_b("input_health.require_tilt_perception", false);
  p.emergency_release_delay_s = get_d("output.emergency_release_delay_s", 0.15);
  p.neutral_stop_filter_time_constant_s =
    get_d("output.neutral_stop_filter_time_constant_s", 0.30);

  // ── Footprint 轨迹采样 ──
  p.tilt_perception.input_type = get_s("perception.tilt.input_type", "pointcloud2");
  p.tilt_perception.publish_debug_scan = get_b("perception.tilt.publish_debug_scan", true);
  p.tilt_perception.enabled = get_b("perception.tilt.enabled", true);
  p.tilt_perception.feed_local_map = get_b("perception.tilt.feed_local_map", false);
  p.tilt_perception.max_range_m = get_d("perception.tilt.max_range_m", 1.0);
  p.tilt_perception.min_forward_x_m = get_d("perception.tilt.min_forward_x_m", 0.0);
  p.tilt_perception.roi_x_min_m = get_d("perception.tilt.roi_x_min_m", -0.3);
  p.tilt_perception.roi_x_max_m = get_d("perception.tilt.roi_x_max_m", 1.0);
  p.tilt_perception.roi_y_abs_m = get_d("perception.tilt.roi_y_abs_m", 0.45);
  p.tilt_perception.z_filter_threshold =
    get_d("perception.tilt.z_filter_threshold", 0.0);
  p.tilt_perception.voxel_size_m = get_d("perception.tilt.voxel_size_m", 0.08);
  p.tilt_perception.max_primitives = get_i("perception.tilt.max_primitives", 48);

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
  p.avoidance.braking_lookahead_m =
    get_d("avoidance.braking_lookahead_m", 5.0);
  p.avoidance.braking_path_step_m =
    get_d("avoidance.braking_path_step_m", 0.025);
  p.avoidance.braking_reaction_delay_s =
    get_d("avoidance.braking_reaction_delay_s", 0.10);
  p.avoidance.braking_comfort_decel_m_s2 =
    get_d("avoidance.braking_comfort_decel_m_s2", 0.60);
  p.avoidance.braking_max_jerk_m_s3 =
    get_d("avoidance.braking_max_jerk_m_s3", 2.5);
  p.avoidance.braking_stop_buffer_m =
    get_d("avoidance.braking_stop_buffer_m", 0.02);
  p.avoidance.tilt_ttc.enabled = get_b("avoidance.tilt_ttc.enabled", true);
  p.avoidance.tilt_ttc.horizon_s = get_d("avoidance.tilt_ttc.horizon_s", 0.5);
  p.avoidance.tilt_ttc.max_probe_distance_m =
    get_d("avoidance.tilt_ttc.max_probe_distance_m", 1.0);
  p.avoidance.tilt_ttc.hard_stop_distance_m =
    get_d("avoidance.tilt_ttc.hard_stop_distance_m", 0.12);
  p.avoidance.tilt_ttc.limiting_distance_m =
    get_d("avoidance.tilt_ttc.limiting_distance_m", 0.35);
  p.avoidance.tilt_ttc.release_distance_m =
    get_d("avoidance.tilt_ttc.release_distance_m", 0.40);
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
  p.avoidance.candidate_speed_up_hold_s = get_d("avoidance.candidate_speed_up_hold_s", 0.30);
  p.avoidance.recovery_mode_enabled = get_b("avoidance.recovery_mode_enabled", true);
  p.avoidance.recovery_max_linear_speed =
    get_d("avoidance.recovery_max_linear_speed", 0.12);
  p.avoidance.recovery_max_angular_speed =
    get_d("avoidance.recovery_max_angular_speed", 0.35);
  p.avoidance.recovery_lookahead_s = get_d("avoidance.recovery_lookahead_s", 0.35);
  p.avoidance.recovery_min_command = get_d("avoidance.recovery_min_command", 0.02);
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
  p.local_map.tilt_z_threshold = get_d("local_map.tilt_z_threshold", 0.5);
  p.local_map.integrate_primitive_centers = get_b("local_map.integrate_primitives", false);
  p.local_map.primitive_observation_weight = get_d("local_map.primitive_observation_weight", 0.6);

  // ── Map 硬约束 ──
  p.map_constraint.enabled = get_b("map_constraint.enabled", true);
  p.map_constraint.veto_on_lethal = get_b("map_constraint.veto_on_lethal", true);
  p.map_constraint.check_footprint_corners = get_b("map_constraint.check_footprint_corners", true);
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

  p.default_mode = parseSafetyMode(get_s("mode.default", "narrow"));

  return p;
}
