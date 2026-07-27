/**
 * @file decision_maker_params_io.cpp
 * @brief DecisionParams ↔ ROS2 参数映射实现
 */

#include "decision_maker_params_io.h"

namespace {

bool try_apply_one_parameter(DecisionParams& t, const rclcpp::Parameter& param, bool& known, std::string& err)
{
    known = false;
    err.clear();
    const std::string& n = param.get_name();

    auto set_double = [&](double& field) -> bool {
        try {
            field = param.as_double();
            return true;
        } catch (const rclcpp::ParameterTypeException& e) {
            err = std::string("参数 ") + n + " 需要浮点类型: " + e.what();
            return false;
        }
    };
    auto set_int_sz = [&](size_t& field) -> bool {
        try {
            const int v = param.as_int();
            if (v < 1) {
                err = "参数 " + n + " 须 >= 1";
                return false;
            }
            field = static_cast<size_t>(v);
            return true;
        } catch (const rclcpp::ParameterTypeException& e) {
            err = std::string("参数 ") + n + " 需要整数类型: " + e.what();
            return false;
        }
    };
    auto set_bool = [&](bool& field) -> bool {
        try {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                field = param.as_bool();
                return true;
            }
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                field = (param.as_int() != 0);
                return true;
            }
            err = "参数 " + n + " 需要布尔或整型";
            return false;
        } catch (const rclcpp::ParameterTypeException& e) {
            err = std::string("参数 ") + n + ": " + e.what();
            return false;
        }
    };

#define DP_PARAM(NAME)                                                                                                 \
    if (n == #NAME) {                                                                                                  \
        known = true;                                                                                                  \
        return set_double(t.NAME);                                                                                     \
    }

    DP_PARAM(radar_frequency);
    DP_PARAM(prediction_time);
    DP_PARAM(robot_length);
    DP_PARAM(robot_width);
    DP_PARAM(robot_height);
    DP_PARAM(robot_safety_margin_front);
    DP_PARAM(robot_safety_margin_back);
    DP_PARAM(robot_safety_margin_side);
    DP_PARAM(warning_zone);
    DP_PARAM(deceleration_zone);
    DP_PARAM(emergency_brake_distance);
    DP_PARAM(fixed_stopping_distance_forward);
    DP_PARAM(warning_ttc);
    DP_PARAM(deceleration_ttc);
    DP_PARAM(emergency_brake_ttc);
    DP_PARAM(forward_ttc_high_speed_threshold_m_s);
    DP_PARAM(deceleration_ttc_high_speed);
    DP_PARAM(emergency_brake_ttc_high_speed);
    DP_PARAM(emergency_brake_ttc_decel);
    DP_PARAM(emergency_brake_ttc_base);
    DP_PARAM(emergency_brake_ttc_margin);
    DP_PARAM(min_approach_velocity_for_ttc);
    DP_PARAM(foot_distance_to_radar);
    DP_PARAM(radar_distance_to_vehicle_rear);
    DP_PARAM(forward_brake_joystick_y_deadzone);
    DP_PARAM(forward_brake_chassis_speed_threshold);
    DP_PARAM(forward_brake_zero_speed_window_ms);
    DP_PARAM(forward_brake_zero_speed_threshold_kmh);
    DP_PARAM(smooth_comfort_max_linear_delta);
    DP_PARAM(smooth_comfort_max_angular_delta);
    DP_PARAM(smooth_comfort_max_linear_accel_m_s2);
    DP_PARAM(smooth_sport_max_linear_delta);
    DP_PARAM(smooth_sport_max_angular_delta);
    DP_PARAM(smooth_sport_max_linear_accel_m_s2);
    DP_PARAM(lateral_dominance_ratio);
    DP_PARAM(warning_zone_lateral);
    DP_PARAM(deceleration_zone_lateral);
    DP_PARAM(emergency_brake_distance_lateral);
    DP_PARAM(fixed_stopping_distance_lateral);
    DP_PARAM(warning_ttc_lateral);
    DP_PARAM(deceleration_ttc_lateral);
    DP_PARAM(emergency_brake_ttc_lateral);
    DP_PARAM(warning_zone_backward);
    DP_PARAM(deceleration_zone_backward);
    DP_PARAM(emergency_brake_distance_backward);
    DP_PARAM(fixed_stopping_distance_backward);
    DP_PARAM(warning_ttc_backward);
    DP_PARAM(deceleration_ttc_backward);
    DP_PARAM(emergency_brake_ttc_backward);
    DP_PARAM(warning_distance_backward);
    DP_PARAM(backward_brake_decel_max_m_s2);
    DP_PARAM(backward_recover_accel_max_m_s2);
    DP_PARAM(backward_emergency_creep_speed);
    DP_PARAM(backward_emergency_hold_ms);
    DP_PARAM(min_speed_limit);
    DP_PARAM(max_speed_limit);
    DP_PARAM(max_speed_limit_backward);
    DP_PARAM(absolute_max_linear_speed);
    DP_PARAM(absolute_max_angular_speed);
    DP_PARAM(state_hold_time);
    DP_PARAM(speed_recovery_time);
    DP_PARAM(radar_timeout);
    DP_PARAM(obstacle_prediction_max_age);
    DP_PARAM(obstacle_prediction_drift_ratio);
    DP_PARAM(obstacle_prediction_min_valid_x);
    DP_PARAM(evasion_angular_threshold);
    DP_PARAM(evasion_angular_full);
    DP_PARAM(evasion_min_factor);
#undef DP_PARAM

    if (n == "max_history_size") {
        known = true;
        return set_int_sz(t.max_history_size);
    }
    if (n == "enable_forward_collision") {
        known = true;
        return set_bool(t.enable_forward_collision);
    }
    if (n == "enable_lateral_collision") {
        known = true;
        return set_bool(t.enable_lateral_collision);
    }
    if (n == "enable_backward_collision") {
        known = true;
        return set_bool(t.enable_backward_collision);
    }
    if (n == "enable_forward_brake_joystick_y_deadzone") {
        known = true;
        return set_bool(t.enable_forward_brake_joystick_y_deadzone);
    }
    if (n == "enable_obstacle_prediction") {
        known = true;
        return set_bool(t.enable_obstacle_prediction);
    }
    if (n == "enable_evasion_detection") {
        known = true;
        return set_bool(t.enable_evasion_detection);
    }

    return true;
}

} // namespace

namespace decision_maker_params_io {

bool validate_decision_params(const DecisionParams& p, std::string& reason)
{
    if (p.robot_length <= 0.0 || p.robot_width <= 0.0 || p.robot_height <= 0.0) {
        reason = "robot_length、robot_width、robot_height 须为正数";
        return false;
    }
    if (p.robot_safety_margin_front < 0.0 || p.robot_safety_margin_back < 0.0 || p.robot_safety_margin_side < 0.0) {
        reason = "robot_safety_margin_front/back/side 不可为负";
        return false;
    }
    if (!(p.emergency_brake_distance < p.deceleration_zone && p.deceleration_zone < p.warning_zone)) {
        reason = "前向需满足: emergency_brake_distance < deceleration_zone < warning_zone";
        return false;
    }
    if (!(p.emergency_brake_distance_lateral < p.deceleration_zone_lateral
            && p.deceleration_zone_lateral < p.warning_zone_lateral)) {
        reason = "侧向需满足: emergency_brake_distance_lateral < deceleration_zone_lateral < warning_zone_lateral";
        return false;
    }
    if (!(p.emergency_brake_distance_backward < p.deceleration_zone_backward
            && p.deceleration_zone_backward < p.warning_zone_backward)) {
        reason = "后向需满足: emergency_brake_distance_backward < deceleration_zone_backward < warning_zone_backward";
        return false;
    }
    if (p.fixed_stopping_distance_forward >= p.emergency_brake_distance) {
        reason = "fixed_stopping_distance_forward 应小于 emergency_brake_distance";
        return false;
    }
    if (p.min_speed_limit >= p.max_speed_limit) {
        reason = "min_speed_limit 应小于 max_speed_limit";
        return false;
    }
    if (p.max_speed_limit > p.absolute_max_linear_speed) {
        reason = "max_speed_limit 不应大于 absolute_max_linear_speed";
        return false;
    }
    if (p.state_hold_time <= 0.0 || p.speed_recovery_time <= 0.0 || p.radar_timeout <= 0.0) {
        reason = "state_hold_time、speed_recovery_time、radar_timeout 须为正";
        return false;
    }
    if (p.max_history_size < 1) {
        reason = "max_history_size 至少为 1";
        return false;
    }
    if (p.deceleration_ttc <= p.emergency_brake_ttc) {
        reason = "deceleration_ttc 应大于 emergency_brake_ttc";
        return false;
    }
    if (p.deceleration_ttc_high_speed <= p.emergency_brake_ttc_high_speed) {
        reason = "deceleration_ttc_high_speed 应大于 emergency_brake_ttc_high_speed";
        return false;
    }
    if (p.forward_ttc_high_speed_threshold_m_s < 0.0) {
        reason = "forward_ttc_high_speed_threshold_m_s 不可为负（0 表示不启用高速 TTC 剖面）";
        return false;
    }
    if (p.deceleration_ttc_lateral <= p.emergency_brake_ttc_lateral) {
        reason = "deceleration_ttc_lateral 应大于 emergency_brake_ttc_lateral";
        return false;
    }
    if (p.deceleration_ttc_backward <= p.emergency_brake_ttc_backward) {
        reason = "deceleration_ttc_backward 应大于 emergency_brake_ttc_backward";
        return false;
    }
    if (p.backward_brake_decel_max_m_s2 <= 0.0) {
        reason = "backward_brake_decel_max_m_s2 须为正";
        return false;
    }
    if (p.backward_recover_accel_max_m_s2 <= 0.0) {
        reason = "backward_recover_accel_max_m_s2 须为正";
        return false;
    }
    if (p.backward_emergency_creep_speed < 0.0) {
        reason = "backward_emergency_creep_speed 不可为负";
        return false;
    }
    if (p.backward_emergency_hold_ms < 0.0) {
        reason = "backward_emergency_hold_ms 不可为负";
        return false;
    }
    if (p.forward_brake_joystick_y_deadzone < 0.0) {
        reason = "forward_brake_joystick_y_deadzone 不可为负（<=0 表示关闭角速度置零逻辑）";
        return false;
    }
    if (p.forward_brake_chassis_speed_threshold < 0.0) {
        reason = "forward_brake_chassis_speed_threshold 不可为负";
        return false;
    }
    if (p.forward_brake_zero_speed_window_ms < 0.0) {
        reason = "forward_brake_zero_speed_window_ms 不可为负";
        return false;
    }
    if (p.forward_brake_zero_speed_threshold_kmh < 0.0) {
        reason = "forward_brake_zero_speed_threshold_kmh 不可为负";
        return false;
    }
    if (p.smooth_comfort_max_linear_delta <= 0.0 || p.smooth_comfort_max_angular_delta <= 0.0) {
        reason = "smooth_comfort_max_linear_delta / smooth_comfort_max_angular_delta 须为正";
        return false;
    }
    if (p.smooth_sport_max_linear_delta <= 0.0 || p.smooth_sport_max_angular_delta <= 0.0) {
        reason = "smooth_sport_max_linear_delta / smooth_sport_max_angular_delta 须为正";
        return false;
    }
    if (p.smooth_comfort_max_linear_accel_m_s2 < 0.0 || p.smooth_sport_max_linear_accel_m_s2 < 0.0) {
        reason = "smooth_*_max_linear_accel_m_s2 不可为负（0 表示不按加速度额外收紧，仅按每周期 delta）";
        return false;
    }
    if (p.evasion_angular_full <= p.evasion_angular_threshold) {
        reason = "evasion_angular_full 须大于 evasion_angular_threshold";
        return false;
    }
    if (p.evasion_min_factor < 0.0 || p.evasion_min_factor > 1.0) {
        reason = "evasion_min_factor 须在 [0, 1] 内";
        return false;
    }
    return true;
}

void declare_and_load_decision_params(rclcpp::Node& node, DecisionParams& p)
{
    p.radar_frequency = node.declare_parameter("radar_frequency", p.radar_frequency);
    p.prediction_time = node.declare_parameter("prediction_time", p.prediction_time);
    p.robot_length = node.declare_parameter("robot_length", p.robot_length);
    p.robot_width = node.declare_parameter("robot_width", p.robot_width);
    p.robot_height = node.declare_parameter("robot_height", p.robot_height);
    p.robot_safety_margin_front = node.declare_parameter("robot_safety_margin_front", p.robot_safety_margin_front);
    p.robot_safety_margin_back = node.declare_parameter("robot_safety_margin_back", p.robot_safety_margin_back);
    p.robot_safety_margin_side = node.declare_parameter("robot_safety_margin_side", p.robot_safety_margin_side);
    p.warning_zone = node.declare_parameter("warning_zone", p.warning_zone);
    p.deceleration_zone = node.declare_parameter("deceleration_zone", p.deceleration_zone);
    p.emergency_brake_distance = node.declare_parameter("emergency_brake_distance", p.emergency_brake_distance);
    p.fixed_stopping_distance_forward = node.declare_parameter("fixed_stopping_distance_forward", p.fixed_stopping_distance_forward);
    p.warning_ttc = node.declare_parameter("warning_ttc", p.warning_ttc);
    p.deceleration_ttc = node.declare_parameter("deceleration_ttc", p.deceleration_ttc);
    p.emergency_brake_ttc = node.declare_parameter("emergency_brake_ttc", p.emergency_brake_ttc);
    p.forward_ttc_high_speed_threshold_m_s = node.declare_parameter("forward_ttc_high_speed_threshold_m_s", p.forward_ttc_high_speed_threshold_m_s);
    p.deceleration_ttc_high_speed = node.declare_parameter("deceleration_ttc_high_speed", p.deceleration_ttc);
    p.emergency_brake_ttc_high_speed = node.declare_parameter("emergency_brake_ttc_high_speed", p.emergency_brake_ttc);
    p.emergency_brake_ttc_decel = node.declare_parameter("emergency_brake_ttc_decel", p.emergency_brake_ttc_decel);
    p.emergency_brake_ttc_base = node.declare_parameter("emergency_brake_ttc_base", p.emergency_brake_ttc_base);
    p.emergency_brake_ttc_margin = node.declare_parameter("emergency_brake_ttc_margin", p.emergency_brake_ttc_margin);
    p.min_approach_velocity_for_ttc = node.declare_parameter("min_approach_velocity_for_ttc", p.min_approach_velocity_for_ttc);
    p.foot_distance_to_radar = node.declare_parameter("foot_distance_to_radar", p.foot_distance_to_radar);
    p.radar_distance_to_vehicle_rear = node.declare_parameter("radar_distance_to_vehicle_rear", p.radar_distance_to_vehicle_rear);
    p.forward_brake_joystick_y_deadzone = node.declare_parameter("forward_brake_joystick_y_deadzone", p.forward_brake_joystick_y_deadzone);
    p.forward_brake_chassis_speed_threshold = node.declare_parameter("forward_brake_chassis_speed_threshold", p.forward_brake_chassis_speed_threshold);
    p.forward_brake_zero_speed_window_ms = node.declare_parameter("forward_brake_zero_speed_window_ms", p.forward_brake_zero_speed_window_ms);
    p.forward_brake_zero_speed_threshold_kmh = node.declare_parameter(
        "forward_brake_zero_speed_threshold_kmh", p.forward_brake_zero_speed_threshold_kmh);
    p.smooth_comfort_max_linear_delta = node.declare_parameter("smooth_comfort_max_linear_delta", p.smooth_comfort_max_linear_delta);
    p.smooth_comfort_max_angular_delta = node.declare_parameter("smooth_comfort_max_angular_delta", p.smooth_comfort_max_angular_delta);
    p.smooth_comfort_max_linear_accel_m_s2 = node.declare_parameter("smooth_comfort_max_linear_accel_m_s2", p.smooth_comfort_max_linear_accel_m_s2);
    p.smooth_sport_max_linear_delta = node.declare_parameter("smooth_sport_max_linear_delta", p.smooth_sport_max_linear_delta);
    p.smooth_sport_max_angular_delta = node.declare_parameter("smooth_sport_max_angular_delta", p.smooth_sport_max_angular_delta);
    p.smooth_sport_max_linear_accel_m_s2 = node.declare_parameter("smooth_sport_max_linear_accel_m_s2", p.smooth_sport_max_linear_accel_m_s2);
    p.lateral_dominance_ratio = node.declare_parameter("lateral_dominance_ratio", p.lateral_dominance_ratio);
    p.warning_zone_lateral = node.declare_parameter("warning_zone_lateral", p.warning_zone_lateral);
    p.deceleration_zone_lateral = node.declare_parameter("deceleration_zone_lateral", p.deceleration_zone_lateral);
    p.emergency_brake_distance_lateral = node.declare_parameter("emergency_brake_distance_lateral", p.emergency_brake_distance_lateral);
    p.fixed_stopping_distance_lateral = node.declare_parameter("fixed_stopping_distance_lateral", p.fixed_stopping_distance_lateral);
    p.warning_ttc_lateral = node.declare_parameter("warning_ttc_lateral", p.warning_ttc_lateral);
    p.deceleration_ttc_lateral = node.declare_parameter("deceleration_ttc_lateral", p.deceleration_ttc_lateral);
    p.emergency_brake_ttc_lateral = node.declare_parameter("emergency_brake_ttc_lateral", p.emergency_brake_ttc_lateral);
    p.warning_zone_backward = node.declare_parameter("warning_zone_backward", p.warning_zone_backward);
    p.deceleration_zone_backward = node.declare_parameter("deceleration_zone_backward", p.deceleration_zone_backward);
    p.emergency_brake_distance_backward = node.declare_parameter("emergency_brake_distance_backward", p.emergency_brake_distance_backward);
    p.fixed_stopping_distance_backward = node.declare_parameter("fixed_stopping_distance_backward", p.fixed_stopping_distance_backward);
    p.warning_ttc_backward = node.declare_parameter("warning_ttc_backward", p.warning_ttc_backward);
    p.deceleration_ttc_backward = node.declare_parameter("deceleration_ttc_backward", p.deceleration_ttc_backward);
    p.emergency_brake_ttc_backward = node.declare_parameter("emergency_brake_ttc_backward", p.emergency_brake_ttc_backward);
    p.warning_distance_backward = node.declare_parameter("warning_distance_backward", p.warning_distance_backward);
    p.backward_brake_decel_max_m_s2 = node.declare_parameter("backward_brake_decel_max_m_s2", p.backward_brake_decel_max_m_s2);
    p.backward_recover_accel_max_m_s2 = node.declare_parameter("backward_recover_accel_max_m_s2", p.backward_recover_accel_max_m_s2);
    p.backward_emergency_creep_speed = node.declare_parameter("backward_emergency_creep_speed", p.backward_emergency_creep_speed);
    p.backward_emergency_hold_ms = node.declare_parameter("backward_emergency_hold_ms", p.backward_emergency_hold_ms);
    p.min_speed_limit = node.declare_parameter("min_speed_limit", p.min_speed_limit);
    p.max_speed_limit = node.declare_parameter("max_speed_limit", p.max_speed_limit);
    p.max_speed_limit_backward = node.declare_parameter("max_speed_limit_backward", p.max_speed_limit_backward);
    p.absolute_max_linear_speed = node.declare_parameter("absolute_max_linear_speed", p.absolute_max_linear_speed);
    p.absolute_max_angular_speed = node.declare_parameter("absolute_max_angular_speed", p.absolute_max_angular_speed);
    p.state_hold_time = node.declare_parameter("state_hold_time", p.state_hold_time);
    p.speed_recovery_time = node.declare_parameter("speed_recovery_time", p.speed_recovery_time);
    p.radar_timeout = node.declare_parameter("radar_timeout", p.radar_timeout);
    p.max_history_size = static_cast<size_t>(node.declare_parameter<int64_t>("max_history_size", static_cast<int64_t>(p.max_history_size)));
    p.enable_forward_collision = node.declare_parameter("enable_forward_collision", p.enable_forward_collision);
    p.enable_lateral_collision = node.declare_parameter("enable_lateral_collision", p.enable_lateral_collision);
    p.enable_backward_collision = node.declare_parameter("enable_backward_collision", p.enable_backward_collision);
    p.enable_forward_brake_joystick_y_deadzone = node.declare_parameter(
        "enable_forward_brake_joystick_y_deadzone", p.enable_forward_brake_joystick_y_deadzone);

    p.enable_evasion_detection = node.declare_parameter("enable_evasion_detection", p.enable_evasion_detection);
    p.evasion_angular_threshold = node.declare_parameter("evasion_angular_threshold", p.evasion_angular_threshold);
    p.evasion_angular_full = node.declare_parameter("evasion_angular_full", p.evasion_angular_full);
    p.evasion_min_factor = node.declare_parameter("evasion_min_factor", p.evasion_min_factor);

    p.enable_obstacle_prediction = node.declare_parameter("enable_obstacle_prediction", p.enable_obstacle_prediction);
    p.obstacle_prediction_max_age = node.declare_parameter("obstacle_prediction_max_age", p.obstacle_prediction_max_age);
    p.obstacle_prediction_drift_ratio = node.declare_parameter("obstacle_prediction_drift_ratio", p.obstacle_prediction_drift_ratio);
    p.obstacle_prediction_min_valid_x = node.declare_parameter("obstacle_prediction_min_valid_x", p.obstacle_prediction_min_valid_x);

    std::string vr;
    if (!validate_decision_params(p, vr)) {
        RCLCPP_WARN(node.get_logger(), "决策参数校验未通过（已仍使用当前加载值）: %s", vr.c_str());
    }
}

rcl_interfaces::msg::SetParametersResult apply_ros_parameter_updates(
    DecisionParams& in_out, const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    DecisionParams next = in_out;

    for (const auto& param : parameters) {
        bool known = false;
        std::string err;
        if (!try_apply_one_parameter(next, param, known, err)) {
            result.successful = false;
            result.reason = err.empty() ? "参数应用失败" : err;
            return result;
        }
        (void)known;
    }

    std::string vr;
    if (!validate_decision_params(next, vr)) {
        result.successful = false;
        result.reason = vr;
        return result;
    }

    in_out = next;
    return result;
}

} // namespace decision_maker_params_io
