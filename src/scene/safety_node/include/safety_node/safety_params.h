/**
 * @file safety_params.h
 * @brief 从 ROS2 参数服务器加载 SafetyNode 全部配置
 */
#pragma once

#include "safety_node/core/avoidance_types.h"

#include <rclcpp/rclcpp.hpp>

/** 节点级参数聚合 */
struct TiltPerceptionConfig
{
  /** pointcloud2: /perception/left_tilt/pointcloud; tilt_scan: /perception/left_tilt/scan */
  std::string input_type = "pointcloud2";
  bool publish_debug_scan = true;
  bool enabled = true;
  bool feed_local_map = false;
  double max_range_m = 1.0;
  double min_forward_x_m = 0.0;
  double roi_x_min_m = -0.3;
  double roi_x_max_m = 1.0;
  double roi_y_abs_m = 0.45;
  double z_filter_threshold = 0.0;
  double voxel_size_m = 0.08;
  int max_primitives = 48;
};

struct SafetyNodeParams
{
  double control_rate_hz = 20.0;
  double control_command_timeout_s = 0.25;
  double joystick_timeout_s = 0.25;
  int joystick_deadzone = 5;
  double wheel_feedback_timeout_s = 0.25;
  bool require_wheel_feedback = true;
  double perception_timeout_s = 0.35;
  double tf_lookup_timeout_s = 0.02;
  int min_fresh_perception_sources = 1;
  bool require_perception = true;
  int min_fresh_horizontal_sources = 1;
  bool require_tilt_perception = false;
  double emergency_release_delay_s = 0.15;
  double neutral_stop_filter_time_constant_s = 0.30;

  AvoidanceParams avoidance;
  TiltPerceptionConfig tilt_perception;
  NarrowPassageConfig narrow_passage;
  LocalMapConfig local_map;
  MapConstraintConfig map_constraint;
  SafetyMode default_mode = SafetyMode::SAFE_ASSIST;

  /** 从已 declare 的 ROS 参数读取，键名见 config/safety_node_params.yaml */
  static SafetyNodeParams loadFromNode(rclcpp::Node & node);
};
