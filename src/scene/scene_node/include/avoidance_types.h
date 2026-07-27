#pragma once

#include <cstddef>
#include <limits>
#include <rclcpp/time.hpp>
#include <string>
#include <vector>

namespace scene_node::avoidance {

  struct MotionCommand
  {
    double linear_x = 0.0;
    double angular_z = 0.0;
  };

  struct VehicleFeedback
  {
    double linear_x = 0.0;
    double angular_z = 0.0;
    rclcpp::Time stamp;
  };

  struct PredictedPose
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double t = 0.0;
  };

  struct CandidateTrajectory
  {
    MotionCommand command;
    std::vector < PredictedPose > poses;
  };

  struct FootprintBox
  {
    double front = 0.88;
    double back = 0.25;
    double half_width = 0.35;
  };

  struct FootprintMargins
  {
    double front = 0.10;
    double back = 0.12;
    double side = 0.05;
  };

  struct ObstaclePrimitive
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double radius = 0.08;
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    std::string source;
    rclcpp::Time stamp;
    bool valid = false;
  };

  struct ObstacleField
  {
    std::vector < ObstaclePrimitive > obstacles;
    rclcpp::Time stamp;
  };

  enum class RiskLevel
  {
    CLEAR,
    WARNING,
    LIMITING,
    BRAKING
  };

  struct CollisionRisk
  {
    RiskLevel level = RiskLevel::CLEAR;
    double min_distance = std::numeric_limits < double > ::infinity();
    double time_to_collision = std::numeric_limits < double > ::infinity();
    double collision_x = 0.0;
    double collision_y = 0.0;
    std::string source;
    bool has_collision = false;
  };

  struct SafetyFilterResult
  {
    MotionCommand selected_command;
    CollisionRisk risk;
    bool emergency_brake = false;
    std::size_t obstacle_count = 0;
    std::size_t trajectory_count = 0;
  };

  struct RolloutParams
  {
    double horizon_s = 1.2;
    double dt_s = 0.1;
    int speed_samples = 6;
    int angular_samples = 5;
    double linear_deadband = 0.01;
    double angular_deadband = 0.01;
  };

  struct ObstacleFieldParams
  {
    double radar_timeout_s = 0.15;
    std::size_t max_obstacles = 128;
    double default_radius = 0.08;
    double min_radius = 0.04;
    double max_radius = 0.35;
    bool include_tilt = true;
    bool include_back = true;
  };

  struct RiskParams
  {
    double warning_ttc_s = 1.5;
    double limiting_ttc_s = 1.0;
    double emergency_ttc_s = 0.45;
    double hard_stop_distance_m = 0.12;
    double comfort_clearance_m = 0.30;
  };

  struct SafetyFilterParams
  {
    double angular_weight = 0.30;
    double emergency_ttc_s = 0.45;
  };

  struct RiskAssessmentParams
  {
    RolloutParams rollout;
    ObstacleFieldParams obstacle;
    FootprintBox footprint;
    FootprintMargins margins;
    RiskParams risk;
    SafetyFilterParams filter;
  };

  const char * riskLevelName(RiskLevel level);

}  // namespace scene_node::avoidance
