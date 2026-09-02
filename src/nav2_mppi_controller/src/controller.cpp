// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <stdint.h>
#include <chrono>
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

// #define BENCHMARK_TESTING  #define BENCHMARK_TESTING

namespace nav2_mppi_controller
{

void MPPIController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;
  parameters_handler_ = std::make_unique<ParametersHandler>(parent);

  auto node = parent_.lock();
  clock_ = node->get_clock();
  last_time_called_ = clock_->now();
  // Get high-level controller parameters
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(visualize_, "visualize", false);
  getParam(reset_period_, "reset_period", 1.0);
  getParam(
    reset_seed_min_vx_after_side_pass_enable_,
    "reset_seed_min_vx_after_side_pass_enable", false);
  getParam(reset_seed_min_vx_mps_, "reset_seed_min_vx_mps", 0.0);
  if (reset_seed_min_vx_mps_ < 0.0) {
    reset_seed_min_vx_mps_ = 0.0;
  }
  getParam(
    narrow_gate_side_pass_mode_topic_,
    "narrow_gate_side_pass_mode_topic", std::string("/mppi/narrow_gate_side_pass_mode"));
  getParam(rocker_sector_vx_min_enable_, "rocker_sector_vx_min_enable", false);
  getParam(vx_min_forward_, "vx_min", 0.001f);
  getParam(vx_min_lateral_, "vx_min_lateral", -0.50f);
  getParam(
    rocker_forward_sector_topic_, "rocker_forward_sector_topic",
    std::string("/mppi/rocker_forward_sector_mode"));
  getParam(wz_density_cap_enable_, "wz_density_cap_enable", false);
  getParam(
    wz_density_cap_topic_, "wz_density_cap_topic",
    std::string("/mppi/wz_density_cap_rps"));
  getParam(gear_vx_cap_enable_, "gear_vx_cap_enable", false);
  getParam(
    gear_vx_cap_topic_, "gear_vx_cap_topic",
    std::string("/mppi/gear_vx_max_mps"));

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  trajectory_visualizer_.on_configure(
    parent_, name_,
    costmap_ros_->getGlobalFrameID(), parameters_handler_.get());

  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
  if (rocker_sector_vx_min_enable_) {
    RCLCPP_INFO(
      logger_,
      "rocker sector vx_min: forward=%.3f lateral=%.2f topic=%s",
      vx_min_forward_, vx_min_lateral_, rocker_forward_sector_topic_.c_str());
  }
  if (wz_density_cap_enable_) {
    RCLCPP_INFO(
      logger_,
      "wz density cap: topic=%s base_wz_max=%.2f",
      wz_density_cap_topic_.c_str(), optimizer_.getBaseWzMax());
  }
  if (gear_vx_cap_enable_) {
    RCLCPP_INFO(
      logger_,
      "gear vx cap: topic=%s base_vx_max=%.2f",
      gear_vx_cap_topic_.c_str(), optimizer_.getBaseVxMax());
  }
}

void MPPIController::cleanup()
{
  optimizer_.shutdown();
  trajectory_visualizer_.on_cleanup();
  parameters_handler_.reset();
  RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
}

void MPPIController::activate()
{
  trajectory_visualizer_.on_activate();
  parameters_handler_->start();
  auto node = parent_.lock();
  if (reset_seed_min_vx_after_side_pass_enable_) {
    side_pass_mode_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      narrow_gate_side_pass_mode_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&MPPIController::onSidePassMode, this, std::placeholders::_1));
    RCLCPP_INFO(
      logger_,
      "side_pass exit seed: min_vx=%.2f topic=%s",
      reset_seed_min_vx_mps_, narrow_gate_side_pass_mode_topic_.c_str());
  }
  if (rocker_sector_vx_min_enable_) {
    rocker_forward_sector_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      rocker_forward_sector_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&MPPIController::onRockerForwardSector, this, std::placeholders::_1));
  }
  if (wz_density_cap_enable_) {
    wz_density_cap_sub_ = node->create_subscription<std_msgs::msg::Float64>(
      wz_density_cap_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&MPPIController::onWzDensityCap, this, std::placeholders::_1));
  }
  if (gear_vx_cap_enable_) {
    gear_vx_cap_sub_ = node->create_subscription<std_msgs::msg::Float64>(
      gear_vx_cap_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&MPPIController::onGearVxCap, this, std::placeholders::_1));
  }
  RCLCPP_INFO(logger_, "Activated MPPI Controller: %s", name_.c_str());
}

void MPPIController::deactivate()
{
  side_pass_mode_sub_.reset();
  rocker_forward_sector_sub_.reset();
  wz_density_cap_sub_.reset();
  gear_vx_cap_sub_.reset();
  trajectory_visualizer_.on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated MPPI Controller: %s", name_.c_str());
}

void MPPIController::reset()
{
  optimizer_.reset();
  side_pass_exit_seed_pending_ = false;
}

void MPPIController::onSidePassMode(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    side_pass_active_ = true;
    side_pass_exit_seed_pending_ = false;
    return;
  }
  if (side_pass_active_) {
    side_pass_exit_seed_pending_ = true;
    RCLCPP_INFO(
      logger_,
      "side_pass exit: next idle reset seeds |vx|>=%.2f m/s",
      reset_seed_min_vx_mps_);
  }
  side_pass_active_ = false;
}

void MPPIController::onRockerForwardSector(const std_msgs::msg::Bool::SharedPtr msg)
{
  rocker_forward_sector_active_.store(msg->data, std::memory_order_relaxed);
}

void MPPIController::onWzDensityCap(const std_msgs::msg::Float64::SharedPtr msg)
{
  wz_density_cap_rps_.store(msg->data, std::memory_order_relaxed);
}

void MPPIController::onGearVxCap(const std_msgs::msg::Float64::SharedPtr msg)
{
  gear_vx_cap_mps_.store(msg->data, std::memory_order_relaxed);
}

void MPPIController::applyRockerSectorVxMin()
{
  if (!rocker_sector_vx_min_enable_) {
    return;
  }

  const float vx_min = rocker_forward_sector_active_.load(std::memory_order_relaxed) ?
    vx_min_forward_ : vx_min_lateral_;
  optimizer_.setVxMin(vx_min);
}

void MPPIController::applyGearVxCap()
{
  const float base = optimizer_.getBaseVxMax();
  if (!gear_vx_cap_enable_) {
    optimizer_.setVxMax(base);
    return;
  }

  const double cap = gear_vx_cap_mps_.load(std::memory_order_relaxed);
  if (cap <= 1e-6) {
    optimizer_.setVxMax(base);
    return;
  }
  optimizer_.setVxMax(std::min(base, static_cast<float>(cap)));
}

void MPPIController::applyWzDensityCap()
{
  const float base = optimizer_.getBaseWzMax();
  if (!wz_density_cap_enable_) {
    optimizer_.setWzMax(base);
    return;
  }

  const double cap = wz_density_cap_rps_.load(std::memory_order_relaxed);
  if (cap <= 1e-6) {
    optimizer_.setWzMax(base);
    return;
  }
  optimizer_.setWzMax(std::min(base, static_cast<float>(cap)));
}

void MPPIController::applyMinAbsVxSeed(
  geometry_msgs::msg::Twist & speed, double min_vx_mps, double vel_eps)
{
  if (min_vx_mps <= 0.0) {
    return;
  }
  const double vx = speed.linear.x;
  if (std::abs(vx) >= min_vx_mps) {
    return;
  }
  if (std::abs(vx) <= vel_eps) {
    speed.linear.x = min_vx_mps;
  } else {
    speed.linear.x = (vx >= 0.0 ? 1.0 : -1.0) * min_vx_mps;
  }
}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  // [DEBUG-FRAME] 记录单帧起始时间
  const auto _frame_t0 = std::chrono::steady_clock::now();

#ifdef BENCHMARK_TESTING
  auto start = std::chrono::system_clock::now();
#endif

  const bool idle_reset =
    clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_);
  const bool fail_recovery_seed = optimizer_.consumeFailRecoverySeedPending();
  if (idle_reset || fail_recovery_seed) {
    geometry_msgs::msg::Twist seed_speed = robot_speed;
    if (reset_seed_min_vx_after_side_pass_enable_ && side_pass_exit_seed_pending_) {
      applyMinAbsVxSeed(seed_speed, reset_seed_min_vx_mps_, 1e-3);
      side_pass_exit_seed_pending_ = false;
    }
    optimizer_.reset(&seed_speed);
    if (fail_recovery_seed) {
      RCLCPP_INFO(
        logger_,
        "Optimizer fail recovery seed from odom (vx=%.3f wz=%.3f, elapsed=%.3fs < reset_period=%.2fs)",
        seed_speed.linear.x, seed_speed.angular.z,
        (clock_->now() - last_time_called_).seconds(), reset_period_);
    }
  }
  last_time_called_ = clock_->now();

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

  applyRockerSectorVxMin();
  applyGearVxCap();
  applyWzDensityCap();

  geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal_checker);

#ifdef BENCHMARK_TESTING
  auto end = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
#endif

  // [DEBUG-FRAME] 单帧总耗时
  const auto _frame_t1 = std::chrono::steady_clock::now();
  const double _frame_ms = std::chrono::duration<double, std::milli>(
    _frame_t1 - _frame_t0).count();
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 500,
    "[DEBUG-FRAME] total=%.1fms cmd=(v=%.2f w=%.2f)",
    _frame_ms, cmd.twist.linear.x, cmd.twist.angular.z);

  if (visualize_) {
    visualize(std::move(transformed_plan));
  }

  return cmd;
}

void MPPIController::visualize(nav_msgs::msg::Path transformed_plan)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories");
  trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory");
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_.setPath(path);
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace nav2_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::MPPIController, nav2_core::Controller)
