#include "wheelchair_obstacle/gate_centerline_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace mppi::critics
{

// ==== 窄门中心线引导 cost 试验版 BEGIN ====
void GateCenterlineCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(cost_power_, "cost_power", 1);
  getParam(critic_weight_, "critic_weight", 4.0);
  getParam(center_weight_, "center_weight", 1.0);
  getParam(heading_weight_, "heading_weight", 0.3);
  getParam(gate_width_min_m_, "gate_width_min_m", 0.80);
  getParam(gate_width_max_m_, "gate_width_max_m", 1.50);
  getParam(gate_min_distance_m_, "gate_min_distance_m", 0.50);
  getParam(gate_max_distance_m_, "gate_max_distance_m", 1.20);
  getParam(scan_y_abs_m_, "scan_y_abs_m", 1.60);
  getParam(scan_x_step_m_, "scan_x_step_m", 0.05);
  getParam(scan_y_step_m_, "scan_y_step_m", 0.05);
  getParam(center_clear_half_width_m_, "center_clear_half_width_m", 0.25);
  getParam(center_clear_x_margin_m_, "center_clear_x_margin_m", 0.12);
  getParam(active_x_band_m_, "active_x_band_m", 0.45);
  getParam(active_hold_s_, "active_hold_s", 0.30);
  getParam(stage_prepare_ratio_, "stage_prepare_ratio", 0.35);
  getParam(stage_turn_ratio_, "stage_turn_ratio", 0.20);
  getParam(stage_exit_ratio_, "stage_exit_ratio", 0.45);
  getParam(stage_far_center_scale_, "stage_far_center_scale", 0.35);
  getParam(stage_prepare_center_scale_, "stage_prepare_center_scale", 0.85);
  getParam(stage_turn_center_scale_, "stage_turn_center_scale", 1.35);
  getParam(stage_exit_center_scale_, "stage_exit_center_scale", 0.90);
  getParam(stage_far_heading_scale_, "stage_far_heading_scale", 0.55);
  getParam(stage_prepare_heading_scale_, "stage_prepare_heading_scale", 1.00);
  getParam(stage_turn_heading_scale_, "stage_turn_heading_scale", 1.25);
  getParam(stage_exit_heading_scale_, "stage_exit_heading_scale", 0.95);
  getParam(stage_weight_gain_, "stage_weight_gain", 0.85);
  getParam(critical_mix_ratio_, "critical_mix_ratio", 0.35);
  getParam(stick_forward_min_, "stick_forward_min", 0.20);
  getParam(stick_lateral_min_, "stick_lateral_min", 0.12);
  getParam(gate_center_y_deadband_m_, "gate_center_y_deadband_m", 0.18);
  getParam(intent_lateral_ratio_min_, "intent_lateral_ratio_min", 0.20);
  getParam(trajectory_point_step_, "trajectory_point_step", 4);
  getParam(joystick_topic_, "joystick_topic", std::string("/sensor/joystick"));
  getParam(debug_topic_, "debug_topic", std::string("/mppi/gate_centerline_critic/debug"));
  getParam(debug_publish_period_s_, "debug_publish_period_s", 0.10);

  int obstacle_cost_threshold = 253;
  getParam(obstacle_cost_threshold, "obstacle_cost_threshold", 253);
  obstacle_cost_threshold_ = static_cast<unsigned char>(
    std::clamp(obstacle_cost_threshold, 0, 255));

  auto node = parent_.lock();
  if (node) {
    joystick_sub_ = node->create_subscription<interface::msg::Joystick>(
      joystick_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&GateCenterlineCritic::onJoystick, this, std::placeholders::_1));
    debug_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>(
      debug_topic_, rclcpp::SystemDefaultsQoS());
  }

  RCLCPP_INFO(
    logger_,
    "GateCenterlineCritic enabled=%s width=[%.2f, %.2f]m distance=[%.2f, %.2f]m debug=%s",
    enabled_ ? "true" : "false", gate_width_min_m_, gate_width_max_m_,
    gate_min_distance_m_, gate_max_distance_m_, debug_topic_.c_str());
}

void GateCenterlineCritic::score(mppi::CriticData & data)
{
  if (!enabled_) {
    return;
  }

  const auto intent = getStickIntent();
  auto gate = detectGate(data, intent);

  const auto node = parent_.lock();
  const auto now_time = node ? node->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);

  if (gate) {
    last_active_gate_ = gate;
    last_active_gate_stamp_ = now_time;
  } else if (
    last_active_gate_ && now_time.nanoseconds() != 0 &&
    (now_time - last_active_gate_stamp_).seconds() <= active_hold_s_)
  {
    // 只对摇杆单帧抖动做短暂保持；如果中心被障碍占住，detectGate() 不会更新，
    // 下一帧中心检测仍失败时会很快退出，不会长期和避障 cost 对抗。
    gate = last_active_gate_;
  } else {
    last_active_gate_.reset();
    publishDebug(false, std::nullopt, intent, now_time);
    return;
  }

  publishDebug(true, gate, intent, now_time);

  const double robot_yaw = getYaw(data.state.pose.pose.orientation);
  const double cos_y = std::cos(robot_yaw);
  const double sin_y = std::sin(robot_yaw);
  const double robot_x = data.state.pose.pose.position.x;
  const double robot_y = data.state.pose.pose.position.y;

  const auto shape = data.trajectories.x.shape();
  const size_t batch_size = shape[0];
  const size_t time_steps = shape[1];
  const size_t step = static_cast<size_t>(std::max(1, trajectory_point_step_));

  const double active_band = std::max(0.10, active_x_band_m_);
  const auto guide_curve = buildGuideCurve(*gate);
  if (guide_curve.empty()) {
    publishDebug(false, std::nullopt, intent, now_time);
    return;
  }

  for (size_t i = 0; i < batch_size; ++i) {
    double total = 0.0;
    double total_weight = 0.0;
    double max_point_cost = 0.0;

    for (size_t t = 0; t < time_steps; t += step) {
      const double wx = data.trajectories.x(i, t);
      const double wy = data.trajectories.y(i, t);
      const double dx = wx - robot_x;
      const double dy = wy - robot_y;
      const double bx = cos_y * dx + sin_y * dy;
      const double by = -sin_y * dx + cos_y * dy;

      if (bx < -0.05 || bx > gate->distance_m + active_band + 0.80) {
        continue;
      }

      // ==== 分段引导参考曲线 BEGIN ====
      // 不再用“一条斜直线”做参考，而是用：
      // 1) 主走廊中心直行；
      // 2) 一段三次 Hermite 平滑转弯；
      // 3) 接入侧边窄道中心线。
      // 这里对每个候选点，取其在参考曲线上的最近点来计算横向偏差和航向偏差。
      const GuidePoint guide_point = findClosestGuidePoint(guide_curve, bx, by);
      const double rel_x = bx - guide_point.x_body;
      const double rel_y = by - guide_point.y_body;
      const double guide_cos = std::cos(guide_point.yaw_body);
      const double guide_sin = std::sin(guide_point.yaw_body);
      const double lateral_error = -guide_sin * rel_x + guide_cos * rel_y;
      const double yaw_body = normalizeAngle(data.trajectories.yaws(i, t) - robot_yaw);
      const double yaw_error = normalizeAngle(yaw_body - guide_point.yaw_body);
      // ==== 分段引导参考曲线 END ====

      // ==== 分阶段引导加权 BEGIN ====
      // 不同位置的候选点，使用不同的中心/航向权重：
      // 1) 入口较远时，只给较弱的中心保持，避免过早被吸向侧边；
      // 2) 接近入口时逐步增强，引导先把姿态准备好；
      // 3) 真正切入入口时达到最强；
      // 4) 进入后回到中等，避免和其他 critic 长时间“打架”。
      const StageWeights stage = computeStageWeights(guide_point.x_body, *gate);
      const double point_cost =
        (center_weight_ * stage.center_scale) * lateral_error * lateral_error +
        (heading_weight_ * stage.heading_scale) * yaw_error * yaw_error;
      const double point_weight =
        1.0 + std::max(0.0, stage_weight_gain_) *
        std::max(stage.center_scale, stage.heading_scale);
      // ==== 分阶段引导加权 END ====
      total += point_weight * point_cost;
      total_weight += point_weight;
      max_point_cost = std::max(max_point_cost, point_cost);
    }

    if (total_weight <= 1e-6) {
      continue;
    }

    // ==== 关键切入段更重 BEGIN ====
    // weighted_mean: 更看重入口准备/切入阶段，而不是把全段平均摊平；
    // max_point_cost: 再盯住最差那一小段，避免“整体还行但临门一脚很歪”仍被选中。
    const double weighted_mean = total / total_weight;
    const double critical_mix = std::clamp(critical_mix_ratio_, 0.0, 1.0);
    float cost = static_cast<float>(
      (1.0 - critical_mix) * weighted_mean + critical_mix * max_point_cost);
    // ==== 关键切入段更重 END ====
    if (cost_power_ > 1) {
      cost = std::pow(cost, static_cast<float>(cost_power_));
    }
    data.costs(i) += static_cast<float>(critic_weight_) * cost;
  }
}

void GateCenterlineCritic::onJoystick(const interface::msg::Joystick::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joystick_mutex_);
  latest_joystick_ = msg;
  auto node = parent_.lock();
  latest_joystick_stamp_ = node ? node->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void GateCenterlineCritic::publishDebug(
  bool active,
  const std::optional<GateInfo> & gate,
  const StickIntent & intent,
  const rclcpp::Time & now_time)
{
  if (!debug_pub_) {
    return;
  }

  const double publish_period = std::max(0.02, debug_publish_period_s_);
  if (
    now_time.nanoseconds() != 0 && last_debug_publish_stamp_.nanoseconds() != 0 &&
    (now_time - last_debug_publish_stamp_).seconds() < publish_period)
  {
    return;
  }
  last_debug_publish_stamp_ = now_time;

  const float nan = std::numeric_limits<float>::quiet_NaN();
  std_msgs::msg::Float32MultiArray msg;
  // ==== 窄门中心线引导 cost 调试输出 BEGIN ====
  // data 含义：
  // [0] active，1 表示本帧已触发并参与 MPPI 打分，0 表示未触发；
  // [1] distance_m，识别到的入口/窄门距离；
  // [2] width_m，识别到的入口/窄门宽度；
  // [3] center_y_m，窄门中心线相对车体中心的横向偏差，左正右负；
  // [4] axis_yaw_rad，侧边窄道参考方向角；
  // [5] intent_forward，当前摇杆前向分量；
  // [6] intent_lateral，当前摇杆侧向分量。
  // ==== 窄门中心线引导 cost 调试输出 END ====
  msg.data.reserve(7);
  msg.data.push_back(active ? 1.0F : 0.0F);
  msg.data.push_back(gate ? static_cast<float>(gate->distance_m) : nan);
  msg.data.push_back(gate ? static_cast<float>(gate->width_m) : nan);
  msg.data.push_back(gate ? static_cast<float>(gate->center_y_m) : nan);
  msg.data.push_back(gate ? static_cast<float>(gate->axis_yaw_body) : nan);
  msg.data.push_back(static_cast<float>(intent.forward));
  msg.data.push_back(static_cast<float>(intent.lateral));
  debug_pub_->publish(msg);
}

GateCenterlineCritic::StickIntent GateCenterlineCritic::getStickIntent()
{
  StickIntent intent;
  interface::msg::Joystick::SharedPtr joy;
  rclcpp::Time stamp;
  {
    std::lock_guard<std::mutex> lock(joystick_mutex_);
    joy = latest_joystick_;
    stamp = latest_joystick_stamp_;
  }

  auto node = parent_.lock();
  if (!joy || !node || (node->now() - stamp).seconds() > 0.35) {
    return intent;
  }

  intent.forward = std::clamp(static_cast<double>(joy->rocker_x) / 100.0, -1.0, 1.0);
  intent.lateral = std::clamp(static_cast<double>(joy->rocker_y) / 100.0, -1.0, 1.0);
  if (std::hypot(intent.forward, intent.lateral) < 1e-3) {
    return intent;
  }
  intent.yaw_body = std::atan2(intent.lateral, intent.forward);
  intent.fresh = true;
  return intent;
}

std::optional<GateCenterlineCritic::GateInfo> GateCenterlineCritic::detectGate(
  const mppi::CriticData & data, const StickIntent & intent)
{
  if (!costmap_) {
    return std::nullopt;
  }

  const auto & pose = data.state.pose.pose;
  const double x_step = std::max(0.03, scan_x_step_m_);
  const double y_step = std::max(0.03, scan_y_step_m_);

  std::optional<GateInfo> best;
  double best_score = std::numeric_limits<double>::infinity();

  for (double x = gate_min_distance_m_; x <= gate_max_distance_m_ + 1e-6; x += x_step) {
    double left_y = std::numeric_limits<double>::quiet_NaN();
    double right_y = std::numeric_limits<double>::quiet_NaN();

    // 横向扫描左右最近障碍边界：左侧 y>0，右侧 y<0。
    for (double y = 0.0; y <= scan_y_abs_m_ + 1e-6; y += y_step) {
      if (!std::isfinite(left_y) && isLethalInBody(x, y, pose)) {
        left_y = y;
      }
      if (!std::isfinite(right_y) && isLethalInBody(x, -y, pose)) {
        right_y = -y;
      }
      if (std::isfinite(left_y) && std::isfinite(right_y)) {
        break;
      }
    }

    if (!std::isfinite(left_y) || !std::isfinite(right_y)) {
      continue;
    }

    const double width = left_y - right_y;
    if (width < gate_width_min_m_ || width > gate_width_max_m_) {
      continue;
    }

    const double center_y = 0.5 * (left_y + right_y);
    if (centerBandBlocked(x, center_y, pose)) {
      // 如果走廊中间有人/障碍，这里会直接不启用中心线 cost，让原避障 cost 接管。
      continue;
    }

    GateInfo gate;
    gate.distance_m = x;
    gate.center_y_m = center_y;
    gate.width_m = width;
    const double side_sign =
      (std::abs(center_y) > 1e-3) ? ((center_y > 0.0) ? 1.0 : -1.0) :
      ((intent.lateral >= 0.0) ? 1.0 : -1.0);
    gate.axis_yaw_body = side_sign * (M_PI / 2.0);

    if (!stickMatchesGate(gate, intent)) {
      continue;
    }

    // 优先选择更近、中心更明确的窄门，避免远处开口提前影响当前行驶。
    const double target_width = 0.5 * (gate_width_min_m_ + gate_width_max_m_);
    const double score =
      1.5 * x +
      std::abs(width - target_width) +
      0.5 * std::abs(center_y);
    if (score < best_score) {
      best_score = score;
      best = gate;
    }
  }

  return best;
}

std::vector<GateCenterlineCritic::GuidePoint> GateCenterlineCritic::buildGuideCurve(
  const GateInfo & gate) const
{
  std::vector<GuidePoint> guide;
  const double side_sign = (gate.center_y_m >= 0.0) ? 1.0 : -1.0;
  const double x_end = gate.distance_m;
  const double turn_lead_m =
    std::clamp(0.30 + 0.45 * std::abs(gate.center_y_m), 0.35, std::max(0.40, x_end - 0.10));
  const double x0 = std::clamp(x_end - turn_lead_m, 0.15, std::max(0.15, x_end - 0.08));
  const double y0 = 0.0;
  const double y1 = gate.center_y_m;
  const double curve_len = std::hypot(x_end - x0, y1 - y0);
  const double tangent_len0 = std::max(0.20, 0.75 * curve_len);
  const double tangent_len1 = std::max(0.20, 0.75 * curve_len);
  const double final_yaw = gate.axis_yaw_body;
  const double curve_step = 0.06;

  for (double x = 0.0; x < x0 - 1e-6; x += curve_step) {
    guide.push_back(GuidePoint{x, 0.0, 0.0});
  }
  guide.push_back(GuidePoint{x0, 0.0, 0.0});

  const int hermite_samples = 18;
  const double mx0 = tangent_len0;
  const double my0 = 0.0;
  const double mx1 = tangent_len1 * std::cos(final_yaw);
  const double my1 = tangent_len1 * std::sin(final_yaw);
  for (int i = 1; i <= hermite_samples; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(hermite_samples);
    guide.push_back(makeHermiteGuidePoint(u, x0, y0, x_end, y1, mx0, my0, mx1, my1));
  }

  const double tail_m = std::clamp(0.45 + 0.25 * std::abs(gate.center_y_m), 0.45, 0.80);
  for (double s = curve_step; s <= tail_m + 1e-6; s += curve_step) {
    guide.push_back(GuidePoint{x_end, y1 + side_sign * s, final_yaw});
  }

  return guide;
}

GateCenterlineCritic::GuidePoint GateCenterlineCritic::findClosestGuidePoint(
  const std::vector<GuidePoint> & guide, double x_body, double y_body) const
{
  GuidePoint best = guide.front();
  double best_dist2 = std::numeric_limits<double>::infinity();
  for (const auto & point : guide) {
    const double dx = x_body - point.x_body;
    const double dy = y_body - point.y_body;
    const double dist2 = dx * dx + dy * dy;
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      best = point;
    }
  }
  return best;
}

GateCenterlineCritic::GuidePoint GateCenterlineCritic::makeHermiteGuidePoint(
  double u,
  double x0,
  double y0,
  double x1,
  double y1,
  double mx0,
  double my0,
  double mx1,
  double my1) const
{
  const double u2 = u * u;
  const double u3 = u2 * u;

  const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
  const double h10 = u3 - 2.0 * u2 + u;
  const double h01 = -2.0 * u3 + 3.0 * u2;
  const double h11 = u3 - u2;

  const double x = h00 * x0 + h10 * mx0 + h01 * x1 + h11 * mx1;
  const double y = h00 * y0 + h10 * my0 + h01 * y1 + h11 * my1;

  const double dh00 = 6.0 * u2 - 6.0 * u;
  const double dh10 = 3.0 * u2 - 4.0 * u + 1.0;
  const double dh01 = -6.0 * u2 + 6.0 * u;
  const double dh11 = 3.0 * u2 - 2.0 * u;

  const double dx_du = dh00 * x0 + dh10 * mx0 + dh01 * x1 + dh11 * mx1;
  const double dy_du = dh00 * y0 + dh10 * my0 + dh01 * y1 + dh11 * my1;

  return GuidePoint{x, y, std::atan2(dy_du, dx_du)};
}

bool GateCenterlineCritic::isLethalInBody(
  double x_body, double y_body, const geometry_msgs::msg::Pose & pose) const
{
  const double yaw = getYaw(pose.orientation);
  const double wx = pose.position.x + std::cos(yaw) * x_body - std::sin(yaw) * y_body;
  const double wy = pose.position.y + std::sin(yaw) * x_body + std::cos(yaw) * y_body;

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return false;
  }

  return costmap_->getCost(mx, my) >= obstacle_cost_threshold_;
}

bool GateCenterlineCritic::centerBandBlocked(
  double distance_m, double center_y_m, const geometry_msgs::msg::Pose & pose) const
{
  const double x_margin = std::max(0.0, center_clear_x_margin_m_);
  const double half_width = std::max(0.05, center_clear_half_width_m_);
  const double step = std::max(0.03, scan_y_step_m_);
  for (double x = distance_m - x_margin; x <= distance_m + x_margin + 1e-6; x += step) {
    for (double y = center_y_m - half_width; y <= center_y_m + half_width + 1e-6; y += step) {
      if (isLethalInBody(x, y, pose)) {
        return true;
      }
    }
  }
  return false;
}

bool GateCenterlineCritic::stickMatchesGate(const GateInfo & gate, const StickIntent & intent)
{
  if (!intent.fresh || intent.forward < stick_forward_min_) {
    return false;
  }

  // ==== 摇杆意图范围判定 BEGIN ====
  // 这里不再要求摇杆角度和窄门方向“精确对齐”，而是按范围做粗判：
  // 1) 先确认用户确实在明显前推；
  // 2) 如果入口中心基本在正前方附近，只要前推即可；
  // 3) 如果入口明显偏左/偏右，只要求摇杆侧向分量同号，且占前推分量达到一个比例范围。
  // 这样更符合真实推杆习惯，不需要每一帧都精确指向入口轴线。
  if (std::abs(gate.center_y_m) <= gate_center_y_deadband_m_) {
    return true;
  }

  if (std::abs(intent.lateral) < stick_lateral_min_) {
    return false;
  }
  if (gate.center_y_m * intent.lateral < 0.0) {
    return false;
  }

  const double lateral_ratio =
    std::abs(intent.lateral) / std::max(std::abs(intent.forward), 1e-3);
  if (lateral_ratio < intent_lateral_ratio_min_) {
    return false;
  }
  // ==== 摇杆意图范围判定 END ====

  return true;
}

double GateCenterlineCritic::getYaw(const geometry_msgs::msg::Quaternion & q) const
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  quat.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

double GateCenterlineCritic::normalizeAngle(double a) const
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

GateCenterlineCritic::StageWeights GateCenterlineCritic::computeStageWeights(
  double x_body, const GateInfo & gate) const
{
  const double active_band = std::max(0.10, active_x_band_m_);
  const double x_far = gate.distance_m - active_band;
  const double x_prepare = gate.distance_m - std::max(0.0, stage_prepare_ratio_) * active_band;
  const double x_turn = gate.distance_m + std::max(0.0, stage_turn_ratio_) * active_band;
  const double x_exit = gate.distance_m + std::max(0.0, stage_exit_ratio_) * active_band;

  StageWeights weights;

  if (x_body <= x_prepare) {
    const double t = smoothstep01((x_body - x_far) / std::max(1e-3, x_prepare - x_far));
    weights.center_scale = lerp(stage_far_center_scale_, stage_prepare_center_scale_, t);
    weights.heading_scale = lerp(stage_far_heading_scale_, stage_prepare_heading_scale_, t);
    return weights;
  }

  if (x_body <= x_turn) {
    const double t = smoothstep01((x_body - x_prepare) / std::max(1e-3, x_turn - x_prepare));
    weights.center_scale = lerp(stage_prepare_center_scale_, stage_turn_center_scale_, t);
    weights.heading_scale = lerp(stage_prepare_heading_scale_, stage_turn_heading_scale_, t);
    return weights;
  }

  if (x_body <= x_exit) {
    const double t = smoothstep01((x_body - x_turn) / std::max(1e-3, x_exit - x_turn));
    weights.center_scale = lerp(stage_turn_center_scale_, stage_exit_center_scale_, t);
    weights.heading_scale = lerp(stage_turn_heading_scale_, stage_exit_heading_scale_, t);
    return weights;
  }

  weights.center_scale = stage_exit_center_scale_;
  weights.heading_scale = stage_exit_heading_scale_;
  return weights;
}

double GateCenterlineCritic::smoothstep01(double value) const
{
  const double v = std::clamp(value, 0.0, 1.0);
  return v * v * (3.0 - 2.0 * v);
}

double GateCenterlineCritic::lerp(double a, double b, double t) const
{
  return a + (b - a) * std::clamp(t, 0.0, 1.0);
}
// ==== 窄门中心线引导 cost 试验版 END ====

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::GateCenterlineCritic,
  mppi::critics::CriticFunction)
