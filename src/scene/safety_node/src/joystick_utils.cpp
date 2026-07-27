#include "safety_node/joystick_utils.h"

#include <algorithm>
#include <cmath>

namespace
{
int applyDeadzone(int raw, int deadzone)
{
  const int dz = std::max(0, deadzone);
  return (std::abs(raw) <= dz) ? 0 : raw;
}
} // namespace

geometry_msgs::msg::Twist JoystickUtils::toTwist(
  const interface::msg::Joystick & joystick,
  const JoystickTwistMapping & mapping,
  bool fresh)
{
  geometry_msgs::msg::Twist out{};
  if (!fresh) {
    return out;
  }

  const int x = applyDeadzone(static_cast<int>(joystick.rocker_x), mapping.deadzone);
  const int y = applyDeadzone(static_cast<int>(joystick.rocker_y), mapping.deadzone);
  if (x == 0 && y == 0) {
    return out;
  }

  constexpr double kRockerScale = 100.0;
  out.linear.x = (static_cast<double>(x) / kRockerScale) * mapping.max_linear_m_s;
  // 右拨 rocker_y 为正 → 车体右转 → angular.z 为负（ROS 左转为正）
  out.angular.z = -(static_cast<double>(y) / kRockerScale) * mapping.max_angular_rad_s;
  return out;
}

std::vector<geometry_msgs::msg::Point> JoystickUtils::integrateUnicycleTrajectory(
  double linear_x,
  double angular_z,
  double horizon_s,
  double dt_s,
  double z)
{
  std::vector<geometry_msgs::msg::Point> points;
  const double dt = std::max(0.02, dt_s);
  const double horizon = std::max(dt, horizon_s);

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double elapsed = 0.0;

  geometry_msgs::msg::Point origin;
  origin.x = x;
  origin.y = y;
  origin.z = z;
  points.push_back(origin);

  while (elapsed < horizon - 1e-9) {
    x += linear_x * std::cos(theta) * dt;
    y += linear_x * std::sin(theta) * dt;
    theta += angular_z * dt;
    elapsed += dt;

    geometry_msgs::msg::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    points.push_back(pt);
  }
  return points;
}
