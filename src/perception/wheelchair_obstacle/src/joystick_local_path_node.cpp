// Handle (joystick) velocity intent -> short local path -> Nav2 FollowPath.
//
// MPPI (nav2_mppi_controller) is a path-tracking controller: it consumes a
// nav_msgs/Path via the FollowPath action. On this wheelchair the physical
// rocker joystick is combined with the selected gear (挡位) *inside the chassis
// firmware*, which republishes the resulting linear/angular velocity as
// geometry_msgs/Twist on /handle/control (CAN 0x220 top_input_vel, ~50 Hz).
//
// The raw rocker topic (/sensor/joystick, interface/msg/Joystick, 0x226) only
// carries the stick deflection (rocker_x/rocker_y in [-100,100]) and does NOT
// encode the gear, so it cannot express how fast the user actually wants to go.
// Therefore this node takes the *gear-scaled* velocity from /handle/control as
// the intent, integrates it into a short predicted path in the odom frame and
// forwards it to controller_server::FollowPath. Because gear scaling is done
// once in firmware, the nav layer just mirrors the result and stays consistent
// with manual driving. The path is also published on /mppi/local_intent_path
// for debugging.
//
// Behavior summary:
//   - intent in deadband or stale -> cancel any active goal, publish empty path
//   - intent active               -> regenerate path at send_rate_hz, preempt previous goal
//
// NOTE: MPPI vx_max/vx_min/wz_max and the cmd_vel safety gate limits MUST be
// >= the top gear's max speed, otherwise high gears get clamped downstream.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "nav2_msgs/action/follow_path.hpp"

namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  quat.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace

class JoystickLocalPathNode : public rclcpp::Node
{
public:
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;
  using Path = nav_msgs::msg::Path;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  JoystickLocalPathNode()
  : rclcpp::Node("joystick_local_path_node")
  {
    // Gear-scaled velocity intent from the chassis firmware (0x220), Twist.
    control_topic_ = declare_parameter<std::string>("control_topic", "/handle/control");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom/fused");
    intent_path_topic_ = declare_parameter<std::string>("intent_path_topic", "/mppi/local_intent_path");
    action_name_ = declare_parameter<std::string>("follow_path_action", "/follow_path");

    global_frame_ = declare_parameter<std::string>("global_frame", "odom");
    controller_id_ = declare_parameter<std::string>("controller_id", "FollowPath");
    goal_checker_id_ = declare_parameter<std::string>("goal_checker_id", "general_goal_checker");

    // Deadbands are now in physical units (m/s, rad/s) because the intent is a
    // real velocity, not a normalized rocker deflection. /handle/control is 0
    // at rest, so these only reject noise / firmware jitter.
    lin_deadband_ = declare_parameter<double>("lin_deadband", 0.02);
    ang_deadband_ = declare_parameter<double>("ang_deadband", 0.02);
    // Forward creep injected when the intent is (near) pure rotation so the
    // generated path has a non-zero goal displacement and the goal checker does
    // not report "reached" before the chair turns. Set 0.0 for pure in-place
    // rotation (needs a yaw-aware goal checker).
    turn_assist_vx_ = declare_parameter<double>("turn_assist_vx", 0.08);
    control_timeout_ = declare_parameter<double>("control_timeout", 0.30);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.30);

    obstacle_points_topic_ = declare_parameter<std::string>( "obstacle_points_topic", "/perception/obstacle_points");
    avoidance_assist_enabled_ = declare_parameter<bool>("avoidance_assist_enabled", false);
    assist_obstacle_timeout_ = declare_parameter<double>("assist_obstacle_timeout", 0.30);
    assist_x_min_ = declare_parameter<double>("assist_x_min", 0.35);
    assist_x_max_ = declare_parameter<double>("assist_x_max", 1.35);
    assist_y_abs_ = declare_parameter<double>("assist_y_abs", 0.75);
    assist_center_y_abs_ = declare_parameter<double>("assist_center_y_abs", 0.35);
    assist_wz_ = declare_parameter<double>("assist_wz", 0.16);
    assist_max_wz_ = declare_parameter<double>("assist_max_wz", 0.32);
    assist_smoothing_ = declare_parameter<double>("assist_smoothing", 0.75);
    assist_user_turn_override_ = declare_parameter<double>("assist_user_turn_override", 0.10);

    horizon_ = declare_parameter<double>("horizon", 1.5);
    dt_ = declare_parameter<double>("dt", 0.10);
    send_rate_hz_ = declare_parameter<double>("send_rate_hz", 5.0);

    control_sub_ = create_subscription<Twist>(
      control_topic_, 10,
      std::bind(&JoystickLocalPathNode::onControl, this, std::placeholders::_1));
    odom_sub_ = create_subscription<Odometry>(
      odom_topic_, 10,
      std::bind(&JoystickLocalPathNode::onOdom, this, std::placeholders::_1));

    obstacle_sub_ = create_subscription<PointCloud2>(
      obstacle_points_topic_, 10,
      std::bind(&JoystickLocalPathNode::onObstaclePoints, this, std::placeholders::_1));

    path_pub_ = create_publisher<Path>(intent_path_topic_, 1);

    action_client_ = rclcpp_action::create_client<FollowPath>(this, action_name_);

    const double period = 1.0 / std::max(0.1, send_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(period * 1000)),
      std::bind(&JoystickLocalPathNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "joystick_local_path_node: control=%s odom=%s action=%s frame=%s "
      "lin_deadband=%.3f ang_deadband=%.3f horizon=%.2f dt=%.2f send_rate=%.1f",
      control_topic_.c_str(), odom_topic_.c_str(), action_name_.c_str(),
      global_frame_.c_str(), lin_deadband_, ang_deadband_,
      horizon_, dt_, send_rate_hz_);
  }

private:
  void onControl(const Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(ctrl_mutex_);
    latest_ctrl_ = msg;
    // /handle/control is an unstamped Twist, so use reception time for the
    // freshness check.
    latest_ctrl_stamp_ = now();
  }

  void onOdom(const Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = msg;
  }
  void onObstaclePoints(const PointCloud2::SharedPtr msg)
  {
   std::lock_guard<std::mutex> lock(obstacle_mutex_);
   latest_obstacles_ = msg;
   latest_obstacles_stamp_ = now();
  }

  void onTimer()
  {
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "FollowPath action server not ready, waiting...");
      return;
    }

    // Snapshot intent + odom under locks.
    Twist::SharedPtr ctrl;
    rclcpp::Time ctrl_recv_stamp;
    {
      std::lock_guard<std::mutex> lock(ctrl_mutex_);
      ctrl = latest_ctrl_;
      ctrl_recv_stamp = latest_ctrl_stamp_;
    }
    Odometry::SharedPtr odom;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      odom = latest_odom_;
    }

    const auto tnow = now();

    // Intent timeout -> cancel + zero.
    if (!ctrl || (tnow - ctrl_recv_stamp).seconds() > control_timeout_) {
      cancelActiveGoal();
      publishEmptyPath();
      return;
    }

    // Odom timeout -> cannot build path safely -> cancel + zero. A zero
    // header.stamp (unset or clock-source mismatch) is treated as fresh so the
    // node does not permanently refuse to build paths.
    const rclcpp::Time odom_stamp = odom ? rclcpp::Time(odom->header.stamp)
                                         : rclcpp::Time(0, 0, RCL_ROS_TIME);
    const double odom_age =
      (odom_stamp.nanoseconds() == 0) ? 0.0 : (tnow - odom_stamp).seconds();
    if (!odom || odom_age > odom_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "odom stale or missing, cancelling FollowPath goal");
      cancelActiveGoal();
      publishEmptyPath();
      return;
    }

    // Gear-scaled velocity intent is taken verbatim from /handle/control: the
    // chassis firmware already applied the gear (挡位), so no scaling here.
    double v_user = ctrl->linear.x;
    double w_user = ctrl->angular.z;

    const bool zero_intent =
      (std::abs(v_user) <= lin_deadband_) && (std::abs(w_user) <= ang_deadband_);
    if (zero_intent) {
      cancelActiveGoal();
      publishEmptyPath();
      return;
    }

    // Turn-in-place assist: a rotation-only intent (w active, v in deadband)
    // yields a path whose goal position equals the start pose, which many goal
    // checkers report as "reached" immediately so the chair never rotates.
    // Inject a small forward speed so the path becomes a trackable gentle arc.
    // Set turn_assist_vx=0.0 to keep pure in-place rotation (this then requires
    // a yaw-aware goal checker to actually execute the turn).
    if (std::abs(w_user) > 1e-3 && std::abs(v_user) < turn_assist_vx_) {
      v_user = std::copysign(turn_assist_vx_, (v_user != 0.0) ? v_user : 1.0);
    }

    // w_user = applyAvoidanceAssist(v_user, w_user, tnow);
    const Path path = buildPath(*odom, v_user, w_user);
    path_pub_->publish(path);
    sendGoal(path);
  }
 
  double applyAvoidanceAssist(double v_user, double w_user, const rclcpp::Time & tnow)
{
  if (!avoidance_assist_enabled_ || v_user <= lin_deadband_) {
    assist_wz_state_ *= assist_smoothing_;
    return w_user + assist_wz_state_;
  }

  if (std::abs(w_user) >= assist_user_turn_override_) {
    assist_wz_state_ *= assist_smoothing_;
    return w_user + assist_wz_state_;
  }

  PointCloud2::SharedPtr cloud;
  rclcpp::Time stamp;
  {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    cloud = latest_obstacles_;
    stamp = latest_obstacles_stamp_;
  }

  if (!cloud || (tnow - stamp).seconds() > assist_obstacle_timeout_) {
    assist_wz_state_ *= assist_smoothing_;
    return w_user + assist_wz_state_;
  }

  double center_score = 0.0;
  double left_score = 0.0;
  double right_score = 0.0;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    const double x = *iter_x;
    const double y = *iter_y;

    if (x < assist_x_min_ || x > assist_x_max_ || std::abs(y) > assist_y_abs_) {
      continue;
    }

    const double weight = 1.0 / std::max(0.20, x);

    if (std::abs(y) <= assist_center_y_abs_) {
      center_score += weight;
    } else if (y > 0.0) {
      left_score += weight;
    } else {
      right_score += weight;
    }
  }

  double target_assist = 0.0;
  if (center_score > 0.0) {
    target_assist = (left_score <= right_score) ? assist_wz_ : -assist_wz_;
  }

  target_assist = std::clamp(target_assist, -assist_max_wz_, assist_max_wz_);
  assist_wz_state_ =
    assist_smoothing_ * assist_wz_state_ + (1.0 - assist_smoothing_) * target_assist;

  return w_user + assist_wz_state_;
}

  Path buildPath(const Odometry & odom, double v_user, double w_user) const
  {
    Path path;
    path.header.stamp = now();
    path.header.frame_id = global_frame_;

    double x = odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y;
    double yaw = yawFromQuaternion(odom.pose.pose.orientation);

    const int N = std::max(1, static_cast<int>(std::round(horizon_ / dt_)));

    PoseStamped pose;
    pose.header = path.header;
    pose.pose = odom.pose.pose;
    path.poses.push_back(pose);

    for (int i = 1; i <= N; ++i) {
      x += v_user * std::cos(yaw) * dt_;
      y += v_user * std::sin(yaw) * dt_;
      yaw += w_user * dt_;

      PoseStamped p;
      p.header = path.header;
      p.pose.position.x = x;
      p.pose.position.y = y;
      p.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      p.pose.orientation.x = q.x();
      p.pose.orientation.y = q.y();
      p.pose.orientation.z = q.z();
      p.pose.orientation.w = q.w();
      path.poses.push_back(p);
    }

    return path;
  }

  void sendGoal(const Path & path)
  {
    auto goal = FollowPath::Goal();
    goal.path = path;
    goal.controller_id = controller_id_;
    goal.goal_checker_id = goal_checker_id_;

    auto send_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    send_options.goal_response_callback =
      [this](const GoalHandle::SharedPtr & goal_handle) {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal_handle_ = goal_handle;
      };
    send_options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_DEBUG(get_logger(), "FollowPath result: SUCCEEDED");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_DEBUG(get_logger(), "FollowPath result: ABORTED");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_DEBUG(get_logger(), "FollowPath result: CANCELED");
            break;
          default:
            break;
        }
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal_handle_.reset();
      };

    // Drop the stale handle before sending: Nav2 FollowPath preempts the
    // previous goal server-side when a new one arrives, but we also clear our
    // local reference so a cancel issued in the same cycle does not race.
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      current_goal_handle_.reset();
    }

    action_client_->async_send_goal(goal, send_options);
  }

  void cancelActiveGoal()
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!current_goal_handle_) {
      return;
    }
    action_client_->async_cancel_goal(current_goal_handle_);
    current_goal_handle_.reset();
  }

  void publishEmptyPath()
  {
    Path path;
    path.header.stamp = now();
    path.header.frame_id = global_frame_;
    path_pub_->publish(path);
  }

  // Parameters
  std::string control_topic_;
  std::string odom_topic_;
  std::string intent_path_topic_;
  std::string obstacle_points_topic_;
  std::string action_name_;
  std::string global_frame_;
  std::string controller_id_;
  std::string goal_checker_id_;
  double lin_deadband_{0.02};
  double ang_deadband_{0.02};
  double turn_assist_vx_{0.08};
  double control_timeout_{0.30};
  double odom_timeout_{0.30};
  bool avoidance_assist_enabled_{false};
  double assist_obstacle_timeout_{0.30};  
  double assist_x_min_{0.35};
  double assist_x_max_{1.35};
  double assist_y_abs_{0.75};
  double assist_center_y_abs_{0.35};
  double assist_wz_{0.16};
  double assist_max_wz_{0.32};
  double assist_smoothing_{0.75};
  double assist_user_turn_override_{0.10};
  double assist_wz_state_{0.0};
  double horizon_{1.5};
  double dt_{0.10};
  double send_rate_hz_{5.0};

  // Subscriptions / publishers / action client
  rclcpp::Subscription<Twist>::SharedPtr control_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_;
  rclcpp_action::Client<FollowPath>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  std::mutex ctrl_mutex_;
  Twist::SharedPtr latest_ctrl_;
  rclcpp::Time latest_ctrl_stamp_{0, 0, RCL_ROS_TIME};

  std::mutex odom_mutex_;
  Odometry::SharedPtr latest_odom_;

  std::mutex obstacle_mutex_;
  PointCloud2::SharedPtr latest_obstacles_;
  rclcpp::Time latest_obstacles_stamp_{0, 0, RCL_ROS_TIME};

  std::mutex goal_mutex_;
  GoalHandle::SharedPtr current_goal_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickLocalPathNode>());
  rclcpp::shutdown();
  return 0;
}
