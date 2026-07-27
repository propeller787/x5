#include "shared_control_assist/shared_control_assist_node.hpp"

#include "shared_control_assist/math_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace shared_control_assist {
namespace {

template <typename T>
T declareAndGet(rclcpp::Node& node, const std::string& name, const T& default_value)
{
    node.declare_parameter<T>(name, default_value);
    return node.get_parameter(name).get_value<T>();
}

geometry_msgs::msg::Point point(double x, double y, double z = 0.0)
{
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

}  // namespace

SharedControlAssistNode::SharedControlAssistNode()
    : rclcpp::Node("shared_control_assist"),
      perception_(params_),
      door_detector_(params_)
{
    declareAndLoadParams();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto now_time = now();
    last_timer_time_ = now_time;
    last_output_time_ = now_time;
    last_debug_publish_time_ = now_time;
    state_enter_time_ = now_time;

    createIo();

    RCLCPP_INFO(
        get_logger(),
        "shared_control_assist_node started: %s -> %s, scans=[%s, %s, %s]",
        params_.input_control_topic.c_str(),
        params_.output_control_topic.c_str(),
        params_.left_scan_topic.c_str(),
        params_.right_scan_topic.c_str(),
        params_.back_scan_topic.c_str());
}

void SharedControlAssistNode::declareAndLoadParams()
{
    params_.input_control_topic = declareAndGet(*this, "input_control_topic", params_.input_control_topic);
    params_.output_control_topic = declareAndGet(*this, "output_control_topic", params_.output_control_topic);
    params_.wheel_speed_topic = declareAndGet(*this, "wheel_speed_topic", params_.wheel_speed_topic);
    params_.left_scan_topic = declareAndGet(*this, "left_scan_topic", params_.left_scan_topic);
    params_.right_scan_topic = declareAndGet(*this, "right_scan_topic", params_.right_scan_topic);
    params_.back_scan_topic = declareAndGet(*this, "back_scan_topic", params_.back_scan_topic);
    params_.assist_enable_topic = declareAndGet(*this, "assist_enable_topic", params_.assist_enable_topic);
    params_.assist_points_topic = declareAndGet(*this, "assist_points_topic", params_.assist_points_topic);
    params_.assist_local_grid_topic = declareAndGet(*this, "assist_local_grid_topic", params_.assist_local_grid_topic);
    params_.assist_state_topic = declareAndGet(*this, "assist_state_topic", params_.assist_state_topic);
    params_.debug_markers_topic = declareAndGet(*this, "debug_markers_topic", params_.debug_markers_topic);
    params_.debug_reason_topic = declareAndGet(*this, "debug_reason_topic", params_.debug_reason_topic);
    params_.debug_door_score_topic = declareAndGet(*this, "debug_door_score_topic", params_.debug_door_score_topic);
    params_.base_frame = declareAndGet(*this, "base_frame", params_.base_frame);

    params_.control_rate = declareAndGet(*this, "control_rate", params_.control_rate);
    params_.debug_publish_rate = declareAndGet(*this, "debug_publish_rate", params_.debug_publish_rate);
    params_.command_timeout = declareAndGet(*this, "command_timeout", params_.command_timeout);
    params_.lidar_timeout = declareAndGet(*this, "lidar_timeout", params_.lidar_timeout);
    params_.tf_fail_timeout = declareAndGet(*this, "tf_fail_timeout", params_.tf_fail_timeout);
    params_.min_fresh_lidars = std::clamp(
        declareAndGet(*this, "min_fresh_lidars", params_.min_fresh_lidars),
        1,
        3);
    params_.publish_debug_topics = declareAndGet(*this, "publish_debug_topics", params_.publish_debug_topics);

    params_.vehicle_length = declareAndGet(*this, "vehicle_length", params_.vehicle_length);
    params_.vehicle_width = declareAndGet(*this, "vehicle_width", params_.vehicle_width);
    params_.front_edge = declareAndGet(*this, "front_edge", params_.front_edge);
    params_.rear_edge = declareAndGet(*this, "rear_edge", params_.rear_edge);
    params_.footprint_padding = declareAndGet(*this, "footprint_padding", params_.footprint_padding);

    params_.scan_min_range = declareAndGet(*this, "scan_min_range", params_.scan_min_range);
    params_.scan_max_range = declareAndGet(*this, "scan_max_range", params_.scan_max_range);
    params_.scan_sample_step = std::max(1, declareAndGet(*this, "scan_sample_step", params_.scan_sample_step));

    params_.grid_resolution = declareAndGet(*this, "grid_resolution", params_.grid_resolution);
    params_.grid_x_min = declareAndGet(*this, "grid_x_min", params_.grid_x_min);
    params_.grid_x_max = declareAndGet(*this, "grid_x_max", params_.grid_x_max);
    params_.grid_y_min = declareAndGet(*this, "grid_y_min", params_.grid_y_min);
    params_.grid_y_max = declareAndGet(*this, "grid_y_max", params_.grid_y_max);
    params_.grid_inflation = declareAndGet(*this, "grid_inflation", params_.grid_inflation);

    params_.front_stop_distance = declareAndGet(*this, "front_stop_distance", params_.front_stop_distance);
    params_.front_slow_distance = declareAndGet(*this, "front_slow_distance", params_.front_slow_distance);
    params_.rear_stop_distance = declareAndGet(*this, "rear_stop_distance", params_.rear_stop_distance);
    params_.rear_slow_distance = declareAndGet(*this, "rear_slow_distance", params_.rear_slow_distance);
    params_.side_stop_distance = declareAndGet(*this, "side_stop_distance", params_.side_stop_distance);
    params_.side_slow_distance = declareAndGet(*this, "side_slow_distance", params_.side_slow_distance);
    params_.min_valid_points_for_stop =
        declareAndGet(*this, "min_valid_points_for_stop", params_.min_valid_points_for_stop);
    params_.emergency_stop_hold_time =
        declareAndGet(*this, "emergency_stop_hold_time", params_.emergency_stop_hold_time);

    params_.door_roi_x_min = declareAndGet(*this, "door_roi_x_min", params_.door_roi_x_min);
    params_.door_roi_x_max = declareAndGet(*this, "door_roi_x_max", params_.door_roi_x_max);
    params_.door_roi_y_min = declareAndGet(*this, "door_roi_y_min", params_.door_roi_y_min);
    params_.door_roi_y_max = declareAndGet(*this, "door_roi_y_max", params_.door_roi_y_max);
    params_.min_door_width = declareAndGet(*this, "min_door_width", params_.min_door_width);
    params_.max_door_width = declareAndGet(*this, "max_door_width", params_.max_door_width);
    params_.required_side_margin = declareAndGet(*this, "required_side_margin", params_.required_side_margin);
    params_.door_bin_resolution = declareAndGet(*this, "door_bin_resolution", params_.door_bin_resolution);
    params_.door_x_sample_step = declareAndGet(*this, "door_x_sample_step", params_.door_x_sample_step);
    params_.door_x_window = declareAndGet(*this, "door_x_window", params_.door_x_window);
    params_.door_stable_frame_count =
        std::max(1, declareAndGet(*this, "door_stable_frame_count", params_.door_stable_frame_count));
    params_.door_confidence_enter = declareAndGet(*this, "door_confidence_enter", params_.door_confidence_enter);
    params_.door_confidence_keep = declareAndGet(*this, "door_confidence_keep", params_.door_confidence_keep);
    params_.candidate_lost_timeout = declareAndGet(*this, "candidate_lost_timeout", params_.candidate_lost_timeout);

    params_.user_forward_threshold = declareAndGet(*this, "user_forward_threshold", params_.user_forward_threshold);
    params_.user_cancel_angular_threshold =
        declareAndGet(*this, "user_cancel_angular_threshold", params_.user_cancel_angular_threshold);
    params_.approach_distance = declareAndGet(*this, "approach_distance", params_.approach_distance);
    params_.align_distance = declareAndGet(*this, "align_distance", params_.align_distance);
    params_.enter_distance = declareAndGet(*this, "enter_distance", params_.enter_distance);
    params_.lateral_error_enter = declareAndGet(*this, "lateral_error_enter", params_.lateral_error_enter);
    params_.angle_error_enter_deg = declareAndGet(*this, "angle_error_enter_deg", params_.angle_error_enter_deg);
    params_.complete_hold_time = declareAndGet(*this, "complete_hold_time", params_.complete_hold_time);

    params_.manual_v_max = declareAndGet(*this, "manual_v_max", params_.manual_v_max);
    params_.manual_w_max = declareAndGet(*this, "manual_w_max", params_.manual_w_max);
    params_.obstacle_assist_v_max = declareAndGet(*this, "obstacle_assist_v_max", params_.obstacle_assist_v_max);
    params_.obstacle_assist_w_max = declareAndGet(*this, "obstacle_assist_w_max", params_.obstacle_assist_w_max);
    params_.door_v_max = declareAndGet(*this, "door_v_max", params_.door_v_max);
    params_.door_align_v_max = declareAndGet(*this, "door_align_v_max", params_.door_align_v_max);
    params_.door_w_max = declareAndGet(*this, "door_w_max", params_.door_w_max);
    params_.k_lateral = declareAndGet(*this, "k_lateral", params_.k_lateral);
    params_.k_angle = declareAndGet(*this, "k_angle", params_.k_angle);
    params_.k_gap = declareAndGet(*this, "k_gap", params_.k_gap);
    params_.blend_ratio_approach = declareAndGet(*this, "blend_ratio_approach", params_.blend_ratio_approach);
    params_.blend_ratio_align = declareAndGet(*this, "blend_ratio_align", params_.blend_ratio_align);
    params_.blend_ratio_passing = declareAndGet(*this, "blend_ratio_passing", params_.blend_ratio_passing);
    params_.max_accel_linear = declareAndGet(*this, "max_accel_linear", params_.max_accel_linear);
    params_.max_accel_angular = declareAndGet(*this, "max_accel_angular", params_.max_accel_angular);
}

void SharedControlAssistNode::createIo()
{
    using std::placeholders::_1;

    control_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        params_.input_control_topic,
        rclcpp::QoS(10),
        std::bind(&SharedControlAssistNode::onControl, this, _1));

    wheel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        params_.wheel_speed_topic,
        rclcpp::QoS(10),
        std::bind(&SharedControlAssistNode::onWheelTwist, this, _1));

    assist_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        params_.assist_enable_topic,
        rclcpp::QoS(1).transient_local(),
        std::bind(&SharedControlAssistNode::onAssistEnable, this, _1));

    const auto scan_qos = rclcpp::SensorDataQoS();
    left_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        params_.left_scan_topic,
        scan_qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            onLaserScan(msg, LidarSlot::FRONT_LEFT);
        });
    right_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        params_.right_scan_topic,
        scan_qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            onLaserScan(msg, LidarSlot::FRONT_RIGHT);
        });
    back_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        params_.back_scan_topic,
        scan_qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            onLaserScan(msg, LidarSlot::REAR);
        });

    control_pub_ = create_publisher<geometry_msgs::msg::Twist>(params_.output_control_topic, rclcpp::QoS(10));
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(params_.assist_points_topic, rclcpp::QoS(2));
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(params_.assist_local_grid_topic, rclcpp::QoS(2));
    state_pub_ = create_publisher<std_msgs::msg::String>(params_.assist_state_topic, rclcpp::QoS(10));
    marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(params_.debug_markers_topic, rclcpp::QoS(2));
    reason_pub_ = create_publisher<std_msgs::msg::String>(params_.debug_reason_topic, rclcpp::QoS(10));
    door_score_pub_ = create_publisher<std_msgs::msg::Float32>(params_.debug_door_score_topic, rclcpp::QoS(10));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, params_.control_rate)));
    timer_ = create_wall_timer(period, std::bind(&SharedControlAssistNode::onTimer, this));
}

void SharedControlAssistNode::onControl(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_user_cmd_ = *msg;
    has_user_cmd_ = true;
    last_user_cmd_time_ = now();
}

void SharedControlAssistNode::onWheelTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_wheel_twist_ = msg->twist;
    has_wheel_twist_ = true;
}

void SharedControlAssistNode::onAssistEnable(const std_msgs::msg::Bool::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    assist_enabled_ = msg->data;
}

void SharedControlAssistNode::onLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, LidarSlot slot)
{
    if (!msg || msg->header.frame_id.empty()) {
        return;
    }
    const auto slot_index = static_cast<std::size_t>(slot);

    geometry_msgs::msg::TransformStamped transform_msg;
    try {
        transform_msg = tf_buffer_->lookupTransform(
            params_.base_frame,
            msg->header.frame_id,
            msg->header.stamp,
            rclcpp::Duration::from_seconds(0.03));
    } catch (const tf2::TransformException&) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (tf_fail_since_[slot_index].nanoseconds() == 0) {
            tf_fail_since_[slot_index] = now();
        }
        return;
    }

    tf2::Quaternion rotation;
    tf2::fromMsg(transform_msg.transform.rotation, rotation);
    const tf2::Transform transform(
        rotation,
        tf2::Vector3(
            transform_msg.transform.translation.x,
            transform_msg.transform.translation.y,
            transform_msg.transform.translation.z));

    const double range_max = std::min(
        params_.scan_max_range,
        msg->range_max > 0.0 ? static_cast<double>(msg->range_max) : params_.scan_max_range);
    const double range_min = std::max(params_.scan_min_range, static_cast<double>(msg->range_min));

    std::vector<Point2D> points;
    points.reserve(msg->ranges.size() / static_cast<std::size_t>(params_.scan_sample_step) + 1);

    for (std::size_t i = 0; i < msg->ranges.size(); i += static_cast<std::size_t>(params_.scan_sample_step)) {
        const double range = msg->ranges[i];
        if (!std::isfinite(range) || range < range_min || range > range_max) {
            continue;
        }

        const double angle = static_cast<double>(msg->angle_min) +
            static_cast<double>(i) * static_cast<double>(msg->angle_increment);
        const tf2::Vector3 source_point(range * std::cos(angle), range * std::sin(angle), 0.0);
        const tf2::Vector3 base_point = transform * source_point;

        const double x = base_point.x();
        const double y = base_point.y();
        if (math::isInsideFootprint(params_, x, y)) {
            continue;
        }
        if (x < params_.grid_x_min - 0.5 || x > params_.grid_x_max + 0.5 ||
            y < params_.grid_y_min - 0.5 || y > params_.grid_y_max + 0.5) {
            continue;
        }

        points.push_back({x, y});
    }

    rclcpp::Time scan_stamp(msg->header.stamp);
    if (scan_stamp.nanoseconds() == 0) {
        scan_stamp = now();
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        scan_points_[slot_index] = std::move(points);
        scan_stamps_[slot_index] = scan_stamp;
        scan_valid_[slot_index] = true;
        tf_fail_since_[slot_index] = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
}

void SharedControlAssistNode::onTimer()
{
    const auto now_time = now();
    double dt = (now_time - last_timer_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
        dt = 1.0 / std::max(1.0, params_.control_rate);
    }
    last_timer_time_ = now_time;

    geometry_msgs::msg::Twist user_cmd;
    geometry_msgs::msg::Twist wheel_twist;
    bool has_wheel = false;
    bool assist_enabled = true;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        user_cmd = latest_user_cmd_;
        wheel_twist = latest_wheel_twist_;
        has_wheel = has_wheel_twist_;
        assist_enabled = assist_enabled_;
    }

    // Perception is intentionally pulled once per control tick so the rest of
    // the loop works on a consistent snapshot.
    int fresh_lidar_count = 0;
    const auto points = currentPointsLocked(now_time, fresh_lidar_count);
    const auto dist = perception_.computeObstacleDistance(points);
    const auto grid = perception_.buildLocalGrid(points, now_time);
    const auto door_result = door_detector_.detect(points, now_time);
    const auto& door = door_result.candidate;
    debug_reason_ = door_result.reason;

    const bool command_timeout = isCommandTimedOut(now_time);
    const double forward_velocity = has_wheel ? wheel_twist.linear.x : last_output_cmd_.linear.x;

    geometry_msgs::msg::Twist raw_cmd;
    bool immediate_stop = false;

    // The main decision path is: human command -> assist candidate -> safety
    // limit -> acceleration smoothing -> /nav/control.
    if (command_timeout) {
        mode_ = assist_enabled ? AssistMode::OBSTACLE_ASSIST : AssistMode::MANUAL_ONLY;
        setDoorState(DoorState::IDLE, now_time);
        debug_reason_ = "handle_command_timeout";
        immediate_stop = true;
    } else if (!assist_enabled) {
        mode_ = AssistMode::MANUAL_ONLY;
        setDoorState(DoorState::IDLE, now_time);
        raw_cmd = computeObstacleAssist(user_cmd);
        debug_reason_ = "manual_only";
    } else {
        updateDoorState(user_cmd, door, dist, now_time, dt, forward_velocity);
        if (mode_ == AssistMode::NARROW_DOOR_ASSIST) {
            raw_cmd = computeDoorAssist(user_cmd, door);
        } else {
            raw_cmd = computeObstacleAssist(user_cmd);
        }
    }

    auto safe_cmd = applySafetyLimit(raw_cmd, dist, fresh_lidar_count, now_time);
    if (immediate_stop) {
        safe_cmd = geometry_msgs::msg::Twist();
    }

    const bool emergency_active = immediate_stop || now_time < emergency_until_;
    const auto output_cmd = applyAccelerationLimit(safe_cmd, dt, emergency_active);

    control_pub_->publish(output_cmd);
    last_output_cmd_ = output_cmd;
    last_output_time_ = now_time;

    publishDebug(points, grid, door, dist, user_cmd, output_cmd, now_time);
}

std::vector<Point2D> SharedControlAssistNode::currentPointsLocked(
    const rclcpp::Time& now_time,
    int& fresh_lidar_count) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    fresh_lidar_count = 0;
    std::vector<Point2D> points;
    for (std::size_t i = 0; i < scan_points_.size(); ++i) {
        if (!scan_valid_[i]) {
            continue;
        }
        if (math::timeAgeSeconds(now_time, scan_stamps_[i]) > params_.lidar_timeout) {
            continue;
        }
        ++fresh_lidar_count;
        points.insert(points.end(), scan_points_[i].begin(), scan_points_[i].end());
    }
    return points;
}

void SharedControlAssistNode::updateDoorState(
    const geometry_msgs::msg::Twist& user_cmd,
    const DoorCandidate& door,
    const ObstacleDistance& dist,
    const rclcpp::Time& now_time,
    double dt,
    double forward_velocity)
{
    travel_in_state_ += std::max(0.0, forward_velocity) * std::max(0.0, dt);

    const bool user_forward = user_cmd.linear.x > params_.user_forward_threshold;
    const bool user_cancel =
        user_cmd.linear.x < -params_.user_forward_threshold ||
        std::abs(user_cmd.angular.z) > params_.user_cancel_angular_threshold;
    const bool door_keep = door.detected && door.confidence >= params_.door_confidence_keep;
    const bool door_enter = door.detected &&
        door.confidence >= params_.door_confidence_enter &&
        door.stable_count >= params_.door_stable_frame_count;
    const bool front_blocked = user_cmd.linear.x > 0.0 && dist.front_stop;

    // Any user release/cancel or safety stop wins over the narrow-door state.
    if (door_state_ != DoorState::IDLE && door_state_ != DoorState::COMPLETE && door_state_ != DoorState::ABORT) {
        if (!user_forward) {
            debug_reason_ = "door_assist_user_released";
            setDoorState(DoorState::ABORT, now_time);
        } else if (user_cancel) {
            debug_reason_ = "door_assist_user_cancel";
            setDoorState(DoorState::ABORT, now_time);
        } else if (front_blocked) {
            debug_reason_ = "door_assist_front_stop";
            setDoorState(DoorState::ABORT, now_time);
        } else if (!door_keep) {
            debug_reason_ = "door_candidate_lost";
            setDoorState(DoorState::ABORT, now_time);
        }
    }

    switch (door_state_) {
    case DoorState::IDLE:
        mode_ = AssistMode::OBSTACLE_ASSIST;
        if (user_forward && door_keep) {
            setDoorState(DoorState::SEARCH, now_time);
        }
        break;
    case DoorState::SEARCH:
        // SEARCH only watches/stabilizes the candidate. Automatic steering
        // starts after the detector reaches the stronger enter threshold.
        mode_ = AssistMode::OBSTACLE_ASSIST;
        if (door_enter) {
            setDoorState(DoorState::APPROACH, now_time);
            mode_ = AssistMode::NARROW_DOOR_ASSIST;
        }
        break;
    case DoorState::APPROACH:
        mode_ = AssistMode::NARROW_DOOR_ASSIST;
        if (door.distance <= params_.align_distance) {
            setDoorState(DoorState::ALIGN, now_time);
        }
        break;
    case DoorState::ALIGN:
        mode_ = AssistMode::NARROW_DOOR_ASSIST;
        if (std::abs(door.lateral_error) <= params_.lateral_error_enter &&
            std::abs(door.angle_error) <= params_.angle_error_enter_deg * math::kPi / 180.0) {
            setDoorState(DoorState::ENTER, now_time);
        }
        break;
    case DoorState::ENTER:
        mode_ = AssistMode::NARROW_DOOR_ASSIST;
        if (travel_in_state_ >= std::max(0.25, params_.enter_distance)) {
            setDoorState(DoorState::PASSING, now_time);
        }
        break;
    case DoorState::PASSING:
        mode_ = AssistMode::NARROW_DOOR_ASSIST;
        if (travel_in_state_ >= std::max(0.35, params_.vehicle_length * 0.65)) {
            setDoorState(DoorState::EXIT, now_time);
        }
        break;
    case DoorState::EXIT:
        mode_ = AssistMode::NARROW_DOOR_ASSIST;
        if (travel_in_state_ >= std::max(0.25, params_.vehicle_length * 0.35)) {
            setDoorState(DoorState::COMPLETE, now_time);
        }
        break;
    case DoorState::COMPLETE:
        mode_ = AssistMode::OBSTACLE_ASSIST;
        if ((now_time - state_enter_time_).seconds() >= params_.complete_hold_time) {
            setDoorState(DoorState::IDLE, now_time);
        }
        break;
    case DoorState::ABORT:
        mode_ = AssistMode::OBSTACLE_ASSIST;
        if (!user_forward || (now_time - state_enter_time_).seconds() >= params_.complete_hold_time) {
            setDoorState(DoorState::IDLE, now_time);
        }
        break;
    }
}

void SharedControlAssistNode::setDoorState(DoorState state, const rclcpp::Time& now_time)
{
    if (door_state_ == state) {
        return;
    }
    door_state_ = state;
    state_enter_time_ = now_time;
    travel_in_state_ = 0.0;
}

geometry_msgs::msg::Twist SharedControlAssistNode::computeObstacleAssist(
    const geometry_msgs::msg::Twist& user_cmd) const
{
    geometry_msgs::msg::Twist cmd;
    const bool assist_active = mode_ != AssistMode::MANUAL_ONLY;
    const double v_max = assist_active ? params_.obstacle_assist_v_max : params_.manual_v_max;
    const double w_max = assist_active ? params_.obstacle_assist_w_max : params_.manual_w_max;
    cmd.linear.x = math::clamp(user_cmd.linear.x, -v_max, v_max);
    cmd.angular.z = math::clampAbs(user_cmd.angular.z, w_max);
    return cmd;
}

geometry_msgs::msg::Twist SharedControlAssistNode::computeDoorAssist(
    const geometry_msgs::msg::Twist& user_cmd,
    const DoorCandidate& door) const
{
    geometry_msgs::msg::Twist cmd;
    const bool aligning = door_state_ == DoorState::ALIGN || door_state_ == DoorState::ENTER;
    const double v_max = aligning ? params_.door_align_v_max : params_.door_v_max;
    double blend = params_.blend_ratio_approach;
    if (door_state_ == DoorState::ALIGN || door_state_ == DoorState::ENTER) {
        blend = params_.blend_ratio_align;
    } else if (door_state_ == DoorState::PASSING || door_state_ == DoorState::EXIT) {
        blend = params_.blend_ratio_passing;
    }
    blend = math::clamp(blend, 0.0, 1.0);

    const double gap_error = math::finiteOr(door.left_clearance, 0.0) - math::finiteOr(door.right_clearance, 0.0);
    const double w_assist =
        params_.k_lateral * door.lateral_error +
        params_.k_angle * door.angle_error +
        params_.k_gap * gap_error;

    cmd.linear.x = math::clamp(user_cmd.linear.x, 0.0, v_max);
    cmd.angular.z = math::clampAbs(blend * w_assist + (1.0 - blend) * user_cmd.angular.z, params_.door_w_max);
    return cmd;
}

geometry_msgs::msg::Twist SharedControlAssistNode::applySafetyLimit(
    const geometry_msgs::msg::Twist& cmd,
    const ObstacleDistance& dist,
    int fresh_lidar_count,
    const rclcpp::Time& now_time)
{
    geometry_msgs::msg::Twist out = cmd;
    bool emergency = false;

    // TF failures are tracked per lidar slot. One healthy lidar must not mask a
    // broken transform from another required sensor.
    std::array<rclcpp::Time, 3> tf_fail_since;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tf_fail_since = tf_fail_since_;
    }

    const bool command_nonzero = std::abs(out.linear.x) > 1.0e-4 || std::abs(out.angular.z) > 1.0e-4;
    if (command_nonzero && fresh_lidar_count < params_.min_fresh_lidars) {
        emergency = true;
        mode_ = AssistMode::FAULT;
        debug_reason_ = "lidar_timeout";
    } else {
        for (const auto& fail_since : tf_fail_since) {
            if (fail_since.nanoseconds() == 0) {
                continue;
            }
            if (math::timeAgeSeconds(now_time, fail_since) >= params_.tf_fail_timeout) {
                emergency = true;
                mode_ = AssistMode::FAULT;
                debug_reason_ = "tf_failure_timeout";
                break;
            }
        }
    }

    if (now_time < emergency_until_) {
        emergency = true;
    }

    const bool front_block = out.linear.x > 0.0 && dist.front_stop;
    const bool rear_block = out.linear.x < 0.0 && dist.rear_stop;
    const bool left_turn_block = out.angular.z > 0.0 &&
        dist.left_points >= params_.min_valid_points_for_stop &&
        dist.left_min <= params_.side_stop_distance;
    const bool right_turn_block = out.angular.z < 0.0 &&
        dist.right_points >= params_.min_valid_points_for_stop &&
        dist.right_min <= params_.side_stop_distance;
    const bool corner_block =
        (out.linear.x > 0.0 && out.angular.z > 0.0 && dist.front_left_min <= params_.side_stop_distance) ||
        (out.linear.x > 0.0 && out.angular.z < 0.0 && dist.front_right_min <= params_.side_stop_distance) ||
        (out.linear.x < 0.0 && out.angular.z < 0.0 && dist.rear_left_min <= params_.side_stop_distance) ||
        (out.linear.x < 0.0 && out.angular.z > 0.0 && dist.rear_right_min <= params_.side_stop_distance);

    if (front_block || rear_block || left_turn_block || right_turn_block || corner_block) {
        emergency = true;
        debug_reason_ = "software_emergency_stop";
        emergency_until_ = now_time + rclcpp::Duration::from_seconds(params_.emergency_stop_hold_time);
    }

    if (emergency) {
        return geometry_msgs::msg::Twist();
    }

    if (dist.valid) {
        if (out.linear.x > 0.0 && std::isfinite(dist.front_min)) {
            out.linear.x *= math::distanceLimitScale(
                dist.front_min,
                params_.front_stop_distance,
                params_.front_slow_distance);
        } else if (out.linear.x < 0.0 && std::isfinite(dist.rear_min)) {
            out.linear.x *= math::distanceLimitScale(
                dist.rear_min,
                params_.rear_stop_distance,
                params_.rear_slow_distance);
        }

        if (out.angular.z > 0.0 && std::isfinite(dist.left_min)) {
            out.angular.z *= math::distanceLimitScale(
                dist.left_min,
                params_.side_stop_distance,
                params_.side_slow_distance);
        } else if (out.angular.z < 0.0 && std::isfinite(dist.right_min)) {
            out.angular.z *= math::distanceLimitScale(
                dist.right_min,
                params_.side_stop_distance,
                params_.side_slow_distance);
        }
    }

    return out;
}

geometry_msgs::msg::Twist SharedControlAssistNode::applyAccelerationLimit(
    const geometry_msgs::msg::Twist& cmd,
    double dt,
    bool emergency) const
{
    if (emergency) {
        return cmd;
    }

    geometry_msgs::msg::Twist out = cmd;
    out.linear.x = math::rateLimit(cmd.linear.x, last_output_cmd_.linear.x, params_.max_accel_linear * dt);
    out.angular.z = math::rateLimit(cmd.angular.z, last_output_cmd_.angular.z, params_.max_accel_angular * dt);
    return out;
}

void SharedControlAssistNode::publishPointCloud(const std::vector<Point2D>& points, const rclcpp::Time& stamp)
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = params_.base_frame;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto& p : points) {
        *iter_x = static_cast<float>(p.x);
        *iter_y = static_cast<float>(p.y);
        *iter_z = 0.0F;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    points_pub_->publish(cloud);
}

void SharedControlAssistNode::publishDebug(
    const std::vector<Point2D>& points,
    const nav_msgs::msg::OccupancyGrid& grid,
    const DoorCandidate& door,
    const ObstacleDistance& dist,
    const geometry_msgs::msg::Twist& user_cmd,
    const geometry_msgs::msg::Twist& output_cmd,
    const rclcpp::Time& now_time)
{
    if (!params_.publish_debug_topics) {
        return;
    }
    const double interval = 1.0 / std::max(1.0, params_.debug_publish_rate);
    if ((now_time - last_debug_publish_time_).seconds() < interval) {
        return;
    }
    last_debug_publish_time_ = now_time;

    publishPointCloud(points, now_time);
    grid_pub_->publish(grid);

    std_msgs::msg::String state_msg;
    std::ostringstream state;
    state << std::fixed << std::setprecision(3)
          << "{"
          << "\"mode\":\"" << modeName() << "\","
          << "\"door_state\":\"" << doorStateName() << "\","
          << "\"active\":" << (mode_ != AssistMode::MANUAL_ONLY ? "true" : "false") << ","
          << "\"door_detected\":" << (door.detected ? "true" : "false") << ","
          << "\"door_confidence\":" << door.confidence << ","
          << "\"door_width\":" << door.width << ","
          << "\"door_center_y\":" << door.center_y << ","
          << "\"front_min\":" << math::finiteOr(dist.front_min, -1.0) << ","
          << "\"rear_min\":" << math::finiteOr(dist.rear_min, -1.0) << ","
          << "\"left_min\":" << math::finiteOr(dist.left_min, -1.0) << ","
          << "\"right_min\":" << math::finiteOr(dist.right_min, -1.0) << ","
          << "\"user_linear\":" << user_cmd.linear.x << ","
          << "\"user_angular\":" << user_cmd.angular.z << ","
          << "\"output_linear\":" << output_cmd.linear.x << ","
          << "\"output_angular\":" << output_cmd.angular.z << ","
          << "\"reason\":\"" << debug_reason_ << "\""
          << "}";
    state_msg.data = state.str();
    state_pub_->publish(state_msg);

    std_msgs::msg::String reason_msg;
    reason_msg.data = debug_reason_;
    reason_pub_->publish(reason_msg);

    std_msgs::msg::Float32 score_msg;
    score_msg.data = static_cast<float>(door.confidence);
    door_score_pub_->publish(score_msg);

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = params_.base_frame;
    clear.header.stamp = now_time;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    if (door.detected) {
        visualization_msgs::msg::Marker door_line;
        door_line.header.frame_id = params_.base_frame;
        door_line.header.stamp = now_time;
        door_line.ns = "door";
        door_line.id = 1;
        door_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        door_line.action = visualization_msgs::msg::Marker::ADD;
        door_line.scale.x = 0.035;
        door_line.color.r = 0.1F;
        door_line.color.g = 0.9F;
        door_line.color.b = 0.2F;
        door_line.color.a = 1.0F;
        door_line.points.push_back(point(door.left_boundary_x, door.left_boundary_y, 0.05));
        door_line.points.push_back(point(door.right_boundary_x, door.right_boundary_y, 0.05));
        markers.markers.push_back(door_line);

        visualization_msgs::msg::Marker center_line;
        center_line.header.frame_id = params_.base_frame;
        center_line.header.stamp = now_time;
        center_line.ns = "door";
        center_line.id = 2;
        center_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        center_line.action = visualization_msgs::msg::Marker::ADD;
        center_line.scale.x = 0.025;
        center_line.color.r = 0.1F;
        center_line.color.g = 0.4F;
        center_line.color.b = 1.0F;
        center_line.color.a = 1.0F;
        center_line.points.push_back(point(0.0, 0.0, 0.08));
        center_line.points.push_back(point(door.center_x, door.center_y, 0.08));
        markers.markers.push_back(center_line);

        visualization_msgs::msg::Marker text;
        text.header.frame_id = params_.base_frame;
        text.header.stamp = now_time;
        text.ns = "door";
        text.id = 3;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position = point(door.center_x, door.center_y, 0.35);
        text.scale.z = 0.16;
        text.color.r = 1.0F;
        text.color.g = 1.0F;
        text.color.b = 1.0F;
        text.color.a = 1.0F;
        std::ostringstream label;
        label << std::fixed << std::setprecision(2) << door.width << "m " << door.confidence;
        text.text = label.str();
        markers.markers.push_back(text);
    }

    marker_pub_->publish(markers);
}

bool SharedControlAssistNode::isCommandTimedOut(const rclcpp::Time& now_time) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return !has_user_cmd_ || math::timeAgeSeconds(now_time, last_user_cmd_time_) > params_.command_timeout;
}

std::string SharedControlAssistNode::modeName() const
{
    switch (mode_) {
    case AssistMode::MANUAL_ONLY:
        return "MANUAL_ONLY";
    case AssistMode::OBSTACLE_ASSIST:
        return "OBSTACLE_ASSIST";
    case AssistMode::NARROW_DOOR_ASSIST:
        return "NARROW_DOOR_ASSIST";
    case AssistMode::RECOVERY_ASSIST:
        return "RECOVERY_ASSIST";
    case AssistMode::FAULT:
        return "FAULT";
    }
    return "UNKNOWN";
}

std::string SharedControlAssistNode::doorStateName() const
{
    switch (door_state_) {
    case DoorState::IDLE:
        return "IDLE";
    case DoorState::SEARCH:
        return "SEARCH";
    case DoorState::APPROACH:
        return "APPROACH";
    case DoorState::ALIGN:
        return "ALIGN";
    case DoorState::ENTER:
        return "ENTER";
    case DoorState::PASSING:
        return "PASSING";
    case DoorState::EXIT:
        return "EXIT";
    case DoorState::COMPLETE:
        return "COMPLETE";
    case DoorState::ABORT:
        return "ABORT";
    }
    return "UNKNOWN";
}

}  // namespace shared_control_assist
