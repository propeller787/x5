#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <proj.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_localization/srv/from_ll.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

const std::string base_frameid = "base_link";

// 全局 Node 指针
std::shared_ptr<rclcpp::Node> global_node;

std::shared_ptr<rclcpp::Node> get_node() { return global_node; }

bool g_debug = false;
geometry_msgs::msg::TwistStamped::SharedPtr g_last_wheel_twist;

using namespace std::chrono_literals;

class BaseWheel {
  public:
    double yaw;
    rclcpp::Time prev_wheel_stamp;

  public:
    BaseWheel() : yaw(0.0) {}
    virtual ~BaseWheel() = default;

    void reset_yaw(double new_yaw) { yaw = new_yaw; }

    std::tuple<double, double, double> on_twist(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        rclcpp::Time current_stamp = msg->header.stamp;

        if (prev_wheel_stamp.nanoseconds() == 0) {
            prev_wheel_stamp = current_stamp;
            return {0.0, 0.0, yaw};
        }

        double delta_time = (current_stamp - prev_wheel_stamp).seconds();
        prev_wheel_stamp = current_stamp;

        double delta_x = msg->twist.linear.x * delta_time * std::cos(yaw);
        double delta_y = msg->twist.linear.x * delta_time * std::sin(yaw);
        double delta_yaw = msg->twist.angular.z * delta_time;
        return {delta_x, delta_y, delta_yaw};
    }
};

class Wheel : public BaseWheel {
  public:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_vel_sub;
    std::function<void(double, double, double, const geometry_msgs::msg::TwistStamped::SharedPtr)> delta_odom_callback;
    double dt;

  public:
    Wheel(std::function<void(double, double, double, const geometry_msgs::msg::TwistStamped::SharedPtr)> callback) : delta_odom_callback(callback), dt(-2.2) {
        wheel_vel_sub = global_node->create_subscription<geometry_msgs::msg::TwistStamped>("/wheel/twist", 10, std::bind(&Wheel::wheel_callback, this, _1));
    }

    void wheel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        g_last_wheel_twist = msg;
        auto [delta_x, delta_y, delta_yaw] = on_twist(msg);
        yaw += delta_yaw;
        RCLCPP_INFO(global_node->get_logger(), "delta_x: %f", delta_x);
        double current_yaw = dt + yaw;
        delta_odom_callback(delta_x, delta_y, current_yaw, msg);
        if (g_debug)
            printf("debug Wheel wheel_callback %d\n", __LINE__);
    }
};

class ImuWheel : public BaseWheel {
  public:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_vel_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr imu_wheel_publisher_;
    std::function<void(double, double, double, const geometry_msgs::msg::TwistStamped::SharedPtr)> delta_odom_callback;
    double imu_yaw;
    double dt;
    rclcpp::TimerBase::SharedPtr watchdog_timer;
    rclcpp::Time last_wheel_msg_time;
    bool wheel_data_active;

  public:
    ImuWheel(std::function<void(double, double, double, const geometry_msgs::msg::TwistStamped::SharedPtr)> callback)
        : delta_odom_callback(callback), imu_yaw(0.0), dt(0.0), wheel_data_active(false) {

        if (g_debug)
            printf("debug ImuWheel init %d\n", __LINE__);
        wheel_vel_sub = global_node->create_subscription<geometry_msgs::msg::TwistStamped>("/wheel/twist", 10, std::bind(&ImuWheel::wheel_callback, this, _1));
        imu_sub = global_node->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&ImuWheel::imu_callback, this, _1));

        imu_wheel_publisher_ = global_node->create_publisher<geometry_msgs::msg::TwistStamped>("/imu/twist", 10);
        // 添加看门狗定时器，每秒检查一次
        watchdog_timer = global_node->create_wall_timer(std::chrono::seconds(1), std::bind(&ImuWheel::watchdog_callback, this));
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        if (g_debug)
            printf("debug ImuWheel imu_callback %d\n", __LINE__);

        double imu_yaw_new = yaw + dt;
        if (std::abs(imu_yaw - imu_yaw_new) > 0.01) {
            RCLCPP_INFO(global_node->get_logger(), "imu_yaw: %f", imu_yaw_new);
        }
        if (false) {
            geometry_msgs::msg::TwistStamped imu_twist;
            imu_twist.header.stamp = global_node->get_clock()->now();
            imu_twist.header.frame_id = "base_link";
            imu_twist.twist.linear.x = 0.0;
            imu_twist.twist.linear.y = 0.0;
            imu_twist.twist.linear.z = 0.0;
            imu_twist.twist.angular.z = msg->angular_velocity.z;
            imu_wheel_publisher_->publish(imu_twist);
        }

        imu_yaw = imu_yaw_new;
        reset_yaw(imu_yaw);
    }

    void wheel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        last_wheel_msg_time = global_node->get_clock()->now();
        wheel_data_active = true;
        auto [delta_x, delta_y, delta_yaw] = on_twist(msg);
        delta_odom_callback(delta_x, delta_y, imu_yaw, msg);
        if (g_debug)
            printf("debug ImuWheel wheel_callback %d\n", __LINE__);
    }

    void watchdog_callback() {
        if (!wheel_data_active) {
            // 还没有收到过任何数据，不处理
            return;
        }

        auto now = global_node->get_clock()->now();
        auto time_since_last_msg = (now - last_wheel_msg_time).seconds();

        if (time_since_last_msg > 1.0) {
            // 超过1秒没有收到新数据，发送零速度
            auto zero_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
            zero_msg->header.stamp = now;
            zero_msg->twist.linear.x = 0.0;
            zero_msg->twist.angular.z = 0.0;

            // 模拟收到零速度消息
            wheel_callback(zero_msg);

            RCLCPP_WARN(global_node->get_logger(), "No wheel data for %.3f seconds, sending zero velocity", time_since_last_msg);
        }
    }
};

class CmdvelWheel : public BaseWheel {
  public:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr wheel_vel_sub;
    std::function<void(double, double, double, const geometry_msgs::msg::TwistStamped::SharedPtr)> delta_odom_callback;
    double dt;

  public:
    CmdvelWheel(std::function<void(double, double, double, const geometry_msgs::msg::TwistStamped::SharedPtr)> callback)
        : delta_odom_callback(callback), dt(0.0) {
        wheel_vel_sub = global_node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 30, std::bind(&CmdvelWheel::wheel_callback, this, _1));
    }

    void wheel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto twist_stamped = std::make_shared<geometry_msgs::msg::TwistStamped>();
        twist_stamped->header.stamp = global_node->get_clock()->now();
        twist_stamped->header.frame_id = base_frameid;
        twist_stamped->twist = *msg;
        g_last_wheel_twist = twist_stamped;

        auto [delta_x, delta_y, delta_yaw] = on_twist(twist_stamped);
        yaw += delta_yaw;
        double current_yaw = dt + yaw;
        delta_odom_callback(delta_x, delta_y, current_yaw, twist_stamped);
        if (g_debug)
            printf("debug CmdvelWheel wheel_callback %d\n", __LINE__);
    }
};

class OdomPub {
  public:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;
    rclcpp::TimerBase::SharedPtr timer_tf;
    rclcpp::TimerBase::SharedPtr timer_odom;

    double odom_x;
    double odom_y;
    double yaw;
    geometry_msgs::msg::TwistStamped::SharedPtr twist_;
    rclcpp::Time last_report_time_;

    double last_report_x;
    double last_report_y;
    double last_report_yaw;

  public:
    OdomPub() : odom_x(0.0), odom_y(0.0), yaw(0.0) {
        last_report_time_ = global_node->now();
        if (g_debug)
            printf("debug OdomPub init %d\n", __LINE__);
        odom_pub = global_node->create_publisher<nav_msgs::msg::Odometry>("/odom_update", 10);
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(global_node);
        static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(global_node);

        timer_tf = global_node->create_wall_timer(std::chrono::milliseconds(160), std::bind(&OdomPub::timer_tf_callback, this));
        // timer_odom = global_node->create_wall_timer(
        //     std::chrono::milliseconds(100),
        //     std::bind(&OdomPub::timer_odom_callback, this));

        publish_static_transforms();
    }

    void timer_tf_callback() {
        auto current_time = global_node->now();
        rclcpp::Duration duration = current_time - last_report_time_;
        int64_t milliseconds_diff = duration.nanoseconds() / 1'000'000; // 将纳秒转换为毫秒
        const int64_t diff_min = 50;
        if (std::abs(milliseconds_diff) > diff_min) {
            if (twist_.get()) {
                twist_->header.stamp = global_node->get_clock()->now();
            }
            publish_odom();
            publish_map_to_odom_transform();
            publish_static_transforms();
        }
    }

    void timer_odom_callback() {
        // publish_odom();
        // publish_map_to_odom_transform();
    }

    void publish_static_transforms() {
        // Static transform: map -> utm
        geometry_msgs::msg::TransformStamped static_transform_utm;
        static_transform_utm.header.stamp = global_node->get_clock()->now();
        if (twist_.get()) {
            static_transform_utm.header.stamp = twist_->header.stamp;
        }
        static_transform_utm.header.frame_id = "map";
        static_transform_utm.child_frame_id = "utm";
        static_transform_utm.transform.translation.x = 0.0;
        static_transform_utm.transform.translation.y = 0.0;
        static_transform_utm.transform.translation.z = 0.0;
        static_transform_utm.transform.rotation.x = 0.0;
        static_transform_utm.transform.rotation.y = 0.0;
        static_transform_utm.transform.rotation.z = 0.0;
        static_transform_utm.transform.rotation.w = 1.0;

        static_tf_broadcaster->sendTransform(static_transform_utm);

        // Static transform: odom -> base_link
        geometry_msgs::msg::TransformStamped static_transform_odom;
        static_transform_odom.header.stamp = global_node->get_clock()->now();
        if (twist_.get()) {
            static_transform_odom.header.stamp = twist_->header.stamp;
        }
        static_transform_odom.header.frame_id = "odom";
        static_transform_odom.child_frame_id = base_frameid;
        static_transform_odom.transform.translation.x = 0.0;
        static_transform_odom.transform.translation.y = 0.0;
        static_transform_odom.transform.translation.z = 0.0;
        static_transform_odom.transform.rotation.x = 0.0;
        static_transform_odom.transform.rotation.y = 0.0;
        static_transform_odom.transform.rotation.z = 0.0;
        static_transform_odom.transform.rotation.w = 1.0;

        static_tf_broadcaster->sendTransform(static_transform_odom);
    }

    void publish_odom() {
        last_report_x = odom_x;
        last_report_y = odom_y;
        last_report_yaw = yaw;
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = global_node->get_clock()->now();
        last_report_time_ = odom_msg.header.stamp;
        odom_msg.header.frame_id = "odom";

        odom_msg.pose.pose.position.x = odom_x;
        odom_msg.pose.pose.position.y = odom_y;
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.pose.pose.orientation = yaw_to_quaternion(yaw);
        if (twist_.get()) {
            odom_msg.header.stamp = twist_->header.stamp;
            odom_msg.twist.twist = twist_->twist;
        }
        odom_pub->publish(odom_msg);
    }

    void publish_map_to_odom_transform() {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = global_node->get_clock()->now();
        if (twist_.get()) {
            transform.header.stamp = twist_->header.stamp;
        }
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";
        transform.transform.translation.x = odom_x;
        transform.transform.translation.y = odom_y;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = yaw_to_quaternion(yaw);

        tf_broadcaster->sendTransform(transform);
    }

    void update_yaw(double new_yaw) { yaw = new_yaw; }

    void update_diff_pose(double x, double y) {
        odom_x += x;
        odom_y += y;
    }

    void update_twist(const geometry_msgs::msg::TwistStamped::SharedPtr twist) { twist_ = twist; }

    void reset_pos(double x, double y) {
        odom_x = x;
        odom_y = y;
    }

    geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
        geometry_msgs::msg::Quaternion q;
        q.z = std::sin(yaw / 2.0);
        q.w = std::cos(yaw / 2.0);
        return q;
    }
};

class GpsOdom {
  public:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
    rclcpp::Service<robot_localization::srv::FromLL>::SharedPtr srv;
    std::function<void(double, double)> odom_callback;

    PJ_CONTEXT *ctx;
    PJ *utm_proj;
    double origin_utm_x;
    double origin_utm_y;
    double prev_utm_x;
    double prev_utm_y;
    double yaw;

  public:
    GpsOdom(std::function<void(double, double)> callback)
        : odom_callback(callback), origin_utm_x(0.0), origin_utm_y(0.0), prev_utm_x(0.0), prev_utm_y(0.0), yaw(0.0) {
        ctx = proj_context_create();
        utm_proj = proj_create(ctx, "+proj=utm +zone=50 +ellps=WGS84 +datum=WGS84 +units=m +no_defs");

        gps_sub = global_node->create_subscription<sensor_msgs::msg::NavSatFix>("/sensor/gps/data", 10, std::bind(&GpsOdom::gps_callback, this, _1));
        srv = global_node->create_service<robot_localization::srv::FromLL>("fromLL1", std::bind(&GpsOdom::from_ll_callback, this, _1, _2));

        init_map_origin();
        init_gps();
    }

    ~GpsOdom() {
        proj_destroy(utm_proj);
        proj_context_destroy(ctx);
    }

    void init_map_origin() {
        double latitude = 39.8924;
        double longitude = 116.121;
        auto [utm_x, utm_y] = convert_gps_to_utm(latitude, longitude);
        origin_utm_x = utm_x;
        origin_utm_y = utm_y;
    }

    void init_gps() {
        auto msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
        msg->latitude = 39.893729;
        msg->longitude = 116.121481;
        msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        msg->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        msg->header.frame_id = "gps_link";
        gps_callback(msg);

        msg->latitude = 39.893729;
        msg->longitude = 116.124481;
        gps_callback(msg);
    }

    void from_ll_callback(const std::shared_ptr<robot_localization::srv::FromLL::Request> request,
                          std::shared_ptr<robot_localization::srv::FromLL::Response> response) {
        double latitude = request->ll_point.latitude;
        double longitude = request->ll_point.longitude;

        auto [utm_x, utm_y] = convert_gps_to_utm(latitude, longitude);
        double odom_x = utm_x - origin_utm_x;
        double odom_y = utm_y - origin_utm_y;
        RCLCPP_INFO(global_node->get_logger(), "FromLL x:%f y:%f", odom_x, odom_y);

        response->map_point.x = odom_x;
        response->map_point.y = odom_y;
        response->map_point.z = 0.0;
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (msg->latitude < 10 || msg->longitude < 10)
            return;
        if (msg->latitude < 39.8 || msg->latitude > 39.9)
            return;
        if (msg->longitude < 116.1 || msg->longitude > 116.2)
            return;
        if (msg->status.status < 0.0)
            return;

        auto [utm_x, utm_y] = convert_gps_to_utm(msg->latitude, msg->longitude);
        if (utm_x == prev_utm_x && utm_y == prev_utm_y)
            return;

        if (origin_utm_x == 0.0 && origin_utm_y == 0.0) {
            init_map_origin();
            prev_utm_x = utm_x;
            prev_utm_y = utm_y;
            RCLCPP_INFO(global_node->get_logger(), "Set origin UTM: x=%f, y=%f", origin_utm_x, origin_utm_y);
            return;
        }

        if (prev_utm_x != 0.0 && prev_utm_y != 0.0) {
            double delta_x = utm_x - prev_utm_x;
            double delta_y = utm_y - prev_utm_y;
            yaw = std::atan2(delta_y, delta_x);
        }

        prev_utm_x = utm_x;
        prev_utm_y = utm_y;

        double odom_x = utm_x - origin_utm_x;
        double odom_y = utm_y - origin_utm_y;
        odom_callback(odom_x, odom_y);
    }

    std::tuple<double, double> convert_gps_to_utm(double latitude, double longitude) {
        PJ_COORD gps = proj_coord(longitude, latitude, 0, 0);
        PJ_COORD utm = proj_trans(utm_proj, PJ_FWD, gps);
        return {utm.enu.e, utm.enu.n};
    }
};

class GPSOdomPublisher : public rclcpp::Node {
  public:
    std::shared_ptr<OdomPub> odompub;
    std::shared_ptr<ImuWheel> wheel;
    // std::shared_ptr<GpsOdom> gpsodom;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub;

  public:
    GPSOdomPublisher() : Node("gps_odom_publisher") {
        if (g_debug)
            printf("debug GPSOdomPublisher init %d\n", __LINE__);
        odompub = std::make_shared<OdomPub>();

        wheel = std::make_shared<ImuWheel>(std::bind(&GPSOdomPublisher::delta_odom_callback, this, _1, _2, _3, _4));

        // gpsodom = std::make_shared<GpsOdom>(
        //     std::bind(&GPSOdomPublisher::gps_odom_callback, this, _1, _2));

        pose_subscription = global_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&GPSOdomPublisher::pose_callback, this, _1));

        clicked_sub = global_node->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10,
                                                                                         std::bind(&GPSOdomPublisher::clicked_point_callback, this, _1));

        odompub->publish_static_transforms();
    }

    // void gps_odom_callback(double odom_x, double odom_y) {
    //     if (std::abs(odom_x) > 500 || std::abs(odom_y) > 500) {
    //         RCLCPP_WARN(get_logger(), "Invalid odom coordinates: x=%f, y=%f", odom_x, odom_y);
    //         return;
    //     }

    //     double r = 0.25;
    //     double yaw = wheel->yaw - 0.5 * M_PI;
    //     double diff_x = r * std::cos(yaw);
    //     double diff_y = r * std::sin(yaw);
    //     double new_x = odom_x + diff_x;
    //     double new_y = odom_y + diff_y;

    //     odompub->reset_pos(new_x, new_y);

    //     double diff1 = std::abs(odompub->odom_x - last_report_x);
    //     double diff2 = std::abs(odompub->odom_y - last_report_y);
    //     double diff3 = std::abs(odompub->yaw - last_report_yaw);
    //     if (std::abs(odompub->odom_x - last_report_x) < 0.02 &&
    //         std::abs(odompub->odom_y - last_report_y) < 0.02 &&
    //         std::abs(odompub->yaw - last_report_yaw) < 0.03) {
    //         //不需要发布
    //         // printf("不需要发布 diff1:%f d2:%f d3:%f\n", diff1, diff2, diff3);
    //         return;
    //     }
    //     last_report_x = odompub->odom_x;
    //     last_report_y = odompub->odom_y;
    //     last_report_yaw = odompub->yaw;
    //     // printf("发布 diff1:%f d2:%f d3:%f\n", diff1, diff2, diff3);

    //     odompub->publish_odom();
    //     odompub->publish_map_to_odom_transform();
    // }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double dt = yaw + wheel->dt - wheel->yaw;
        if (dt > 2 * M_PI)
            dt -= 2 * M_PI;
        if (dt < -2 * M_PI)
            dt += 2 * M_PI;

        double a = wheel->dt;
        wheel->dt = dt;
        if (dt != a) {
            RCLCPP_INFO(get_logger(), "yaw: %f self.wheel.yaw:%f dt:%f -> %f", yaw, wheel->yaw, a, dt);
        }

        RCLCPP_INFO(get_logger(), "目标位姿: x=%f, y=%f, yaw=%f", x, y, yaw);

        odompub->update_yaw(yaw);
        odompub->reset_pos(x, y);
    }

    void delta_odom_callback(double delta_x, double delta_y, double yaw, const geometry_msgs::msg::TwistStamped::SharedPtr twist) {
        if (g_debug)
            printf("debug GPSOdomPublisher delta_odom_callback %d\n", __LINE__);
        odompub->update_yaw(yaw);
        odompub->update_diff_pose(delta_x, delta_y);
        odompub->update_twist(twist);

        // double diff1 = std::abs(odompub->odom_x - odompub->last_report_x);
        // double diff2 = std::abs(odompub->odom_y - odompub->last_report_y);
        // double diff3 = std::abs(odompub->yaw - odompub->last_report_yaw);
        if (std::abs(odompub->odom_x - odompub->last_report_x) < 0.016 && std::abs(odompub->odom_y - odompub->last_report_y) < 0.016 &&
            std::abs(odompub->yaw - odompub->last_report_yaw) < 0.021) {
            // 不需要发布
            //  printf("不需要发布 diff1:%f d2:%f d3:%f\n", diff1, diff2, diff3);
            return;
        }
        // printf("发布 diff1:%f d2:%f d3:%f\n", diff1, diff2, diff3);

        odompub->publish_odom();
        odompub->publish_map_to_odom_transform();
    }

    void clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        double x = msg->point.x;
        double y = msg->point.y;
        odompub->reset_pos(x, y);
        RCLCPP_INFO(get_logger(), "Received point: x=%f, y=%f, z=%f", x, y, msg->point.z);
    }
};

class KeyStatus {
  private:
    struct JoystickData {
        std::chrono::time_point<std::chrono::steady_clock> timestamp;
        geometry_msgs::msg::Twist msg;
        std::string direction;
        int level;
    };

    std::deque<JoystickData> joystick_history_;
    double history_duration_ = 2.0;
    std::string confirm_key_direct_ = "stop";
    std::string last_key_direct_ = "stop";

    std::function<void(const std::string &, int)> cb_new_key_;
    std::function<void(const std::string &, int)> cb_key_hold_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_key_hit_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_key_hold_;

  public:
    KeyStatus() {
        auto node = get_node();
        pub_key_hit_ = node->create_publisher<std_msgs::msg::String>("/narrow/key/hit", 10);
        pub_key_hold_ = node->create_publisher<std_msgs::msg::String>("/narrow/key/hold", 10);
    }

    bool is_up() const { return confirm_key_direct_ == "up"; }

    bool is_stop() const { return confirm_key_direct_ == "stop"; }

    bool is_back() const {
        return confirm_key_direct_ == "stop"; // Note: Same as Python version, may need review
    }

    void set_cb(const std::function<void(const std::string &, int)> &new_key_cb, const std::function<void(const std::string &, int)> &key_hold_cb) {
        cb_new_key_ = new_key_cb;
        cb_key_hold_ = key_hold_cb;
    }

    void on_joystick(const geometry_msgs::msg::Twist &msg) {
        auto current_time = std::chrono::steady_clock::now();
        std::string key_direct = "stop";
        int key_level = 0;

        if (msg.linear.x > 0.08) {
            key_direct = "up";
            key_level = 1;
        } else if (msg.linear.x < -0.08) {
            key_direct = "back";
            key_level = 0;
        } else {
            key_direct = "stop";
            key_level = 0;
        }

        if (!joystick_history_.empty()) {
            const auto &last_direct = joystick_history_.back().direction;
            if (last_direct != key_direct) {
                joystick_history_.clear();
            }
        }

        joystick_history_.push_back({current_time, msg, key_direct, key_level});

        // Remove data older than history duration
        while (!joystick_history_.empty() &&
               std::chrono::duration_cast<std::chrono::duration<double>>(current_time - joystick_history_.front().timestamp).count() > history_duration_) {
            joystick_history_.pop_front();
        }

        auto t1 = std::chrono::steady_clock::now();
        if (joystick_history_.size() == 1) {
            key_hit(msg, 0.0, key_direct, key_level, true);
        } else {
            double dur = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - joystick_history_.front().timestamp).count();
            key_hit(msg, dur, key_direct, key_level, false);
        }
        auto t2 = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        if (duration > 0.08) {
            RCLCPP_WARN(get_node()->get_logger(), "阻塞错误");
        }
    }

  private:
    void key_hit(const geometry_msgs::msg::Twist &key_value, double dur, const std::string &key_direct, int key_level, bool is_new) {
        auto pub_msg = std_msgs::msg::String();

        if (is_new) {
            auto now = std::chrono::steady_clock::now();
            auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
            RCLCPP_INFO(get_node()->get_logger(), "key is_new %s --------------------- %ld", key_direct.c_str(), timestamp);
            pub_msg.data = key_direct;
            pub_key_hit_->publish(pub_msg);
            if (cb_new_key_) {
                cb_new_key_(key_direct, key_level);
            }
            last_key_direct_ = key_direct;
        }

        if (confirm_key_direct_ != key_direct) {
            if (dur > 0.5) {
                pub_msg.data = key_direct;
                pub_key_hold_->publish(pub_msg);
                confirm_key_direct_ = key_direct;
                if (cb_key_hold_) {
                    auto now = std::chrono::steady_clock::now();
                    auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
                    RCLCPP_INFO(get_node()->get_logger(), "key change %s ++++++++++++++++++++++++++ %ld", key_direct.c_str(), timestamp);

                    cb_key_hold_(key_direct, key_level);
                }
            }
        }
    }
};

class TopicForwarder : public rclcpp::Node {
  public:
    TopicForwarder(KeyStatus *keyStatus) : Node("vlago_ctrl"), last_watchar_speed_(1.40) {
        keyStatus_ = keyStatus;
        // Initialize parameters with default values
        global_node->declare_parameter("max_speed", 1.2);
        global_node->declare_parameter("max_angular", 0.6);
        global_node->declare_parameter("imu_offset", 2.327402);

        // Get initial parameter values
        max_speed_ = global_node->get_parameter("max_speed").as_double();
        max_angular_ = global_node->get_parameter("max_angular").as_double();
        imu_offset_ = global_node->get_parameter("imu_offset").as_double();

        RCLCPP_INFO(global_node->get_logger(), "Initial max_speed: %.2f", max_speed_);
        RCLCPP_INFO(global_node->get_logger(), "Initial max_angular: %.2f", max_angular_);
        RCLCPP_INFO(global_node->get_logger(), "Initial imu_offset: %.2f", imu_offset_);

        last_mode_ = "";
        last_status_ = "start";
        last_status_time_ = global_node->now();
        ;

        // Add parameter callback
        param_callback_handle_ = global_node->add_on_set_parameters_callback(std::bind(&TopicForwarder::parameter_callback, this, std::placeholders::_1));

        // Initialize subscribers
        cmd_vel_raw_sub_ = global_node->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_raw", 10, std::bind(&TopicForwarder::listener_raw_callback, this, std::placeholders::_1));
        cmd_vel_sub_ = global_node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
                                                                                   std::bind(&TopicForwarder::listener_callback, this, std::placeholders::_1));

        handle_ctrl_sub_ = global_node->create_subscription<geometry_msgs::msg::Twist>(
            "/handle/control", 10, std::bind(&TopicForwarder::listener_handle_ctrl_callback, this, std::placeholders::_1));

        scene_mode_sub_ = global_node->create_subscription<std_msgs::msg::String>(
            "/scene_services/mode", 10, std::bind(&TopicForwarder::listener_scene_mode_callback, this, std::placeholders::_1));

        narrow_status_sub_ = global_node->create_subscription<std_msgs::msg::String>(
            "/narrow/status", 10, std::bind(&TopicForwarder::listener_narrow_status_callback, this, std::placeholders::_1));

        // Initialize publisher
        control_pub_ = global_node->create_publisher<geometry_msgs::msg::Twist>("/nav/control", 10);

        // Initialize timers
        timer_ = global_node->create_wall_timer(std::chrono::milliseconds(15), std::bind(&TopicForwarder::timer_callback, this));
        // watchcar_timer_ = global_node->create_wall_timer(std::chrono::seconds(1),
        // std::bind(&TopicForwarder::timer_watchcar_callback, this));

        // Initialize TF
        // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(global_node->get_clock());
        // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize time points
        last_nav2_time_ = global_node->now();
        last_nav2_raw_time_ = global_node->now();
        last_handle_time_ = global_node->now();
        last_app_time_ = global_node->now();
    }

  private:
    void change_maxspeed(double maxspeed) {
        max_speed_ = maxspeed;
        RCLCPP_INFO(global_node->get_logger(), "Updated max_speed to: %.2f", max_speed_);
    }

    bool is_navctrl() {
        if (is_narrow()) {
            return true;
        } else if (is_followme()) {
            return true;
        } else if (is_autoindoor()) {
            return true;
        } else if (is_recharge()) {
            return false;
        }
        return false;
    }

    bool is_disable() { return false; }

    bool is_narrow() { return last_mode_ == "narrow"; }
    bool is_recharge() { return last_mode_ == "recharge"; }
    double narrow_duration() 
    {
        auto current_time = std::chrono::steady_clock::now();
        auto a = mode_change_time_;
        auto mode = last_mode_;
        if (mode == "narrow") {
            double dur = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - mode_change_time_).count();
            return dur;
        }
        return -1;
    }

    bool is_followme() { return last_mode_ == "followme"; }

    bool is_autoindoor() { return last_mode_ == "autoindoor"; }

    // void timer_watchcar_callback()
    // {
    //     if (!is_navctrl()) return;
    //     try {
    //         // Get transform from map to base_link
    //         auto trans = tf_buffer_->lookupTransform(
    //             "map", "base_link", tf2::TimePointZero);

    //         auto pos = trans.transform.translation;
    //         bool check_in = false;

    //         if (pos.x > 162.0 && pos.x < 168.5 && pos.y > 128.0 && pos.y < 138.0) {
    //             check_in = true;
    //             if (last_watchar_speed_ > 0.42) {
    //                 last_watchar_speed_ -= 0.2;
    //                 change_maxspeed(last_watchar_speed_);
    //             }
    //         }

    //         if (!check_in && last_watchar_speed_ < 1.39) {
    //             last_watchar_speed_ += 0.2;
    //             change_maxspeed(last_watchar_speed_);
    //         }
    //     } catch (const tf2::TransformException &ex) {
    //         RCLCPP_ERROR(global_node->get_logger(), "Could not transform base_link to odom: %s", ex.what());
    //     }
    // }

    rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
            if (param.get_name() == "max_speed") {
                max_speed_ = param.as_double();
                RCLCPP_INFO(global_node->get_logger(), "Updated max_speed to: %.2f", max_speed_);
            } else if (param.get_name() == "max_angular") {
                max_angular_ = param.as_double();
                RCLCPP_INFO(global_node->get_logger(), "Updated max_angular to: %.2f", max_angular_);
            } else if (param.get_name() == "imu_offset") {
                imu_offset_ = param.as_double();
                RCLCPP_INFO(global_node->get_logger(), "Updated imu_offset to: %.2f", imu_offset_);
            }
        }

        return result;
    }

    void listener_handle_ctrl_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_handle_msg_ = *msg;
        last_handle_time_ = global_node->now();
        if (keyStatus_) {
            keyStatus_->on_joystick(*msg);
        }
    }

    void listener_handle_app_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!is_navctrl())
            return;
        last_app_msg_ = *msg;
        last_app_time_ = global_node->now();
    }

    void listener_scene_mode_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string scene_mode = msg->data;
        if (last_mode_ != scene_mode) {
            mode_change_time_ = std::chrono::steady_clock::now();
            printf("change mode %s -> %s\n", last_mode_.c_str(), scene_mode.c_str());
        }
        last_mode_ = scene_mode;
        if (last_mode_ != "narrow") {
            last_status_ = "start";
        }
    }

    void listener_narrow_status_callback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string &status = msg->data;
        if (last_status_ != status) {
            if (last_mode_ == "narrow") {
                printf("change status %s -> %s\n", last_status_.c_str(), status.c_str());
                last_status_ = status;
            } else {
                last_status_ = "start";
            }
            last_status_time_ = global_node->now();
        }
    }

    void listener_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!is_navctrl())
            return;
        auto current_time = global_node->now();
        rclcpp::Duration duration = current_time - last_nav2_raw_time_;
        int64_t milliseconds_diff = duration.nanoseconds() / 1'000'000; // 将纳秒转换为毫秒
        const int diff_max = 100;
        if (std::abs(milliseconds_diff) > diff_max) {
            last_nav2_time_ = current_time;
            geometry_msgs::msg::Twist modified_msg = *msg;
            last_nav2_msg_ = modified_msg;
        }
    }

    void listener_raw_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!is_navctrl())
            return;
        last_nav2_time_ = last_nav2_raw_time_ = global_node->now();
        geometry_msgs::msg::Twist modified_msg = *msg;
        last_nav2_msg_ = modified_msg;
    }

    void timer_callback() {
        if (is_disable())
            return;
        if (!is_navctrl())
            return;
        auto current_time = global_node->now();
        geometry_msgs::msg::Twist msg;
        bool msg_set = false;

        if (last_mode_ != "narrow") {
            last_status_ = "start";
        }
        if (0) {
            rclcpp::Duration duration = current_time - last_status_time_;
            int64_t milliseconds_diff = duration.nanoseconds() / 1'000'000; // 将纳秒转换为毫秒
            const int diff_max = 100;
            if (std::abs(milliseconds_diff) < diff_max) {
                if ((last_status_ == "stop") && last_mode_ != "narrow") {
                    // 20个台阶
                    const int max_count = 20;
                    int index = milliseconds_diff / (diff_max / max_count);
                    if (last_handle_msg_ && last_nav2_msg_) {
                        float speed1_x = last_handle_msg_->linear.x;
                        float speed2_x = last_nav2_msg_->linear.x;
                        float dx = (speed1_x - speed2_x) / max_count;
                        float target_x = speed2_x + (index * dx);
                        geometry_msgs::msg::Twist send_msg = *last_handle_msg_;
                        send_msg.linear.x = target_x;
                        printf("hand_x: %.3f  cmd_x: %.3f  target_x :%.3f\n", speed1_x, speed2_x, target_x);
                        control_pub_->publish(send_msg);
                    }
                }
                return;
            }
        }

        if (last_status_ == "start") {
            if ((current_time - last_nav2_time_).seconds() < 0.5 && last_nav2_msg_) {
                msg = *last_nav2_msg_;
                msg_set = true;
            }
        } else if (last_status_ == "pause") {
            // control_pub_->publish(msg);
        } else if (last_status_ == "stop") {
            if ((current_time - last_app_time_).seconds() < 0.5 && last_app_msg_) {
                msg = *last_handle_msg_;
                msg_set = true;
            }
        } else {
            RCLCPP_ERROR(global_node->get_logger(), "last_status error %s ", last_status_.c_str());
        }
        if (!msg_set) {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
        }
        if (is_followme()) {
            msg.linear.x = msg.linear.x * 1.3;
            msg.angular.z = msg.angular.z * 1.3;
        }
        /*else if (is_narrow()) {
            static float narrow_max_x = 0.2;
            if (msg.linear.x > narrow_max_x)
                narrow_max_x = msg.linear.x;
            float ratio = std::abs(msg.linear.x) / narrow_max_x + 0.1;
            ratio = std::min(1.0f, ratio);
            msg.angular.z = ratio * msg.angular.z;
        }*/
        if (is_narrow()) {
            double dur = narrow_duration();
            printf("debug narrow dur %.3f g_last_wheel_twist:%p\n", dur, g_last_wheel_twist.get());
            const float wait_sec = 3.0;
            if (dur < wait_sec and g_last_wheel_twist.get()) { 
                float radio = 1.0;
                double ctrl_x = msg.linear.x;
                double ctrl_z = msg.angular.z;
                double wheel_x = g_last_wheel_twist->twist.linear.x;
                double wheel_z = g_last_wheel_twist->twist.angular.z;
                printf("debug narrow dur %.3f ctrl_x:%.3f ctrl_z:%.3f wheel_x:%.3f wheel_z:%.3f\n", dur, ctrl_x, ctrl_z, wheel_x, wheel_z);
                wheel_z = 0.0;
                
                float radio_ctrl = (1.0/wait_sec) * dur;
                float radio_wheel = 1.0 - radio_ctrl;
                /*
                dur = 0.0 -> radio = 1.0
                dur = 0.5 -> radio = 0.7
                dur = 1.0 -> radio = 0.5
                dur = 1.5 -> radio = 0.25
                dur = 2.0 -> radio = 0.0
                */
                double x = ctrl_x * radio_ctrl + wheel_x * radio_wheel;
                double z = ctrl_z * radio_ctrl + wheel_z * radio_wheel;
                printf("debug x: %.3f z:%.3f\n", x, z);
                msg.linear.x = x;
                msg.angular.z = z;
                printf("debug 1 x: %.3f z:%.3f\n", msg.linear.x, msg.angular.z);
            }
        }
        control_pub_->publish(msg);
        static uint32_t s_count = 0;
        if (s_count++ % 10 == 1) {
            RCLCPP_INFO(global_node->get_logger(), "send cmd %s ctrl : %.4f, z: %.4f", last_status_.c_str(), msg.linear.x, msg.angular.z);
        }
        send_count_++;
    }

    // Parameters
    double max_speed_;
    double max_angular_;
    double imu_offset_;
    double last_watchar_speed_;

    KeyStatus *keyStatus_;

    // Messages and timestamps
    std::optional<geometry_msgs::msg::Twist> last_nav2_msg_;
    std::optional<geometry_msgs::msg::Twist> last_handle_msg_;
    std::optional<geometry_msgs::msg::Twist> last_app_msg_;
    std::string last_mode_;
    std::chrono::time_point<std::chrono::steady_clock> mode_change_time_;
    std::string last_status_; // start pause stop
    rclcpp::Time last_nav2_time_;
    rclcpp::Time last_nav2_raw_time_;
    rclcpp::Time last_handle_time_;
    rclcpp::Time last_app_time_;
    rclcpp::Time last_status_time_;

    // ROS2 objects
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_raw_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr handle_ctrl_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scene_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr narrow_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr watchcar_timer_;
    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Counter
    int send_count_ = 0;
};


class OdomUpdater : public rclcpp::Node
{
public:
    OdomUpdater() : Node("odom_updater")
    {
        // 初始化Odometry消息，位置设为(0,0,0)
        initializeOdomMessage();

        scene_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/scene_services/mode", 10, std::bind(&OdomUpdater::listener_scene_mode_callback, this, std::placeholders::_1));
        // 订阅/odom/update话题
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_update", 10, std::bind(&OdomUpdater::odomUpdateCallback, this, _1));
        last_report_time_ = std::chrono::steady_clock::now();
        // 发布/odom话题
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        publisher_slow_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_slow", 10);

        // 创建30Hz的定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / 30),  // 33ms ≈ 30Hz
            std::bind(&OdomUpdater::timerCallback, this));
    }

private:
    void initializeOdomMessage()
    {
        // 初始化header
        current_odom_.header.frame_id = "odom";
        current_odom_.child_frame_id = "base_link";

        // 初始化位置为(0,0,0)
        current_odom_.pose.pose.position.x = 0.0;
        current_odom_.pose.pose.position.y = 0.0;
        current_odom_.pose.pose.position.z = 0.0;

        // 初始化方向为单位四元数(无旋转)
        current_odom_.pose.pose.orientation.x = 0.0;
        current_odom_.pose.pose.orientation.y = 0.0;
        current_odom_.pose.pose.orientation.z = 0.0;
        current_odom_.pose.pose.orientation.w = 1.0;

        // 初始化速度为零
        current_odom_.twist.twist.linear.x = 0.0;
        current_odom_.twist.twist.linear.y = 0.0;
        current_odom_.twist.twist.linear.z = 0.0;
        current_odom_.twist.twist.angular.x = 0.0;
        current_odom_.twist.twist.angular.y = 0.0;
        current_odom_.twist.twist.angular.z = 0.0;
    }

    void listener_scene_mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string scene_mode = msg->data;
        if (last_mode_ != scene_mode) {
            printf("change mode %s -> %s\n", last_mode_.c_str(), scene_mode.c_str());
        }
        last_mode_ = scene_mode;
        if (last_mode_ == "narrow") {
            slow_dur_ = 80;
        } else {
            slow_dur_ = 250;
        }
    }

    void odomUpdateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;  // 保存最新数据
        auto wait_duration = std::chrono::milliseconds(slow_dur_);
        if ((std::chrono::steady_clock::now() - last_report_time_ > wait_duration))
        {
            publisher_slow_->publish(current_odom_);
            last_report_time_ = std::chrono::steady_clock::now();
        }
    }

    void timerCallback()
    {
        // 更新时间戳为当前时间
        current_odom_.header.stamp = this->now();
        // 发布消息
        publisher_->publish(current_odom_);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scene_mode_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_slow_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry current_odom_;
    std::chrono::steady_clock::time_point last_report_time_;
    std::string last_mode_;
    int slow_dur_ = 250;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    global_node = std::make_shared<rclcpp::Node>("narrow_localization_gloabl");
    auto node = std::make_shared<GPSOdomPublisher>();
    auto key_status = std::make_shared<KeyStatus>();
    auto odom_update = std::make_shared<OdomUpdater>();
    auto topicForwarder = std::make_shared<TopicForwarder>(key_status.get());

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(global_node);
    exec.add_node(odom_update);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
