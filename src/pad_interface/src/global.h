#pragma once

#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" // for doTransform
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
//#include <utils/singleton.h>
#include <vector>
using geometry_msgs::msg::Quaternion;
namespace service {
class Global {
    struct Point {
        double x;
        double y;
    };
    // 计算两点间的距离
    double distance(const Point& p1, const Point& p2)
    {
        return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
    }

    // 计算C点坐标
    Point calculateCPoint(const Point& A, const Point& B, double distanceBC)
    {
        Point C;
        // 计算AB向量的长度
        double lengthAB = distance(A, B);

        // 通过单位向量缩放来计算C点
        C.x = B.x - (B.x - A.x) / lengthAB * distanceBC;
        C.y = B.y - (B.y - A.y) / lengthAB * distanceBC;

        return C;
    }

private:
    Global(void);

public:
    static Global* getInstance()
    {
        // 双检锁
        if (instance == nullptr) {
            std::lock_guard<std::mutex> lock(mtx); // 加锁保证线程安全
            if (instance == nullptr) {
                instance = new Global();
            }
        }
        return instance;
    }

public:
    std::shared_ptr<rclcpp::Node> node_ { nullptr };
    // followme
    void send_goal_pose(const geometry_msgs::msg::PointStamped::SharedPtr& point);
    void cannel_goal_pose();

    // summon
    void send_goal_summon(const geometry_msgs::msg::PointStamped::SharedPtr& point);
    // 电梯第一阶段位置
    void send_elevator_goal_pose(const geometry_msgs::msg::PointStamped::SharedPtr& point,
        const geometry_msgs::msg::PointStamped::SharedPtr& pointdire);
    // 电梯第一阶段方向
    void send_elevator_direction(const geometry_msgs::msg::PointStamped::SharedPtr& point);

    void monitor_position();
    // 电梯第二阶段 位置
    void send_elecator_second_goal_pose(const geometry_msgs::msg::PointStamped::SharedPtr& point,
        const geometry_msgs::msg::PointStamped::SharedPtr& pointdire);
    void monitor_second_position();
    void send_elecator_third_goal_pose(const geometry_msgs::msg::PointStamped::SharedPtr& point,
        const geometry_msgs::msg::PointStamped::SharedPtr& pointdire);
    void monitor_third_position();

    void onImuResult(const sensor_msgs::msg::Imu msg);

    void send_playback_pose(const geometry_msgs::msg::PointStamped::SharedPtr& point);

    void rotate(double angle, bool clockwise);
    // 前进固定米数
    void advance(double length);

public:
    // 进电梯第一阶段状态 0 未开始  1 第一阶段 2 第二阶段  3第三阶段 4 第四阶段 5 第一阶段方向调整 6 出电梯
    std::atomic<int16_t> elevator_first_step_ = 0;
    std::atomic<int16_t> elevator_first_ = 0;

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; // 用于缓存tf变换
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // 监听tf变换
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_; // 发布转换后的点
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    // 等待电梯停止的坐标
    double elevator_robox_x = 0.0;
    double elevator_robox_y = 0.0;
    // 修改朝向的坐标
    double heading_x = 0.0;
    double heading_y = 0.0;
    static Global* instance;
    static std::mutex mtx;
    // 监控机器人位置线程
    std::thread monitoring_thread_;
    std::atomic<bool> target_reached_ = false;
    std::atomic<bool> monitoring_thread_reached_ = false;
    // 第一阶段方向
    geometry_msgs::msg::PointStamped directStan_;
    // 第二阶段方向
    geometry_msgs::msg::PointStamped directSecondtan_;
    std::thread monitoring_secode_thread_;
    std::atomic<bool> target_secode_reached_ = false;
    std::atomic<bool> monitoring_thread_secode_reached_ = false;
    // 第三阶段方向
    geometry_msgs::msg::PointStamped directThirdtan_;
    std::thread monitoring_third_thread_;
    std::atomic<bool> target_third_reached_ = false;
    std::atomic<bool> monitoring_thread_third_reached_ = false;
    // IMU
    sensor_msgs::msg::Imu imu_;
    double imu_z_;
    geometry_msgs::msg::Quaternion quate_;
    double heading_1_ = -132.767;
};
}