#include "global.h"
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <utils/topic.h>
using std::placeholders::_1;
namespace service {
Global* Global::instance = nullptr;
std::mutex Global::mtx;
Global::Global(void)
{
    node_ = std::make_shared<rclcpp::Node>("service");
    // 创建一个Buffer对象
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    // 创建一个TransformListener，并将其与Buffer关联
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(std::string("/imu"), 10, std::bind(&Global::onImuResult, this, _1));
}

void Global::onImuResult(const sensor_msgs::msg::Imu msg)
{
    imu_ = msg;
    tf2::Quaternion quat(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // double yaw_deg = yaw * 180.0 / M_PI;
    imu_z_ = yaw;
    // std::cout << "onImuResult yaw_deg: " << yaw_deg << std::endl;
}
void Global::rotate(double angle, bool clockwise)
{
    geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    auto rotation = transformStamped.transform.rotation;

    // 提取当前的yaw角
    tf2::Quaternion quat(rotation.x, rotation.y, rotation.z, rotation.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, current_yaw;
    m.getRPY(roll, pitch, current_yaw);
}

void Global::send_goal_summon(const geometry_msgs::msg::PointStamped::SharedPtr& point)
{
    geometry_msgs::msg::PoseStamped poseStamp;

    poseStamp.header = point->header;
    poseStamp.header.frame_id = "map";
    poseStamp.pose.position.x = point->point.x;
    poseStamp.pose.position.y = point->point.y;
    pose_publisher_->publish(poseStamp);
}

void Global::send_goal_pose(const geometry_msgs::msg::PointStamped::SharedPtr& point)
{
    // 获取当前时间
    rclcpp::Time target_time
        = point->header.stamp;
    // 设置一个时间容忍度，允许较小的时间误差
    rclcpp::Duration tolerance(0, 300000000); // 0.2秒的时间容忍度
    // 检查是否存在从base_link到map的变换
    try {
        // 如果存在，获取这个变换
        auto transform = tf_buffer_->lookupTransform("map", "base_link", target_time, tolerance);
        // 机器人的当前位置
        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;
        // 创建一个新的PointStamped，用于存储转换后的点
        geometry_msgs::msg::PointStamped transformed_point;
        transformed_point.header = point->header;
        transformed_point.header.frame_id = "map";
        transformed_point.point = point->point;

        // 应用变换
        tf2::doTransform(*point, transformed_point, transform);
        geometry_msgs::msg::PoseStamped poseStamp;
        poseStamp.header = transformed_point.header;
        poseStamp.pose.position.x = transformed_point.point.x;
        poseStamp.pose.position.y = transformed_point.point.y;
        // 停止到前1.5米位置
        {
            Point A = { robot_x, robot_y };
            Point B = { poseStamp.pose.position.x, poseStamp.pose.position.y };
            double distanceBC = 1.5f;
            Point c = calculateCPoint(A, B, distanceBC);
            poseStamp.pose.position.x = c.x;
            poseStamp.pose.position.y = c.y;
        }

        // 计算机器人需要朝向的角度
        double delta_x = poseStamp.pose.position.x - robot_x;
        double delta_y = poseStamp.pose.position.y - robot_y;
        double angle_to_goal = atan2(delta_y, delta_x); // 目标点相对于机器人的角度

        tf2::Quaternion goal_orientation;
        goal_orientation.setRPY(0, 0, angle_to_goal); // 设置目标朝向的四元数
        poseStamp.pose.orientation.x = goal_orientation.x();
        poseStamp.pose.orientation.y = goal_orientation.y();
        poseStamp.pose.orientation.z = goal_orientation.z();
        poseStamp.pose.orientation.w = goal_orientation.w();

        // 发布转换后的点
        pose_publisher_->publish(poseStamp);
        std::cout << " ------------send_goal_pose----------------- x " << poseStamp.pose.position.x
                  << " y " << poseStamp.pose.position.y << std::endl;
    } catch (tf2::TransformException& ex) {
        // 如果没有找到变换，输出错误信息
        RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    }
}

void Global::cannel_goal_pose()
{
    try {
        // 设置目标时间和容差
        rclcpp::Time target_time = rclcpp::Time();
        rclcpp::Duration tolerance = rclcpp::Duration::from_seconds(0.2);

        auto transform = tf_buffer_->lookupTransform("map", "base_link", 
                                                        target_time, tolerance);
        
        // 创建base_link坐标系下的目标点
        geometry_msgs::msg::PoseStamped base_link_pose;
        base_link_pose.header.frame_id = "base_link";
        base_link_pose.header.stamp = target_time;
        base_link_pose.pose.position.x = 0.1;
        base_link_pose.pose.position.y = 0.0;
        base_link_pose.pose.position.z = 0.0;
        
        // 设置方向（相对于base_link的正前方）
        tf2::Quaternion quat_base;
        quat_base.setRPY(0, 0, 0);
        base_link_pose.pose.orientation.x = quat_base.x();
        base_link_pose.pose.orientation.y = quat_base.y();
        base_link_pose.pose.orientation.z = quat_base.z();
        base_link_pose.pose.orientation.w = quat_base.w();
        
        // 使用tf2::doTransform进行坐标变换
        geometry_msgs::msg::PoseStamped map_pose;
        tf2::doTransform(base_link_pose, map_pose, transform);

        // 发布转换后的坐标
        pose_publisher_->publish(map_pose);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(node_->get_logger(), "lookupTransform失败: %s", ex.what());
        RCLCPP_WARN(node_->get_logger(), "请确保以下TF存在: map -> base_link");
    }
}

void Global::advance(double length)
{

    // 获取当前时间
    rclcpp::Time target_time = rclcpp::Time();
    // 设置一个时间容忍度，允许较小的时间误差
    rclcpp::Duration tolerance(0, 200000000); // 0.2秒的时间容忍度
    geometry_msgs::msg::PointStamped::SharedPtr pointSta = std::make_shared<geometry_msgs::msg::PointStamped>();
    pointSta->header.stamp = rclcpp::Time();
    pointSta->point.x = length;
    pointSta->point.y = 0;
    // 检查是否存在从base_link到map的变换
    try {
        // 如果存在，获取这个变换
        auto transform = tf_buffer_->lookupTransform("map", "base_link", target_time, tolerance);
        // 机器人的当前位置
        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;
        // 创建一个新的PointStamped，用于存储转换后的点
        geometry_msgs::msg::PointStamped transformed_point;
        transformed_point.header.frame_id = "map";
        transformed_point.point.x = length;
        transformed_point.point.y = 0;
        // 应用变换
        tf2::doTransform(*pointSta, transformed_point, transform);
        geometry_msgs::msg::PoseStamped poseStamp;
        poseStamp.header = transformed_point.header;
        poseStamp.pose.position.x = transformed_point.point.x;
        poseStamp.pose.position.y = transformed_point.point.y;
        // 停止到前1.5米位置
        {
            Point A = { robot_x, robot_y };
            Point B = { poseStamp.pose.position.x, poseStamp.pose.position.y };
            double distanceBC = 0.3f;
            Point c = calculateCPoint(A, B, distanceBC);
            poseStamp.pose.position.x = c.x;
            poseStamp.pose.position.y = c.y;
        }

        // 计算机器人需要朝向的角度
        double delta_x = poseStamp.pose.position.x - robot_x;
        double delta_y = poseStamp.pose.position.y - robot_y;
        double angle_to_goal = atan2(delta_y, delta_x); // 目标点相对于机器人的角度

        tf2::Quaternion goal_orientation;
        goal_orientation.setRPY(0, 0, angle_to_goal); // 设置目标朝向的四元数
        poseStamp.pose.orientation.x = goal_orientation.x();
        poseStamp.pose.orientation.y = goal_orientation.y();
        poseStamp.pose.orientation.z = goal_orientation.z();
        poseStamp.pose.orientation.w = goal_orientation.w();

        // 发布转换后的点
        pose_publisher_->publish(poseStamp);
        std::cout << " ------------advance----------------- x " << poseStamp.pose.position.x
                  << " y " << poseStamp.pose.position.y << std::endl;
    } catch (tf2::TransformException& ex) {
        // 如果没有找到变换，输出错误信息
        RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    }
}

void Global::send_elevator_goal_pose(const geometry_msgs::msg::PointStamped::SharedPtr& point,
    const geometry_msgs::msg::PointStamped::SharedPtr& pointdire)
{
    // 获取当前时间
    rclcpp::Time target_time = point->header.stamp;
    // 设置一个时间容忍度，允许较小的时间误差
    rclcpp::Duration tolerance(0, 200000000); // 0.2秒的时间容忍度
    // 检查是否存在从base_link到map的变换
    try {
        // 如果存在，获取这个变换
        auto transform = tf_buffer_->lookupTransform("map", "base_link", target_time, tolerance);
        // 机器人的当前位置
        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;
        // 创建一个新的PointStamped，用于存储转换后的点
        geometry_msgs::msg::PointStamped transformed_point;
        transformed_point.header = point->header;
        transformed_point.header.frame_id = "map";
        transformed_point.point = point->point;

        // 应用变换
        tf2::doTransform(*point, transformed_point, transform);
        geometry_msgs::msg::PoseStamped poseStamp;
        poseStamp.header = transformed_point.header;
        poseStamp.pose.position.x = transformed_point.point.x;
        poseStamp.pose.position.y = transformed_point.point.y;
        // 方向应用变换
        directStan_.header = pointdire->header;
        directStan_.header.frame_id = "map";
        directStan_.point = pointdire->point;

        tf2::doTransform(*pointdire, directStan_, transform);
        // 记录发送位置
        elevator_robox_x = transformed_point.point.x;
        elevator_robox_y = transformed_point.point.y;
        // 启动线程监控是否已到达指定位置，然后进行方向纠正
        if (!monitoring_thread_reached_) {
            // 创建监控线程
            std::cout << " 监控线程启动 " << std::endl;
            target_reached_ = false;
            monitoring_thread_ = std::thread(&Global::monitor_position, this);
            monitoring_thread_.detach();
            monitoring_thread_reached_ = true;
        }

        //  计算机器人需要朝向的角度
        double delta_x = poseStamp.pose.position.x - robot_x;
        double delta_y = poseStamp.pose.position.y - robot_y;
        double angle_to_goal = atan2(delta_y, delta_x); // 目标点相对于机器人的角度

        tf2::Quaternion goal_orientation;
        goal_orientation.setRPY(0, 0, angle_to_goal); // 设置目标朝向的四元数
        poseStamp.pose.orientation.x = goal_orientation.x();
        poseStamp.pose.orientation.y = goal_orientation.y();
        poseStamp.pose.orientation.z = goal_orientation.z();
        poseStamp.pose.orientation.w = goal_orientation.w();

        // 发布转换后的点
        pose_publisher_->publish(poseStamp);
        std::cout << " ------------send_elevator_goal_pose----------------- x "
                  << poseStamp.pose.position.x
                  << " y " << poseStamp.pose.position.y << std::endl;
    } catch (tf2::TransformException& ex) {
        // 如果没有找到变换，输出错误信息
        RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    }
}

// 计算两个四元数的点积
double quaternionDotProduct(const Quaternion& q1, const Quaternion& q2)
{
    return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
}

// 计算两个四元数之间的夹角（以弧度表示）
double computeAngleBetweenQuaternions(const Quaternion& q1, const Quaternion& q2)
{
    double dot_product = quaternionDotProduct(q1, q2);
    // 确保点积在 [-1, 1] 范围内，避免精度问题
    dot_product = std::fmax(-1.0, std::fmin(1.0, dot_product));
    double angle = 2 * std::acos(dot_product);
    return angle; // 以弧度返回
}

// 计算两个四元数之间的夹角（以度数表示）
double computeAngleBetweenQuaternionsInDegrees(const Quaternion& q1, const Quaternion& q2)
{
    double dot_product = quaternionDotProduct(q1, q2);
    // 确保点积在 [-1, 1] 范围内，避免精度问题
    dot_product = std::fmax(-1.0, std::fmin(1.0, dot_product));
    double angle_in_radians = 2 * std::acos(dot_product); // 以弧度表示
    double angle_in_degrees = angle_in_radians * 180.0 / M_PI; // 转换为度数
    return angle_in_degrees;
}

void Global::monitor_position()
{
    while (!target_reached_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 定期检查位置
        // 获取当前时间
        rclcpp::Time target_time = rclcpp::Time();
        // 设置一个时间容忍度，允许较小的时间误差
        rclcpp::Duration tolerance(0, 200000000); // 0.2秒的时间容忍度
        // 检查是否存在从base_link到map的变换
        try {
            // 如果存在，获取这个变换
            auto transform = tf_buffer_->lookupTransform("map", "base_link", target_time, tolerance);
            // 机器人的当前位置
            double robot_x = transform.transform.translation.x;
            double robot_y = transform.transform.translation.y;
            double distance = sqrt(pow(elevator_robox_x - robot_x, 2) + pow(elevator_robox_y - robot_y, 2));
            std::cout << " monitor_position " << distance << std::endl;
            if (distance < 0.22) { // 目标位置容差
                sleep(2);
                // RCLCPP_INFO(this->get_logger(), "Target position reached!");
                target_reached_ = true;
                // elevator_first_step_ = true;
                monitoring_thread_reached_ = false;
                geometry_msgs::msg::PoseStamped poseStamp;
                poseStamp.header.stamp = rclcpp::Time();
                poseStamp.header.frame_id = "map";
                poseStamp.pose.position.x = elevator_robox_x;
                poseStamp.pose.position.y = elevator_robox_y;
                //  计算机器人需要朝向的角度
                double delta_x = directStan_.point.x - robot_x;
                double delta_y = directStan_.point.y - robot_y;
                double angle_to_goal = atan2(delta_y, delta_x) /*+ (20 * π / 180)*/; // 目标点相对于机器人的角度

                tf2::Quaternion goal_orientation;
                goal_orientation.setRPY(0, 0, angle_to_goal); // 设置目标朝向的四元数
                poseStamp.pose.orientation.x = goal_orientation.x();
                poseStamp.pose.orientation.y = goal_orientation.y();
                poseStamp.pose.orientation.z = goal_orientation.z();
                poseStamp.pose.orientation.w = goal_orientation.w();
                pose_publisher_->publish(poseStamp);
                std::cout << " 发送角度 " << std::endl;
                sleep(4);
                auto transform = tf_buffer_->lookupTransform("map", "base_link", target_time, tolerance);
                auto rotation = transform.transform.rotation;

                // 提取当前的yaw角
                tf2::Quaternion quat(rotation.x, rotation.y, rotation.z, rotation.w);
                tf2::Matrix3x3 m(quat);
                double roll, pitch, current_yaw;
                m.getRPY(roll, pitch, current_yaw);
                // 使用IMU进行计算朝向* 180.0 / M_PI
                std::cout << " imu_z_ " << imu_z_ * 180.0 / M_PI << " 弧度 " << imu_z_
                          << " heading_1_ 弧度 " << heading_1_ * M_PI / 180.0 << " heading_1_ " << heading_1_
                          << " 车身弧度 " << current_yaw << std::endl;
                elevator_first_step_ = 5;
            }
        } catch (tf2::TransformException& ex) {
            // 如果没有找到变换，输出错误信息
            RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
        }
    }
}

void Global::send_elecator_second_goal_pose(const geometry_msgs::msg::PointStamped::SharedPtr& point,
    const geometry_msgs::msg::PointStamped::SharedPtr& pointdire)
{
    // 获取当前时间
    rclcpp::Time target_time = point->header.stamp;
    // 设置一个时间容忍度，允许较小的时间误差
    rclcpp::Duration tolerance(0, 200000000); // 0.2秒的时间容忍度
    // 检查是否存在从base_link到map的变换
    try {
        // 如果存在，获取这个变换
        auto transform = tf_buffer_->lookupTransform("map", "base_link", target_time, tolerance);
        // 机器人的当前位置
        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;
        // 创建一个新的PointStamped，用于存储转换后的点
        geometry_msgs::msg::PointStamped transformed_point;
        transformed_point.header = point->header;
        transformed_point.header.frame_id = "map";
        transformed_point.point = point->point;

        // 应用变换
        tf2::doTransform(*point, transformed_point, transform);
        geometry_msgs::msg::PoseStamped poseStamp;
        poseStamp.header = transformed_point.header;
        poseStamp.pose.position.x = transformed_point.point.x;
        poseStamp.pose.position.y = transformed_point.point.y;
        // 方向应用变换
        directSecondtan_.header = pointdire->header;
        directSecondtan_.header.frame_id = "map";
        directSecondtan_.point = pointdire->point;

        tf2::doTransform(*pointdire, directSecondtan_, transform);
        // 记录发送位置
        elevator_robox_x = transformed_point.point.x;
        elevator_robox_y = transformed_point.point.y;
        // 启动线程监控是否已到达指定位置，然后进行方向纠正
        if (!monitoring_thread_secode_reached_) {
            // 创建监控线程
            std::cout << " 监控线程启动 " << std::endl;
            target_secode_reached_ = false;
            monitoring_secode_thread_ = std::thread(&Global::monitor_second_position, this);
            monitoring_secode_thread_.detach();
            monitoring_thread_secode_reached_ = true;
        }

        //  计算机器人需要朝向的角度
        double delta_x = poseStamp.pose.position.x - robot_x;
        double delta_y = poseStamp.pose.position.y - robot_y;
        double angle_to_goal = atan2(delta_y, delta_x); // 目标点相对于机器人的角度

        tf2::Quaternion goal_orientation;
        goal_orientation.setRPY(0, 0, angle_to_goal); // 设置目标朝向的四元数
        poseStamp.pose.orientation.x = goal_orientation.x();
        poseStamp.pose.orientation.y = goal_orientation.y();
        poseStamp.pose.orientation.z = goal_orientation.z();
        poseStamp.pose.orientation.w = goal_orientation.w();

        // 发布转换后的点
        pose_publisher_->publish(poseStamp);
        std::cout << " ------------send_elevator_goal_pose----------------- x "
                  << poseStamp.pose.position.x
                  << " y " << poseStamp.pose.position.y << std::endl;
    } catch (tf2::TransformException& ex) {
        // 如果没有找到变换，输出错误信息
        RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    }
}

void Global::monitor_second_position()
{
    while (!target_secode_reached_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 定期检查位置
        // 获取当前时间
        rclcpp::Time target_time = rclcpp::Time();
        // 设置一个时间容忍度，允许较小的时间误差
        rclcpp::Duration tolerance(0, 200000000); // 0.2秒的时间容忍度
        // 检查是否存在从base_link到map的变换
        try {
            // 如果存在，获取这个变换
            auto transform = tf_buffer_->lookupTransform("map", "base_link", target_time, tolerance);
            // 机器人的当前位置
            double robot_x = transform.transform.translation.x;
            double robot_y = transform.transform.translation.y;
            double distance = sqrt(pow(elevator_robox_x - robot_x, 2) + pow(elevator_robox_y - robot_y, 2));
            std::cout << " monitor_second_position " << distance << std::endl;
            if (distance < 0.22) { // 目标位置容差
                sleep(2);
                // RCLCPP_INFO(this->get_logger(), "Target position reached!");
                target_secode_reached_ = true;
                // elevator_first_step_ = true;
                monitoring_thread_secode_reached_ = false;
                geometry_msgs::msg::PoseStamped poseStamp;
                poseStamp.header.stamp = rclcpp::Time();
                poseStamp.header.frame_id = "map";
                poseStamp.pose.position.x = elevator_robox_x;
                poseStamp.pose.position.y = elevator_robox_y;
                //  计算机器人需要朝向的角度
                double delta_x = directSecondtan_.point.x - robot_x;
                double delta_y = directSecondtan_.point.y - robot_y;
                double angle_to_goal = atan2(delta_y, delta_x) /*+ (10 * π / 180)*/; // 目标点相对于机器人的角度
                tf2::Quaternion goal_orientation;
                goal_orientation.setRPY(0, 0, angle_to_goal); // 设置目标朝向的四元数
                poseStamp.pose.orientation.x = goal_orientation.x();
                poseStamp.pose.orientation.y = goal_orientation.y();
                poseStamp.pose.orientation.z = goal_orientation.z();
                poseStamp.pose.orientation.w = goal_orientation.w();
                pose_publisher_->publish(poseStamp);
                std::cout << " 发送角度 " << std::endl;
                sleep(4);
                elevator_first_step_ = 3;
            }
        } catch (tf2::TransformException& ex) {
            // 如果没有找到变换，输出错误信息
            RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
        }
    }
}

void Global::send_elecator_third_goal_pose(const geometry_msgs::msg::PointStamped::SharedPtr& point,
    const geometry_msgs::msg::PointStamped::SharedPtr& pointdire)
{
    // 获取当前时间
    rclcpp::Time target_time = point->header.stamp;
    // 设置一个时间容忍度，允许较小的时间误差
    rclcpp::Duration tolerance(0, 200000000); // 0.2秒的时间容忍度
    // 检查是否存在从base_link到map的变换
    try {
        // 如果存在，获取这个变换
        auto transform = tf_buffer_->lookupTransform("map", "base_link", target_time, tolerance);
        // 机器人的当前位置
        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;
        // 创建一个新的PointStamped，用于存储转换后的点
        geometry_msgs::msg::PointStamped transformed_point;
        transformed_point.header = point->header;
        transformed_point.header.frame_id = "map";
        transformed_point.point = point->point;

        // 应用变换
        tf2::doTransform(*point, transformed_point, transform);
        geometry_msgs::msg::PoseStamped poseStamp;
        poseStamp.header = transformed_point.header;
        poseStamp.pose.position.x = transformed_point.point.x;
        poseStamp.pose.position.y = transformed_point.point.y;
        // 方向应用变换
        directSecondtan_.header = pointdire->header;
        directSecondtan_.header.frame_id = "map";
        directSecondtan_.point = pointdire->point;

        tf2::doTransform(*pointdire, directSecondtan_, transform);
        // 记录发送位置
        elevator_robox_x = transformed_point.point.x;
        elevator_robox_y = transformed_point.point.y;
        // 启动线程监控是否已到达指定位置，然后进行方向纠正
        if (!monitoring_thread_third_reached_) {
            // 创建监控线程
            std::cout << " 监控线程启动 " << std::endl;
            target_third_reached_ = false;
            monitoring_third_thread_ = std::thread(&Global::monitor_third_position, this);
            monitoring_third_thread_.detach();
            monitoring_thread_third_reached_ = true;
        }

        //  计算机器人需要朝向的角度
        double delta_x = poseStamp.pose.position.x - robot_x;
        double delta_y = poseStamp.pose.position.y - robot_y;
        double angle_to_goal = atan2(delta_y, delta_x); // 目标点相对于机器人的角度

        tf2::Quaternion goal_orientation;
        goal_orientation.setRPY(0, 0, angle_to_goal); // 设置目标朝向的四元数
        poseStamp.pose.orientation.x = goal_orientation.x();
        poseStamp.pose.orientation.y = goal_orientation.y();
        poseStamp.pose.orientation.z = goal_orientation.z();
        poseStamp.pose.orientation.w = goal_orientation.w();

        // 发布转换后的点
        pose_publisher_->publish(poseStamp);
        std::cout << " ------------send_elevator_goal_pose----------------- x "
                  << poseStamp.pose.position.x
                  << " y " << poseStamp.pose.position.y << std::endl;
    } catch (tf2::TransformException& ex) {
        // 如果没有找到变换，输出错误信息
        RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    }
}

void Global::monitor_third_position()
{
    while (!target_third_reached_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 定期检查位置
        // 获取当前时间
        rclcpp::Time target_time = rclcpp::Time();
        // 设置一个时间容忍度，允许较小的时间误差
        rclcpp::Duration tolerance(0, 200000000); // 0.2秒的时间容忍度
        // 检查是否存在从base_link到map的变换
        try {
            // 如果存在，获取这个变换
            auto transform = tf_buffer_->lookupTransform("map", "base_link", target_time, tolerance);
            // 机器人的当前位置
            double robot_x = transform.transform.translation.x;
            double robot_y = transform.transform.translation.y;
            double distance = sqrt(pow(elevator_robox_x - robot_x, 2) + pow(elevator_robox_y - robot_y, 2));
            std::cout << " monitor_second_position " << distance << std::endl;
            if (distance < 0.2) { // 目标位置容差
                // sleep(2);
                //  RCLCPP_INFO(this->get_logger(), "Target position reached!");
                target_third_reached_ = true;
                // elevator_first_step_ = true;
                monitoring_thread_third_reached_ = false;
                geometry_msgs::msg::PoseStamped poseStamp;
                poseStamp.header.stamp = rclcpp::Time();
                poseStamp.header.frame_id = "map";
                poseStamp.pose.position.x = elevator_robox_x;
                poseStamp.pose.position.y = elevator_robox_y;
                //  计算机器人需要朝向的角度
                double delta_x = directSecondtan_.point.x - robot_x;
                double delta_y = directSecondtan_.point.y - robot_y;
                double angle_to_goal = atan2(delta_y, delta_x) /*+ (10 * π / 180)*/; // 目标点相对于机器人的角度
                tf2::Quaternion goal_orientation;
                goal_orientation.setRPY(0, 0, angle_to_goal); // 设置目标朝向的四元数
                poseStamp.pose.orientation.x = goal_orientation.x();
                poseStamp.pose.orientation.y = goal_orientation.y();
                poseStamp.pose.orientation.z = goal_orientation.z();
                poseStamp.pose.orientation.w = goal_orientation.w();
                pose_publisher_->publish(poseStamp);
                std::cout << " 发送角度 " << std::endl;
                sleep(1);
                // 前进2米

                // auto client = std::make_shared<rclcpp::Node>("parameter_client");
                // auto result = client->set_parameter(rclcpp::Parameter("max_speed", 0.3));

                // advance(2.0);
                elevator_first_step_ = 4;
            }
        } catch (tf2::TransformException& ex) {
            // 如果没有找到变换，输出错误信息
            RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
        }
    }
}
void Global::send_playback_pose(const geometry_msgs::msg::PointStamped::SharedPtr& point)
{
    // 获取当前时间
    rclcpp::Time target_time
        = point->header.stamp;
    // 设置一个时间容忍度，允许较小的时间误差
    rclcpp::Duration tolerance(0, 200000000); // 0.2秒的时间容忍度
    // 检查是否存在从base_link到map的变换
    try {
        // 如果存在，获取这个变换
        auto transform = tf_buffer_->lookupTransform("map", "base_link", target_time, tolerance);
        // 机器人的当前位置
        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;
        // 创建一个新的PointStamped，用于存储转换后的点
        geometry_msgs::msg::PointStamped transformed_point;
        transformed_point.header = point->header;
        transformed_point.header.frame_id = "map";
        transformed_point.point = point->point;

        // 应用变换
        tf2::doTransform(*point, transformed_point, transform);
        geometry_msgs::msg::PoseStamped poseStamp;
        poseStamp.header = transformed_point.header;
        poseStamp.pose.position.x = transformed_point.point.x;
        poseStamp.pose.position.y = transformed_point.point.y;
        // 停止到前1.5米位置
        {
            Point A = { robot_x, robot_y };
            Point B = { poseStamp.pose.position.x, poseStamp.pose.position.y };
            double distanceBC = 0.5f;
            Point c = calculateCPoint(A, B, distanceBC);
            poseStamp.pose.position.x = c.x;
            poseStamp.pose.position.y = c.y;
        }

        // 计算机器人需要朝向的角度
        double delta_x = poseStamp.pose.position.x - robot_x;
        double delta_y = poseStamp.pose.position.y - robot_y;
        double angle_to_goal = atan2(delta_y, delta_x); // 目标点相对于机器人的角度

        tf2::Quaternion goal_orientation;
        goal_orientation.setRPY(0, 0, angle_to_goal); // 设置目标朝向的四元数
        poseStamp.pose.orientation.x = goal_orientation.x();
        poseStamp.pose.orientation.y = goal_orientation.y();
        poseStamp.pose.orientation.z = goal_orientation.z();
        poseStamp.pose.orientation.w = goal_orientation.w();

        // 发布转换后的点
        pose_publisher_->publish(poseStamp);
        std::cout << " ------------send_goal_pose----------------- x " << poseStamp.pose.position.x
                  << " y " << poseStamp.pose.position.y << std::endl;
    } catch (tf2::TransformException& ex) {
        // 如果没有找到变换，输出错误信息
        RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    }
}
void Global::send_elevator_direction(const geometry_msgs::msg::PointStamped::SharedPtr& point)
{
    // 获取当前时间
    rclcpp::Time target_time
        = point->header.stamp;
    // 设置一个时间容忍度，允许较小的时间误差
    rclcpp::Duration tolerance(0, 200000000); // 0.2秒的时间容忍度
    // 检查是否存在从base_link到map的变换
    try {
        // 如果存在，获取这个变换
        auto transform = tf_buffer_->lookupTransform("map", "base_link", target_time, tolerance);
        // 机器人的当前位置
        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;
        // 创建一个新的PointStamped，用于存储转换后的点
        geometry_msgs::msg::PointStamped transformed_point;
        transformed_point.header = point->header;
        transformed_point.header.frame_id = "map";
        transformed_point.point = point->point;

        // 应用变换
        tf2::doTransform(*point, transformed_point, transform);
        geometry_msgs::msg::PoseStamped poseStamp;
        poseStamp.header = transformed_point.header;
        poseStamp.pose.position.x = transformed_point.point.x;
        poseStamp.pose.position.y = transformed_point.point.y;
        // 停止到前1.5米位置
        {
            Point A = { robot_x, robot_y };
            Point B = { poseStamp.pose.position.x, poseStamp.pose.position.y };
            double distanceBC = 2.3f;
            Point c = calculateCPoint(A, B, distanceBC);
            poseStamp.pose.position.x = c.x;
            poseStamp.pose.position.y = c.y;
        }

        // 计算机器人需要朝向的角度
        double delta_x = poseStamp.pose.position.x - robot_x;
        double delta_y = poseStamp.pose.position.y - robot_y;
        double angle_to_goal = atan2(delta_y, delta_x); // 目标点相对于机器人的角度

        tf2::Quaternion goal_orientation;
        goal_orientation.setRPY(0, 0, angle_to_goal); // 设置目标朝向的四元数
        poseStamp.pose.orientation.x = goal_orientation.x();
        poseStamp.pose.orientation.y = goal_orientation.y();
        poseStamp.pose.orientation.z = goal_orientation.z();
        poseStamp.pose.orientation.w = goal_orientation.w();

        // 发布转换后的点
        pose_publisher_->publish(poseStamp);
        std::cout << " ------------send_elevator_direction----------------- x " << poseStamp.pose.position.x
                  << " y " << poseStamp.pose.position.y << std::endl;
    } catch (tf2::TransformException& ex) {
        // 如果没有找到变换，输出错误信息
        RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    }
}
}