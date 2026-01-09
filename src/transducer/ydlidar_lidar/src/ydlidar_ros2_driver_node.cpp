/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "src/CYdLidar.h"
#include "std_srvs/srv/empty.hpp"
#include "common/utils/error_event.h"
#include "common/utils/topic.h"
#include "interface/msg/hex_data.hpp"
#include <chrono>
#include <iostream>
#include <math.h>
#include <memory>
#include <signal.h>
#include <string>
#include <vector>

#define ROS2Verision "1.0.1"

// 将错误级别编码到错误码中（错误级别编码在最低字节）
uint32_t getErrorCodeNotFound(std::string scan_topic) {
    if (scan_topic == "scan/right") {
        return LIDAR2_NOT_FOUND;
    } else if (scan_topic == "scan/left") {
        return LIDAR1_NOT_FOUND;
    } else if (scan_topic == "scan/back") {
        return LIDAR5_NOT_FOUND;
    } else if (scan_topic == "scan/left_tilt") {
        return LIDAR3_NOT_FOUND;
    } else if (scan_topic == "scan/right_tilt") {
        return LIDAR4_NOT_FOUND;
    } else {
        return 0;
    }
}

uint32_t getErrorCodeDisconnected(std::string scan_topic) {
    if (scan_topic == "scan/right") {
        return LIDAR2_DISCONNECTED;
    } else if (scan_topic == "scan/left") {
        return LIDAR1_DISCONNECTED;
    } else if (scan_topic == "scan/back") {
        return LIDAR5_DISCONNECTED;
    } else if (scan_topic == "scan/left_tilt") {
        return LIDAR3_DISCONNECTED;
    } else if (scan_topic == "scan/right_tilt") {
        return LIDAR4_DISCONNECTED;
    } else {
        return 0;
    }
}

uint32_t getErrorCodeTimeout(std::string scan_topic) {
    if (scan_topic == "scan/right") {
        return LIDAR2_TIMEOUT;
    } else if (scan_topic == "scan/left") {
        return LIDAR1_TIMEOUT;
    } else if (scan_topic == "scan/back") {
        return LIDAR5_TIMEOUT;
    } else if (scan_topic == "scan/left_tilt") {
        return LIDAR3_TIMEOUT;
    } else if (scan_topic == "scan/right_tilt") {
        return LIDAR4_TIMEOUT;
    } else {
        return 0;
    }
}

uint32_t getErrorCodeDataInvalid(std::string scan_topic) {
    if (scan_topic == "scan/right") {
        return LIDAR2_DATA_INVALID;
    } else if (scan_topic == "scan/left") {
        return LIDAR1_DATA_INVALID;
    } else if (scan_topic == "scan/back") {
        return LIDAR5_DATA_INVALID;
    } else if (scan_topic == "scan/left_tilt") {
        return LIDAR3_DATA_INVALID;
    } else if (scan_topic == "scan/right_tilt") {
        return LIDAR4_DATA_INVALID;
    } else {
        return 0;
    }
}

uint32_t getErrorCodeTimeSyncError(std::string scan_topic) {
    if (scan_topic == "scan/right") {
        return LIDAR2_TIME_SYNC_ERROR;
    } else if (scan_topic == "scan/left") {
        return LIDAR1_TIME_SYNC_ERROR;
    } else if (scan_topic == "scan/back") {
        return LIDAR5_TIME_SYNC_ERROR;
    } else if (scan_topic == "scan/left_tilt") {
        return LIDAR3_TIME_SYNC_ERROR;
    } else if (scan_topic == "scan/right_tilt") {
        return LIDAR4_TIME_SYNC_ERROR;
    } else {
        return 0;
    }
}

uint32_t getErrorCodeDriverError(std::string scan_topic) {
    if (scan_topic == "scan/right") {
        return LIDAR2_DRIVER_ERROR;
    } else if (scan_topic == "scan/left") {
        return LIDAR1_DRIVER_ERROR;
    } else if (scan_topic == "scan/back") {
        return LIDAR5_DRIVER_ERROR;
    } else if (scan_topic == "scan/left_tilt") {
        return LIDAR3_DRIVER_ERROR;
    } else if (scan_topic == "scan/right_tilt") {
        return LIDAR4_DRIVER_ERROR;
    } else {
        return 0;
    }
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

    RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());

    CYdLidar laser;
    std::string str_optvalue = "/dev/ttyUSB1";
    node->declare_parameter("port", str_optvalue);
    node->get_parameter("port", str_optvalue);
    /// lidar port
    laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

    /// ignore array
    str_optvalue = "";
    node->declare_parameter("ignore_array", str_optvalue);
    node->get_parameter("ignore_array", str_optvalue);
    laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

    std::string frame_id = "laser_frame";
    node->declare_parameter("frame_id", frame_id);
    node->get_parameter("frame_id", frame_id);

    //////////////////////int property/////////////////
    /// lidar baudrate
    int optval = 230400;
    node->declare_parameter("baudrate", optval);
    node->get_parameter("baudrate", optval);
    laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
    /// tof lidar
    optval = TYPE_TRIANGLE;
    node->declare_parameter("lidar_type", optval);
    node->get_parameter("lidar_type", optval);
    laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
    /// device type
    optval = YDLIDAR_TYPE_SERIAL;
    node->declare_parameter("device_type", optval);
    node->get_parameter("device_type", optval);
    laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
    /// sample rate
    optval = 9;
    node->declare_parameter("sample_rate", optval);
    node->get_parameter("sample_rate", optval);
    laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
    /// abnormal count
    optval = 4;
    node->declare_parameter("abnormal_check_count", optval);
    node->get_parameter("abnormal_check_count", optval);
    laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

    /// Intenstiy bit count
    optval = 0;
    node->declare_parameter("intensity_bit", optval);
    node->get_parameter("intensity_bit", optval);
    laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

    //////////////////////bool property/////////////////
    /// fixed angle resolution
    bool b_optvalue = false;
    node->declare_parameter("fixed_resolution", b_optvalue);
    node->get_parameter("fixed_resolution", b_optvalue);
    laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
    /// rotate 180
    b_optvalue = false;
    node->declare_parameter("reversion", b_optvalue);
    node->get_parameter("reversion", b_optvalue);
    laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
    /// Counterclockwise
    b_optvalue = true;
    node->declare_parameter("inverted", b_optvalue);
    node->get_parameter("inverted", b_optvalue);
    laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
    b_optvalue = true;
    node->declare_parameter("auto_reconnect", b_optvalue);
    node->get_parameter("auto_reconnect", b_optvalue);
    laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
    /// one-way communication
    b_optvalue = false;
    node->declare_parameter("isSingleChannel", b_optvalue);
    node->get_parameter("isSingleChannel", b_optvalue);
    laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
    /// intensity
    b_optvalue = false;
    node->declare_parameter("intensity", b_optvalue);
    node->get_parameter("intensity", b_optvalue);
    laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
    /// Motor DTR
    b_optvalue = false;
    node->declare_parameter("support_motor_dtr", b_optvalue);
    node->get_parameter("support_motor_dtr", b_optvalue);
    laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
    // 是否启用调试
    b_optvalue = false;
    node->declare_parameter("debug", b_optvalue);
    node->get_parameter("debug", b_optvalue);
    laser.setEnableDebug(b_optvalue);

    //////////////////////float property/////////////////
    /// unit: °
    float f_optvalue = 180.0f;
    node->declare_parameter("angle_max", f_optvalue);
    node->get_parameter("angle_max", f_optvalue);
    laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
    f_optvalue = -180.0f;
    node->declare_parameter("angle_min", f_optvalue);
    node->get_parameter("angle_min", f_optvalue);
    laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
    /// unit: m
    f_optvalue = 12.f;
    node->declare_parameter("range_max", f_optvalue);
    node->get_parameter("range_max", f_optvalue);
    laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
    f_optvalue = 0.03f;
    node->declare_parameter("range_min", f_optvalue);
    node->get_parameter("range_min", f_optvalue);
    laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
    /// unit: Hz
    f_optvalue = 10.f;
    node->declare_parameter("frequency", f_optvalue);
    node->get_parameter("frequency", f_optvalue);
    laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

    bool invalid_range_is_inf = false;
    node->declare_parameter("invalid_range_is_inf", invalid_range_is_inf);
    node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);

    std::string scan_topic = "scan";
    node->declare_parameter("scan_topic", scan_topic);
    node->get_parameter("scan_topic", scan_topic);
    auto error_event_pub = node->create_publisher<interface::msg::HexData>(errorEventTopic, 1);
    
    // 错误状态跟踪：记录当前处于错误状态的错误码（0表示无错误）
    // 需要在服务回调中访问，所以定义在这里
    uint32_t current_error_code = 0;
    
    bool ret = laser.initialize();
    if (ret) {
        // 设置GS工作模式（非GS雷达请无视该代码）
        int i_v = 0;
        node->declare_parameter("m1_mode", i_v);
        node->get_parameter("m1_mode", i_v);
        laser.setWorkMode(i_v, 0x01);
        i_v = 0;
        node->declare_parameter("m2_mode", i_v);
        node->get_parameter("m2_mode", i_v);
        laser.setWorkMode(i_v, 0x02);
        i_v = 1;
        node->declare_parameter("m3_mode", i_v);
        node->get_parameter("m3_mode", i_v);
        laser.setWorkMode(i_v, 0x04);
        // 启动扫描
        ret = laser.turnOn();
        if (!ret) {
            // 启动扫描失败，播报断连异常
            RCLCPP_ERROR(node->get_logger(), "Failed to start scan: %s\n", laser.DescribeError());
            uint32_t disconnected_error_code = getErrorCodeDisconnected(scan_topic);
            interface::msg::HexData error_event_msg;
            error_event_msg.hex_value = encodeErrorLevel(disconnected_error_code, ErrorEventLevel::OCCUR);
            error_event_pub->publish(error_event_msg);
            current_error_code = disconnected_error_code; // 记录错误状态
        }
    } else {
        // 初始化失败，播报设备未找到异常
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize lidar: %s\n", laser.DescribeError());
        uint32_t not_found_error_code = getErrorCodeNotFound(scan_topic);
        interface::msg::HexData error_event_msg;
        error_event_msg.hex_value = encodeErrorLevel(not_found_error_code, ErrorEventLevel::OCCUR);
        error_event_pub->publish(error_event_msg);
        current_error_code = not_found_error_code; // 记录错误状态
    }

    auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::SensorDataQoS());
    //auto pc_pub = node->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud", rclcpp::SensorDataQoS());
    auto stop_scan_service =
        [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::Empty::Request> req,
            std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool {
        return laser.turnOff();
    };

    auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan", stop_scan_service);

    auto start_scan_service =
        [&laser, node, scan_topic, error_event_pub, &current_error_code](const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::Empty::Request> req,
            std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool {
        bool result = laser.turnOn();
        if (!result) {
            RCLCPP_ERROR(node->get_logger(), "Failed to start scan via service: %s\n", laser.DescribeError());
            uint32_t disconnected_error_code = getErrorCodeDisconnected(scan_topic);
            interface::msg::HexData error_event_msg;
            error_event_msg.hex_value = encodeErrorLevel(disconnected_error_code, ErrorEventLevel::OCCUR);
            error_event_pub->publish(error_event_msg);
            current_error_code = disconnected_error_code; // 记录错误状态
        } else {
            // 启动成功，如果之前处于错误状态，发布恢复消息
            if (current_error_code != 0) {
                RCLCPP_INFO(node->get_logger(), "Lidar recovered from error via service (0x%08X)", current_error_code);
                interface::msg::HexData recover_event_msg;
                recover_event_msg.hex_value = encodeErrorLevel(current_error_code, ErrorEventLevel::RECOVER);
                error_event_pub->publish(recover_event_msg);
                current_error_code = 0; // 清除错误状态
            }
        }
        return result;
    };

    auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan", start_scan_service);

    rclcpp::WallRate loop_rate(20);
    
    // 用于跟踪连续失败次数，避免频繁播报
    int consecutive_fail_count = 0;
    const int FAIL_THRESHOLD = 10; // 连续失败10次后播报异常
    std::chrono::steady_clock::time_point last_error_report_time = std::chrono::steady_clock::now() - std::chrono::seconds(10); // 初始化为10秒前，确保第一次可以播报
    const std::chrono::seconds ERROR_REPORT_INTERVAL(5); // 错误播报最小间隔5秒

    while (ret && rclcpp::ok()) {

        LaserScan scan; //

        if (laser.doProcessSimple(scan)) {
            // 成功获取数据，重置失败计数
            consecutive_fail_count = 0;
            
            // 如果之前处于错误状态，现在恢复，发布恢复消息
            if (current_error_code != 0) {
                RCLCPP_INFO(node->get_logger(), "Lidar recovered from error (0x%08X)", current_error_code);
                interface::msg::HexData recover_event_msg;
                recover_event_msg.hex_value = encodeErrorLevel(current_error_code, ErrorEventLevel::RECOVER);
                error_event_pub->publish(recover_event_msg);
                current_error_code = 0; // 清除错误状态
            }

            auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
            auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud>();

            scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
            scan_msg->header.stamp.nanosec = scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
            scan_msg->header.frame_id = frame_id;
            pc_msg->header = scan_msg->header;
            scan_msg->angle_min = scan.config.min_angle;
            scan_msg->angle_max = scan.config.max_angle;
            scan_msg->angle_increment = scan.config.angle_increment;
            scan_msg->scan_time = scan.config.scan_time;
            scan_msg->time_increment = scan.config.time_increment;
            scan_msg->range_min = scan.config.min_range;
            scan_msg->range_max = scan.config.max_range;

            int size = (scan.config.max_angle - scan.config.min_angle) / scan.config.angle_increment + 1;
            scan_msg->ranges.resize(size);
            scan_msg->intensities.resize(size);

            pc_msg->channels.resize(2);
            int idx_intensity = 0;
            pc_msg->channels[idx_intensity].name = "intensities";
            int idx_timestamp = 1;
            pc_msg->channels[idx_timestamp].name = "stamps";

            for (size_t i = 0; i < scan.points.size(); i++) {
                int index = std::ceil((scan.points[i].angle - scan.config.min_angle) / scan.config.angle_increment);
                if (index >= 0 && index < size) {
                    if (scan.points[i].range >= scan.config.min_range) {
                        scan_msg->ranges[index] = scan.points[i].range;
                        scan_msg->intensities[index] = scan.points[i].intensity;
                    }
                }

                if (scan.points[i].range >= scan.config.min_range && scan.points[i].range <= scan.config.max_range) {
                    geometry_msgs::msg::Point32 point;
                    point.x = scan.points[i].range * cos(scan.points[i].angle);
                    point.y = scan.points[i].range * sin(scan.points[i].angle);
                    point.z = 0.0;
                    pc_msg->points.push_back(point);
                    pc_msg->channels[idx_intensity].values.push_back(scan.points[i].intensity);
                    pc_msg->channels[idx_timestamp].values.push_back(i * scan.config.time_increment);
                }
            }

            laser_pub->publish(*scan_msg);
           // pc_pub->publish(*pc_msg);

        } else {
            consecutive_fail_count++;
            RCLCPP_WARN(node->get_logger(), "Failed to get scan (consecutive failures: %d)", consecutive_fail_count);
            
            // 连续失败达到阈值且距离上次播报超过最小间隔时，播报异常
            auto now = std::chrono::steady_clock::now();
            bool should_report = (consecutive_fail_count >= FAIL_THRESHOLD) && 
                                (consecutive_fail_count == FAIL_THRESHOLD || 
                                 (now - last_error_report_time) >= ERROR_REPORT_INTERVAL);
            
            if (should_report) {
                RCLCPP_ERROR(node->get_logger(), "Lidar timeout: failed to get scan data for %d consecutive times", consecutive_fail_count);
                uint32_t timeout_error_code = getErrorCodeTimeout(scan_topic);
                interface::msg::HexData error_event_msg;
                error_event_msg.hex_value = encodeErrorLevel(timeout_error_code, ErrorEventLevel::OCCUR);
                error_event_pub->publish(error_event_msg);
                current_error_code = timeout_error_code; // 记录当前错误状态
                last_error_report_time = now;
            }
        }
        if (!rclcpp::ok()) {
            break;
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
    laser.turnOff();
    laser.disconnecting();
    rclcpp::shutdown();

    return 0;
}
