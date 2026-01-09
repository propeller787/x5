#include "src/ord_lidar_driver.h"
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <thread>
#include <vector>
#define Degree2Rad(X) ((X) * M_PI / 180.)
// 数据结构：每个设备的信息
struct LidarDevice {
    std::string port;
    int baudrate;
    std::string frame_id;
    std::string scan_topic;
    double angle_min;
    double angle_max;
    double range_min;
    double range_max;
    bool clockwise;
    int motor_speed;
    std::shared_ptr<ordlidar::OrdlidarDriver> driver;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
};

/**
 * @brief 发布激光雷达扫描数据为ROS消息
 *
 * 此函数将full_scan_data_st结构中的扫描数据转换为sensor_msgs::msg::LaserScan消息格式，
 * 并通过给定的ROS发布者发布该消息。
 *
 * @param pub ROS发布者，用于发布LaserScan消息
 * @param scan_frame 包含扫描数据的full_scan_data_st结构指针
 * @param start 扫描开始的时间戳
 * @param scan_time 扫描时间间隔
 * @param frame_id 扫描数据的坐标系ID
 * @param clockwise 扫描方向标志，true表示顺时针，false表示逆时针
 * @param angle_min 扫描的最小角度（弧度）
 * @param angle_max 扫描的最大角度（弧度）
 * @param min_range 扫描的最小距离
 * @param max_range 扫描的最大距离
 */
void publish_msg(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub, full_scan_data_st* scan_frame, rclcpp::Time start,
    double scan_time, std::string frame_id, bool clockwise,
    double angle_min, double angle_max, double min_range, double max_range)
{
    sensor_msgs::msg::LaserScan scanMsg;
    int point_nums = scan_frame->vailtidy_point_num;

    scanMsg.header.stamp = start;
    scanMsg.header.frame_id = frame_id;
    scanMsg.angle_min = Degree2Rad(scan_frame->data[0].angle);
    scanMsg.angle_max = Degree2Rad(scan_frame->data[point_nums - 1].angle);
    double diff = scan_frame->data[point_nums - 1].angle - scan_frame->data[0].angle;
    scanMsg.angle_increment = Degree2Rad(diff / point_nums);
    scanMsg.scan_time = scan_time;
    scanMsg.time_increment = scan_time / point_nums;
    scanMsg.range_min = min_range;
    scanMsg.range_max = max_range;

    scanMsg.ranges.assign(point_nums, std::numeric_limits<float>::quiet_NaN());
    scanMsg.intensities.assign(point_nums, std::numeric_limits<float>::quiet_NaN());

    float range = 0.0;
    float intensity = 0.0;
    float dir_angle;
    unsigned int last_index = 0;

    for (int i = 0; i < point_nums; i++) {
        range = scan_frame->data[i].distance * 0.001;
        intensity = scan_frame->data[i].intensity;

        if ((range > max_range) || (range < min_range)) {
            range = 0.0;
            intensity = 0.0;
        }

        if (!clockwise) {
            dir_angle = static_cast<float>(360.f - scan_frame->data[i].angle);
        } else {
            dir_angle = scan_frame->data[i].angle;
        }

        if ((dir_angle < angle_min) || (dir_angle > angle_max)) {
            range = 0;
            intensity = 0;
        }

        float angle = Degree2Rad(dir_angle);
        unsigned int index = (unsigned int)((angle - scanMsg.angle_min) / scanMsg.angle_increment);
        if (index < point_nums) {
            // If the current content is Nan, it is assigned directly
            if (std::isnan(scanMsg.ranges[index])) {
                scanMsg.ranges[index] = range;
                unsigned int err = index - last_index;
                if (err == 2) {
                    scanMsg.ranges[index - 1] = range;
                    scanMsg.intensities[index - 1] = intensity;
                }
            } else { // Otherwise, only when the distance is less than the current
                //   value, it can be re assigned
                if (range < scanMsg.ranges[index]) {
                    scanMsg.ranges[index] = range;
                }
            }
            scanMsg.intensities[index] = intensity;
            last_index = index;
        }
    }

    pub->publish(scanMsg);
}
// 单设备线程函数
void lidar_thread_function(std::shared_ptr<rclcpp::Node> node, LidarDevice device)
{
    // 为每个设备创建独立的驱动实例
    auto driver = std::make_shared<ordlidar::OrdlidarDriver>(ORADAR_TYPE_SERIAL, ORADAR_MS200);
    device.driver = driver;
    
    device.driver->SetSerialPort(device.port, device.baudrate);
    double min_thr = 0.0, max_thr = 0.0, cur_speed = 0.0;
    int motor_speed = 10;
    bool ret = false;
    // 尝试连接设备
    while (rclcpp::ok()) {
        if (device.driver->isConnected() == true) {
            device.driver->Disconnect();
            std::cout << "Disconnect lidar device." << std::endl;
        }
        if (device.driver->Connect()) {
            RCLCPP_INFO(node->get_logger(), "Connected to lidar device on port %s", device.port.c_str());
            break;
        } else {
            RCLCPP_WARN(node->get_logger(), "Failed to connect to lidar device on port %s, retrying...", device.port.c_str());
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }

    // 设置电机转速
    full_scan_data_st scan_data;
    rclcpp::Time start_scan_time;
    rclcpp::Time end_scan_time;
    double scan_duration;

    min_thr = (double)motor_speed - ((double)motor_speed * 0.1);
    max_thr = (double)motor_speed + ((double)motor_speed * 0.1);
    cur_speed = device.driver->GetRotationSpeed();
    if (cur_speed < min_thr || cur_speed > max_thr) {
        device.driver->SetRotationSpeed(motor_speed);
    }

    // 主循环：采集数据并发布消息
    while (rclcpp::ok()) {
        start_scan_time = node->now();
        ret = device.driver->GrabFullScanBlocking(scan_data, 1000);
        end_scan_time = node->now();
        scan_duration = (end_scan_time.seconds() - start_scan_time.seconds());
        // 阻塞模式获取扫描数据
        if(ret) {
            publish_msg(device.publisher, &scan_data, start_scan_time, scan_duration, device.frame_id,
                device.clockwise, device.angle_min, device.angle_max, device.range_min, device.range_max);
        }
    }

    // 断开设备连接
    device.driver->Disconnect();
    RCLCPP_INFO(node->get_logger(), "Lidar device on port %s disconnected.", device.port.c_str());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("multi_threaded_oradar_ros");

    // 配置设备
    int device_count = 1; // 假设两个设备
    std::vector<LidarDevice> devices(device_count);
    std::vector<std::thread> lidar_threads; // 存储线程对象

    for (int i = 0; i < device_count; ++i) {
        std::string prefix = "lidar" + std::to_string(i + 1) + "_";
        node->declare_parameter<std::string>(prefix + "port_name", devices[i].port);
        node->declare_parameter<int>(prefix + "baudrate", devices[i].baudrate);
        node->declare_parameter<std::string>(prefix + "frame_id", devices[i].frame_id);
        node->declare_parameter<std::string>(prefix + "scan_topic", devices[i].scan_topic);
        node->declare_parameter<double>(prefix + "angle_min", devices[i].angle_min);
        node->declare_parameter<double>(prefix + "angle_max", devices[i].angle_max);
        node->declare_parameter<double>(prefix + "range_min", devices[i].range_min);
        node->declare_parameter<double>(prefix + "range_max", devices[i].range_max);
        node->declare_parameter<bool>(prefix + "clockwise", devices[i].clockwise);
        node->declare_parameter<int>(prefix + "motor_speed", devices[i].motor_speed);

        node->get_parameter(prefix + "port_name", devices[i].port);
        node->get_parameter(prefix + "baudrate", devices[i].baudrate);
        node->get_parameter(prefix + "frame_id", devices[i].frame_id);
        node->get_parameter(prefix + "scan_topic", devices[i].scan_topic);
        node->get_parameter(prefix + "angle_min", devices[i].angle_min);
        node->get_parameter(prefix + "angle_max", devices[i].angle_max);
        node->get_parameter(prefix + "range_min", devices[i].range_min);
        node->get_parameter(prefix + "range_max", devices[i].range_max);
        node->get_parameter(prefix + "clockwise", devices[i].clockwise);
        node->get_parameter(prefix + "motor_speed", devices[i].motor_speed);

        devices[i].publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(devices[i].scan_topic, 10);
    }

    // 为每个设备启动独立线程
    for (auto& device : devices) {
        lidar_threads.emplace_back(std::thread(lidar_thread_function, node, device));
    }

    // 等待所有线程完成 
    for (auto& t : lidar_threads) {
        if (t.joinable()) {
            t.join();
        }
    }

    rclcpp::shutdown();
    return 0;
}
