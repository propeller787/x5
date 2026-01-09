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
    std::vector<std::pair<double, double>> angle_ranges; // Multiple angle ranges
    double range_min;
    double range_max;
    bool clockwise;
    int motor_speed;
    std::shared_ptr<ordlidar::OrdlidarDriver> driver;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
};

static const int FILTER_WINDOW_SIZE = 7; // 滤波窗口大小,必须为奇数

// 点云滤波函数
// FilterLevel: 滤波等级(1-5),数值越大滤波强度越大
// MotorSpeed: 电机转速(10/15/20/25/30)
// distances: 距离数据数组,单位mm
// distances参数说明:
// 1. 类型为std::vector<uint16_t>&,以引用方式传入,可以直接修改原始数据
// 2. 存储一圈激光雷达扫描得到的距离数据,每个元素表示一个采样点的距离值(单位:mm)
// 3. 函数会对distances中的异常点进行滤波,将噪点的距离值设为0
// 4. 滤波处理会保持distances数组大小不变,仅修改部分元素的值
/**
 * @brief 对激光雷达点云数据进行滤波处理
 *
 * @param FilterLevel 滤波等级(1-5),等级越高滤波强度越大
 * @param MotorSpeed 电机转速(10/15/20/25/30 Hz)
 * @param distances 点云距离数据,单位为毫米
 *
 * @details 该函数通过分析点云数据中的异常点进行滤波:
 *          1. 基于滑动窗口计算相邻点的夹角
 *          2. 根据滤波等级设置不同的夹角阈值和判定条件
 *          3. 结合电机转速调整滤波参数
 *          4. 对判定为噪点的数据进行剔除处理
 *
 * @note 滤波条件:
 *       - 点云数量必须大于滤波窗口大小
 *       - 滤波等级必须在1-5之间
 *       - 电机转速必须为指定值(10/15/20/25/30 Hz)
 */
void point_cloud_filter(uint32_t FilterLevel, uint32_t MotorSpeed, std::vector<uint16_t> &distances) {
    uint16_t SumLeftAbnormal = 0;      // 滑窗左侧点云夹角异常数量
    uint16_t SumRightAbnormal = 0;     // 滑窗右侧点云夹角异常数量
    int32_t Neighbors_ = 0;            // 相邻滤除处理点个数
    int32_t PointNumThreshold = 0;     // 临近点云超过阈值的数量门限
    float TanAngleThreshold = 0;       // 点云滤波角度判决阈值，依据临近点云夹角
    float AngleResSin = 0;             // 角度间隔对应的正弦值
    float AngleResCos = 0;             // 角度间隔对应的余弦值
    double TanAngle = 0;               // 夹角的正切值
    uint16_t abnormal_point_diff = 25; // 与左右两边的点进行预判断的差值阈值
                                       // 10转速:15mm(3cm), 15转速:30mm(4cm)
                                       // 20转速:25mm(5cm), 25转速:35mm(7cm)
                                       // 30转速:40mm(8cm)
    uint16_t PointDistanceCmp = 0;     // 点云距离比较量暂存值
    uint16_t cur_distance = 0;         // 当前处理点的距离值
    uint16_t diff_1, diff_2;           // 与相邻点的距离差值
    uint16_t left_aver = 0;            // 左侧点云平均值
    uint16_t right_aver = 0;           // 右侧点云平均值
    uint16_t aver_thr = 30;            // 平均值差异阈值
    int count = distances.size();      // 一圈的点数

    std::vector<uint16_t> temp_distance_data; // 临时存储距离数据

    // 数据点数小于滤波窗口大小,直接返回
    if (count < FILTER_WINDOW_SIZE) {
        return;
    }

    // 滤波等级为0,不进行滤波处理
    if (FilterLevel == 0) {
        return;
    }

    // 电机转速不在允许范围内,直接返回
    if ((MotorSpeed != 10) && (MotorSpeed != 15) && (MotorSpeed != 20) && (MotorSpeed != 25) && (MotorSpeed != 30)) {
        return;
    }

    // 复制距离数据到临时数组
    temp_distance_data.resize(count);
    std::copy(distances.begin(), distances.end(), temp_distance_data.begin());

    // 根据滤波等级设置相关参数
    switch (FilterLevel) {
    case 1: {
        // 拖尾、噪点角度范围在±4°以内,剔除相邻点个数为1,判断一侧至少3点符合
        TanAngleThreshold = 0.03492076949175; // tan(3°)
        Neighbors_ = 0;
        PointNumThreshold = 3;
        break;
    }
    case 2: {
        // 拖尾、噪点角度范围在±4°以内,剔除相邻点个数为1，判断一侧至少2点符合
        TanAngleThreshold = 0.052407779283041; // tan(3°)
        Neighbors_ = 0;
        PointNumThreshold = 3;
        break;
    }
    case 3: {
        // 拖尾、噪点角度范围在±7°以内,剔除相邻点个数为1,判断一侧至少2点符合
        TanAngleThreshold = 0.052407779283041; // tan(5.5°)
        Neighbors_ = 1;
        PointNumThreshold = 3;
        break;
    }
    case 4: {
        // 拖尾、噪点角度范围在±9°以内,剔除相邻点个数为1，判断一侧至少2点符合
        TanAngleThreshold = 0.096289048197539; // tan(9°)
        Neighbors_ = 0;
        PointNumThreshold = 3;
        break;
    }
    case 5: {
        // 拖尾、噪点角度范围在±9°以内,剔除相邻点个数为1,判断一侧至少1点符合
        TanAngleThreshold = 0.096289048197539; // tan(9°)
        Neighbors_ = 1;
        PointNumThreshold = 3;
        break;
    }
    default: {
        TanAngleThreshold = 0;
        break;
    }
    }

    // 根据电机转速设置角度参数
    switch (MotorSpeed) {
    case 10: {
        AngleResSin = -0.002073449665673; // sin(角分辨率)
        AngleResCos = 0.999997850400932;  // cos(角分辨率)
        abnormal_point_diff = 15;         // 异常点判断阈值15mm
        break;
    }
    case 15: {
        AngleResSin = -0.003110171712830;
        AngleResCos = 0.999995163404262;
        abnormal_point_diff = 30;
        break;
    }
    case 20: {
        AngleResSin = -0.004146890417175;
        AngleResCos = 0.999991401612968;
        abnormal_point_diff = 25;
        break;
    }
    case 25: {
        AngleResSin = -0.005183604664443;
        AngleResCos = 0.999986565031092;
        abnormal_point_diff = 35;
        break;
    }
    case 30: {
        AngleResSin = -0.006220313340373;
        AngleResCos = 0.999980653663833;
        abnormal_point_diff = 40;
        break;
    }
    default: {
        break;
    }
    }

    // 遍历所有点进行滤波处理
    for (int i = (FILTER_WINDOW_SIZE / 2); i < (count - (FILTER_WINDOW_SIZE / 2)); i++) {
        cur_distance = temp_distance_data[i];

        // 计算与相邻点的距离差值
        if (cur_distance > temp_distance_data[i + 1]) {
            diff_1 = cur_distance - temp_distance_data[i + 1];
        } else {
            diff_1 = temp_distance_data[i + 1] - cur_distance;
        }

        if (cur_distance > temp_distance_data[i - 1]) {
            diff_2 = cur_distance - temp_distance_data[i - 1];
        } else {
            diff_2 = temp_distance_data[i - 1] - cur_distance;
        }

        // 如果与相邻点距离差值都小于阈值,且当前点距离大于500mm,则跳过该点
        if ((diff_1 < abnormal_point_diff) && (diff_2 < abnormal_point_diff)) {
            if (cur_distance > 500) {
                continue;
            }
        }

        int half_window = FILTER_WINDOW_SIZE / 2;

        // 计算窗口内左右两侧点的平均值
        int l_temp = 0, r_temp = 0;
        for (int n = (i - half_window); n < i + half_window + 1; n++) {
            if (n < (i + half_window))
                l_temp = distances[n];
            else if (n > i + half_window)
                r_temp = distances[n];
        }

        left_aver = l_temp / half_window;
        right_aver = r_temp / half_window;

        // 遍历窗口内的点,计算夹角
        for (int pos = (i - half_window); pos < i + half_window + 1; pos++) {
            // 跳过当前点和距离为0的点
            if (pos == i || cur_distance == 0) {
                continue;
            }

            PointDistanceCmp = temp_distance_data[pos];
            // 计算当前点与窗口内其他点的夹角正切值
            TanAngle = (cur_distance * AngleResSin) / (PointDistanceCmp - cur_distance * AngleResCos);

            // 如果夹角在阈值范围内,累计异常点数
            if ((TanAngle < TanAngleThreshold) && (TanAngle > ((-1) * TanAngleThreshold))) {
                if (pos < i)
                    SumLeftAbnormal++; // 左侧异常点数+1
                else
                    SumRightAbnormal++; // 右侧异常点数+1
            }

            // 如果任一侧异常点数超过阈值,且与平均值差异大于阈值,则认为是噪点
            if ((SumLeftAbnormal >= PointNumThreshold) || (SumRightAbnormal >= PointNumThreshold)) {
                if ((abs(distances[i] - left_aver) > aver_thr) || (abs(distances[i] - right_aver) > aver_thr)) {
                    // 将当前点及邻近点的距离值设为0
                    for (int num = ((-1) * Neighbors_); num <= Neighbors_; num++) {
                        distances[i + num] = 0;
                    }
                }

                break;
            }
        }

        // 重置异常点计数
        SumLeftAbnormal = 0;
        SumRightAbnormal = 0;
    }
}

/**
 * @brief Check if an angle is within any of the specified angle ranges
 * @param angle The angle to check (in degrees)
 * @param ranges Vector of angle range pairs (in degrees)
 * @return True if the angle is outside all ranges, False if inside any range
 */
bool is_angle_outside_ranges(float angle, const std::vector<std::pair<double, double>> &ranges) {
    for (const auto &range : ranges) {
        if (angle >= range.first && angle <= range.second) {
            return false; // Inside a valid range
        }
    }
    return true; // Outside all ranges
}

/**
 * @brief 发布激光雷达扫描数据为ROS消息
 */
void publish_msg(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr &pub, full_scan_data_st *scan_frame, rclcpp::Time start, double scan_time,
                 std::string frame_id, bool clockwise, const std::vector<std::pair<double, double>> &angle_ranges, double min_range, double max_range,
                 std::string scan_topic) {
    sensor_msgs::msg::LaserScan scanMsg;
    int point_nums = scan_frame->vailtidy_point_num;

    // 将距离数据转换为毫米单位的vector用于滤波
    std::vector<uint16_t> distances;
    distances.reserve(point_nums);
    for (int i = 0; i < point_nums; i++) {
        distances.push_back(static_cast<uint16_t>(scan_frame->data[i].distance));
    }

    // 对数据进行滤波处理  该处理会影响狭窄空间的点
    // point_cloud_filter(3, 10, distances);
    // if (scan_topic.find("tilt") == std::string::npos) {
    //     point_cloud_filter(3, 10, distances);
    // }

    scanMsg.header.stamp = start;
    scanMsg.header.frame_id = frame_id;
    scanMsg.angle_min = Degree2Rad(scan_frame->data[0].angle);              // Full range of data
    scanMsg.angle_max = Degree2Rad(scan_frame->data[point_nums - 1].angle); // Full range of data
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
    bool filter_near_origin = (scan_topic.find("tilt") == std::string::npos);
    bool back_origin = (scan_topic.find("back") != std::string::npos);

    for (int i = 0; i < point_nums; i++) {
        // 使用滤波后的距离数据
        range = distances[i] * 0.001;
        intensity = scan_frame->data[i].intensity;

        // 当话题名不包含"tilt"时，过滤距离原点0.05米以内的数据
        if (!filter_near_origin && range < 0.1f) {
            range = 0.0f;
            intensity = 0.0f;
        }
        if (back_origin && range < 0.1f) {
            range = 0.0f;
            intensity = 0.0f;
        }
        if ((range > max_range) || (range < min_range)) {
            range = 0.0;
            intensity = 0.0;
        }

        if (!clockwise) {
            dir_angle = static_cast<float>(360.f - scan_frame->data[i].angle);
        } else {
            dir_angle = scan_frame->data[i].angle;
        }

        // Filter based on multiple angle ranges
        if (is_angle_outside_ranges(dir_angle, angle_ranges)) {
            range = 0;
            intensity = 0;
        }

        float angle = Degree2Rad(dir_angle);
        // 依据极坐标过滤：去除 0<x<0.6 且 -0.02<y<0.02 的点
        if (range > 0.0f) {
            float x = range * std::cos(angle);
            float y = range * std::sin(angle);
            if (x > 0.0f && x < 0.06f && y < 0.006f && y > -0.006f) {
                range = 0.0f;
                intensity = 0.0f;
            }
        }
        unsigned int index = (unsigned int)((angle - scanMsg.angle_min) / scanMsg.angle_increment);
        if (index < point_nums) {
            if (std::isnan(scanMsg.ranges[index])) {
                scanMsg.ranges[index] = range;
                unsigned int err = index - last_index;
                if (err == 2) {
                    scanMsg.ranges[index - 1] = range;
                    scanMsg.intensities[index - 1] = intensity;
                }
            } else {
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
void lidar_thread_function(std::shared_ptr<rclcpp::Node> node, LidarDevice device) {
    auto driver = std::make_shared<ordlidar::OrdlidarDriver>(ORADAR_TYPE_SERIAL, ORADAR_MS200);
    device.driver = driver;

    device.driver->SetSerialPort(device.port, device.baudrate);
    double min_thr = 0.0, max_thr = 0.0, cur_speed = 0.0;
    int motor_speed = 10;
    bool ret = false;

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

    min_thr = (double)motor_speed - ((double)motor_speed * 0.1);
    max_thr = (double)motor_speed + ((double)motor_speed * 0.1);
    cur_speed = device.driver->GetRotationSpeed();
    if (cur_speed < min_thr || cur_speed > max_thr) {
        device.driver->SetRotationSpeed(motor_speed);
    }

    full_scan_data_st scan_data;
    rclcpp::Time start_scan_time;
    rclcpp::Time end_scan_time;
    double scan_duration;

    while (rclcpp::ok()) {
        start_scan_time = node->now();
        ret = device.driver->GrabFullScanBlocking(scan_data, 1000);
        end_scan_time = node->now();
        scan_duration = (end_scan_time.seconds() - start_scan_time.seconds());
        if (ret) {
            publish_msg(device.publisher, &scan_data, start_scan_time, scan_duration, device.frame_id, device.clockwise, device.angle_ranges, device.range_min,
                        device.range_max, device.scan_topic);
        }
    }

    device.driver->Disconnect();
    RCLCPP_INFO(node->get_logger(), "Lidar device on port %s disconnected.", device.port.c_str());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("multi_threaded_oradar_ros");

    int device_count = 1; // 假设一个设备
    std::vector<LidarDevice> devices(device_count);
    std::vector<std::thread> lidar_threads;

    for (int i = 0; i < device_count; ++i) {
        std::string prefix = "lidar" + std::to_string(i + 1) + "_";
        node->declare_parameter<std::string>(prefix + "port_name", devices[i].port);
        node->declare_parameter<int>(prefix + "baudrate", devices[i].baudrate);
        node->declare_parameter<std::string>(prefix + "frame_id", devices[i].frame_id);
        node->declare_parameter<std::string>(prefix + "scan_topic", devices[i].scan_topic);
        node->declare_parameter<std::vector<double>>(prefix + "angle_ranges", std::vector<double>{}); // List of ranges
        node->declare_parameter<double>(prefix + "range_min", devices[i].range_min);
        node->declare_parameter<double>(prefix + "range_max", devices[i].range_max);
        node->declare_parameter<bool>(prefix + "clockwise", devices[i].clockwise);
        node->declare_parameter<int>(prefix + "motor_speed", devices[i].motor_speed);

        node->get_parameter(prefix + "port_name", devices[i].port);
        node->get_parameter(prefix + "baudrate", devices[i].baudrate);
        node->get_parameter(prefix + "frame_id", devices[i].frame_id);
        node->get_parameter(prefix + "scan_topic", devices[i].scan_topic);
        std::vector<double> angle_ranges_flat;
        node->get_parameter(prefix + "angle_ranges", angle_ranges_flat);
        if (angle_ranges_flat.size() % 2 != 0) {
            RCLCPP_ERROR(node->get_logger(), "Angle ranges for %s must be provided as pairs (min, max).", prefix.c_str());
            return -1;
        }
        for (size_t j = 0; j < angle_ranges_flat.size(); j += 2) {
            devices[i].angle_ranges.emplace_back(angle_ranges_flat[j], angle_ranges_flat[j + 1]);
        }
        node->get_parameter(prefix + "range_min", devices[i].range_min);
        node->get_parameter(prefix + "range_max", devices[i].range_max);
        node->get_parameter(prefix + "clockwise", devices[i].clockwise);
        node->get_parameter(prefix + "motor_speed", devices[i].motor_speed);

        devices[i].publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(devices[i].scan_topic, 10);
    }

    for (auto &device : devices) {
        lidar_threads.emplace_back(std::thread(lidar_thread_function, node, device));
    }

    for (auto &t : lidar_threads) {
        if (t.joinable()) {
            t.join();
        }
    }

    rclcpp::shutdown();
    return 0;
}