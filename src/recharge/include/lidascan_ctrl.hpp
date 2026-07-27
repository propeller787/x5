// lidascan_ctrl.hpp
#ifndef LIDAR_SCAN_CONTROLLER_HPP
#define LIDAR_SCAN_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "wheelchair_types.hpp"
#include "recharge_tool.hpp"
#include "recharge_ctrl.hpp"
#include <map>
#include <memory>
#include <tuple>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "recharge_status.hpp"
#include <sstream>


class LidarScanController
{
public:
    using LaserScan = sensor_msgs::msg::LaserScan;
    using LaserScanSharedPtr = sensor_msgs::msg::LaserScan::SharedPtr;

    LidarScanController(rclcpp::Node *node);
    ~LidarScanController();

    // 初始化
    void initialize();

    // 导航控制接口（外部调用）
    void startNavigation();
    void stopNavigation();
    void stopMotion();

    // 充电电压检测接口
    void setChargingVoltage(bool value);
    bool getChargingVoltage() const;

    // 状态查询
    bool isActive() const { return active_; }
    ChargingState getChargingState() const { return charging_state_; };

    void publishStationDetected(bool detected);

    void resetStationDetection();

private:
    rclcpp::Node *node_;
    std::mutex data_mutex_;

    // ROS2 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_ctrl_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_rviz_;
    rclcpp::TimerBase::SharedPtr navigation_timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_filtered_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_station_detected_; 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_charging_voltage_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub1_;//对外
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub2_;//对外

    // 状态变量
    bool active_;
    ChargingState charging_state_;
    geometry_msgs::msg::Twist ctrl_twist_;
    rclcpp::Time ctrl_twist_time_;
    rclcpp::Time detect_station_time_;

    bool station_detected_;
    rclcpp::Time last_station_publish_time_;

    // 参数
    ScanFilterParam param_;
    double distance_threshold_;
    int fit_window_size_;
    double fit_residual_threshold_;
    int frame_index_;

    // 充电检测
    bool detect_charging_voltage_;
    rclcpp::Time detect_charging_voltage_time_;

    struct RechargeResult last_result_;

    // 异步控制（RechargeCtrl 实例）
    std::unique_ptr<RechargeCtrl> recharge_ctrl_;

    // ========== 核心算法函数 ==========

    // 激光数据处理
    void processLaserScan(const LaserScanSharedPtr &msg);
    bool processSegmentData(LaserScanSharedPtr msg, int begin_index, int end_index);
    void procData(RechargeResult &result);

    // 过滤算法
    void correctIndexOffset(LaserScanSharedPtr msg);  // 修正索引偏移（ranges/intensities 错位重排）
    void filterByAngleRange(LaserScanSharedPtr msg);  // 按角度范围过滤，保留后向 90°
    void filterByDistance(LaserScanSharedPtr msg);    // 按距离阈值过滤（min_distance ~ max_distance）
    void filterByIntensity(LaserScanSharedPtr msg);   // 按强度阈值过滤 + 距离二次检查

    // 坐标转换
    std::map<int, std::pair<double, double>> laserScanToXY(const LaserScanSharedPtr &scan_msg);
    std::map<int, std::pair<double, double>> laserScanToXY1(int begin_index, int end_index,
                                                            const LaserScanSharedPtr &scan_msg);

    // 分段处理算法
    std::vector<std::pair<int, int>> getSections(
        int begin_index, int end_index,
        const LaserScanSharedPtr &msg,
        const std::map<int, std::pair<double, double>> &xy_list);

    std::map<int, std::tuple<int, int, double, double, double, double, double, int>>
    getSegments(const std::map<int, std::pair<double, double>> &xy_points_map);

    std::map<int, std::tuple<int, int, double, double, double, double, double, int>>
    filterSegments(const std::map<int, std::tuple<int, int, double, double, double, double, double, int>> &turn_segments);

    // 可视化
    void publishRechargeVisualization(const RechargeResult &result);

    // ========== 辅助函数 ==========
    void publishFilteredScan(const LaserScanSharedPtr &original_msg,
                             const std::vector<float> &filtered_ranges);
    LaserScanSharedPtr resetLaserData(LaserScanSharedPtr msg, int b_index, int e_index);
    int getFixedIndex(const LaserScanSharedPtr &msg, int i) const;
    int getFixed() const { return param_.fixed_index; }
    // 定时器回调
    void navigationTimerCallback();

    // 控制发布
    void publishControl();
};

#endif // LIDAR_SCAN_CONTROLLER_HPP
