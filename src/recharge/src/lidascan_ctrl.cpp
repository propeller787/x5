// lidascan_ctrl.cpp
#include "lidascan_ctrl.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <limits>
#include <deque>
#include <numeric>
#include <thread>
#include <Eigen/Dense>
#include "../../common/utils/topic.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

bool g_debug_last_scan = false;

bool isValid(float value)
{
    if (fabs(value) < 0.001)
        return false;
    return !std::isinf(value) && !std::isnan(value);
}


// 片段结构
struct Segment {
    size_t start_idx;   // 片段起始索引（包含）
    size_t end_idx;     // 片段结束索引（包含）
    size_t size() const { return end_idx - start_idx + 1; }
    
    // 判断是否为有效片段（大小>=3）
    bool isValid() const { return size() >= 3; }
};

// 主处理函数
std::pair<int, int>  filterScanModify4(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!msg || msg->ranges.empty()) {
        // std::cout << "Empty scan data!" << std::endl;
        return {0, 0};
    }
    
    std::vector<Segment> all_segments;  // 存储所有片段（包括无效的）
    size_t n = msg->ranges.size();
    
    // 第一步：遍历数据，根据连续丢失点数进行分片
    size_t i = 0;
    while (i < n) {
        // 跳过开头的无效点
        while (i < n && !isValid(msg->ranges[i])) {
            i++;
        }
        if (i >= n) break;
        
        // 开始一个片段
        size_t seg_start = i;
        int consecutive_invalid = 0;
        
        // 扩展当前片段，直到遇到连续3个无效点
        while (i < n) {
            if (isValid(msg->ranges[i])) {
                consecutive_invalid = 0;
                i++;
            } else {
                consecutive_invalid++;
                if (consecutive_invalid >= 3) {
                    // 遇到中断，结束当前片段
                    break;
                }
                i++;
            }
        }
        
        size_t seg_end = i - consecutive_invalid;  // 片段结束索引
        all_segments.push_back({seg_start, seg_end});
    }
    
    // 第二步：将无效片段（大小<3）中的所有数据置零
    std::vector<Segment> valid_segments;
    for (const auto& seg : all_segments) {
        if (seg.isValid()) {
            valid_segments.push_back(seg);
        } else {
            // 将无效片段中的所有点置零
            for (size_t idx = seg.start_idx; idx <= seg.end_idx; ++idx) {
                msg->ranges[idx] = 0.0f;
            }
        }
    }
    
    // 第三步：提取整体范围
    if (valid_segments.empty()) {
        return {0, 0};
    }
    
    int global_begin = valid_segments.front().start_idx;
    int global_end = valid_segments.back().end_idx;
    
    return {global_begin, global_end};
}


LidarScanController::LidarScanController(rclcpp::Node *node)
    : node_(node), active_(false), charging_state_(ChargingState::IDLE), detect_charging_voltage_(false), frame_index_(0), station_detected_(false)
{
    last_result_.middle_point.x = 0.0;
    last_result_.middle_point.y = 0.0;
    last_result_.middle_point.index = 0;
}

LidarScanController::~LidarScanController()
{
    recharge_ctrl_.reset();
}

void LidarScanController::initialize()
{
    // ==================== 声明 ScanFilterParam 参数 ====================
    node_->declare_parameter<int>("recharge_filter.min_index", 80);
    node_->declare_parameter<int>("recharge_filter.max_index", 170);
    node_->declare_parameter<double>("recharge_filter.min_intensities", 100.0);
    node_->declare_parameter<double>("recharge_filter.max_intensities", 220.0);
    node_->declare_parameter<double>("recharge_filter.min_distance", 0.20);
    node_->declare_parameter<double>("recharge_filter.max_distance", 3.5);
    node_->declare_parameter<int>("recharge_filter.min_count", 4);
    node_->declare_parameter<double>("recharge_filter.dock_length", 0.42);
    node_->declare_parameter<int>("recharge_filter.baselink_back_index", 225);
    node_->declare_parameter<double>("recharge_filter.dy_diff", 0.09);  // 默认 0.1 - 0.01
    node_->declare_parameter<int>("recharge_filter.fixed_index", 212);  // 默认 214 - 2
    node_->declare_parameter<double>("recharge_filter.scan_angle_diff", 0.0);

    // 获取参数
    param_.min_index = node_->get_parameter("recharge_filter.min_index").as_int();
    param_.max_index = node_->get_parameter("recharge_filter.max_index").as_int();
    param_.min_intensities = node_->get_parameter("recharge_filter.min_intensities").as_double();
    param_.max_intensities = node_->get_parameter("recharge_filter.max_intensities").as_double();
    param_.min_distance = node_->get_parameter("recharge_filter.min_distance").as_double();
    param_.max_distance = node_->get_parameter("recharge_filter.max_distance").as_double();
    param_.min_count = node_->get_parameter("recharge_filter.min_count").as_int();
    param_.dock_length = node_->get_parameter("recharge_filter.dock_length").as_double();
    param_.baselink_back_index = node_->get_parameter("recharge_filter.baselink_back_index").as_int();
    param_.dy_diff = node_->get_parameter("recharge_filter.dy_diff").as_double();
    param_.fixed_index = node_->get_parameter("recharge_filter.fixed_index").as_int();
    param_.scan_angle_diff = node_->get_parameter("recharge_filter.scan_angle_diff").as_double();

    RCLCPP_INFO(node_->get_logger(), "ScanFilterParam - min_index:%d max_index:%d min_intensities:%.1f max_intensities:%.1f",
                param_.min_index, param_.max_index, param_.min_intensities, param_.max_intensities);
    RCLCPP_INFO(node_->get_logger(), "ScanFilterParam - min_dist:%.2f max_dist:%.2f min_count:%d dock_length:%.2f dy_diff:%.3f fixed_index:%d",
                param_.min_distance, param_.max_distance, param_.min_count, param_.dock_length, param_.dy_diff, param_.fixed_index, param_.scan_angle_diff);

    // 声明原有参数
    node_->declare_parameter<double>("distance_threshold", 0.5);
    node_->declare_parameter<int>("fit_window_size", 10);
    node_->declare_parameter<double>("fit_residual_threshold", 0.09);

    // 获取原有参数
    distance_threshold_ = node_->get_parameter("distance_threshold").as_double();
    fit_window_size_ = node_->get_parameter("fit_window_size").as_int();
    fit_residual_threshold_ = node_->get_parameter("fit_residual_threshold").as_double();

    // QoS配置
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
                           .best_effort()
                           .durability_volatile();

    // 创建激光雷达订阅者
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan/back", qos_profile,
        [this](const LaserScanSharedPtr msg)
        {
            rclcpp::Time current_time = node_->now();
            rclcpp::Time msg_time = msg->header.stamp;
            double time_diff = (current_time - msg_time).seconds();
            static LaserScanSharedPtr s_last_scan;
            if (g_debug_last_scan) {
                processLaserScan(s_last_scan);
                return;
            }
            s_last_scan = msg;
            if (time_diff < 0.2) {
                processLaserScan(msg);
            }
        });

    // 创建控制发布者
    pub_ctrl_ = node_->create_publisher<geometry_msgs::msg::Twist>(nav2ControlTopic, 10);

    // 创建可视化发布者
    pub_rviz_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/line_marker_array", 10);

    // 创建过滤后的激光数据发布者
    pub_filtered_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("/scan/out", 10);
    // 创建基站检测状态发布者（使用 transient_local 确保新订阅者能收到最后一条消息）
    pub_station_detected_ = node_->create_publisher<std_msgs::msg::Bool>(
        "/station/detected",
        rclcpp::QoS(10).transient_local()); // 修改为 transient_local
    pub_charging_voltage_ = node_->create_publisher<std_msgs::msg::String>(
        "/charging/voltage_detected", 10);
    pub1_=node_->create_publisher<std_msgs::msg::Int32>(rechargeStateTopic,10);
    pub2_=node_->create_publisher<std_msgs::msg::Int32>("/vlago/recharge/feedback",10);

    if (0)
    {
        auto msg =std_msgs::msg::Int32();
        // msg.data = RECHARGE_FEEDBACK_SUCCESS_DOCKED;
        msg.data = RECHARGE_EVENT_STATION_NOT_FOUND;
        pub1_->publish(msg);
        RCLCPP_WARN(node_->get_logger(), "开始上报状态 2");
    }
    // 初始化控制消息
    ctrl_twist_.linear.x = 0.0;
    ctrl_twist_.angular.z = 0.0;
    ctrl_twist_time_ = node_->now();

    // 初始化 RechargeCtrl（依赖控制所需的 publisher 和状态指针）
    {
        RechargeCtrlDeps deps;
        deps.node = node_;
        deps.pub_ctrl = pub_ctrl_;
        deps.pub1 = pub1_;
        deps.pub2 = pub2_;
        deps.pub_station_detected = pub_station_detected_;
        deps.pub_charging_voltage = pub_charging_voltage_;
        deps.ctrl_twist = &ctrl_twist_;
        deps.ctrl_twist_time = &ctrl_twist_time_;
        deps.active = &active_;
        deps.last_result = &last_result_;
        deps.stop_navigation = [this]() { stopNavigation(); };
        deps.start_navigation = [this]() { startNavigation(); };
        deps.set_charging_voltage = [this](bool v) { setChargingVoltage(v); };
        deps.get_charging_voltage = [this]() { return getChargingVoltage(); };
        recharge_ctrl_ = std::make_unique<RechargeCtrl>(deps);
    }

    // 创建导航定时器
    navigation_timer_ = node_->create_wall_timer(
        100ms, std::bind(&LidarScanController::navigationTimerCallback, this));

    detect_station_time_ = node_->now() - rclcpp::Duration(100.0, 0);
    detect_charging_voltage_time_ = node_->now();

    // 初始化基站检测状态
    station_detected_ = false;
    last_station_publish_time_ = node_->now();

    RCLCPP_INFO(node_->get_logger(), "激光扫描控制器已初始化");
}

void LidarScanController::resetStationDetection()
{
    // 重置内部状态
    station_detected_ = false;
    detect_charging_voltage_ = false;

    // 发布false信号
    auto false_msg = std_msgs::msg::Bool();
    false_msg.data = false;
    pub_station_detected_->publish(false_msg);

    // 重置时间戳
    last_station_publish_time_ = node_->now();

    RCLCPP_INFO(node_->get_logger(), "基站检测状态已重置: false");
}

void LidarScanController::startNavigation()
{
    active_ = true;
    charging_state_ = ChargingState::NAVIGATING;
    station_detected_ = false;
    RCLCPP_INFO(node_->get_logger(), "导航功能已启动");
}

void LidarScanController::stopNavigation()
{
    active_ = false;
    charging_state_ = ChargingState::IDLE;
    stopMotion();
    RCLCPP_INFO(node_->get_logger(), "导航功能已停止");
}

void LidarScanController::publishStationDetected(bool detected)
{
    auto now = node_->now();
    auto time_diff = (now - last_station_publish_time_).seconds();
    // if (station_detected_ == false && detected == false)
    // {
    //     auto msg = std_msgs::msg::Bool();
    //     msg.data = detected;
    //     pub_station_detected_->publish(msg);
    // }
    // 防止频繁发布，至少间隔0.5秒
    if (station_detected_ != detected)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = detected;
        pub_station_detected_->publish(msg);
        station_detected_ = detected;
    }
    last_station_publish_time_ = now;

    // if (detected)
    // {
    //     RCLCPP_INFO(node_->get_logger(), "发布基站检测成功信号: true");
    // }
    // else
    // {
    //     RCLCPP_DEBUG(node_->get_logger(), "发布基站检测信号: false");
    // }
}

void LidarScanController::stopMotion()
{
    ctrl_twist_.linear.x = 0.0;
    ctrl_twist_.angular.z = 0.0;
    ctrl_twist_time_ = node_->now();
    pub_ctrl_->publish(ctrl_twist_);
    RCLCPP_INFO(node_->get_logger(), "停止运动");
}

void LidarScanController::navigationTimerCallback()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!active_)
        return;

    auto now = node_->now();
    auto time_diff = (now - ctrl_twist_time_).seconds();

    if (time_diff < 2.0)
    {
        pub_ctrl_->publish(ctrl_twist_);
    }
}

void LidarScanController::processLaserScan(const LaserScanSharedPtr &input_msg)
{
    LaserScanSharedPtr output_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    //拷貝input_msg 到 output_msg;
    output_msg->header = input_msg->header;
    output_msg->angle_min = input_msg->angle_min;
    output_msg->angle_max = input_msg->angle_max;
    output_msg->angle_increment = input_msg->angle_increment;
    output_msg->time_increment = input_msg->time_increment;
    output_msg->scan_time = input_msg->scan_time;
    output_msg->range_min = input_msg->range_min;
    output_msg->range_max = input_msg->range_max;
    output_msg->ranges = input_msg->ranges;
    output_msg->intensities = input_msg->intensities;

    if (frame_index_ == 0) {
        printf("LaserScan size:%ld\n", input_msg->ranges.size());
    }
    frame_index_++;
    auto& msg = output_msg;
    // 应用过滤算法
    correctIndexOffset(msg);
    filterByAngleRange(msg);
    filterByDistance(msg);
    filterByIntensity(msg);
    // publishFilteredScan(msg,msg->ranges);
    auto [begin_index, end_index] = filterScanModify4(msg);

    if (end_index - begin_index < 2)
    {
        return;
    }
    // 处理分段数据
    if (processSegmentData(msg, begin_index, end_index))
    {
        detect_station_time_ = node_->now();
    }
}

bool LidarScanController::processSegmentData(LaserScanSharedPtr msg,
                                             int begin_index, int end_index)
{
    // 转换到XY坐标系
    auto xy_list = laserScanToXY1(begin_index, end_index, msg);
    if (xy_list.size() < 2)
    {
        RCLCPP_WARN(node_->get_logger(), "XY转换结果点数不足: %zu", xy_list.size());
        return false;
    }

    // 获取分段
    auto sections = getSections(begin_index, end_index, msg, xy_list);
    if (sections.empty())
    {
        // RCLCPP_WARN(node_->get_logger(), "没有获取到分段");
        return false;
    }

    // 选择最近的分段
    int nearest_index = -1;
    double nearest_x = 100.0;

    for (size_t i = 0; i < sections.size(); i++)
    {
        int section_start = sections[i].first;
        int section_end = sections[i].second;

        if (section_end - section_start < 10)
            continue;

        double avg_total = 0.0;
        int avg_count = 0;

        for (int index = section_start; index <= section_end; index++)
        {
            if (xy_list.find(index) == xy_list.end())
                continue;

            double x = msg->ranges[index];
            avg_count++;
            avg_total += x;
        }

        if (avg_count == 0)
            continue;

        double avg_value = avg_total / avg_count;

        if (nearest_x > avg_value)
        {
            nearest_x = avg_value;
            nearest_index = i;
        }
    }

    if (nearest_index < 0)
    {
        return false;
    }

    begin_index = sections[nearest_index].first;
    end_index = sections[nearest_index].second;

    // 重置激光数据
    auto filtered_msg = resetLaserData(msg, begin_index, end_index);

    // 发布过滤后的数据
    publishFilteredScan(filtered_msg, filtered_msg->ranges);

    // 进一步处理
    auto xy_list_full = laserScanToXY(filtered_msg);
    if (xy_list_full.size() < 2)
    {
        return false;
    }

    // 获取分段
    std::map<int, std::tuple<int, int, double, double, double, double, double, int>> turn_segments = getSegments(xy_list_full);
    std::map<int, std::tuple<int, int, double, double, double, double, double, int>> filtered_segments = filterSegments(turn_segments);

    if (filtered_segments.empty())
    {
        return false;
    }

    // 创建结果对象
    RechargeResult result;
    result.frameid = frame_index_;

    // 处理左右点
    int segment_index = 0;
    for (const auto &seg : filtered_segments)
    {
        const auto &info = seg.second;
        int start_idx = std::get<0>(info);
        int end_idx = std::get<1>(info);
        double x1 = xy_list_full[start_idx].first;
        if (x1 < 1.2) {
            if (segment_index == 0) {
                start_idx += 3;
            }
            if (segment_index == 1) {
                end_idx += 6;
            }
        }

        for (int index = start_idx; index <= end_idx; index++)
        {
            if (xy_list_full.find(index) == xy_list_full.end())
                continue;

            double x = xy_list_full[index].first;
            double y = xy_list_full[index].second;
            IndexPoint item(x, y, index);

            if (segment_index == 0)
            {
                result.left_points.push_back(item);
            }
            else if (segment_index == 1)
            {
                result.right_points.push_back(item);
            }
        }
        segment_index++;

        if (segment_index >= 2)
            break; // 只需要两个分段
    }

    // 如果成功获取左右点，进行控制
    if (!result.left_points.empty() && !result.right_points.empty())
    {
        procData(result);
        return true;
    }
    return false;
}

void LidarScanController::procData(RechargeResult &result)
{
    if (result.left_points.size() < 2 || result.right_points.size() < 2)
    {
        RCLCPP_WARN(node_->get_logger(), "点数量不足，跳过拟合");
        return;
    }
    {
        auto& vec = result.left_points;
        if (vec.size() > 10) {
            vec.assign(vec.begin() + 3, vec.end());
        } else if (vec.size() > 6) {
            vec.assign(vec.begin() + 1, vec.end());
        }
    }
    {
        auto& vec = result.right_points;
        if (vec.size() > 10) {
            vec.resize(vec.size() * 3 / 5);
        } else if (vec.size() > 5) {
            vec.resize(vec.size() - 2);
        }
    }

    // 拟合左边直线
    auto [k1, b1, yaw1] = RechargeTool::simpleFitLine(result.left_points);
    RCLCPP_DEBUG(node_->get_logger(), "左直线拟合: y = %.4fx + %.4f", k1, b1);

    // 拟合右边直线
    auto [k2, b2, yaw2] = RechargeTool::simpleFitLine(result.right_points);
    {
        yaw1 += M_PI;
        if (yaw1 > M_PI) yaw1-= 2 * M_PI;
        if (yaw1 < -M_PI) yaw1 += 2 * M_PI;
    }
    RCLCPP_DEBUG(node_->get_logger(), "右直线拟合: y = %.4fx + %.4f", k2, b2);

    // 检查是否平行
    if (fabs(k1 - k2) < 1e-10)
    {
        RCLCPP_WARN(node_->get_logger(), "两直线平行，无交点");
        result.middle_point = IndexPoint(0.0, 0.0, 0);
        result.middle_line = StraightLine(0.0, 0.0, 0.0);
        result.left_line = StraightLine(k1, b1, yaw1);
        result.right_line = StraightLine(k2, b2, yaw2);
        return;
    }

    // 计算交点
    double x = (b2 - b1) / (k1 - k2);
    double y = k1 * x + b1;

    // 更新中间点
    result.middle_point = IndexPoint(x, y, 0);

    // 计算角平分线
    auto [k3, b3] = RechargeTool::calculateAngleBisector(k1, b1, k2, b2);

    // 计算角度信息
    double left_angle = atan(k1) * 180.0 / M_PI;
    double right_angle = atan(k2) * 180.0 / M_PI;
    double bisector_angle = atan(k3) * 180.0 / M_PI;

    auto getBisectorAngle = [](double yaw1, double yaw2) {
        // 方法1：直接平均（需要处理角度环绕）
        double avg = (yaw1 + yaw2) / 2.0;
        
        // 处理角度范围 [-π, π) 或 [0, 2π)
        // 确保结果在 [-π, π) 范围内
        if (avg > M_PI) avg -= 2 * M_PI;
        if (avg < -M_PI) avg += 2 * M_PI;
        
        return avg;
    };
    double yaw3 = getBisectorAngle(yaw2, yaw1);
    // 更新结果
    result.middle_line = StraightLine(k3, b3, yaw3);
    result.left_line = StraightLine(k1, b1, yaw1);
    result.right_line = StraightLine(k2, b2, yaw2);

    // RCLCPP_INFO(node_->get_logger(), "k1: %.2f b1: %.2f", k1, b1);
    // RCLCPP_INFO(node_->get_logger(), "k2: %.2f b2: %.2f", k2, b2);
    // RCLCPP_INFO(node_->get_logger(), "k3: %.2f b3: %.2f", k3, b3);


    // 详细打印角平分线信息（用于测试对齐）
    // RCLCPP_INFO(node_->get_logger(), "========== 基站检测信息 ==========frameid: %d", result.frameid);
    // RCLCPP_INFO(node_->get_logger(), "左直线角度: %.2f°", left_angle);
    // RCLCPP_INFO(node_->get_logger(), "右直线角度: %.2f°", right_angle);
    // RCLCPP_INFO(node_->get_logger(), "角平分线角度: %.2f°", bisector_angle);
    // RCLCPP_INFO(node_->get_logger(), "交点: (%.4f, %.4f)", x, (y - param_.dy_diff));

    
    {
        result.rechargePose.dx = result.middle_point.x;
        result.rechargePose.dy_center = result.middle_point.y;
        result.rechargePose.k = result.middle_line.k;
        result.rechargePose.yaw_diff = result.middle_line.yaw;
        result.rechargePose.dy = result.rechargePose.dy_center-param_.dy_diff;
    }

    {
        //过滤跳帧偏差严重的
        if (last_result_.middle_point.x > 0.01) {
            
            double dx = last_result_.middle_point.x - result.middle_point.x;
            double dy = last_result_.middle_point.y - result.middle_point.y;
            last_result_ = result;
            const float threshold = std::min(0.2, result.middle_point.x * 0.2); //越远阈值越大
            double distance = sqrt(dx * dx + dy * dy);
            if (distance > 0.2) {
                return;
            }
        }
    
        if (std::fabs(result.rechargePose.yaw_diff * 180.0 / M_PI) > 60.0) {
            return ;
        }
        last_result_ = result;
    }
    // 可视化
    publishRechargeVisualization(result);
    if (active_) {
        publishStationDetected(true);
    }

    // 异步：将结果推入 RechargeCtrl 队列，由独立工作线程消费
    if (recharge_ctrl_)
        recharge_ctrl_->pushResult(result);
}

// ==================== 过滤函数实现 ====================
void LidarScanController::correctIndexOffset(LaserScanSharedPtr msg)
{
    int fixed = getFixed();
    double diff_yaw = msg->angle_increment * fixed;
    // if (msg->angle_min + diff_yaw < 1.57) {
    //     msg->angle_min += diff_yaw;
    //     msg->angle_max += diff_yaw;
    // } else {
    //     msg->angle_min += diff_yaw;
    //     msg->angle_max += diff_yaw;
    // }
{
    std::vector<float> ranges = msg->ranges;
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
        int index = getFixedIndex(msg, i);
        ranges[index] = msg->ranges[i];
    }
    msg->ranges = ranges;
}
     {
        //新增 intensities错位
        std::vector<float> intensities = msg->intensities;
        for (size_t i = 0; i < msg->intensities.size(); i++)
        {
            int index = getFixedIndex(msg, i);
            intensities[index] = msg->intensities[i];
        }
        msg->intensities = intensities;
    }
}

void LidarScanController::filterByAngleRange(LaserScanSharedPtr  msg)
{
    // 过滤角度，只接受后面90度
    int min_index = param_.min_index + 1;
    int max_index = param_.max_index - 1;

    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
        if (i < min_index || i > max_index)
        {
            msg->ranges[i] = std::numeric_limits<float>::infinity();
        }
    }
}

void LidarScanController::filterByDistance(LaserScanSharedPtr  msg)
{
    // 过滤距离
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
        if (!isValid(msg->ranges[i]))
            continue;

        if (msg->ranges[i] < param_.min_distance ||
            msg->ranges[i] > param_.max_distance)
        {
            msg->ranges[i] = std::numeric_limits<float>::infinity();
        }
    }
}

void LidarScanController::filterByIntensity(LaserScanSharedPtr msg)
{
    int min_intensities = param_.min_intensities;
    int max_intensities = param_.max_intensities;
    // 过滤强度
    int max_index = static_cast<int>(std::min(msg->ranges.size(), msg->intensities.size()));
    for (size_t i = 0; i < max_index; i++)
    {
        if (!isValid(msg->ranges[i]))
            continue;

        float value = msg->intensities[i];
        if (value < min_intensities || value > max_intensities)
        {
            msg->ranges[i] = std::numeric_limits<float>::infinity();
        }

        // 再次检查距离
        if (msg->ranges[i] < param_.min_distance ||
            msg->ranges[i] > param_.max_distance)
        {
            msg->ranges[i] = std::numeric_limits<float>::infinity();
        }
    }
}

// ==================== 坐标转换函数 ====================
std::map<int, std::pair<double, double>> LidarScanController::laserScanToXY(
    const LaserScanSharedPtr &scan_msg)
{
    std::map<int, std::pair<double, double>> xy_points_map;
    double angle = scan_msg->angle_min + param_.scan_angle_diff;

    for (size_t i = 0; i < scan_msg->ranges.size(); i++)
    {
        float range_val = scan_msg->ranges[i];

        // 跳过无效测量值
        if (range_val >= scan_msg->range_min && range_val <= scan_msg->range_max)
        {
            // 极坐标转笛卡尔坐标
            double x1 = range_val * cos(angle);
            double y1 = range_val * sin(angle);
            double x = y1;
            double y = -x1;
            xy_points_map[i] = {x, y};
        }
        angle += scan_msg->angle_increment;
    }
    return xy_points_map;
}

std::map<int, std::pair<double, double>> LidarScanController::laserScanToXY1(
    int begin_index, int end_index,
    const LaserScanSharedPtr &scan_msg)
{
    std::map<int, std::pair<double, double>> xy_points_map;
    double angle = scan_msg->angle_min + param_.scan_angle_diff;
    angle += (begin_index * scan_msg->angle_increment);

    for (int i = begin_index - 1; i <= end_index; i++)
    {
        float range_val = scan_msg->ranges[i];

        if (range_val >= scan_msg->range_min && range_val <= scan_msg->range_max)
        {
            double x1 = range_val * cos(angle);
            double y1 = range_val * sin(angle);
            double x = y1;
            double y = -x1;
            xy_points_map[i] = {x, y};
        }
        angle += scan_msg->angle_increment;
    }
    return xy_points_map;
}

// ==================== 分段处理函数 ====================
std::vector<std::pair<int, int>> LidarScanController::getSections(
    int begin_index, int end_index,
    const LaserScanSharedPtr &msg,
    const std::map<int, std::pair<double, double>> &xy_list)
{
    std::vector<std::pair<int, int>> sections;

    int section_start = begin_index + 1;
    int section_end = 0;

    int last_index = -1;
    double last_x = 0.0, last_y = 0.0;

    for (int i = begin_index + 1; i < end_index - 1; i++)
    {
        if (xy_list.find(i) == xy_list.end())
        {
            continue;
        }

        auto curr = xy_list.at(i);
        if (last_index < 0)
        {
            last_index = i;
            last_x = curr.first;
            last_y = curr.second;
            continue;
        }

        double dx = curr.first - last_x;
        double dy = curr.second - last_y;
        double dist = sqrt(dx * dx + dy * dy);

        if (dist > 0.05 || (i - last_index) > 2)
        {
            section_end = i;
            if ((section_end - section_start) >= param_.min_count)
            {
                sections.emplace_back(section_start, section_end);
            }
            section_start = i;
        }

        last_index = i;
        last_x = curr.first;
        last_y = curr.second;
    }

    if (last_index > section_end + 10)
    {
        section_end = last_index;
        if ((section_end - section_start) > param_.min_count)
        {
            sections.emplace_back(section_start, section_end);
        }
    }

    return sections;
}

std::map<int, std::tuple<int, int, double, double, double, double, double, int>>
LidarScanController::getSegments(const std::map<int, std::pair<double, double>> &xy_points_map)
{
    std::map<int, std::tuple<int, int, double, double, double, double, double, int>> turn_segments;

    if (xy_points_map.empty())
    {
        return turn_segments;
    }

    // 将点转换为路径
    std::vector<IndexPoint> path;
    for (const auto &point : xy_points_map)
    {
        path.emplace_back(point.second.first, point.second.second, point.first);
    }
    IndexPoint center_point = path[path.size()/2];

    // 使用相同的参数进行分段
    /*
        远距离  24.0, 4
        近距离 31.0, 4
    */
    double angle_threshold_deg = 25.0;
    double distance_threshold_deg = 0.2 - 0.06;
    int history_window_size = 4;
    if(center_point.x < 0.64) {
        //近距离 31.0, 4
        angle_threshold_deg = 24.0;
        history_window_size = 6;
        distance_threshold_deg =  0.145;
    } else if(center_point.x < 0.7) {
        angle_threshold_deg = 20.0;
        history_window_size = 5;
        distance_threshold_deg =  0.145;
    } else if(center_point.x < 0.9) {
        angle_threshold_deg = 25.0;
        history_window_size = 4;
        distance_threshold_deg =  0.13;
    } else if (center_point.x < 1.2) {
        angle_threshold_deg = 24.0;
        history_window_size = 3;
        distance_threshold_deg =  0.2 - 0.09;
    }  else {
        angle_threshold_deg = 18.0;
        history_window_size = 3;
        distance_threshold_deg =  0.2 - 0.09;
    }
    std::vector<std::vector<IndexPoint>> segments;
    if(center_point.x < 0.7) {
        segments = RechargeTool::splitPointsToSegments_near(path, angle_threshold_deg, history_window_size);
    } else {
        segments = RechargeTool::splitPointsToSegments(path, angle_threshold_deg, history_window_size);
    }

    //第二段少0.03米
    float distance_second_diff = 0.028;
    // 处理每个分段
    for (size_t i = 0; i < segments.size(); i++)
    {
        if (segments[i].size() < 2)
            continue;

        // 获取分段的前两个点
        const auto &s_prev = segments[i][0];
        const auto &s_next = segments[i][1];

        // 计算距离和角度
        double dx = s_next.x  - s_prev.x;
        double dy = -1*(s_next.y - s_prev.y);
        double distance = sqrt(dx * dx + dy * dy);
        double line_angle = std::atan2(dy, dx);
        double angle_deg = line_angle * 180.0 / M_PI;
        // printf("line_angle: %.2f弧度, angle_deg: %.2f度\n", line_angle,angle_deg);
        int index_start = s_prev.index;
        int index_end = s_next.index;
        if ((center_point.x < 0.7) && (turn_segments.size() > 0) && (i > 0)) {
            distance_second_diff = 0.06; // 非常近的时候，第二段雷达扫描空缺或者变形
        }

        //基站参数
        const float station_sidelength_max = 0.22;

        // 应用过滤条件
        if (distance < station_sidelength_max  && distance > (distance_threshold_deg - i*distance_second_diff) && (index_end - index_start) > 3)
        {
            turn_segments[index_start] = std::make_tuple(
                index_start, // 0: 开始索引
                index_end,   // 1: 结束索引
                0.0,         // 2: 0.0
                s_prev.x,    // 3: x坐标
                s_prev.y,    // 4: y坐标
                distance,    // 5: 距离
                line_angle,  // 6: 线角度
                1            // 7: 1
            );
        }
    }

    return turn_segments;
}

std::map<int, std::tuple<int, int, double, double, double, double, double, int>>
LidarScanController::filterSegments(
    const std::map<int, std::tuple<int, int, double, double, double, double, double, int>> &turn_segments)
{
    std::map<int, std::tuple<int, int, double, double, double, double, double, int>> new_segments;

    if (turn_segments.empty() || turn_segments.size() < 2)
    {
        return new_segments;
    }

    bool last_line_angle_valid = false;
    double last_line_angle = 0.0;
    std::tuple<int, int, double, double, double, double, double, int> last_seg;
    int last_key = -1;

    // 遍历所有分段
    for (const auto &seg : turn_segments)
    {
        const auto &info = seg.second;
        float dx = std::get<3>(info);
        double line_angle = std::get<6>(info);

        if (last_line_angle_valid)
        {
            // 计算角度差
            double diff_angle = -1* (line_angle - last_line_angle);
            // 目标角度计算
            double target_angle1 = 0.5 * M_PI * (120.0 / 90.0); // 120°
            double target_angle2 = 0.5 * M_PI * (60.0 / 90.0); // 120°反向 180-120=60°
            double target_angle = target_angle2;
            // if (line_angle < 0) {
            //     target_angle = target_angle2;
            // }
            double tolerance = 0.24;

            // 角度条件判断
            if (diff_angle > (target_angle - tolerance) && diff_angle < (target_angle + tolerance))
            {
                // 索引检查
                int right_start_idx = std::get<0>(info);
                int left_end_idx = std::get<1>(last_seg);
                int max_dur = 3;
                if (dx < 0.5) {
                    max_dur = 6;
                } else if (dx < 0.7) {
                    max_dur = 5;
                }

                if (right_start_idx - left_end_idx <= max_dur)
                {
                    // 添加到结果
                    new_segments[last_key] = last_seg;
                    new_segments[seg.first] = info;
                    break; // 找到一对就跳出循环
                } else {
                    printf(" ");
                }
            } else {
                printf(" ");
            }
        }

        // 更新上一个分段信息
        last_line_angle = line_angle;
        last_line_angle_valid = true;
        last_seg = info;
        last_key = seg.first;
    }

    return new_segments;
}

// ==================== 辅助函数实现 ====================
void LidarScanController::publishRechargeVisualization(const RechargeResult &result)
{
    auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();

    // 添加中线标记
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "laser_frame_back2";
    line_marker.header.stamp = node_->now();
    line_marker.ns = "rotating_line";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;

    int color_index = result.frameid % 3;

    switch (color_index)
    {
    case 0:
        line_marker.scale.x = 0.02;
        line_marker.color.a = 1.0;
        line_marker.color.r = 1.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 0.0;
        break;
    case 1:
        line_marker.scale.x = 0.02;
        line_marker.color.a = 1.0;
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        break;
    case 2:
        line_marker.scale.x = 0.02;
        line_marker.color.a = 1.0;
        line_marker.color.r = 0.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 1.0;
        break;
    
    default:
        break;
    }


    geometry_msgs::msg::Point p1, p2;
    {
        double x1 = result.middle_point.x;
        double y1 = result.middle_point.y;
        double yaw = result.middle_line.yaw + M_PI;
        double length = 2.0;
        // 计算方向向量
        double dx = std::cos(yaw);
        double dy = std::sin(yaw);
        
        // 计算第二个点
        double x2 = x1 + dx * length;
        double y2 = y1 + dy * length;
        
        // 返回线段: (起点, 终点)
    p1.x = x1;
        p1.y = y1;
    p2.x = x2;
        p2.y = y2;
    }

    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);
    marker_array->markers.push_back(line_marker);

    {
    // 添加中间点标记
    visualization_msgs::msg::Marker middle_marker;
    middle_marker.header.frame_id = "laser_frame_back2";
    middle_marker.header.stamp = node_->now();
    middle_marker.ns = "middle_point";
    middle_marker.id = 1;
    middle_marker.type = visualization_msgs::msg::Marker::SPHERE;
    middle_marker.action = visualization_msgs::msg::Marker::ADD;
    middle_marker.pose.orientation.w = 1.0;
    middle_marker.scale.x = 0.03;
    middle_marker.scale.y = 0.03;
    middle_marker.color.b = 1.0;
    middle_marker.color.a = 1.0;

    middle_marker.pose.position.x = result.middle_point.x;
    middle_marker.pose.position.y = result.middle_point.y;
    middle_marker.pose.position.z = 0.0;

    marker_array->markers.push_back(middle_marker);
    }

    if (result.left_points.size() > 1)
    {
        // 添加中间点标记
        visualization_msgs::msg::Marker middle_marker;
        middle_marker.header.frame_id = "laser_frame_back2";
        middle_marker.header.stamp = node_->now();
        middle_marker.ns = "left_point";
        middle_marker.id = 1;
        middle_marker.type = visualization_msgs::msg::Marker::SPHERE;
        middle_marker.action = visualization_msgs::msg::Marker::ADD;
        middle_marker.pose.orientation.w = 1.0;
        middle_marker.scale.x = 0.03;
        middle_marker.scale.y = 0.03;
        middle_marker.color.b = 1.0;
        middle_marker.color.a = 1.0;

        const IndexPoint& point = result.left_points[0];

        middle_marker.pose.position.x = point.x;
        middle_marker.pose.position.y = point.y;
        middle_marker.pose.position.z = 0.0;

        marker_array->markers.push_back(middle_marker);
    }
    if (result.right_points.size() > 1)
    {
        // 添加中间点标记
        visualization_msgs::msg::Marker middle_marker;
        middle_marker.header.frame_id = "laser_frame_back2";
        middle_marker.header.stamp = node_->now();
        middle_marker.ns = "right_point";
        middle_marker.id = 1;
        middle_marker.type = visualization_msgs::msg::Marker::SPHERE;
        middle_marker.action = visualization_msgs::msg::Marker::ADD;
        middle_marker.pose.orientation.w = 1.0;
        middle_marker.scale.x = 0.03;
        middle_marker.scale.y = 0.03;
        middle_marker.color.b = 1.0;
        middle_marker.color.a = 1.0;

        const IndexPoint& point = result.right_points[result.right_points.size() - 1];

        middle_marker.pose.position.x = point.x;
        middle_marker.pose.position.y = point.y;
        middle_marker.pose.position.z = 0.0;

        marker_array->markers.push_back(middle_marker);
    }

    // 发布标记数组
    pub_rviz_->publish(*marker_array);
}

void LidarScanController::publishFilteredScan(
    const LaserScanSharedPtr &original_msg,
    const std::vector<float> &filtered_ranges)
{
    auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    msg->header = original_msg->header;
    msg->header.frame_id = "laser_frame_back1";
    msg->angle_min = original_msg->angle_min - param_.scan_angle_diff;
    msg->angle_max = original_msg->angle_max - param_.scan_angle_diff;
    msg->angle_increment = original_msg->angle_increment;
    msg->time_increment = original_msg->time_increment;
    msg->scan_time = original_msg->scan_time;
    msg->range_min = original_msg->range_min;
    msg->range_max = original_msg->range_max;
    msg->ranges = filtered_ranges;
    msg->intensities = original_msg->intensities;

    pub_filtered_->publish(*msg);
    // RCLCPP_DEBUG(node_->get_logger(), "已发布过滤后的激光数据");
}

LidarScanController::LaserScanSharedPtr LidarScanController::resetLaserData(
    LaserScanSharedPtr msg,
    int b_index, int e_index)
{
    std::vector<float> filtered_ranges = msg->ranges;
    std::vector<float> intensities = msg->intensities;

    int max_index = static_cast<int>(std::min(msg->ranges.size(), msg->intensities.size()));

    for (int i = 0; i < b_index && i < max_index; i++)
    {
        filtered_ranges[i] = std::numeric_limits<float>::infinity();
        intensities[i] = 0.0;
    }

    for (int i = e_index; i < max_index; i++)
    {
        filtered_ranges[i] = std::numeric_limits<float>::infinity();
        intensities[i] = 0.0;
    }

    msg->ranges = filtered_ranges;
    msg->intensities = intensities;

    return msg;
}


int LidarScanController::getFixedIndex(const LaserScanSharedPtr &msg, int i) const
{
    int fixed = getFixed();
    if (fixed == 0)
        return i;
  
    int length = msg->ranges.size();
    int max_index = length;
    int index;
    if (i + fixed < 0) {
        index = max_index + (fixed + i);
    } else if (i + fixed >= max_index)
    {
        index = i + fixed - max_index;
    } else {
        index = i + fixed;
    }
    return index;
}

void LidarScanController::publishControl()
{
    if (active_)
    {
        pub_ctrl_->publish(ctrl_twist_);
    }
}


// ==========================================================

void LidarScanController::setChargingVoltage(bool value)
{
    detect_charging_voltage_ = value;
    detect_charging_voltage_time_ = node_->now();
    auto msg = std_msgs::msg::String();
    std::stringstream ss;
    ss << (value ? "CHARGING_VOLTAGE_DETECTED" : "CHARGING_VOLTAGE_LOST")
       << ":timestamp=" << node_->now().seconds()
       << ",voltage_status=" << (value ? "true" : "false");

    if (value)
    {
        ss << ",message=充电电压已检测到，开始充电流程";
        RCLCPP_INFO(node_->get_logger(), "检测到充电电压");
    }
    else
    {
        ss << ",message=充电电压已断开";
        RCLCPP_INFO(node_->get_logger(), "充电电压已断开");
    }
    msg.data = ss.str();
    pub_charging_voltage_->publish(msg);
    if (value)
    {
        auto msg =std_msgs::msg::Int32();
        msg.data = RECHARGE_FEEDBACK_SUCCESS_DOCKED;
        // msg.data = RECHARGE_EVENT_STATION_NOT_FOUND;
        pub1_->publish(msg);
        RCLCPP_WARN(node_->get_logger(), "上报状态 2");
    }
}

bool LidarScanController::getChargingVoltage() const
{
    auto now = node_->now();
    auto diff = now - detect_charging_voltage_time_;

    // 如果检测到充电电压且在0.1秒内，认为有效
    // if (detect_charging_voltage_ && diff.seconds() < 0.1)
    // {
    //     return true;
    // }
    // return false;
    return detect_charging_voltage_; 
}