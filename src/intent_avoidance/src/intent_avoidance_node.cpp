#include "intent_avoidance/intent_avoidance_node.h"
#include "intent_avoidance/scan_processor.h"
#include "intent_avoidance/free_space_analyzer.h"
#include "intent_avoidance/intent_detector.h"
#include "intent_avoidance/avoidance_engine.h"
#include "intent_avoidance/control_generator.h"
#include "intent_avoidance/types.h"

// 话题常量（与 common/utils/topic.h 保持一致，避免额外依赖）
namespace {
constexpr const char* kLeftLaserTopic = "/scan/left";
constexpr const char* kRightLaserTopic = "/scan/right";
constexpr const char* kJoystickTopic = "/sensor/joystick";
constexpr const char* kWheelSpeedStaTopic = "/wheel/twist";
constexpr const char* kNav2ControlTopic = "/nav/control";
}

namespace intent_avoidance {

IntentAvoidanceNode::IntentAvoidanceNode(const rclcpp::NodeOptions& options)
    : Node("intent_avoidance_node", options)
{
    // ========== 参数声明 ==========
    // 感知
    this->declare_parameter("safe_distance_threshold", 1.2);
    this->declare_parameter("min_free_sector_width", 0.349);     // ≈20°
    this->declare_parameter("left_radar_x", 0.690);
    this->declare_parameter("left_radar_y", 0.255);
    this->declare_parameter("left_radar_yaw", 0.0);
    this->declare_parameter("right_radar_x", 0.690);
    this->declare_parameter("right_radar_y", -0.255);
    this->declare_parameter("right_radar_yaw", -0.03);

    // 意图
    this->declare_parameter("joystick_deadzone", 15);

    // 决策
    this->declare_parameter("enable_intent_avoidance", true);
    this->declare_parameter("direction_lock_frames", 8);
    this->declare_parameter("direction_unlock_threshold", 0.524);  // ≈30°
    this->declare_parameter("speed_reduction_factor", 0.6);
    this->declare_parameter("emergency_brake_ttc", 0.5);
    this->declare_parameter("emergency_brake_distance", 0.3);

    // 控制
    this->declare_parameter("max_angular_guidance", 0.5);
    this->declare_parameter("max_linear_accel", 0.8);
    this->declare_parameter("output_smooth_alpha", 0.3);
    this->declare_parameter("speed_recovery_time", 1.5);
    this->declare_parameter("control_frequency", 10.0);

    // 车辆
    this->declare_parameter("robot_width", 0.70);
    this->declare_parameter("narrow_ratio_min", 1.2);
    this->declare_parameter("narrow_ratio_max", 2.5);

    // ========== 组件初始化 ==========

    // 左雷达处理器
    ScanProcessor::Config left_cfg;
    left_cfg.safe_distance_threshold = this->get_parameter("safe_distance_threshold").as_double();
    left_cfg.mount.x = this->get_parameter("left_radar_x").as_double();
    left_cfg.mount.y = this->get_parameter("left_radar_y").as_double();
    left_cfg.mount.yaw = this->get_parameter("left_radar_yaw").as_double();
    left_processor_ = std::make_unique<ScanProcessor>(left_cfg);

    // 右雷达处理器
    ScanProcessor::Config right_cfg;
    right_cfg.safe_distance_threshold = this->get_parameter("safe_distance_threshold").as_double();
    right_cfg.mount.x = this->get_parameter("right_radar_x").as_double();
    right_cfg.mount.y = this->get_parameter("right_radar_y").as_double();
    right_cfg.mount.yaw = this->get_parameter("right_radar_yaw").as_double();
    right_processor_ = std::make_unique<ScanProcessor>(right_cfg);

    // 可通行空间分析器
    FreeSpaceAnalyzer::Config fsa_cfg;
    fsa_cfg.min_free_sector_width_rad = this->get_parameter("min_free_sector_width").as_double();
    fsa_cfg.robot_width = this->get_parameter("robot_width").as_double();
    fsa_cfg.narrow_ratio_min = this->get_parameter("narrow_ratio_min").as_double();
    fsa_cfg.narrow_ratio_max = this->get_parameter("narrow_ratio_max").as_double();
    free_space_analyzer_ = std::make_unique<FreeSpaceAnalyzer>(fsa_cfg);

    // 意图检测器
    IntentDetector::Config id_cfg;
    id_cfg.joystick_deadzone = static_cast<int16_t>(this->get_parameter("joystick_deadzone").as_int());
    intent_detector_ = std::make_unique<IntentDetector>(id_cfg);

    // 避障引擎
    AvoidanceEngine::Config ae_cfg;
    ae_cfg.enable_intent_avoidance = this->get_parameter("enable_intent_avoidance").as_bool();
    ae_cfg.direction_lock_frames = this->get_parameter("direction_lock_frames").as_int();
    ae_cfg.direction_unlock_threshold_rad = this->get_parameter("direction_unlock_threshold").as_double();
    ae_cfg.speed_reduction_factor = this->get_parameter("speed_reduction_factor").as_double();
    ae_cfg.max_angular_guidance = this->get_parameter("max_angular_guidance").as_double();
    ae_cfg.emergency_brake_ttc = this->get_parameter("emergency_brake_ttc").as_double();
    ae_cfg.emergency_brake_distance = this->get_parameter("emergency_brake_distance").as_double();
    avoidance_engine_ = std::make_unique<AvoidanceEngine>(ae_cfg);

    // 控制生成器
    ControlGenerator::Config cg_cfg;
    cg_cfg.max_angular_guidance = this->get_parameter("max_angular_guidance").as_double();
    cg_cfg.max_linear_accel = this->get_parameter("max_linear_accel").as_double();
    cg_cfg.output_smooth_alpha = this->get_parameter("output_smooth_alpha").as_double();
    cg_cfg.speed_recovery_time = this->get_parameter("speed_recovery_time").as_double();
    cg_cfg.cycle_time = 1.0 / this->get_parameter("control_frequency").as_double();
    control_generator_ = std::make_unique<ControlGenerator>(cg_cfg);

    // ========== 订阅 ==========
    left_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        kLeftLaserTopic, rclcpp::SensorDataQoS(),
        std::bind(&IntentAvoidanceNode::onLeftScan, this, std::placeholders::_1));

    right_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        kRightLaserTopic, rclcpp::SensorDataQoS(),
        std::bind(&IntentAvoidanceNode::onRightScan, this, std::placeholders::_1));

    joystick_sub_ = this->create_subscription<interface::msg::Joystick>(
        kJoystickTopic, 10,
        std::bind(&IntentAvoidanceNode::onJoystick, this, std::placeholders::_1));

    wheel_speed_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        kWheelSpeedStaTopic, 10,
        std::bind(&IntentAvoidanceNode::onWheelSpeed, this, std::placeholders::_1));

    // ========== 发布 ==========
    control_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(kNav2ControlTopic, 10);

    // ========== 定时器（主控制循环） ==========
    double freq = this->get_parameter("control_frequency").as_double();
    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / freq),
        std::bind(&IntentAvoidanceNode::onControlTimer, this));

    RCLCPP_INFO(this->get_logger(), "IntentAvoidanceNode started, control freq=%.1f Hz", freq);
}

void IntentAvoidanceNode::onLeftScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    left_processor_->processScan(*msg);
}

void IntentAvoidanceNode::onRightScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    right_processor_->processScan(*msg);
}

void IntentAvoidanceNode::onJoystick(const interface::msg::Joystick::SharedPtr msg)
{
    latest_intent_ = intent_detector_->detect(*msg);
    intent_received_ = true;
}

void IntentAvoidanceNode::onWheelSpeed(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    latest_speed_.linear_x = msg->twist.linear.x;
    latest_speed_.angular_z = msg->twist.angular.z;
}

void IntentAvoidanceNode::onControlTimer()
{
    // 无摇杆数据时不输出
    if (!intent_received_) {
        return;
    }

    // 获取最新的雷达扇区数据
    auto left_sectors = left_processor_->getSectors();
    auto right_sectors = right_processor_->getSectors();

    // 计算可通行空间
    FreeSectorList free_sectors = free_space_analyzer_->computeFreeSpace(
        left_sectors, right_sectors);

    // 窄道检测
    NarrowPassageInfo narrow_info = free_space_analyzer_->detectNarrowPassage(free_sectors);

    // 避障决策
    AvoidanceDecision decision = avoidance_engine_->decide(
        free_sectors, narrow_info, latest_intent_, latest_speed_);

    // 生成控制指令
    geometry_msgs::msg::Twist control_msg = control_generator_->generate(
        decision, latest_intent_, latest_speed_);

    // 发布
    control_pub_->publish(control_msg);
}

}  // namespace intent_avoidance
