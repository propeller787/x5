/**
 * @file safety_node.h
 * @brief Footprint + Rolling LocalMap 混合避障主节点
 *
 * 职责：
 * 1. 订阅多路雷达、手柄、轮速反馈
 * 2. 10 Hz 更新 RollingLocalMap，20 Hz 执行避障控制环
 * 3. 发布经安全干预后的 /nav/control
 *
 * 数据流：
 *   LidarPerception → ObstacleField + 点云
 *   RollingLocalMap ← 点云落格
 *   AvoidanceController ← 手柄 + ObstacleField + LocalMap
 *   OutputShaper → /nav/control
 *
 * 注意：须在 std::make_shared<SafetyNode>() 之后调用 attachComponents()，
 *       因内部需要使用 shared_from_this() 创建订阅与定时器。
 */
#pragma once

#include "safety_node/avoidance_controller.h"
#include "safety_node/lidar_perception.h"
#include "safety_node/mode_supervisor.h"
#include "safety_node/output_shaper.h"
#include "safety_node/joystick_utils.h"
#include "safety_node/core/warning_reporter.h"
#include "safety_node/core/front_tilt_speed_cap.h"
#include "safety_node/rolling_local_map.h"
#include "safety_node/safety_params.h"
#include "safety_node/vehicle_io.h"

#include <geometry_msgs/msg/twist.hpp>
#include <interface/msg/collision_warning.hpp>
#include <interface/msg/tilt_obstacle_scan.hpp>
#include <interface/srv/scene_controls.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

class SafetyNode: public rclcpp::Node {
public:
  explicit SafetyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SafetyNode() override;

  /** 挂载感知、控制、定时器（make_shared 之后调用） */
  void attachComponents();

private:
  void declareAndLoadParams();
  void setupComponents();
  void setupTimers();

  /** 创建运行时开关服务：安全 / 标准 / 巡航（与 scene_node 同名） */
  void setupModeServices();
  /**
   * 从统一用户配置恢复安全开关（对齐 scene_node::loadAuxiliaryModeState 的读法）。
   * 读取 /opt/freego/user_config.yaml 的 safety_warning_enabled（1=开 / 0=关）；
   * 生产默认关闭：文件不存在、字段缺失或解析失败时安全开关保持关闭。
   * 仅当 user_config_file 为空（单测/无持久化）时，按 mode.default 推导启动值。
   * @return true 成功从配置恢复；false 使用默认状态
   */
  bool loadAuxiliaryModeState();
  /**
   * 切换安全（避障）开关。ON → 输出经完整避障链（SAFE_ASSIST 剖面）；
   * OFF → 输出仅做绝对限速透传（PASS_THROUGH 剖面，不做障碍避让）。与巡航正交。
   */
  void setSafetyEnabled(bool enabled);
  /**
   * 切换巡航（定速保持）开关。ON → 注入定速保持速度；OFF → 使用原始手柄命令。
   * 与安全开关正交：巡航是否经过避障由安全开关独立决定。
   * 标准模式关且巡航关时无控制权：不发 /nav/control、不做避障。
   * @param notify_manager true 时（仅自动退出路径）异步通知 /manage/cruise start=0，
   *        让 scene_services 切回默认标准模式；手动 /scene/cruise 关断不通知，避免重入。
   */
  void setCruiseEnabled(bool enabled, bool notify_manager = false);
  /**
   * 是否具备控制输出权：标准模式开，或巡航开。
   * 对齐 scene_node：两者皆关时静默（不发 /nav/control、跳过避障链）。
   */
  bool hasControlAuthority() const;
  /** 进入静默时清空整形器与上一拍输出，避免恢复后带着陈旧速度。 */
  void clearControlOutputState();
  /** 异步调用 /manage/cruise start=0（对齐 scene_node::handleCruiseClose） */
  void notifyManageCruiseClose();
  /** 安全开关服务：start=1 开启安全避障；start=0 关闭（透传，不做避障） */
  void handleSafeService(
    const std::shared_ptr<interface::srv::SceneControls::Request> request,
    std::shared_ptr<interface::srv::SceneControls::Response> response);
  /** 标准模式服务：start=1/0，与 scene_node /scene/standard 同构 */
  void handleStandardService(
    const std::shared_ptr<interface::srv::SceneControls::Request> request,
    std::shared_ptr<interface::srv::SceneControls::Response> response);
  /** 巡航开关服务：start=1 开启定速保持；start=0 关闭巡航（不影响安全开关） */
  void handleCruiseService(
    const std::shared_ptr<interface::srv::SceneControls::Request> request,
    std::shared_ptr<interface::srv::SceneControls::Response> response);
  /**
   * 巡航模式单周期更新：维护定速保持输出（松杆保持速度、转向按平方比降速、
   * 后退意图或启动超时自动退出）。返回注入避障链的等效期望速度；
   * @param gear_max_speed 当前档位/驾驶模式最高线速度 (m/s)；<=0 表示 CarInfo 未知，按绝对上限兜底。
   * @param brake_engaged  手动抱闸是否开启（开启则退出巡航）。
   * @param[out] holding   本周期是否正在注入巡航保持速度（用于绕过回中刹停）。
   */
  geometry_msgs::msg::Twist updateCruise(
    const ControlCommand & control, double gear_max_speed, bool brake_engaged, bool & holding);

  /** 20 Hz 主控制环：读输入 → 避障 → 整形 → 发布 */
  void onControlTick();

  /** 10 Hz 局部地图融合与膨胀 */
  void onLocalMapUpdate();

  /** 5 Hz 占据栅格可视化（可选） */
  void onMapVizTick();

  void onPerceptionVizTick();

  /**
   * 方案 7 控制环看门狗（1 Hz，background 组）：
   * 检查控制环 tick 时效并发布 /safety/health 健康摘要；
   * 控制环停摆超过 10× 周期时兜底发布一帧零速（根本兜底仍是底盘侧 dead-man）。
   */
  void onWatchdogTick();

  /** 方案 9.3 启动时打印生效配置摘要（各层 enabled 位图 + 关键安全参数） */
  void logStartupConfigSummary();

  void publishControl(const geometry_msgs::msg::Twist & cmd);

  /** 干预、速度衰减或刹停时发布 JSON 可解释日志 */
  void publishInterventionLog(const AvoidanceResult & result);

  // ── 对外状态/预警上报（三个话题与 scene_node 同构，见 safety_params.h ReportingConfig） ──

  /** 发布 /scene_auxiliary/status：安全（避障）开关当前状态；1 Hz 心跳与开关跳变共用 */
  void publishAuxiliaryStatus();

  /** 发布 /vlago/cruise/status：巡航过程/结束原因码，取值见 cpp 内 CruiseStatusCode */
  void publishCruiseStatus(int32_t code);

  /**
   * 发布 /vlago/collision/warning：把本周期四向干预等级喂给状态机并发出其产生的事件。
   * @param result           本周期避障裁决（提供 risk.levels 与急停原因）
   * @param fallback_direction 急停但 levels 无方向时的兜底方向（按驾驶员意图/实际速度判定）
   * @param now_time         本周期时刻，用于稳态心跳节流
   */
  void publishCollisionWarning(
    const AvoidanceResult & result, CollisionDirection fallback_direction,
    const rclcpp::Time & now_time);

  /**
   * 巡航建立后的「被障碍物压停」退出判定（对齐 scene_node 的 reason=2）：
   * 干预后输出线速度持续 speed_timeout_s 低于 obstacle_zero_speed_eps_m_s → 退出巡航并上报 2。
   * 每个控制周期在整形后调用。
   * @param output_linear 本周期最终输出线速度 (m/s)
   */
  void updateCruiseObstacleExit(double output_linear, const rclcpp::Time & now_time);

  /** 模式切换时清空地图、滤波器与感知缓存 */
  void resetPipelineState();

  SafetyNodeParams params_;
  ModeSupervisor mode_supervisor_;

  std::unique_ptr < LidarPerception > perception_;
  std::unique_ptr < RollingLocalMap > local_map_;
  std::unique_ptr < AvoidanceController > controller_;
  std::unique_ptr < VehicleIo > vehicle_io_;
  OutputShaper output_shaper_;

  rclcpp::Publisher < geometry_msgs::msg::Twist > ::SharedPtr control_pub_;
  rclcpp::Publisher < nav_msgs::msg::OccupancyGrid > ::SharedPtr map_viz_pub_;
  rclcpp::Publisher < sensor_msgs::msg::PointCloud2 > ::SharedPtr perception_points_pub_;
  rclcpp::Publisher < visualization_msgs::msg::MarkerArray > ::SharedPtr perception_markers_pub_;
  rclcpp::Publisher < interface::msg::TiltObstacleScan > ::SharedPtr tilt_left_debug_pub_;
  rclcpp::Publisher < interface::msg::TiltObstacleScan > ::SharedPtr tilt_right_debug_pub_;
  rclcpp::Publisher < sensor_msgs::msg::PointCloud2 > ::SharedPtr tilt_left_points_pub_;
  rclcpp::Publisher < sensor_msgs::msg::PointCloud2 > ::SharedPtr tilt_right_points_pub_;
  rclcpp::Publisher < std_msgs::msg::String > ::SharedPtr log_pub_;
  rclcpp::Publisher < std_msgs::msg::Bool > ::SharedPtr auxiliary_status_pub_;
  rclcpp::Publisher < std_msgs::msg::Int32 > ::SharedPtr cruise_status_pub_;
  rclcpp::Publisher < interface::msg::CollisionWarning > ::SharedPtr collision_warning_pub_;

  rclcpp::Service < interface::srv::SceneControls > ::SharedPtr safe_mode_service_;
  rclcpp::Service < interface::srv::SceneControls > ::SharedPtr standard_mode_service_;
  rclcpp::Service < interface::srv::SceneControls > ::SharedPtr cruise_mode_service_;
  /** 巡航自动退出时通知 scene_services（/manage/cruise） */
  rclcpp::Client < interface::srv::SceneControls > ::SharedPtr cruise_close_client_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr map_update_timer_;
  rclcpp::TimerBase::SharedPtr map_viz_timer_;
  rclcpp::TimerBase::SharedPtr perception_viz_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr auxiliary_status_timer_;

  // ── 方案 7 回调分组（单线程执行器下无害，多线程执行器下隔离控制环）──
  // control：控制定时器 + 模式服务（服务与控制环同组 → 切换与 evaluate 天然互斥）
  // perception：5 路雷达订阅 + VehicleIo 订阅（内部已有 mutex，组内互斥再加一层保险）
  // background：地图融合 + 双路可视化 + 看门狗（低优先级，不与控制环抢线程）
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::CallbackGroup::SharedPtr perception_group_;
  rclcpp::CallbackGroup::SharedPtr background_group_;

  rclcpp::Publisher < std_msgs::msg::String > ::SharedPtr health_pub_;

  bool perception_viz_enabled_ = false;
  double perception_viz_rate_hz_ = 5.0;
  bool joystick_trajectory_viz_enabled_ = true;
  bool speed_intervention_debug_enabled_ = true;
  double speed_intervention_debug_min_delta_m_s_ = 0.03;
  int64_t speed_intervention_debug_throttle_ms_ = 200;
  JoystickTwistMapping joystick_trajectory_mapping_;

  geometry_msgs::msg::Twist prev_output_;         ///< 上一周期输出，用于斜率限制（仅控制组访问）
  /** prev_output_ 的跨线程只读副本：感知可视化在 background 组读取（方案 7 数据竞争修复） */
  geometry_msgs::msg::Twist viz_prev_output_;
  mutable std::mutex viz_prev_output_mutex_;
  rclcpp::Time last_control_time_ {0, 0, RCL_ROS_TIME};

  /** 前向近障低通刹停闸（仅控制组访问），见 core/front_lowpass_stop.h */
  FrontLowpassStopGate front_lowpass_gate_;
  /** tilt_only 左/右源独立帧替换与轮速外推状态（仅控制组访问） */
  FrontLowpassTiltTracker front_lowpass_tilt_tracker_;
  /** 斜雷达前向近障软限速闸（仅控制组访问），见 core/front_tilt_speed_cap.h */
  FrontTiltSpeedCapGate front_tilt_speed_cap_gate_;

  /** 四向碰撞预警状态机（仅控制组访问），见 core/warning_reporter.h */
  CollisionWarningReporter warning_reporter_;

  // ── 方案 7 看门狗 / 健康摘要状态 ──
  std::atomic < int64_t > last_tick_steady_ns_ {0};   ///< 控制环最近完成时刻（steady ns）
  mutable std::mutex health_mutex_;                   ///< 保护下列健康统计
  std::array < double, 512 > tick_ms_ring_ {};        ///< tick 耗时滑窗（ms）
  size_t tick_ring_index_ = 0;
  size_t tick_ring_count_ = 0;
  uint64_t arbiter_mismatch_count_ = 0;               ///< 方案 1 影子仲裁不一致累计
  uint64_t sudden_shadow_count_ = 0;                  ///< 方案 2.2 影子触发累计
  uint64_t pinned_shadow_count_ = 0;                  ///< 方案 2.3 影子触发累计
  DegradationVerdict last_degradation_;               ///< 方案 4 最近降级裁决
  bool control_loop_degraded_ = false;                ///< 看门狗判定控制环超时

  // ── 方案 6A 自运动积分累加（控制环 50Hz 写入，地图周期 10Hz 消费）──
  mutable std::mutex ego_motion_mutex_;
  double ego_dx_accum_m_ = 0.0;
  double ego_dtheta_accum_rad_ = 0.0;
  // 安全（避障）与巡航（定速保持）正交；标准模式与 scene_node /scene/standard 对齐。
  // 输出权：标准开 或 巡航开 → 跑控制环并发 /nav/control；皆关 → 静默。
  std::atomic < bool > safety_enabled_ {false};   ///< 安全避障是否开启（关闭则透传，不做避障）；启动默认关，再由 user_config 覆盖
  std::atomic < bool > standard_mode_enabled_ {false};  ///< 标准模式（scene_services 默认会开启）
  std::atomic < bool > cruise_enabled_ {false};   ///< 巡航定速保持是否开启
  /** 统一用户配置路径（由 tft_manager 维护）；空字符串表示跳过启动恢复（单测用） */
  std::string user_config_file_path_ {"/opt/freego/user_config.yaml"};
  double cruise_target_speed_ = 0.0;              ///< 巡航保持的目标前向线速度 (m/s)
  double cruise_output_speed_ = 0.0;              ///< 巡航当前实际输出线速度（转向降速后，逐步恢复）(m/s)
  bool cruise_speed_timeout_active_ = false;      ///< 是否处于「进入巡航后等待首个前进速度」的启动超时窗口
  rclcpp::Time cruise_enable_time_ {0, 0, RCL_ROS_TIME};    ///< 进入巡航的时刻，用于启动超时
  bool cruise_forward_speed_reported_ = false;    ///< 本次巡航是否已上报过「检测到前进速度」(status=4)
  bool cruise_obstacle_zero_active_ = false;      ///< 是否正在计时「巡航建立后输出被压到 0」
  rclcpp::Time cruise_obstacle_zero_since_ {0, 0, RCL_ROS_TIME};   ///< 输出被压到 0 的起始时刻
};
