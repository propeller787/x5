#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
// Services (interface)
#include "interface/srv/get_version.hpp"
#include "interface/srv/get_upgrade_content.hpp"
#include "interface/srv/get_package_size.hpp"
#include <cpr/cpr.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <vector>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <optional>
#include <deque>

// 分离后的 CAN 抽象
#include "ota_agent/can_transport.hpp"

// 从 interface 包引入 proto 定义
#include "ota_up.pb.h"
#include "ota_ack.pb.h"
#include "tag_change.pb.h"

// 通用 MQTT/签名模块
#include "mqtt_module/signature_manager.hpp"
#include "mqtt_module/mqtt_connection.hpp"
#include "ota_agent/controller_comm.hpp"
#include "ota_agent/integrity_validator.hpp"
#include "ota_agent/upgrade_info_provider.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

namespace ota_agent {

// 设备标签信息结构体
struct TagValue {
    std::string value_code;
    std::string value_name;
};

struct DeviceTag {
    std::string tag_name;
    std::string tag_code;
    std::vector<TagValue> tag_values;
};

class OtaAgentNode : public rclcpp::Node {
public:
    OtaAgentNode();
    ~OtaAgentNode();

private:
    // 通过 ICanTransport 完成解耦（实现见 ota_agent/can_transport_*）

    // 初始化阶段
    void declare_ros_parameters();
    void load_ros_parameters();
    void initialize_managers();
    bool fetch_device_tags();
    void connect_and_subscribe();

    // MQTT 连接与订阅
    void subscribe_ota_topics();
    bool subscribe_topic(const std::string& topic, int qos = 1);
    void unsubscribe_topics(const std::vector<std::string>& topics);
    void on_mqtt_message(const std::string& topic, const std::string& payload);
    void on_connection_lost(const std::string& cause);
    void on_connected(const std::string& cause);
    // 任务状态变更
    void subscribe_task_status(const std::string& task_id);
    void handle_task_status_change(const std::string& payload);

    // 核心业务逻辑
    void handle_ota_upgrade_msg(const std::string& payload);
    void handle_tag_change_msg(const std::string& payload);
    void handle_can_feedback(const CanFrame& frame);
        // 发布：升级相关通用服务主题（5.1）
        void publish_task_status(int value);
        void publish_upgrade_status(int value);
        void publish_upgrade_message(const std::string& msg);
        void publish_upgrade_progress(double progress);

        // Service handlers (5.2)
        void handle_get_version(const std::shared_ptr<interface::srv::GetVersion::Request> req,
                                std::shared_ptr<interface::srv::GetVersion::Response> res);
        void handle_get_upgrade_content(const std::shared_ptr<interface::srv::GetUpgradeContent::Request> req,
                                        std::shared_ptr<interface::srv::GetUpgradeContent::Response> res);
        void handle_get_package_size(const std::shared_ptr<interface::srv::GetPackageSize::Request> req,
                                     std::shared_ptr<interface::srv::GetPackageSize::Response> res);
    struct OtaStatusEntry {
        std::string package_id;
        std::string ecu;
        std::string ecu_hardware_code;
        std::string current_version;
        int status = 0;           // 0待执行 1下载中 2校验中 3升级中 4回滚中 10升级成功 11升级失败 20回滚成功 21回滚失败
        int error_code = 0;       // 0无 1下载失败 2校验失败 3版本不匹配 4硬件不匹配 5刷写失败 6自检异常 7回滚失败 8设备异常
        std::string sys_error_code;
        std::string message;
    };
    void publish_ota_ack(const std::string& task_id,
                         int task_status,
                         int task_result,
                         int task_error_code,
                         const std::vector<OtaStatusEntry>& packages);
    void maybe_publish_task_summary(const std::string& task_id, bool force = false);
    bool fetch_offline_upgrades(); // 8.5 离线补获
    bool fetch_task_status_http(const std::string& task_id, int& task_status);
    bool process_upgrade_task(const vehicle::ota::up::VehicleOtaUp& up);
    bool download_and_stage(const std::string& url,
                            const std::string& up_id,
                            const vehicle::ota::up::UpgradePackage& pkg,
                            std::string& staged_path,
                            std::string& err);
    bool validate_and_extract_zip(const std::string& zip_path,
                                  const std::string& dest_dir,
                                  const vehicle::ota::up::UpgradePackage& pkg,
                                  std::string& main_file_path,
                                  std::string& err);
    bool run_integrity_pipeline(const IntegrityContext& ctx, IntegrityValidator::ValidationResult& fail);
    void rollback_on_integrity_failure(const std::string& staged_path, const std::string& reason);
    void publish_bin_via_dbc(const std::string& staged_path);
    bool flash_ecu_via_can(const vehicle::ota::up::UpgradePackage& pkg,
                           const std::string& staged_path,
                           std::string& err);
    // 3588 自身升级：调用本地 Bootloader 升级命令
    bool flash_self_via_bootloader(const std::string& staged_path, std::string& err);
    bool reboot_self_and_wait(std::string& err);
    bool verify_self_after_reboot(const std::string& expected_version, std::string& err);
    bool rollback_self_image(std::string& err);
    bool notify_controllers(const std::string& ecu_id, const std::string& version, std::string& err);
    bool send_can(uint32_t can_id, const std::vector<uint8_t>& payload, std::string& err);
    bool send_iso15765_payload(uint32_t can_id, const std::vector<uint8_t>& payload, std::string& err, size_t chunk_payload_size = 7);
    uint32_t compute_crc32_file(const std::string& path, std::string& err);
    // Pull模式：仅发送元数据（URL/MD5/大小/版本），由各控制器自行拉取并刷写
    bool send_pull_metadata_can(uint32_t can_id,
                                const std::string& version,
                                const std::string& url,
                                const std::string& md5,
                                uint64_t size,
                                std::string& err);
    std::string compute_md5_hex(const std::string& path, std::string& err);
    std::string compute_sha256_hex(const std::string& path, std::string& err);
    bool verify_signature(const std::string& payload,
                          const std::string& base64_sig,
                          const std::string& pubkey_path,
                          const std::string& algorithm,
                          std::string& err);
    // 通用：将文件通过 CAN 推送到指定控制器（含版本与大小元数据）
    bool flash_file_via_can(uint32_t target_can_id,
                            const std::string& version,
                            const std::string& staged_path,
                            std::string& err);
    bool send_file_iso_tp(uint32_t can_id,
                          const std::string& staged_path,
                          std::string& err,
                          size_t chunk_payload_size = 7);
    void on_can_frame_rx(const CanFrame& f);
    bool wait_for_can_response(uint32_t can_id, uint8_t expect_sid, int timeout_ms, std::vector<uint8_t>& resp, std::string& err);
    bool is_ack_success(const std::vector<uint8_t>& resp) const;
    std::string hexify(const std::vector<uint8_t>& data) const;
    // exec_time 延迟执行
    rclcpp::TimerBase::SharedPtr exec_timer_;
    std::mutex exec_timer_mutex_;
    std::optional<vehicle::ota::up::VehicleOtaUp> pending_up_;
    // 版本工具与主线版本管理
    int compare_versions(const std::string& a, const std::string& b) const; // return <0 a<b, 0 eq, >0 a>b
    void load_mainline_config();
    void check_boot_version_and_push();
    bool load_stored_domain_version(std::string& out_version);
    void persist_domain_version(const std::string& version);
    void start_boot_broadcast_timer();
    void start_version_broadcast_timer();
    bool validate_package_metadata(const vehicle::ota::up::UpgradePackage& pkg, std::string& reason, int& pkg_error_code) const;
    
    // HTTP 工具
    cpr::Header build_common_headers(const std::string& body_json);
    std::string make_nonce() const;
    std::string make_signature_payload(const std::string& body_json, const std::string& gtime, const std::string& nonce) const;
    std::string build_all_topic() const;
    std::string build_change_topic() const;
    std::string build_tag_topic(const std::string& tag_code, const std::string& value_code) const;
    std::string build_task_status_topic(const std::string& task_id) const { return "vehicle/otatask/" + task_id + "/" + protocol_version_ + "/status"; }

    // ROS 参数
    std::string vin_;
    int model_;
    std::string model_str_;
    std::string api_base_url_;
    std::string session_key_; // 会话密钥
    std::string local_bin_override_path_; // 本地固件路径（非空则跳过下载）
    std::string mqtt_broker_url_;
    std::string mqtt_client_id_;
    std::string mqtt_username_;
    std::string mqtt_password_;
    std::string mqtt_ca_cert_;
    std::string mqtt_cipher_;
    bool mqtt_insecure_skip_verify_ = false;
    int mqtt_retry_attempts_ = 3;
    double mqtt_retry_delay_base_ = 2.0;
    int http_timeout_ms_ = 5000;
    // HTTP头部所需信息（协议8.5）
    std::string protocol_version_ = "1.0";   // Gversion
    std::string http_language_ = "zh-CN";    // Glanguage
    std::string http_client_ = "VehicleGateway/ota_agent"; // GClient
    std::string software_version_ = "1.0.0"; // GApp-Version
    std::string hardware_version_ = "";      // 可选，版本广播用
    std::string domain_version_file_ = "/var/lib/ota_agent/domain_version.json";
    std::string controller_package_path_;
    bool controller_boot_push_enabled_ = false;
    std::unordered_map<std::string, std::string> ecu_hardware_sn_map_;
    std::unordered_map<std::string, std::string> ecu_model_map_;
    bool boot_broadcast_enabled_ = false;
    int boot_broadcast_period_ms_ = 100;
    std::string boot_broadcast_topic_ = "/ota/boot_broadcast";
    std::string boot_broadcast_payload_ = "request_upgrade";
    bool version_broadcast_enabled_ = false;
    int version_broadcast_period_ms_ = 1000;
    std::string version_broadcast_topic_ = "/ota/version_broadcast";
    bool can_use_socketcan_ = false;
    std::string can_ifname_ = "can0";
    int can_send_retries_ = 3;
    int can_retry_delay_ms_ = 50;
    // 可配置的CAN ID（域控/上控/下控）
    int can_id_domain_ = 0x3588;
    int can_id_upper_  = 0x0510;
    int can_id_lower_  = 0x0520;
    // Pull刷写模式：true=通过CAN下发URL/MD5/大小，控制器自行拉取并刷写；false=通过CAN推送镜像
    bool pull_flash_mode_ = false;
    // Hybrid：仅上/下控Pull，自身3588仍Push
    bool pull_to_controllers_only_ = false;
    // Pull模式等待超时（毫秒），超时未收到 S/F 则上报失败
    int pull_timeout_ms_ = 600000; // 默认10分钟
    std::atomic<bool> pull_waiting_{false};
    // Pull模式超时（秒）
    int pull_timeout_sec_ = 600;
    rclcpp::TimerBase::SharedPtr pull_watchdog_timer_;
    std::atomic<bool> pull_watchdog_active_{false};
    std::chrono::steady_clock::time_point pull_deadline_{};
    void pull_watchdog_tick();
        // 自身刷写命令（可配置），例如："/usr/bin/ota_flash.sh {path}"
        std::string self_flash_cmd_template_ = "";
        std::string self_reboot_cmd_template_ = "";
        std::string self_verify_cmd_template_ = "";
        std::string self_rollback_image_path_ = "";
    // Mock CAN 行为配置
    std::string mock_can_outcome_ = "success"; // success | failure | timeout
    int mock_can_fail_after_step_ = 2; // 触发失败前的进度步数
    int mock_can_step_delay_ms_ = 100; // 进度步之间延迟
    size_t iso_tp_chunk_size_ = 7; // 每帧有效载荷（单帧<=7，多帧连续帧<=7）
    struct PendingCanResp {
        uint32_t can_id;
        uint8_t expect_sid;
        std::vector<uint8_t> data;
        bool fulfilled = false;
    };
    std::mutex can_resp_mutex_;
    std::condition_variable can_resp_cv_;
    std::vector<PendingCanResp> pending_can_resps_;

    // DBC 发布（复用 ota_dbc_demo_node 的行为）
    bool dbc_publish_enabled_ = false;
    int dbc_audio_type_ = 1;
    int dbc_publish_count_ = 1;
    int dbc_publish_interval_ms_ = 200;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dbc_audio_pub_;
    // HTTP 兜底定时器
    rclcpp::TimerBase::SharedPtr http_poll_timer_;

    // 管理器
    std::shared_ptr<mqtt_module::SignatureManager> signature_mgr_;

    // 5.1 本地服务主题发布器（与 MQTT/云端完全解耦）
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr task_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr upgrade_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr upgrade_message_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr upgrade_progress_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr boot_broadcast_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr version_broadcast_pub_;
    rclcpp::TimerBase::SharedPtr boot_broadcast_timer_;
    rclcpp::TimerBase::SharedPtr version_broadcast_timer_;

    // 可插拔数据源接口（云端/本地），用于服务实现的解耦
    std::shared_ptr<ota_agent::UpgradeInfoProvider> info_provider_;
    std::shared_ptr<mqtt_module::MqttConnectionManager> mqtt_manager_;
    IntegrityValidator integrity_validator_;

    // Service servers
    rclcpp::Service<interface::srv::GetVersion>::SharedPtr get_version_srv_;
    rclcpp::Service<interface::srv::GetUpgradeContent>::SharedPtr get_upgrade_content_srv_;
    rclcpp::Service<interface::srv::GetPackageSize>::SharedPtr get_package_size_srv_;

    // 状态
    std::vector<DeviceTag> device_tags_;
    std::vector<std::string> subscribed_tag_topics_;
    double protocol_version_d_ = 1.0;
    std::string ack_topic_ = "vehicle/ota/up/1.0/ack";
    bool tags_ready_ = false;
    mutable std::mutex tag_mutex_;

    // CAN 传输实现（当前默认 mock，可后续替换为 socketcan 实现）
    std::shared_ptr<ICanTransport> can_transport_;
    std::shared_ptr<IControllerComm> ctrl_comm_;
    // 升级并发控制
    std::mutex upgrade_mutex_;
    bool upgrade_in_progress_ = false;
    // 当前升级上下文（串行刷写）
    std::string current_up_id_;
    std::string current_task_id_;
    std::atomic<int> current_task_status_{0}; // 0正常 1暂停 2取消
    std::string current_ecu_id_;
    std::string current_version_;
    std::string current_package_id_;
    std::atomic<bool> pause_requested_{false};
    std::atomic<bool> cancel_requested_{false};
    std::condition_variable pause_cv_;
    std::mutex pause_mutex_;
    // 记录最近一次下载的本地固件路径（用于清理）
    std::string last_staged_path_;
    void prune_old_staged_bins(const std::string& keep_path);
    // 是否在 CAN 发送失败时自动上报失败ACK
    bool report_can_tx_fail_ack_ = true;

    // 完整性参数
    void setup_integrity_callbacks();
    std::string integrity_sha256_json_;
    std::string integrity_signature_map_json_;
    std::string integrity_signature_pubkey_path_;
    std::string integrity_signature_algorithm_ = "RSA";
    std::string integrity_signature_payload_template_ = "{version}|{url}|{md5}|{sha256}";
    std::string integrity_expected_versions_json_;
    std::vector<std::string> integrity_l3_required_nodes_;
    std::vector<std::string> integrity_l4_required_nodes_;
    std::unordered_map<std::string, std::string> sha256_expectations_;
    std::unordered_map<std::string, std::string> signature_expectations_;
    std::unordered_map<std::string, std::string> version_expectations_;
    // 主线版本（车型级）与可选的 ECU 主线版本
    std::string mainline_version_;
    std::string mainline_config_json_;
    std::unordered_map<std::string, std::string> mainline_ecu_versions_;

        // ... 原有成员变量 ...
    bool simulate_can_send_;
    std::string simulate_bin_path_;
    int simulate_target_can_id_;

    struct PendingAck {
        std::string task_id;
        std::string payload;
        std::string label;
        int attempts = 0;
    };
    struct TaskAckState {
        std::string task_id;
        size_t expected_total = 0;
        std::unordered_map<std::string, OtaStatusEntry> latest;
        bool summary_sent = false;
    };
    TaskAckState task_ack_state_;
    std::deque<PendingAck> ack_queue_;
    std::mutex ack_mutex_;
    rclcpp::TimerBase::SharedPtr ack_retry_timer_;
    int ack_retry_interval_ms_ = 2000;
    int ack_retry_max_attempts_ = 5;

    void reset_task_ack_state(const std::string& task_id, size_t expected_total);
    void update_task_ack_state(const std::string& task_id, const std::vector<OtaStatusEntry>& packages);
    void send_ack_payload(const vehicle::ota::up::VehicleOtaAck& ack, const std::string& label);
    void drain_ack_queue();
    bool is_final_status(int status) const;
    bool is_failure_status(int status) const;
    bool wait_if_paused_or_cancelled(const std::string& phase);
    bool handle_cancel_and_cleanup(const vehicle::ota::up::VehicleOtaUp& up, const std::string& staged_path, const std::string& reason);
    void remove_staged_file(const std::string& path);

    // 新增：模拟 CAN 发送函数
    void simulate_can_file_send();
    
};

} // namespace ota_agent
