#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
// topic
#include "interface/msg/car_info.hpp"
#include "interface/msg/ota_package_info.hpp"
// Services
#include "interface/srv/package_download.hpp"
#include "interface/srv/package_upgrade.hpp"
#include "interface/srv/package_check.hpp"
#include "interface/srv/id_controls.hpp"
#include "interface/srv/get_token.hpp"
#include "interface/srv/system_version.hpp"
// protobuf
#include "vehicle_ota_up.pb.h"
#include "vehicle_ota_ack.pb.h"
#include "tag_change.pb.h"
// mqtt module
#include "mqtt_module/signature_manager.hpp"
#include "mqtt_module/mqtt_connection.hpp"
#include "mqtt_module/mqtt_client.hpp"
// ota_agent
#include "ota_agent/can_transport.hpp"
#include "ota_agent/oss_manager.hpp"
// cpr
#include <cpr/cpr.h>

#include <chrono>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <functional>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <linux/can.h>
#include <linux/can/raw.h>


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
struct FileInfo {
    uint32_t start_address = 0;
    uint32_t data_length = 0;
    bool is_hex_file = false;
    std::vector<uint8_t> hex_binary_data;
};

class OtaAgentNode : public rclcpp::Node {
public:
    OtaAgentNode();
    ~OtaAgentNode();

private:
    // init
    void declare_ros_parameters();
    void load_ros_parameters();
    void initialize_managers();
    
    // mqtt
    void connect_and_subscribe();
    void subscribe_ota_topics();
    bool subscribe_topic(const std::string& topic, int qos = 1);
    void unsubscribe_topics(const std::vector<std::string>& topics);
    void on_mqtt_message(const std::string& topic, const std::string& payload, const std::vector<std::pair<std::string, std::string>>& properties);
    void on_connection_lost(const std::string& cause);
    void on_connected(const std::string& cause);
    // 任务状态变更
    void subscribe_task_status(const std::string& task_id);
    void handle_task_status_change(const std::string& payload);

    // 核心业务逻辑
    void handle_ota_upgrade_msg(const std::string& payload);
    bool publish_recv_ota_upgrade_msg();

    // 发布：升级相关通用服务主题（5.1）
    void publish_package_info(const std::string& new_version, const std::string& new_description, int32_t new_size);
    void publish_download_message(const std::string& msg);
    void publish_upgrade_message(const std::string& msg);
    void publish_download_progress(double progress);
    void publish_upgrade_progress(double progress);
    void publish_system_upgrade_prepare(const uint32_t& ret);

    // Service handlers (5.2)
    void handle_package_download(const std::shared_ptr<interface::srv::PackageDownload::Request> req,
                            std::shared_ptr<interface::srv::PackageDownload::Response> res);
    void handle_package_upgrade(const std::shared_ptr<interface::srv::PackageUpgrade::Request> req,
                            std::shared_ptr<interface::srv::PackageUpgrade::Response> res);
    void handle_package_check(const std::shared_ptr<interface::srv::PackageCheck::Request> req,
                            std::shared_ptr<interface::srv::PackageCheck::Response> res);
    
    // srv 请求获取
    bool request_sn_via_service(uint32_t id, std::string& sn, std::string& err);
    bool request_auth_info_via_service(std::string& jwt_token, std::string& session_public_key,std::string& session_private_key, std::string& session_no_key, std::string& err);
    bool request_auth_info_via_service_async(std::string& jwt_token, std::string& session_public_key,std::string& session_private_key, std::string& session_no_key, std::string& err);
    bool request_uc_version_via_service_async(const std::string& version, std::string& err);
    bool request_cu_version_via_service_async(const std::string& version, std::string& err);
    // ros2 topic handlers
    void on_car_info_received(const interface::msg::CarInfo::SharedPtr msg);

    // safe
    bool check_critical_nodes_alive(std::string& err_msg);
    std::vector<std::string> monitored_nodes_;
    
    bool read_hex_get_version(const std::string& hex_file_path, std::string& version, std::string& err);

    std::string tmp_version_;
    

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
    bool fetch_task_status_http(const std::string& task_id, int& task_status);
    bool process_download_task(const vehicle::ota::up::VehicleOtaUp& up, std::string& msg);
    bool download_and_stage(const std::string& url,
                            const std::string& up_id,
                            const vehicle::ota::up::UpgradePackage& pkg,
                            std::string& staged_path,
                            std::string& err);

    bool validate_and_extract_zip(const std::string& zip_path, const std::string& dest_dir, std::string& err);

    bool prepare_upgrade_task(std::string& err);
    bool process_upgrade_task(const vehicle::ota::up::VehicleOtaUp& up, std::string& msg);
    bool process_check_task(std::string& msg);

    static int hex_char_to_int(char c) {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    }
    static int hex_str_to_byte(const std::string& hex_str, size_t pos) {
        if (pos + 1 >= hex_str.size()) return -1;
        int high = hex_char_to_int(hex_str[pos]);
        int low = hex_char_to_int(hex_str[pos + 1]);
        if (high < 0 || low < 0) return -1;
        return (high << 4) | low;
    }
    static std::array<uint8_t, 4> u32_to_bytes(uint32_t v) {
        return {{
            static_cast<uint8_t>((v >> 24) & 0xFF),
            static_cast<uint8_t>((v >> 16) & 0xFF),
            static_cast<uint8_t>((v >> 8) & 0xFF),
            static_cast<uint8_t>(v & 0xFF)
        }};
    }
    uint32_t extract_can_id(uint32_t can_id_with_flags);
    uint32_t compute_crc32(uint32_t crc32val, const uint8_t* data, size_t len);
    /**
     * @brief 域控更新
     * @param filename 
     * @return true / false 
     */
    bool falsh_domain_complete(const std::string& filename, const std::string& mainline_version, std::string& err);
    /**
     * @brief 上控/下控刷写
     * 
     * @param filename 
     * @return true / false
     */
    bool flash_firmware_complete(uint32_t driver_rx_id, uint32_t driver_tx_id,const std::string& filename, std::string& err);
    /**
     * @brief 上控/下控刷写完成 发送刷写完成回执
     * 
     * @param driver_uc_id, driver_db_id
     * @return true / false
     */
    bool flash_firmware_success(uint32_t driver_id, uint32_t can_id, std::string& err);
    /**
     * @brief 上控/下控刷写复位原先版本
     * 
     * @param driver_uc_id, driver_db_id
     * @return true / false
     */
    bool flash_firmware_back(uint32_t driver_id, uint32_t can_id, std::string& err);
    /**
     *  @brief 获取上控/下控 版本信息
     * 
     *  @param driver_uc_id, driver_db_id 
     *  @return true / false
     */
    bool flash_firmware_version(uint32_t driver_id, uint32_t can_id, std::string& err);
    bool prepare_file_info(FileInfo& info, const std::string& file);
    bool parse_hex_file(const std::string& hex_path, uint32_t& start_address, uint32_t& data_length, std::string& err);
    bool hex_to_binary(const std::string& hex_path, std::vector<uint8_t>& binary_data, std::string& err);
    bool wait_ack(uint32_t driver_id, uint8_t expect_sid, const char* step);
    void handle_can_rx(uint32_t can_id, const std::vector<uint8_t>& data);
    bool send_file_iso_tp(uint32_t can_id, uint32_t driver_tx_id,const std::string& file_path, size_t chunk_payload_size, 
                          bool is_hex_file, const std::vector<uint8_t>& hex_binary_data , uint32_t& crc32_val,std::string& err);
    bool wait_for_can_response(uint32_t can_id, uint8_t expect_sid, int timeout_ms, std::vector<uint8_t>& resp, std::string& err, int expect_byte_at_pos2 = -1,
        std::function<bool(const std::function<void(uint32_t, const std::vector<uint8_t>&)>&)> send_after_callback_set = nullptr,
        const std::function<void(uint32_t, const std::vector<uint8_t>&)>* restore_cb_override = nullptr);
    bool is_ack_success(const std::vector<uint8_t>& resp, uint8_t expect_sid) const;
    std::string hexify(const std::vector<uint8_t>& data) const;
    bool send_can(uint32_t can_id, const std::vector<uint8_t>& payload, std::string& err);
    bool send_iso15765_payload(uint32_t can_id, uint32_t driver_tx_id,const std::vector<uint8_t>& payload, std::string& err, 
        size_t chunk_payload_size = 7, std::function<void(uint32_t, const std::vector<uint8_t>&)> rx_cb_to_restore = nullptr);

    std::atomic<bool> upgrade_success_ = false;
    std::atomic<bool> upgrade_failed_= false;
    
    std::atomic<bool> is_downloading_{false};
    std::atomic<bool> is_upgrading_{false};

    std::string extract_version(const std::string &version_str);
    int compare_versions(const std::string &a, const std::string &b);
    bool retain_top_versions(const std::string &target_dir);
    
    std::string compute_sha256_hex(const std::string& path, std::string& err);
    bool verify_signature(const std::string& payload,
                          const std::string& base64_sig,
                          const std::string& pubkey_path,
                          const std::string& algorithm,
                          std::string& err);

    void load_mainline_config();
    bool persist_mainline_config();
    void start_boot_broadcast_timer();

    std::string ota_tmp_file_path_ = "/home/zq/tmp/ota_agent";
    std::string ota_real_file_path_ = "/opt/freego/domain";
    std::string ota_link_file_path_ = "/home/zq/freego";
    
    // 任务ID存储管理
    bool load_stored_task_id(std::string& out_task_id);
    void persist_task_id(const std::string& task_id);
    bool persist_up_task(const vehicle::ota::up::VehicleOtaUp& up, const std::string& file);
    bool load_stored_up_task(vehicle::ota::up::VehicleOtaUp& up, const std::string& file);
    const std::string current_ota_task_file_ = ota_tmp_file_path_ + "/current_up_task";

    std::mutex cached_up_mutex_;
    std::optional<vehicle::ota::up::VehicleOtaUp> cached_up_task_;
    // HTTP 
    cpr::Header build_headers(const mqtt_module::DeviceInfo& info, const std::string& body_json, bool use_session_key);
    std::string make_signature_payload(const std::string& body_json, const std::string& gtime, const std::string& nonce) const;
    std::string build_all_topic() const;
    std::string build_change_topic() const;
    std::string build_tag_topic(const std::string& tag_code, const std::string& value_code) const;
    std::string build_task_status_topic(const std::string& task_id) const { return "vehicle/ota/task/" + task_id + "/" + protocol_version_ + "/status"; }
    std::string build_ecu_topic() const { 
        std::string model_for_topic = model_str_;
        model_for_topic.erase(std::remove(model_for_topic.begin(), model_for_topic.end(), ' '), model_for_topic.end());
        return "vehicle/ecu/" + model_for_topic + "/" + vin_ + "/" + protocol_version_ + "/up"; 
    }

    // ROS 参数
    std::string vin_;
    std::string model_;
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
    mqtt_module::TokenResult token_result_;
    std::string protocol_version_ = "1";   // Gversion
    std::string http_language_ = "zh-CN";    // Glanguage
    std::string http_client_ = "VehicleGateway/ota_agent"; // GClient
    std::string software_version_ = "1.0.0"; // GApp-Version
    std::string task_id_file_ = ota_tmp_file_path_ + "/current_task_id.json";
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
    bool can_use_socketcan_ = true;
    std::string can_ifname_ = "can2";
    int can_send_retries_ = 3;
    int can_retry_delay_ms_ = 50;
    // 可配置的CAN ID（域控/上控/下控）
    uint32_t can_id_upper_send_  = 0x70A | CAN_EFF_FLAG;
    uint32_t can_id_upper_recv_  = 0x702 | CAN_EFF_FLAG;
    uint32_t can_id_lower_send_  = 0x709 | CAN_EFF_FLAG;
    uint32_t can_id_lower_recv_  = 0x701 | CAN_EFF_FLAG;
    // Pull刷写模式：true=通过CAN下发URL/MD5/大小，控制器自行拉取并刷写；false=通过CAN推送镜像
    bool pull_flash_mode_ = false;
    // Hybrid：仅上/下控Pull，自身3588仍Push
    bool pull_to_controllers_only_ = false;
    // Pull模式等待超时（毫秒），超时未收到 S/F 则上报失败
    int pull_timeout_ms_ = 600000; // 默认10分钟
    std::atomic<bool> pull_waiting_{false};
    // Pull模式超时（秒）
    int pull_timeout_sec_ = 600;

    // HTTP 兜底定时器
    rclcpp::TimerBase::SharedPtr http_poll_timer_;
    // 管理器
    std::shared_ptr<mqtt_module::SignatureManager> signature_mgr_;
    std::shared_ptr<mqtt_module::MqttClient> mqtt_client_;
    mqtt_module::DeviceInfo device_info_;
    std::shared_ptr<OSSManager> oss_mgr_;

    // 5.1 本地服务主题发布器（与 MQTT/云端完全解耦）
    rclcpp::Publisher<interface::msg::OtaPackageInfo>::SharedPtr package_info_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr download_message_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr upgrade_message_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr download_progress_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr upgrade_progress_pub_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr system_upgrade_prepare_pub_;

    // Service servers
    rclcpp::Service<interface::srv::PackageDownload>::SharedPtr package_download_srv_;
    rclcpp::Service<interface::srv::PackageUpgrade>::SharedPtr package_upgrade_srv_;
    rclcpp::Service<interface::srv::PackageCheck>::SharedPtr package_check_srv_;

    // 车辆信息
    rclcpp::Subscription<interface::msg::CarInfo>::SharedPtr car_info_sub_;
    interface::msg::CarInfo latest_car_info_;
    std::mutex car_info_mutex_;

    // 状态
    std::vector<DeviceTag> device_tags_;
    std::vector<std::string> subscribed_tag_topics_;
    std::set<std::string> subscribed_topics_; // 跟踪所有订阅的主题，便于调试
    double protocol_version_d_ = 1.0;
    std::string ack_topic_ = "vehicle/ota/up/1.0/ack";
    bool tags_ready_ = false;
    mutable std::mutex tag_mutex_;

    // CAN 传输实现 socketcan
    std::shared_ptr<ICanTransport> can_transport_;
    // 升级并发控制
    std::mutex upgrade_mutex_;
    bool upgrade_in_progress_ = false;
    // 当前升级上下文（串行刷写）
    std::string current_up_id_;
    std::string current_task_id_;
    std::string sub_task_id_;
    std::atomic<int> current_task_status_{0}; // 0正常 1暂停 2取消
    std::string current_ecu_id_;
    std::string current_version_;
    std::string current_package_id_;
    std::atomic<bool> pause_requested_{false};
    std::atomic<bool> cancel_requested_{false};
    std::condition_variable pause_cv_;
    std::mutex pause_mutex_;

    // 是否在 CAN 发送失败时自动上报失败ACK
    bool report_can_tx_fail_ack_ = true;

    // 完整性参数
    std::string integrity_sha256_json_;
    std::string integrity_signature_map_json_;
    std::string integrity_signature_pubkey_path_;
    std::string integrity_signature_algorithm_ = "ed25519";
    std::string integrity_signature_payload_template_ = "{version}|{url}|{md5}|{sha256}";
    std::string integrity_expected_versions_json_;
    std::vector<std::string> integrity_l3_required_nodes_;
    std::vector<std::string> integrity_l4_required_nodes_;
    std::unordered_map<std::string, std::string> sha256_expectations_;
    std::unordered_map<std::string, std::string> signature_expectations_;
    std::unordered_map<std::string, std::string> version_expectations_;
    // 主线版本
    std::string mainline_version_;// 主版本
    std::string dc_version_;    // 域控版本
    std::string uc_version_;    // 上控版本
    std::string mcu_version_;    // 下控版本

    std::string mainline_config_json_path_;

    std::string oss_bucket_name_;

    struct PendingAck {
        std::string task_id;
        std::string payload;
        std::string label;
        int attempts = 0;
        std::string signature;
        std::string gkey_no;
        std::string gtime;
        std::string gnonce;
    };
    
    // 生成base36 nonce
    std::string generate_base36_nonce();
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

    /* ack */
    void reset_task_ack_state(const std::string& task_id, size_t expected_total);
    void update_task_ack_state(const std::string& task_id, const std::vector<OtaStatusEntry>& packages);
    void send_ack_payload(const vehicle::ota::ack::VehicleOtaAck& ack, const std::string& label);
    void drain_ack_queue();
    bool is_final_status(int status) const;
    bool is_failure_status(int status) const;
    bool wait_if_paused_or_cancelled(const std::string& phase);
};

} // namespace ota_agent
