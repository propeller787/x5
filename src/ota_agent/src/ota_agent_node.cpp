#include "ota_agent_node.hpp"
#include "ota_agent/can_transport_socketcan.hpp"
#include "ota_agent/can_transport_mock.hpp"
#include "utils/common.h"
#include "utils/topic.h"
#include "vehicle_ota_status.pb.h"
#include <openssl/evp.h>  
#include <openssl/pem.h>
#include <mosquitto.h>

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <random>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <cstdlib>
#include <regex>

#include <linux/can/error.h>
namespace ota_agent {
OtaAgentNode::OtaAgentNode() : rclcpp::Node("ota_agent_node") {
    declare_ros_parameters();
    load_ros_parameters();
    std::string model_for_topic = model_str_;   // ack topic: vehicle/ota/<model>/<sn>/<protocol>/ack
    model_for_topic.erase(std::remove(model_for_topic.begin(), model_for_topic.end(), ' '), model_for_topic.end());
    ack_topic_ = "vehicle/ota/" + model_for_topic + "/" + vin_ + "/" + protocol_version_ + "/ack";
    initialize_managers();

    /*  topic */
    package_info_pub_ = this->create_publisher<interface::msg::OtaPackageInfo>(otaPackageInfoTopic, rclcpp::QoS(3));
    download_message_pub_ = this->create_publisher<std_msgs::msg::String>(otaDownloadMessageTopic, rclcpp::QoS(3));
    upgrade_message_pub_ = this->create_publisher<std_msgs::msg::String>(otaUpgradeMessageTopic, rclcpp::QoS(3));
    download_progress_pub_ = this->create_publisher<std_msgs::msg::Float64>(otaDownloadProgressTopic, rclcpp::QoS(3));
    upgrade_progress_pub_ = this->create_publisher<std_msgs::msg::Float64>(otaUpgradeProgressTopic, rclcpp::QoS(3));
    system_upgrade_prepare_pub_ = this->create_publisher<std_msgs::msg::Int32>(otaSystemUpgradePrepare, rclcpp::QoS(3));
    /*  service */
    package_download_srv_ = this->create_service<interface::srv::PackageDownload>(
        UpgradePackageDownloadService,
        std::bind(&OtaAgentNode::handle_package_download, this, std::placeholders::_1, std::placeholders::_2)
    );
    package_upgrade_srv_ = this->create_service<interface::srv::PackageUpgrade>(
        UpgradePackageUpgradeService,
        std::bind(&OtaAgentNode::handle_package_upgrade, this, std::placeholders::_1, std::placeholders::_2)
    );
    package_check_srv_ = this->create_service<interface::srv::PackageCheck>(
        UpgradePackageCheckService,
        std::bind(&OtaAgentNode::handle_package_check, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    car_info_sub_ = this->create_subscription<interface::msg::CarInfo>(carInfoTopic , 10,
        [this](const interface::msg::CarInfo::SharedPtr msg){
            this->on_car_info_received(msg);
        }
    );

    ack_retry_timer_ = this->create_wall_timer(std::chrono::milliseconds(std::max(100, ack_retry_interval_ms_)), [this]{ this->drain_ack_queue(); });

    start_boot_broadcast_timer();
    connect_and_subscribe();
    {
        std::string msg;
        if(!process_check_task(msg)){
            RCLCPP_WARN(this->get_logger(), "获取版本信息错误：%s", msg.c_str());
        }
        else{
            publish_recv_ota_upgrade_msg();
        }
        std::mt19937 rng{std::random_device{}()}; 
        std::uniform_int_distribution<int> jitter_sec(0, 600); // 0~10min 随机抖动
        const auto period = 2h + std::chrono::seconds(jitter_sec(rng));
        RCLCPP_INFO(this->get_logger(), "HTTP 兜底轮询周期：%ld 秒", std::chrono::duration_cast<std::chrono::seconds>(period).count());
        http_poll_timer_ = this->create_wall_timer(period, [this]{
            std::string msg;
            if(!process_check_task(msg)){
                RCLCPP_WARN(this->get_logger(), "HTTP 兜底轮询失败：%s", msg.c_str());
            }
            else{
                RCLCPP_INFO(this->get_logger(), "HTTP 兜底轮询成功");// 发布待更新通知
                publish_recv_ota_upgrade_msg();
            }
        });
    }
    if (can_use_socketcan_) {
        can_transport_ = std::make_shared<SocketCanTransport>(can_ifname_, this->get_logger());
        RCLCPP_INFO(this->get_logger(), "can socket init success");
    } 
}

OtaAgentNode::~OtaAgentNode() {
    if (can_transport_) {
        RCLCPP_INFO(this->get_logger(), "Shutting down CAN transport");
        can_transport_->close();
    }
}

cpr::Header OtaAgentNode::build_headers(const mqtt_module::DeviceInfo& info, const std::string& body_json, bool use_session_key) {
    cpr::Header headers;
    headers["content-type"] = "application/json; charset=UTF-8";
    // 关键：添加gkey-no头，用于身份认证
    std::string gkey_no = use_session_key && !info.session_key_no.empty() ? info.session_key_no : info.gkey_no;
    headers["gkey-no"] = gkey_no;
    headers["gversion"] = info.version;
    headers["glanguage"] = info.language;
    headers["gclient"] = info.client;
    headers["gapp-version"] = info.app_version;
    auto now_ms = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    headers["gtime"] = now_ms;

    std::uniform_int_distribution<uint64_t> dis;
    static thread_local std::mt19937_64 gen(std::random_device{}());
    uint64_t a = dis(gen), b = dis(gen);
    std::ostringstream ss;
    ss << std::hex << std::setw(16) << std::setfill('0') << a
       << std::hex << std::setw(16) << std::setfill('0') << b;
    std::string nonce = ss.str();
    headers["gnonce"] = nonce;
    std::string sign_source = headers["gkey-no"] + headers["gversion"] + headers["gclient"] +
                               headers["gapp-version"] + headers["gtime"] + headers["gnonce"] + body_json;
    
    headers["gsign"] = signature_mgr_->sign_data(sign_source);
    RCLCPP_DEBUG(this->get_logger(), "签名源: %s", sign_source.c_str());
    RCLCPP_DEBUG(this->get_logger(), "签名结果: %s", headers["gsign"].c_str());
    return headers;
}

std::string OtaAgentNode::make_signature_payload(const std::string& body_json, const std::string& gtime, const std::string& nonce) const {
    return vin_ + "|" + model_str_ + "|" + gtime + "|" + nonce + "|" + body_json;
}


std::string OtaAgentNode::build_all_topic() const {
    // vehicle/ota/<model>/<sn>/<protocol>/up
    std::string model_for_topic = model_str_;
    model_for_topic.erase(std::remove(model_for_topic.begin(), model_for_topic.end(), ' '), model_for_topic.end());
    return "vehicle/ota/" + model_for_topic + "/" + vin_ + "/" + protocol_version_ + "/up";
}
std::string OtaAgentNode::build_change_topic() const {
    std::string model_for_topic = model_str_;
    model_for_topic.erase(std::remove(model_for_topic.begin(), model_for_topic.end(), ' '), model_for_topic.end());
    return "vehicle/ota/change/" + model_for_topic + "/" + vin_ + "/" + protocol_version_ + "/notic";
}
std::string OtaAgentNode::build_tag_topic(const std::string& tag_code, const std::string& value_code) const {
    // Spec: vehicle/ota/tag/<设备型号>/<标签编号>/<标签值编号>/<协议版本>/up
    std::string model_for_topic = model_str_;
    model_for_topic.erase(std::remove(model_for_topic.begin(), model_for_topic.end(), ' '), model_for_topic.end());
    return "vehicle/ota/tag/" + model_for_topic + "/" + tag_code + "/" + value_code + "/" + protocol_version_ + "/up";
}

void OtaAgentNode::on_connection_lost(const std::string& cause) {
    RCLCPP_WARN(this->get_logger(), "MQTT连接丢失: %s", cause.c_str());
}
void OtaAgentNode::on_connected(const std::string& cause) {
    RCLCPP_INFO(this->get_logger(), "MQTT已连接: %s", cause.c_str());
    drain_ack_queue(); // This line is unchanged, but it is included for context.
}


void OtaAgentNode::handle_ota_upgrade_msg(const std::string& payload) {
    vehicle::ota::up::VehicleOtaUp up;
    std::string parsed_task_id;
    if (!up.ParseFromString(payload)) {
        RCLCPP_WARN(this->get_logger(), "ProtoBuf解析失败，尝试按JSON兼容路径处理");
        auto parsed = nlohmann::json::parse(payload, nullptr, false);
        if (parsed.is_discarded()) {
            RCLCPP_ERROR(this->get_logger(), "OTA消息既非ProtoBuf也非有效JSON，忽略");
            return;
        }
        nlohmann::json root = parsed;
        if (root.contains("data")) root = root["data"];
        if (root.contains("up")) root = root["up"];
        if (root.contains("payload")) root = root["payload"];

        auto get_str = [&](const char* k)->std::string {
            if (root.contains(k)) {
                if (root[k].is_string()) return root[k].get<std::string>();
                if (root[k].is_number_integer()) return std::to_string(root[k].get<long long>());
                if (root[k].is_number()) return std::to_string(root[k].get<double>());
            }
            return std::string("");
        };
        auto get_i = [&](const char* k, int def)->int {
            if (root.contains(k)) {
                if (root[k].is_number_integer()) return root[k].get<int>();
                if (root[k].is_string()) {
                    try { return std::stoi(root[k].get<std::string>()); } catch (...) {}
                }
            }
            return def;
        };
        auto get_ll = [&](const char* k, long def)->long {
            if (root.contains(k)) {
                if (root[k].is_number_integer()) return root[k].get<long>();
                if (root[k].is_string()) {
                    try { return std::stol(root[k].get<std::string>()); } catch (...) {}
                }
            }
            return def;
        };
        /* json 转换为 protobuf */
        up.set_task_id(get_str("task_id"));
        up.set_task_status(get_i("task_status", 0));
        up.set_timestamp(get_ll("timestamp", 0));
        up.set_exec_time(get_ll("exec_time", 0));
        up.set_sub_task_id(get_str("sub_task_id"));
        parsed_task_id = up.task_id();
        if (root.contains("strategy") && root["strategy"].is_object()) {
            const auto& s = root["strategy"];
            auto* st = up.mutable_strategy();
            st->set_id(s.value("id", std::string("")));
            st->set_create_time(s.value("create_time", static_cast<int64_t>(0)));
            st->set_up_type(s.value("up_type", 0));
            if (s.contains("preconditions") && s["preconditions"].is_object()) {
                const auto& p = s["preconditions"];
                auto* pre = st->mutable_preconditions();
                pre->set_min_battery_percent(p.value("min_battery_percent", 0));
                pre->set_parking_status(p.value("parking_status", 0));
                pre->set_max_speed(p.value("max_speed", 0));
                pre->set_charging_status(p.value("charging_status", 0));
            }
            st->set_download_retry_count(s.value("download_retry_count", 0));
            st->set_flash_retry_count(s.value("flash_retry_count", 0));
            st->set_up_mode(s.value("up_mode", 0));
            if (s.contains("upgrade_groups") && s["upgrade_groups"].is_array()) {
                for (const auto& g : s["upgrade_groups"]) {
                    if (g.is_string()) st->add_upgrade_groups(g.get<std::string>());
                }
            }
            if (s.contains("rollback_groups") && s["rollback_groups"].is_array()) {
                for (const auto& g : s["rollback_groups"]) {
                    if (g.is_string()) st->add_rollback_groups(g.get<std::string>());
                }
            }
            st->set_desc(s.value("desc", std::string("")));
        }

        nlohmann::json pkg_arr;
        if (root.contains("packages") && root["packages"].is_array()) {
            pkg_arr = root["packages"];
        }
        if (pkg_arr.is_array()) {
            for (const auto& e : pkg_arr) {
                if (!e.is_object()) continue;
                auto* m = up.mutable_strategy()->add_packages();
                m->set_package_id(e.value("package_id", std::string("")));
                m->set_ecu(e.value("ecu", std::string("")));
                m->set_ecu_hardware_code(e.value("ecu_hardware_code", std::string("")));
                m->set_target_version(e.value("target_version", std::string("")));
                m->set_from_version(e.value("from_version", std::string("")));
                m->set_from_max_version(e.value("from_max_version", std::string("")));
                m->set_download_url(e.value("download_url", std::string("")));
                m->set_sha256(e.value("sha256", std::string("")));
                m->set_size(e.value("size", static_cast<int64_t>(0)));
            }
        }
        RCLCPP_INFO(this->get_logger(), "JSON兼容路径生效：映射完成，包数=%d", up.strategy().packages_size());
    }
    std::string task_id_to_use = up.task_id();
    if (task_id_to_use.empty()) {
        task_id_to_use = "01KEHFA68ZQ7M2HWDBNCPAQWA7";
        RCLCPP_WARN(this->get_logger(), "⚠️ 未找到task_id，使用默认值: %s", task_id_to_use.c_str());
    }
    current_task_id_ = task_id_to_use;
    sub_task_id_ = up.sub_task_id();
    
    // 保存task_id到本地文件
    persist_task_id(task_id_to_use);
    subscribe_task_status(task_id_to_use);
    int task_status = 0;
    if (fetch_task_status_http(task_id_to_use, task_status)) {
        current_task_status_.store(task_status);
        if (task_status == 1) {
            RCLCPP_WARN(this->get_logger(), "任务被暂停，暂不执行升级，task_id=%s", task_id_to_use.c_str());
            return;
        } else if (task_status == 2) {
            RCLCPP_WARN(this->get_logger(), "任务被取消，跳过执行，task_id=%s", task_id_to_use.c_str());
            return;
        }
    }
    RCLCPP_INFO(this->get_logger(), "===== OTA消息解析成功 =====");
    RCLCPP_INFO(this->get_logger(), "ota protobuf 字符串打印:\n %s", up.DebugString().c_str());
    if(up.strategy().packages_size() == 0){
        RCLCPP_WARN(this->get_logger(), "OTA消息无升级包：%s", up.task_id().c_str());
        return;
    }
    if(mainline_version_ ==  up.strategy().packages(0).target_version()){
        std::string msg = "当前已是最新版本，无需处理消息"; // 防止升级成功 未发送ack 做防丢失操作
        auto pkg = up.strategy().packages(0);
        publish_ota_ack(up.task_id(),1, 1 ,0 ,{{pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 10, 0, "", msg.c_str()}});
        RCLCPP_INFO(this->get_logger(), "当前已是最新版本，无需处理消息，发送mqtt升级成功消息");
        return;
    }
    {
        std::lock_guard<std::mutex> lk(cached_up_mutex_);
        cached_up_task_ = up;
        RCLCPP_INFO(this->get_logger(), "OTA任务已保存到缓存：task_id=%s, sub_task_id=%s, packages=%d", 
                    up.task_id().c_str(), up.sub_task_id().c_str(), up.strategy().packages_size());
    }
    RCLCPP_INFO(this->get_logger(), "OTA 消息处理完成，等待 Service 调用");
}

bool OtaAgentNode::publish_recv_ota_upgrade_msg(){
    uint64_t new_size = 0;
    std::string new_version, new_description;;
    bool has_cache = false;
    {
        std::lock_guard<std::mutex> lk(cached_up_mutex_);
        if (cached_up_task_.has_value()) {
            has_cache = true;
            const auto& up = cached_up_task_.value();
            if (up.has_strategy() && up.strategy().packages_size() > 0) {
                new_size = up.strategy().packages(0).size();
                new_version = up.strategy().packages(0).target_version();
                new_description = up.strategy().desc();
            }
        }
    }
    if(!has_cache) {
        return false;
    }
    if(new_version != mainline_version_){
        publish_package_info(new_version, new_description,new_size);
    }
    return true;
}

/**
 * @brief 声明所有ROS参数及其默认值
 * 包括网络配置、CAN配置、完整性校验等参数
 */
void OtaAgentNode::declare_ros_parameters() {
    this->declare_parameter<std::string>("vin", "");
    this->declare_parameter<std::string>("model", "");
    this->declare_parameter<std::string>("model_str", "x5");
    this->declare_parameter<std::string>("api_base_url", "");
    this->declare_parameter<std::string>("session_key", "");

    this->declare_parameter<std::string>("mqtt.broker", "mqtts://39.106.140.222:28883");
    this->declare_parameter<std::string>("mqtt.client_id", "ota_agent");
    this->declare_parameter<std::string>("mqtt.username", "");
    this->declare_parameter<std::string>("mqtt.password", "");
    this->declare_parameter<std::string>("mqtt.ca_cert", "");
    this->declare_parameter<std::string>("mqtt.cipher", "");
    this->declare_parameter<bool>("mqtt.insecure_skip_verify", false);
    this->declare_parameter<int>("mqtt.retry_attempts", 3);
    this->declare_parameter<double>("mqtt.retry_delay_base", 2.0);
    this->declare_parameter<int>("http.timeout_ms", 5000);

    this->declare_parameter<std::string>("protocol.version", "1");
    this->declare_parameter<std::string>("http.language", "zh-CN");
    this->declare_parameter<std::string>("http.client", "VehicleGateway/ota_agent");
    this->declare_parameter<std::string>("software.version", "1.0.0");

    this->declare_parameter<std::string>("controller.package_path", "");
    this->declare_parameter<bool>("controller.boot_push_enabled", false);

    this->declare_parameter<bool>("boot.broadcast_enabled", false);
    this->declare_parameter<int>("boot.broadcast_period_ms", 100);
    this->declare_parameter<std::string>("boot.broadcast_topic", "/ota/boot_broadcast");
    this->declare_parameter<std::string>("boot.broadcast_payload", "request_upgrade");
    this->declare_parameter<bool>("version.broadcast_enabled", false);
    this->declare_parameter<int>("version.broadcast_period_ms", 1000);
    this->declare_parameter<std::string>("version.broadcast_topic", "/ota/version_broadcast");
    this->declare_parameter<bool>("can.use_socketcan", true);
    this->declare_parameter<std::string>("can.ifname", "can0");

    this->declare_parameter<std::string>("signature.private_key", "");
    this->declare_parameter<std::string>("signature.public_key", "");
    
    this->declare_parameter<std::string>("integrity.signature.pubkey_path", "");
    this->declare_parameter<std::string>("integrity.signature.algorithm", "ed25519");
    this->declare_parameter<std::string>("integrity.signature.payload_template", "{version}|{url}|{md5}|{sha256}");
    this->declare_parameter<std::string>("integrity.l3.expected_versions_json", "{}");
    this->declare_parameter<std::vector<std::string>>("integrity.l3.required_nodes", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("integrity.l4.required_nodes", std::vector<std::string>{"cloud_manager", "common", "interface", "ota_agent", "postback"});
    // 主线版本管理参数
    this->declare_parameter<std::string>("oss.bucket_name", "");
    this->declare_parameter<std::string>("mainline.version", "");
    this->declare_parameter<std::string>("mainline.config_json_path", "/opt/freego/freego_version.json");

    this->declare_parameter<std::vector<std::string>>("monitored_nodes", std::vector<std::string>{});
}

/**
 * @brief 从ROS参数服务器加载所有配置参数
 */
void OtaAgentNode::load_ros_parameters() {
    // 加载基础配置参数
    this->get_parameter("vin", vin_);
    this->get_parameter("model", model_);
    this->get_parameter("model_str", model_str_);
    this->get_parameter("api_base_url", api_base_url_);
    this->get_parameter("session_key", session_key_);

    this->get_parameter("mqtt.broker", mqtt_broker_url_);
    this->get_parameter("mqtt.client_id", mqtt_client_id_);
    this->get_parameter("mqtt.username", mqtt_username_);
    this->get_parameter("mqtt.password", mqtt_password_);
    this->get_parameter("mqtt.ca_cert", mqtt_ca_cert_);
    this->get_parameter("mqtt.cipher", mqtt_cipher_);
    this->get_parameter("mqtt.insecure_skip_verify", mqtt_insecure_skip_verify_);
    this->get_parameter("mqtt.retry_attempts", mqtt_retry_attempts_);
    this->get_parameter("mqtt.retry_delay_base", mqtt_retry_delay_base_);
    this->get_parameter("http.timeout_ms", http_timeout_ms_);
    this->get_parameter("protocol.version", protocol_version_);
    this->get_parameter("http.language", http_language_);
    this->get_parameter("http.client", http_client_);

    this->get_parameter("controller.package_path", controller_package_path_);
    this->get_parameter("controller.boot_push_enabled", controller_boot_push_enabled_);
    std::string ecu_hw_sn_json = "{}";
    std::string ecu_model_json = "{}";
    this->get_parameter("ecu.hardware_sn_map_json", ecu_hw_sn_json);
    this->get_parameter("ecu.model_map_json", ecu_model_json);
    this->get_parameter("boot.broadcast_enabled", boot_broadcast_enabled_);
    this->get_parameter("boot.broadcast_period_ms", boot_broadcast_period_ms_);
    this->get_parameter("boot.broadcast_topic", boot_broadcast_topic_);
    this->get_parameter("boot.broadcast_payload", boot_broadcast_payload_);
    this->get_parameter("version.broadcast_enabled", version_broadcast_enabled_);
    this->get_parameter("version.broadcast_period_ms", version_broadcast_period_ms_);
    this->get_parameter("version.broadcast_topic", version_broadcast_topic_);
    this->get_parameter("can.use_socketcan", can_use_socketcan_);
    this->get_parameter("can.ifname", can_ifname_);
    this->get_parameter("can.send_retries", can_send_retries_);
    this->get_parameter("can.retry_delay_ms", can_retry_delay_ms_);

    this->get_parameter("flash.pull_mode", pull_flash_mode_);
    this->get_parameter("flash.pull_to_controllers_only", pull_to_controllers_only_);
    this->get_parameter("flash.pull_timeout_sec", pull_timeout_sec_);

    this->get_parameter("integrity.sha256_map_json", integrity_sha256_json_);
    this->get_parameter("integrity.signature.map_json", integrity_signature_map_json_);
    this->get_parameter("integrity.signature.pubkey_path", integrity_signature_pubkey_path_);
    this->get_parameter("integrity.signature.algorithm", integrity_signature_algorithm_);
    this->get_parameter("integrity.signature.payload_template", integrity_signature_payload_template_);
    this->get_parameter("integrity.l3.expected_versions_json", integrity_expected_versions_json_);
    this->get_parameter("integrity.l3.required_nodes", integrity_l3_required_nodes_);
    this->get_parameter("integrity.l4.required_nodes", integrity_l4_required_nodes_);
    this->get_parameter("oss.bucket_name", oss_bucket_name_);


    this->get_parameter("mainline.config_json_path", mainline_config_json_path_);
    this->get_parameter("monitored_nodes", monitored_nodes_);

    load_mainline_config();
    {
        std::string sn, msg;
        if(!request_sn_via_service(0, sn ,msg)){
            // 进程抛出异常 重启节点 保证vin请求成功
            throw std::runtime_error(msg);
        }
        vin_ = sn;
        RCLCPP_INFO(this->get_logger(), "SN: %s", sn.c_str());
    }
    mqtt_client_id_ = vin_ + "_ota";
    {
        std::string token , pub_key , priv_key, no_key, msg;
        if(request_auth_info_via_service(token, pub_key, priv_key, no_key, msg)){
            token_result_.jwt_token = token;
            token_result_.success = true;
            device_info_.session_public_key = pub_key;
            device_info_.session_private_key = priv_key;
            device_info_.session_key_no = no_key;
        }
    }
}


/**
 * @brief 发布OTA升级确认消息
 * @param task_id 任务ID
 * @param task_status 任务状态
 * @param task_result 任务结果
 * @param task_error_code 错误代码
 * @param packages 包状态列表
 */
void OtaAgentNode::publish_ota_ack(const std::string& task_id,
                                   int task_status,
                                   int task_result,
                                   int task_error_code,
                                   const std::vector<OtaStatusEntry>& packages) {
    RCLCPP_INFO(this->get_logger(), "===== 发送8.2.2 OTA升级回执 =====");
    RCLCPP_INFO(this->get_logger(), "回执主题: %s", ack_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "任务ID: %s", task_id.c_str());
    
    if (task_status == 5 && task_result == 0) {
        RCLCPP_INFO(this->get_logger(), "===== 8.2.4 车辆控制器/ECU版本上报 =====");
        RCLCPP_INFO(this->get_logger(), "上报主题: %s", build_ecu_topic().c_str());
    }
    
    vehicle::ota::ack::VehicleOtaAck ack;
    // 优先使用云端下发的字段，若为空则使用默认值
    std::string sn = vin_.empty() ? "ZQKJ001" : vin_;
    std::string model = model_str_.empty() ? "FREEGO X5 Air" : model_str_;
    ack.set_task_id(task_id);
    ack.set_sn(sn);
    ack.set_model(model);
    ack.set_timestamp(static_cast<int64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()));
    ack.set_task_status(task_status);
    ack.set_task_result(task_result);
    ack.set_task_error_code(task_error_code);
    ack.set_sys_error_code("");
    // 新协议：添加sub_task_id字段
    ack.set_sub_task_id(sub_task_id_);
    for (const auto& s : packages) {
        auto* st = ack.add_packages();
        std::string pkg_id = s.package_id.empty() ? s.ecu : s.package_id;
        std::string ecu_hw = s.ecu_hardware_code;
        int status = s.status;
        if (status == 0 && !ecu_hw.empty() && std::all_of(ecu_hw.begin(), ecu_hw.end(), [](char c){ return std::isdigit(static_cast<unsigned char>(c)); })) {
            status = std::stoi(ecu_hw);
            ecu_hw.clear();
        }
        st->set_package_id(pkg_id);
        st->set_ecu(s.ecu.empty() ? this->current_ecu_id_ : s.ecu);
        st->set_ecu_hardware_code(ecu_hw);
        st->set_current_version(s.current_version);
        st->set_status(status);
        st->set_error_code(s.error_code);
        st->set_sys_error_code(s.sys_error_code);
        st->set_message(s.message);
    }

    this->update_task_ack_state(task_id, packages);
    this->send_ack_payload(ack, "ack");
    this->maybe_publish_task_summary(task_id, false);
}

void ota_agent::OtaAgentNode::reset_task_ack_state(const std::string& task_id, size_t expected_total) {
    std::lock_guard<std::mutex> lk(ack_mutex_);
    task_ack_state_.task_id = task_id;
    task_ack_state_.expected_total = expected_total;
    task_ack_state_.latest.clear();
    task_ack_state_.summary_sent = false;
}

void ota_agent::OtaAgentNode::update_task_ack_state(const std::string& task_id, const std::vector<OtaStatusEntry>& packages) {
    std::lock_guard<std::mutex> lk(ack_mutex_);
    if (task_ack_state_.task_id.empty()) {
        task_ack_state_.task_id = task_id;
    }
    if (task_ack_state_.task_id != task_id) {
        return;
    }
    for (const auto& p : packages) {
        std::string key = !p.package_id.empty() ? p.package_id : p.ecu;
        if (key.empty()) {
            key = p.current_version;
        }
        auto& slot = task_ack_state_.latest[key];
        if (!p.package_id.empty()) slot.package_id = p.package_id; else if (slot.package_id.empty()) slot.package_id = p.ecu;
        if (!p.ecu.empty()) slot.ecu = p.ecu;
        if (!p.ecu_hardware_code.empty()) slot.ecu_hardware_code = p.ecu_hardware_code;
        if (!p.current_version.empty()) slot.current_version = p.current_version;
        slot.status = p.status;
        slot.error_code = p.error_code;
        if (!p.sys_error_code.empty()) slot.sys_error_code = p.sys_error_code;
        if (!p.message.empty()) slot.message = p.message;
    }
}

bool ota_agent::OtaAgentNode::is_final_status(int status) const {
    return status == 10 || status == 11 || status == 20 || status == 21;
}

bool ota_agent::OtaAgentNode::is_failure_status(int status) const {
    return status == 11 || status == 21;
}

void ota_agent::OtaAgentNode::maybe_publish_task_summary(const std::string& task_id, bool force) {
    std::vector<OtaStatusEntry> snapshot;
    {
        std::lock_guard<std::mutex> lk(ack_mutex_);
        if (task_ack_state_.task_id != task_id || task_ack_state_.summary_sent) {
            return;
        }
        if (!force) {
            if (task_ack_state_.expected_total == 0 || task_ack_state_.latest.size() < task_ack_state_.expected_total) {
                return;
            }
            const bool all_final = std::all_of(task_ack_state_.latest.begin(), task_ack_state_.latest.end(), [this](const auto& kv){ return is_final_status(kv.second.status); });
            if (!all_final) {
                return;
            }
        }
        snapshot.reserve(task_ack_state_.latest.size());
        for (const auto& kv : task_ack_state_.latest) {
            snapshot.push_back(kv.second);
        }
        task_ack_state_.summary_sent = true;
    }

    if (snapshot.empty()) {
        return;
    }

    int summary_result = 0;
    int summary_error = 0;
    for (const auto& e : snapshot) {
        if (is_failure_status(e.status) || e.error_code != 0) {
            summary_result = 1;
            if (summary_error == 0) {
                summary_error = e.error_code == 0 ? 5 : e.error_code;
            }
        }
    }

    vehicle::ota::ack::VehicleOtaAck ack;
    std::string sn = vin_.empty() ? "ZQKJ001" : vin_;
    std::string model = model_str_.empty() ? "FREEGO X5 Air" : model_str_;
    ack.set_task_id(task_id);
    ack.set_sn(sn);
    ack.set_model(model);
    ack.set_timestamp(static_cast<int64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()));
    ack.set_task_status(5);
    ack.set_task_result(summary_result);
    ack.set_task_error_code(summary_error);
    ack.set_sys_error_code("");
    // 新协议：添加sub_task_id字段
    ack.set_sub_task_id(sub_task_id_);
    for (const auto& s : snapshot) {
        auto* st = ack.add_packages();
        std::string pkg_id = s.package_id.empty() ? s.ecu : s.package_id;
        st->set_package_id(pkg_id);
        st->set_ecu(s.ecu.empty() ? this->current_ecu_id_ : s.ecu);
        st->set_ecu_hardware_code(s.ecu_hardware_code);
        st->set_current_version(s.current_version);
        st->set_status(s.status);
        st->set_error_code(s.error_code);
        st->set_sys_error_code(s.sys_error_code);
        st->set_message(s.message);
    }
    send_ack_payload(ack, "task_summary");
}

std::string ota_agent::OtaAgentNode::generate_base36_nonce() {
    constexpr const char* BASE36_CHARS = "0123456789abcdefghijklmnopqrstuvwxyz";
    constexpr int NONCE_LENGTH = 16;
    std::string nonce;
    nonce.reserve(NONCE_LENGTH);
    
    // 使用系统时间和随机数生成nonce
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    std::mt19937_64 rng(now_ms ^ static_cast<uint64_t>(getpid()));
    
    for (int i = 0; i < NONCE_LENGTH; ++i) {
        nonce += BASE36_CHARS[rng() % 36];
    }
    return nonce;
}

void ota_agent::OtaAgentNode::send_ack_payload(const vehicle::ota::ack::VehicleOtaAck& ack, const std::string& label) {
    RCLCPP_INFO(this->get_logger(), "===== 开始发送ACK消息 =====");
    RCLCPP_INFO(this->get_logger(), "ACK类型: %s, task_id: %s", label.c_str(), ack.task_id().c_str());
    std::string data;
    if (!ack.SerializeToString(&data)) {RCLCPP_ERROR(this->get_logger(), "ACK序列化失败(%s)", label.c_str()); return;}
    RCLCPP_DEBUG(this->get_logger(), "ACK序列化成功，大小: %zu 字节", data.size());
    RCLCPP_INFO(this->get_logger(), "ACK详情: task_status: %d, task_result: %d, task_error_code: %d", 
                ack.task_status(), ack.task_result(), ack.task_error_code());
    RCLCPP_INFO(this->get_logger(), "包数量: %d", ack.packages_size());
    for (int i = 0; i < ack.packages_size(); ++i) {
        const auto& pkg = ack.packages(i);
        RCLCPP_INFO(this->get_logger(), "  包%d: ecu=%s, status=%d, version=%s", i+1, pkg.ecu().c_str(), pkg.status(), pkg.current_version().c_str());
    }
    auto now_ms = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    std::string nonce = generate_base36_nonce();
    std::string gkey_no = device_info_.session_key_no.empty() ? "2;ZQKJ001;3" : device_info_.session_key_no;
    std::vector<unsigned char> payload_bytes(data.begin(), data.end());
    unsigned char hash[crypto_hash_sha256_BYTES];
    crypto_hash_sha256(hash, payload_bytes.data(), payload_bytes.size());
    char hash_hex[65] = {0};
    for (size_t i = 0; i < crypto_hash_sha256_BYTES; ++i) sprintf(hash_hex + 2*i, "%02x", hash[i]);
    RCLCPP_INFO(this->get_logger(), "📌 payload_bytes SHA256: %s", hash_hex);
    // 4. 拼接签名数据（对齐云端Java: gkeyNo+gtime+gnonce+payload）
    std::vector<unsigned char> gkeyno_bytes(gkey_no.begin(), gkey_no.end());
    std::vector<unsigned char> gtime_bytes(now_ms.begin(), now_ms.end());
    std::vector<unsigned char> gnonce_bytes(nonce.begin(), nonce.end());
    std::vector<unsigned char> sign_data;
    sign_data.reserve(gkeyno_bytes.size() + gtime_bytes.size() + gnonce_bytes.size() + payload_bytes.size());
    sign_data.insert(sign_data.end(), gkeyno_bytes.begin(), gkeyno_bytes.end());
    sign_data.insert(sign_data.end(), gtime_bytes.begin(), gtime_bytes.end());
    sign_data.insert(sign_data.end(), gnonce_bytes.begin(), gnonce_bytes.end());
    sign_data.insert(sign_data.end(), payload_bytes.begin(), payload_bytes.end());
    
    // 对消息进行签名
    RCLCPP_INFO(this->get_logger(), "开始对ACK消息进行Ed25519签名...");
    std::string signature = signature_mgr_->sign_data(std::string(sign_data.begin(), sign_data.end()));
    RCLCPP_INFO(this->get_logger(), "✅ ACK消息签名成功: %s", signature.c_str());
    RCLCPP_INFO(this->get_logger(), "🔐 回执加签完成：gkeyNo=%s, gtime=%s, gnonce=%s, payload_len=%zu, sign_data_len=%zu, 签名前20位=%s",
                gkey_no.c_str(), now_ms.c_str(), nonce.c_str(), data.size(), sign_data.size(), signature.substr(0,20).c_str());
    RCLCPP_INFO(this->get_logger(), "🔐 回执 gsign 全量：%s", signature.c_str());

    // 添加到发送队列
    {
        std::lock_guard<std::mutex> lk(ack_mutex_);
        ack_queue_.push_back(PendingAck{ack.task_id(), data, label, 0, signature, gkey_no, now_ms, nonce});
        RCLCPP_INFO(this->get_logger(), "ACK已添加到发送队列，队列大小: %zu", ack_queue_.size());
    }
    
    // 立即尝试发送
    drain_ack_queue();
    RCLCPP_INFO(this->get_logger(), "===== ACK消息发送流程完成 =====");
}

void ota_agent::OtaAgentNode::drain_ack_queue() {
    std::lock_guard<std::mutex> lk(ack_mutex_);
    if (!mqtt_client_ || !mqtt_client_->connection()) {
        RCLCPP_WARN(this->get_logger(), "MQTT客户端未初始化，无法发送ACK");
        return;
    }
    if (!mqtt_client_->connection()->is_connected()) {
        RCLCPP_WARN(this->get_logger(), "MQTT连接中断 尝试重连");
        // 重连mqtt 3次 重连失败 则返回失败 待下次重连 
        for(int attempt = 0; attempt < mqtt_retry_attempts_; ++attempt){
            if(mqtt_client_->connect_with_token_flow_new(token_result_)){
                RCLCPP_INFO(this->get_logger(), "MQTT重连成功");
                break;
            }
            double delay = mqtt_retry_delay_base_ * std::pow(2.0, attempt);
            std::this_thread::sleep_for(std::chrono::duration<double>(delay));
        }
        RCLCPP_WARN(this->get_logger(), "MQTT未连接，无法发送ACK，队列大小: %zu", ack_queue_.size());
        return;
    }

    if (!ack_queue_.empty()) {
        RCLCPP_INFO(this->get_logger(), "===== 开始处理ACK发送队列 =====");
        RCLCPP_INFO(this->get_logger(), "当前队列大小: %zu", ack_queue_.size());
    }
    
    while (!ack_queue_.empty()) {
        auto& item = ack_queue_.front();
        RCLCPP_INFO(this->get_logger(), "发送ACK: task_id=%s, label=%s, 尝试次数: %d/%d", 
                    item.task_id.c_str(), item.label.c_str(), item.attempts + 1, ack_retry_max_attempts_);
        
        // 构造MQTT5 User Properties（和云端一致）
        std::vector<std::pair<std::string, std::string>> user_props;
        user_props.emplace_back("gkey-no", item.gkey_no);
        user_props.emplace_back("gtime", item.gtime);
        user_props.emplace_back("gnonce", item.gnonce);
        user_props.emplace_back("gsign", item.signature);
        
        // 打印MQTT v5属性
        RCLCPP_INFO(this->get_logger(), "📦 即将发送属性 UserPropertyVO{gkey-no='%s', gtime='%s', gnonce='%s', gsign='%s'}", 
                    item.gkey_no.c_str(), item.gtime.c_str(), item.gnonce.c_str(), item.signature.c_str());
        RCLCPP_INFO(this->get_logger(), "📦 属性列表（%zu）:", user_props.size());
        for (const auto &kv : user_props) {
            RCLCPP_INFO(this->get_logger(), "  %s = %s (len=%zu)", kv.first.c_str(), kv.second.c_str(), kv.second.size());
        }
        
        // 使用带属性的发布方法
        int rc = mqtt_client_->connection()->publish_with_properties(ack_topic_, item.payload, 1, user_props);
        if (rc == MOSQ_ERR_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "✅ ACK发布成功，主题: %s", ack_topic_.c_str());
            ack_queue_.pop_front();
            RCLCPP_INFO(this->get_logger(), "队列剩余: %zu 条", ack_queue_.size());
            continue;
        }
        
        ++item.attempts;
        RCLCPP_WARN(this->get_logger(), "❌ ACK发布失败 rc=%d (错误: %s) label=%s task=%s attempt=%d/%d", 
                    rc, mosquitto_strerror(rc), item.label.c_str(), item.task_id.c_str(), item.attempts, ack_retry_max_attempts_);
        
        if (item.attempts >= ack_retry_max_attempts_) {
            RCLCPP_ERROR(this->get_logger(), "❌ ACK重试次数耗尽，丢弃消息 label=%s task=%s", item.label.c_str(), item.task_id.c_str());
            ack_queue_.pop_front();
            RCLCPP_INFO(this->get_logger(), "队列剩余: %zu 条", ack_queue_.size());
        } else {
            RCLCPP_INFO(this->get_logger(), "将在下次重试中发送: label=%s, task=%s", item.label.c_str(), item.task_id.c_str());
        }
        break;
    }
}

bool ota_agent::OtaAgentNode::wait_if_paused_or_cancelled(const std::string& phase) {
    if (cancel_requested_.load()) {
        return false;
    }
    if (!pause_requested_.load()) {
        return true;
    }
    RCLCPP_WARN(this->get_logger(), "任务暂停中，阶段=%s，等待恢复", phase.c_str());
    std::unique_lock<std::mutex> lk(pause_mutex_);
    pause_cv_.wait(lk, [this]{ return !pause_requested_.load() || cancel_requested_.load(); });
    if (cancel_requested_.load()) {
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "任务已恢复，继续阶段=%s", phase.c_str());
    return true;
}

void ota_agent::OtaAgentNode::initialize_managers() {
    // 初始化签名配置
    mqtt_module::SignatureConfig sign_config;

    // 使用真实的根密钥作为默认值
    std::string default_root_private_key = "ydSKNkOGdKx0ITtDp2GtTN9MChiDo6SQXSSRRusli0U=";
    std::string default_root_public_key = "p1c0dmOw6enDGkks5nwuwmr7mMpQm3xfDjrSDl7R0dg=";
    std::string default_root_gkey_no = "2;ZQKJ001;3";
    std::string private_key = this->get_parameter("signature.private_key").as_string();
    std::string public_key = this->get_parameter("signature.public_key").as_string();
    std::string gkey_no;
    
    // 如果没有提供密钥，使用默认的真实根密钥
    if (private_key.empty()) {
        private_key = default_root_private_key;
    }
    if (public_key.empty()) {
        public_key = default_root_public_key;
    }
    if (gkey_no.empty()){
        gkey_no = default_root_gkey_no;
    }
    
    sign_config.private_key = private_key;
    sign_config.public_key = public_key;
    sign_config.sign_separator = "|";

    // 初始化设备信息
    device_info_.vin = vin_;
    device_info_.model = model_str_;
    device_info_.root_private_key = private_key;
    device_info_.root_public_key = public_key;
    device_info_.private_key = private_key;
    // 设置根密钥编号，用于认证
    device_info_.gkey_no = gkey_no;
    device_info_.api_base_url = api_base_url_;
    device_info_.client = "vehicle";
    device_info_.app_version = "1.0";
    device_info_.language = http_language_;
    device_info_.version = "1";

    // 初始化 MQTT 配置
    mqtt_module::MqttConnConfig mqtt_config;
    mqtt_config.broker = mqtt_broker_url_;
    mqtt_config.client_id = mqtt_client_id_;
    mqtt_config.username = mqtt_username_;
    mqtt_config.password = mqtt_password_;
    mqtt_config.ca_cert_path = mqtt_ca_cert_;
    mqtt_config.insecure_skip_verify = mqtt_insecure_skip_verify_;
    mqtt_config.tls_cipher = mqtt_cipher_;
    // 兼容 mqtts:// 前缀
    if (mqtt_config.broker.rfind("mqtts://", 0) == 0) {
        mqtt_config.broker = "ssl://" + mqtt_config.broker.substr(8);
    } else if (mqtt_config.broker.rfind("mqtt://", 0) == 0) {
        mqtt_config.broker = "tcp://" + mqtt_config.broker.substr(7);
    }
    if (mqtt_config.username.empty() && mqtt_config.password.empty()) {
        mqtt_config.use_token_auth = true; // 如无账号密码，则可回退 token 方式
    }

    // 初始化 MQTT 客户端配置
    mqtt_module::MqttClientConfig client_config;
    client_config.device = device_info_;
    client_config.mqtt = mqtt_config;
    client_config.signature = sign_config;

    // 创建 MQTT 客户端
    mqtt_client_ = std::make_shared<mqtt_module::MqttClient>(client_config, this->get_logger());
    // 设置 MQTT 回调
    auto conn = mqtt_client_->connection();
    conn->set_disconnect_callback([this](int rc) {
        RCLCPP_WARN(this->get_logger(), "MQTT连接断开: %s", mosquitto_strerror(rc));
        this->on_connection_lost(std::string(mosquitto_strerror(rc)));
    });
    conn->set_message_callback([this](const std::string& topic, const std::string& payload, const std::vector<std::pair<std::string, std::string>>& properties) {
        this->on_mqtt_message(topic, payload, properties);
    });

    OSSManager::oss_auto_config oss_config;
    oss_config.bucketName = oss_bucket_name_;
    RCLCPP_INFO(this->get_logger(), "oss bucket name : %s", oss_bucket_name_.c_str());
    oss_mgr_ = std::make_shared<OSSManager>(oss_config);
    oss_mgr_->set_receive_handler([this](size_t /*increment*/, int64_t transfered, int64_t total, void* /*userData*/) {
        static double last_logged_progress = 0.0;
        if (total > 0) {
            double progress = (static_cast<double>(transfered) / static_cast<double>(total * 1.2)) * 100.0;
            if (progress - last_logged_progress >= 10.0 || progress >= 100.0) {
                RCLCPP_INFO(this->get_logger(), 
                            "📥 OSS 下载进度：%.1f%% (%.2f MB / %.2f MB)",
                            progress,
                            static_cast<double>(transfered) / 1024.0 / 1024.0,
                            static_cast<double>(total) / 1024.0 / 1024.0);
                last_logged_progress = progress;
                publish_download_progress(progress);
            }
        }
    });
}

void ota_agent::OtaAgentNode::load_mainline_config() {
    // 如果提供 JSON，支持结构：{"dc_version":"","uc_version":"DC_a001_1.4.2","mcu_version":"","version":"1.0.0"}
    if (!mainline_config_json_path_.empty()) {
        std::string config_version_json;
        std::ifstream ifs(mainline_config_json_path_);
        if (!ifs.is_open()){
            mainline_version_ = "1.0.0";
            dc_version_ = "";
            uc_version_ = "";
            mcu_version_ = "";
            RCLCPP_WARN(this->get_logger(), "open file : %s failed", mainline_config_json_path_.c_str());
            return;
        }
        ifs >> config_version_json;
        ifs.close();
        try {
            auto j = nlohmann::json::parse(config_version_json);
            if (j.contains("version") && j["version"].is_string()) {
                mainline_version_ = j["version"].get<std::string>();
            }
            if (j.contains("dc_version") && j["dc_version"].is_string()) {
                dc_version_ = j["dc_version"].get<std::string>();
            }
            if (j.contains("uc_version") && j["uc_version"].is_string()) {
                uc_version_ = j["version"].get<std::string>();
            }
            if (j.contains("mcu_version") && j["mcu_version"].is_string()) {
                mcu_version_ = j["mcu_version"].get<std::string>();
            }
            RCLCPP_INFO(this->get_logger(), "mainline_version_: %s;dc_version_:%s;uc_version:%s;mcu_version:%s",
                        mainline_version_.c_str(), dc_version_.c_str(), uc_version_.c_str(), mcu_version_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "主线版本配置JSON解析失败: %s", e.what());
        }
    }
}
bool ota_agent::OtaAgentNode::persist_mainline_config(){
    try {

        json version_info;
        version_info["version"] = mainline_version_;
        version_info["uc_version"] = uc_version_;
        version_info["mcu_version"] = mcu_version_;
        version_info["dc_version"] = dc_version_;
        std::filesystem::path p(mainline_config_json_path_);
        if (!p.parent_path().empty()) {
            std::error_code ec;
            std::filesystem::create_directories(p.parent_path(), ec);
            if (ec) {
                RCLCPP_ERROR(this->get_logger(), "创建配置目录失败：%s | 错误=%s", p.parent_path().c_str(), ec.message().c_str());
                return false;
            }
        }
        std::ofstream ofs(mainline_config_json_path_, std::ios::trunc);
        if (!ofs.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开配置文件进行写入：%s", mainline_config_json_path_.c_str());
            return false;
        }
        ofs << version_info.dump();
        ofs.close();
        RCLCPP_INFO(this->get_logger(), "✅ 主线版本配置已保存：%s", mainline_config_json_path_.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "保存主线版本配置异常：%s", e.what());
        return false;
    }
}

/**
 * @brief 从本地文件加载保存的任务ID
 * @param out_task_id 输出参数，保存加载的任务ID
 * @return 是否成功加载
 */
bool ota_agent::OtaAgentNode::load_stored_task_id(std::string& out_task_id) {
    try {
        std::filesystem::path p(task_id_file_);
        if (!std::filesystem::exists(p)) {
            return false;
        }
        std::ifstream ifs(p);
        if (!ifs.is_open()) {
            RCLCPP_WARN(this->get_logger(), "无法打开任务ID文件: %s", task_id_file_.c_str());
            return false;
        }
        nlohmann::json j = nlohmann::json::parse(ifs, nullptr, false);
        if (j.is_object() && j.contains("task_id") && j["task_id"].is_string()) {
            out_task_id = j["task_id"].get<std::string>();
            return true;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "读取任务ID文件失败: %s", e.what());
    }
    return false;
}

/**
 * @brief 将任务ID保存到本地文件
 * @param task_id 要保存的任务ID
 */
void ota_agent::OtaAgentNode::persist_task_id(const std::string& task_id) {
    try {
        std::filesystem::path p(task_id_file_);
        if (!p.parent_path().empty()) {
            std::filesystem::create_directories(p.parent_path());
        }
        nlohmann::json j;
        j["task_id"] = task_id;
        std::ofstream ofs(p, std::ios::trunc);
        if (!ofs.is_open()) {
            RCLCPP_WARN(this->get_logger(), "写入任务ID文件失败: %s", task_id_file_.c_str());
            return;
        }
        ofs << j.dump();
        RCLCPP_INFO(this->get_logger(), "✅ 任务ID已保存到本地: %s", task_id.c_str());
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "保存任务ID文件异常: %s", e.what());
    }
}

bool OtaAgentNode::persist_up_task(const vehicle::ota::up::VehicleOtaUp& up, const std::string& file_path){
    std::string binary_data;
    if(!up.SerializeToString(&binary_data)){
        RCLCPP_ERROR(this->get_logger(), "序列化数据失败");
        return false;
    }
    try{
        std::filesystem::path p(file_path);
        if (!p.parent_path().empty()) {
            std::error_code ec;
            std::filesystem::create_directories(p.parent_path(), ec);
            if (ec) {
                RCLCPP_ERROR(this->get_logger(), "创建目录失败：%s | 错误=%s", p.parent_path().c_str(), ec.message().c_str());
                return false;
            }
        }
        std::ofstream ofs(file_path, std::ios::binary | std::ios::trunc);
        if (!ofs.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开文件进行写入：%s", file_path.c_str());
            return false;
        }
        ofs.write(binary_data.data(), static_cast<std::streamsize>(binary_data.size()));
        if (!ofs.good()) {
            RCLCPP_ERROR(this->get_logger(), "文件写入失败：%s", file_path.c_str());
            ofs.close();
            return false;
        }
        ofs.close();
        RCLCPP_INFO(this->get_logger(), "二进制文件写入成功：%s | 大小=%zu bytes", file_path.c_str(), binary_data.size());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "写入二进制文件异常：%s", e.what());
        return false;
    }
    return true;
}
bool OtaAgentNode::load_stored_up_task(vehicle::ota::up::VehicleOtaUp& up, const std::string& file_path){
    std::string binary_data;
    try {
        std::filesystem::path p(file_path);
        if (!std::filesystem::exists(p)) {
            RCLCPP_INFO(this->get_logger(), "ℹ️ 二进制文件不存在：%s", file_path.c_str());
            return false;
        }
        std::ifstream ifs(p, std::ios::binary | std::ios::ate);
        if (!ifs.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "❌ 无法打开文件进行读取：%s", file_path.c_str());
            return false;
        }
        std::streamsize size = ifs.tellg();
        if (size <= 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ 获取文件大小失败：%s", file_path.c_str());
            ifs.close();
            return false;
        }
        ifs.seekg(0, std::ios::beg);
        binary_data.resize(static_cast<size_t>(size));
        if (!ifs.read(&binary_data[0], size)) {
            RCLCPP_ERROR(this->get_logger(), "❌ 读取文件内容失败：%s", file_path.c_str());
            ifs.close();
            binary_data.clear();
            return false;
        }
        ifs.close();
        RCLCPP_INFO(this->get_logger(), "✅ 二进制文件读取成功：%s | 大小=%zu bytes", file_path.c_str(), binary_data.size());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ 读取二进制文件异常：%s", e.what());
        binary_data.clear();
        return false;
    }
    try{
        if (!up.ParseFromString(binary_data)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Protobuf 二进制反序列化失败 | 数据大小=%zu", binary_data.size());
            return false;
        }
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "❌ 二进制反序列化异常：%s", e.what());
        return false;
    }
    return true;
}

void OtaAgentNode::start_boot_broadcast_timer() {
    if (!boot_broadcast_enabled_) {
        return;
    }
    const int period_ms = std::max(10, boot_broadcast_period_ms_);
    RCLCPP_INFO(this->get_logger(), "启动boot广播：topic=%s period=%dms payload=%s", boot_broadcast_topic_.c_str(), period_ms, boot_broadcast_payload_.c_str());
}

void ota_agent::OtaAgentNode::connect_and_subscribe() {
    if (!mqtt_client_) {
        RCLCPP_FATAL(this->get_logger(), "MQTT客户端未初始化");
        throw std::runtime_error("mqtt client null");
    }
    RCLCPP_INFO(this->get_logger(), "开始连接MQTT Broker: %s (client_id: %s)", mqtt_broker_url_.c_str(), mqtt_client_id_.c_str());
    for (int attempt = 0; attempt < mqtt_retry_attempts_; ++attempt) {
        RCLCPP_INFO(this->get_logger(), "MQTT连接尝试 %d/%d，使用完整认证流程", attempt + 1, mqtt_retry_attempts_);
        if (mqtt_client_->connect_with_token_flow_new(token_result_)) {
            RCLCPP_INFO(this->get_logger(), "MQTT连接成功！Broker: %s", mqtt_broker_url_.c_str());
            // mqtt_client_->get_session_key(device_info_.session_public_key,device_info_.session_private_key,device_info_.session_key_no);
            // token_result_ = mqtt_client_->token();

            mqtt_module::SignatureConfig cfg;
            cfg.public_key = device_info_.session_public_key;
            cfg.private_key = device_info_.session_private_key;
            signature_mgr_ = std::make_shared<mqtt_module::SignatureManager>(cfg, this->get_logger());

            subscribe_ota_topics();
            
            std::string stored_task_id;
            if (load_stored_task_id(stored_task_id) && !stored_task_id.empty()) {
                RCLCPP_INFO(this->get_logger(), "🔄 加载到存储的任务ID: %s，开始订阅任务状态主题", stored_task_id.c_str());
                subscribe_task_status(stored_task_id);
            }
            return;
        }
        if (attempt < mqtt_retry_attempts_ - 1) {
            double delay = mqtt_retry_delay_base_ * std::pow(2.0, attempt);
            RCLCPP_WARN(this->get_logger(), "MQTT连接失败，重试 %d/%d, 等待 %.1f 秒", attempt + 1, mqtt_retry_attempts_, delay);
            std::this_thread::sleep_for(std::chrono::duration<double>(delay));
        }
    }
    RCLCPP_FATAL(this->get_logger(), "MQTT连接失败！已尝试 %d 次，Broker: %s", mqtt_retry_attempts_, mqtt_broker_url_.c_str());
    throw std::runtime_error("MQTT连接失败");
}

void ota_agent::OtaAgentNode::subscribe_ota_topics() {
    if (!mqtt_client_ || !mqtt_client_->connection() || !mqtt_client_->connection()->is_connected()) {
        RCLCPP_WARN(this->get_logger(), "MQTT未连接，无法订阅OTA主题");
        return;
    }
    subscribe_topic(build_all_topic());
}
bool ota_agent::OtaAgentNode::subscribe_topic(const std::string& topic, int qos) {
    if (topic.empty()) {
        return false;
    }
    if (!mqtt_client_ || !mqtt_client_->connection() || !mqtt_client_->connection()->is_connected()) {
        return false;
    }
    int rc = mqtt_client_->connection()->subscribe_v5(topic, qos);
    if (rc == MOSQ_ERR_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "已订阅: %s", topic.c_str());
        return true;
    }
    RCLCPP_WARN(this->get_logger(), "订阅失败: %s (%s)", topic.c_str(), mosquitto_strerror(rc));
    return false;
}


void ota_agent::OtaAgentNode::on_mqtt_message(const std::string& topic, const std::string& payload, const std::vector<std::pair<std::string, std::string>>& properties) {
    RCLCPP_INFO(this->get_logger(), "===== 收到MQTT消息 =====");
    RCLCPP_INFO(this->get_logger(), "主题: %s", topic.c_str());
    RCLCPP_INFO(this->get_logger(), "消息长度: %zu 字节", payload.size());
    
    RCLCPP_INFO(this->get_logger(), "MQTT v5属性详情:");
    if (properties.empty()) {
        RCLCPP_INFO(this->get_logger(), "  [无属性]");
    } else {
        RCLCPP_INFO(this->get_logger(), "  属性数量: %zu", properties.size());
        for (size_t i = 0; i < properties.size(); ++i) {
            const auto& [k, v] = properties[i];
            RCLCPP_INFO(this->get_logger(), "  属性[%zu]: 键='%s', 值='%s', 键长度=%zu, 值长度=%zu",
                        i + 1, k.c_str(), v.c_str(), k.size(), v.size());
        }
    }
    
    std::map<std::string, std::string> all_properties;
    std::string gsign, gtime, gnonce, gkeyno;
    RCLCPP_INFO(this->get_logger(), "🔍 开始解析Properties...");
    for (const auto& [k, v] : properties) {
        std::string key_lower = k;
        std::transform(key_lower.begin(), key_lower.end(), key_lower.begin(), ::tolower);
        all_properties[key_lower] = v;
        RCLCPP_INFO(this->get_logger(), "  属性解析: key='%s', key='%s', value='%s'", k.c_str(), key_lower.c_str(), v.c_str());
        if (key_lower == "gsign") {
            gsign = v;
            RCLCPP_INFO(this->get_logger(), "  ✅ 找到签名属性: gsign='%s'", v.substr(0, 20).c_str());
        } else if (key_lower == "gtime") {
            gtime = v;
            RCLCPP_INFO(this->get_logger(), "  ✅ 找到时间属性: gtime='%s'", v.c_str());
        } else if (key_lower == "gnonce") {
            gnonce = v;
            RCLCPP_INFO(this->get_logger(), "  ✅ 找到随机数属性: gnonce='%s'", v.c_str());
        }
    }
    
    // 验证签名
    bool signature_valid = true;
    std::string raw_payload = payload;
    if (!gsign.empty()) {
        RCLCPP_INFO(this->get_logger(), "🔍 开始验证消息签名...");

        if (gtime.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ 指令缺少gtime属性，验签失败");
            signature_valid = false;
        }
        if (gnonce.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ 指令缺少gnonce随机数，验签失败");
            signature_valid = false;
        }
        
        if (signature_valid) {
            // 构造签名源：gtime + gnonce + payload
            std::string sign_source = gtime + gnonce + raw_payload;
            RCLCPP_INFO(this->get_logger(), "🔍 车端拼接待验签数据：gtime='%s', gnonce='%s', payload_len=%zu, total_len=%zu",
                        gtime.c_str(), gnonce.c_str(), raw_payload.size(), sign_source.size());
            
            // 检查是否为任务状态变更消息（使用根公钥验签）
            // bool is_task_status_msg = (!current_task_id_.empty() && topic == build_task_status_topic(current_task_id_));
            bool is_task_status_msg = false;
            if (is_task_status_msg) {
                RCLCPP_INFO(this->get_logger(), "📋 任务状态变更消息，使用根公钥验签");
                const std::string root_public_key_b64 = device_info_.session_public_key;
                
                // 使用直接的libsodium函数进行根公钥验证
                std::vector<unsigned char> root_public_key;
                if (mqtt_module::b64_to_bytes(root_public_key_b64, root_public_key)) {
                    if (root_public_key.size() != crypto_sign_PUBLICKEYBYTES) {
                        RCLCPP_ERROR(this->get_logger(), "❌ 根公钥长度无效: %zu, 期望: %d", root_public_key.size(), crypto_sign_PUBLICKEYBYTES);
                        signature_valid = false;
                    } else {
                        // 转换签名为字节数组
                        std::vector<unsigned char> sig_bytes;
                        if (mqtt_module::b64_to_bytes(gsign, sig_bytes)) {
                            if (sig_bytes.size() != crypto_sign_BYTES) {RCLCPP_ERROR(this->get_logger(), "❌ 根公钥签名长度无效: %zu, 期望: %d", sig_bytes.size(), crypto_sign_BYTES);
                                signature_valid = false;
                            } else {
                                // 执行实际的验签
                                int rc = crypto_sign_verify_detached(
                                    sig_bytes.data(),
                                    reinterpret_cast<const unsigned char*>(sign_source.data()),
                                    sign_source.size(),
                                    root_public_key.data());
                                
                                if (rc == 0) {
                                    RCLCPP_INFO(this->get_logger(), "✅ 任务状态消息根公钥验签通过");
                                    signature_valid = true;
                                } else {
                                    RCLCPP_ERROR(this->get_logger(), "❌ 任务状态消息根公钥验签失败，错误码: %d", rc);
                                    signature_valid = false;
                                }
                            }
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "❌ 根公钥签名Base64解码失败");
                            signature_valid = false;
                        }
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "❌ 根公钥Base64解码失败");
                    signature_valid = false;
                }
            } else {
                // 普通消息或OTA升级消息，使用原有签名管理器验签
                RCLCPP_INFO(this->get_logger(), "🔍 普通消息或OTA升级消息，使用默认公钥验签");
                if (signature_mgr_ && signature_mgr_->verify_data(sign_source, gsign)) {
                    RCLCPP_INFO(this->get_logger(), "✅ 消息签名验证通过");
                    signature_valid = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "❌ 消息签名验证失败");
                    signature_valid = false;
                }
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ 收到未签名消息，暂时允许通过");
        signature_valid = true; // 未签名消息暂时允许通过
    }
    /** 暂忽略签名认证 */
    signature_valid = true;

    if (!signature_valid) {
        RCLCPP_INFO(this->get_logger(), "=======================");
        return;
    }
    
    // 打印完整的原始消息内容（验签通过后）
    RCLCPP_INFO(this->get_logger(), "消息内容: %s", raw_payload.c_str());
    RCLCPP_INFO(this->get_logger(), "=======================");

    // 新规范：仅处理发往本车SN的定向主题，兼容保留 tag/change
    const std::string my_up_topic = build_all_topic();
    if (topic == my_up_topic) {
        RCLCPP_INFO(this->get_logger(), "===== 收到8.2.1 OTA升级消息 =====");
        RCLCPP_INFO(this->get_logger(), "主题: %s", topic.c_str());
        RCLCPP_INFO(this->get_logger(), "===== 处理OTA升级消息 =====");
        RCLCPP_INFO(this->get_logger(), "OTA升级消息属性详情:");
        for (const auto& [key, value] : all_properties) {
            RCLCPP_INFO(this->get_logger(), "  %s: %s", key.c_str(), value.c_str());
        }
        handle_ota_upgrade_msg(payload);
        publish_recv_ota_upgrade_msg();
        
    } else if (topic.find("vehicle/ota/tag") != std::string::npos) {
        RCLCPP_INFO(this->get_logger(), "===== 处理OTA升级消息(tag) =====");
        RCLCPP_INFO(this->get_logger(), "OTA升级消息(tag)属性详情:");
        for (const auto& [key, value] : all_properties) {
            RCLCPP_INFO(this->get_logger(), "  %s: %s", key.c_str(), value.c_str());
        }
        handle_ota_upgrade_msg(payload);
        publish_recv_ota_upgrade_msg();

    } else if (!current_task_id_.empty() && topic == build_task_status_topic(current_task_id_)) {
        RCLCPP_INFO(this->get_logger(), "===== 处理任务状态变更 =====");
        RCLCPP_INFO(this->get_logger(), "收到任务状态变更，task_id=%s", current_task_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "任务状态变更消息属性详情:");
        for (const auto& [key, value] : all_properties) {
            RCLCPP_INFO(this->get_logger(), "  %s: %s", key.c_str(), value.c_str());
        }
        
        handle_task_status_change(raw_payload);
    } else {
        RCLCPP_WARN(this->get_logger(), "===== 收到未知主题消息 =====");
        RCLCPP_WARN(this->get_logger(), "收到未知主题消息: %s", topic.c_str());
        RCLCPP_WARN(this->get_logger(), "未知主题消息属性详情:");
        for (const auto& [key, value] : all_properties) {
            RCLCPP_WARN(this->get_logger(), "  %s: %s", key.c_str(), value.c_str());
        }
    }
}

void ota_agent::OtaAgentNode::subscribe_task_status(const std::string& task_id) {
    if (!mqtt_client_ || !mqtt_client_->connection() || task_id.empty()) {
        RCLCPP_ERROR(this->get_logger(), "❌ 订阅任务状态失败：客户端未初始化或连接断开，或task_id为空");
        return;
    }
    const std::string t = build_task_status_topic(task_id);
    RCLCPP_INFO(this->get_logger(), "📤 开始订阅任务状态主题");
    RCLCPP_INFO(this->get_logger(), "  任务ID: %s", task_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  协议版本: %s", protocol_version_.c_str());
    RCLCPP_INFO(this->get_logger(), "  订阅主题: %s", t.c_str());
    
    int rc = mqtt_client_->connection()->subscribe_v5(t, 1);
    if (rc == MOSQ_ERR_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "✅ 订阅成功！主题: %s", t.c_str());
        RCLCPP_INFO(this->get_logger(), "📋 已经订阅了任务状态主题：%s", t.c_str());
        // 添加到已订阅主题列表，便于调试
        subscribed_topics_.insert(t);
        RCLCPP_INFO(this->get_logger(), "📋 当前已订阅主题列表：");
        for (const auto& topic : subscribed_topics_) {
            RCLCPP_INFO(this->get_logger(), "  - %s", topic.c_str());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ 订阅失败: %s (错误码: %d, 错误信息: %s)", t.c_str(), rc, mosquitto_strerror(rc));
    }
}

void ota_agent::OtaAgentNode::handle_task_status_change(const std::string& payload) {
    // 优先尝试Protobuf解析
    vehicle::ota::status::VehicleOtaStatusChange status_change;
    if (status_change.ParseFromString(payload)) {
        RCLCPP_INFO(this->get_logger(), "✅ 解析Protobuf格式任务状态变动消息成功");
        RCLCPP_INFO(this->get_logger(), "  task_id: %s", status_change.task_id().c_str());
        RCLCPP_INFO(this->get_logger(), "  task_status: %d", status_change.task_status());
        RCLCPP_INFO(this->get_logger(), "  timestamp: %ld", status_change.timestamp());
        RCLCPP_INFO(this->get_logger(), "  desc: %s", status_change.desc().c_str());
        
        // 保存任务ID
        std::string task_id = status_change.task_id();
        if (!task_id.empty()) {
            current_task_id_ = task_id;
        }
        
        // 处理任务状态
        int st = status_change.task_status();
        current_task_status_.store(st);
        if (st == 1) {
            pause_requested_.store(true);
            RCLCPP_WARN(this->get_logger(), "任务被暂停(状态=1)，等待恢复");
        } else if (st == 2) {
            cancel_requested_.store(true);
            pause_requested_.store(false);
            pause_cv_.notify_all();
            RCLCPP_WARN(this->get_logger(), "任务被取消(状态=2)，如在执行应尽快停止");
        } else {
            pause_requested_.store(false);
            pause_cv_.notify_all();
            RCLCPP_INFO(this->get_logger(), "任务状态更新: %d", st);
        }
        return;
    }
    auto j = nlohmann::json::parse(payload, nullptr, false);
    if (!j.is_discarded()) {
        RCLCPP_INFO(this->get_logger(), "✅ 解析JSON格式任务状态变动消息成功");
        int st = 0;
        if (j.contains("task_status")) {
            if (j["task_status"].is_number_integer()) st = j["task_status"].get<int>();
            else if (j["task_status"].is_string()) { try { st = std::stoi(j["task_status"].get<std::string>()); } catch (...) {} }
        }
        
        // 保存任务ID
        if (j.contains("task_id")) {
            current_task_id_ = j["task_id"].get<std::string>();
        }
        
        current_task_status_.store(st);
        if (st == 1) {
            pause_requested_.store(true);
            RCLCPP_WARN(this->get_logger(), "任务被暂停(状态=1)，等待恢复");
        } else if (st == 2) {
            cancel_requested_.store(true);
            pause_requested_.store(false);
            pause_cv_.notify_all();
            RCLCPP_WARN(this->get_logger(), "任务被取消(状态=2)，如在执行应尽快停止");
        } else {
            pause_requested_.store(false);
            pause_cv_.notify_all();
            RCLCPP_INFO(this->get_logger(), "任务状态更新: %d", st);
        }
        return;
    }
    RCLCPP_WARN(this->get_logger(), "无法解析任务状态变更负载，忽略");
}

bool ota_agent::OtaAgentNode::fetch_task_status_http(const std::string& task_id, int& task_status) {
    if (api_base_url_.empty() || task_id.empty()) return false;
    const std::string url = api_base_url_ + "/api/adevice/v1/ota/getTaskStatus";
    nlohmann::json body; body["taskId"] = task_id;
    const std::string body_str = body.dump();
    auto headers = build_headers(device_info_, body_str, true);
    try {
        auto resp = cpr::Post(cpr::Url{url}, cpr::Body{body_str}, headers, cpr::Timeout{http_timeout_ms_});
        if (resp.error) {
            RCLCPP_WARN(this->get_logger(), "getTaskStatus 网络错误: %s", resp.error.message.c_str());
            return false;
        }
        if (resp.status_code != 200) {
            RCLCPP_WARN(this->get_logger(), "getTaskStatus HTTP %ld", static_cast<long>(resp.status_code));
            return false;
        }
        auto jr = nlohmann::json::parse(resp.text, nullptr, false);
        if (jr.is_discarded()) return false;
        if (jr.contains("taskStatus")) {
            if (jr["taskStatus"].is_number_integer()) task_status = jr["taskStatus"].get<int>();
            else if (jr["taskStatus"].is_string()) { try { task_status = std::stoi(jr["taskStatus"].get<std::string>()); } catch (...) { return false; } }
            return true;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "getTaskStatus 异常: %s", e.what());
    }
    return false;
}

void ota_agent::OtaAgentNode::unsubscribe_topics(const std::vector<std::string>& topics) {
    if (!mqtt_client_ || !mqtt_client_->connection() || topics.empty()) return;
    for (const auto& topic : topics) {
        int rc = mqtt_client_->connection()->unsubscribe(topic);
        if (rc == MOSQ_ERR_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "退订: %s", topic.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "退订失败: %s (%s)", topic.c_str(), mosquitto_strerror(rc));
        }
    }
}
/**
 * @brief 下载阶段 
 * @param catch_ota_up_ 
 * @return true 
 * @return false 
 */
bool ota_agent::OtaAgentNode::process_download_task(const vehicle::ota::up::VehicleOtaUp& up, std::string& msg) {
    {
        std::unique_lock<std::mutex> lk(upgrade_mutex_, std::try_to_lock);
        if (!lk.owns_lock() || upgrade_in_progress_) {
            msg = "已有下载在进行，拒绝新的任务: " + up.task_id();
            RCLCPP_WARN(this->get_logger(), "%s",msg.c_str());
            return false;
        }
    }
    upgrade_in_progress_ = true;
    auto guard = std::shared_ptr<void>(this, [this](void*) {
        std::lock_guard<std::mutex> g(upgrade_mutex_);
        upgrade_in_progress_ = false;
        RCLCPP_INFO(this->get_logger(), "upgrade_in_progress_=false");
    });
    const int ts = current_task_status_.load();
    if (ts == 1) {
        msg = "任务当前为暂停状态，跳过执行: task_id=" + up.task_id();
        RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
        return false;
    }
    if (ts == 2) {
        msg = "任务已被取消，跳过执行: task_id=" + up.task_id();
        RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
        return false;
    }
    int latest_status = 0;
    if (fetch_task_status_http(up.task_id(), latest_status)) {
        current_task_status_.store(latest_status);
        if (latest_status == 1) {
            msg = "HTTP 二次确认：任务已暂停，跳过执行 task_id="+ up.task_id();
            RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
            return false;
        }
        if (latest_status == 2) {
            msg = "HTTP 二次确认：任务已取消，跳过执行 task_id=%s" + up.task_id();
            RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
            cancel_requested_.store(true);
            return false;
        }
    }
    /* 开始下载 */
    publish_download_progress(0.0);
    current_task_id_ = up.task_id();
    reset_task_ack_state(up.task_id(), static_cast<size_t>(up.strategy().packages_size()));
    auto summary_guard = std::unique_ptr<void, std::function<void(void*)>>(nullptr, [this, tid = up.task_id()](void*) {
        this->maybe_publish_task_summary(tid, true);
    });
    std::vector<OtaStatusEntry> completed_entries;
    const int download_attempts = std::max(1, 1 + (up.has_strategy() ? up.strategy().download_retry_count() : 0));
    for (int idx = 0; idx < up.strategy().packages_size(); ++idx) {
        const auto& pkg = up.strategy().packages(idx);
        const std::string ecu_id = pkg.ecu();
        const std::string pkg_id = pkg.package_id();
        const std::string version = pkg.target_version();
        const std::string url = pkg.download_url();

        if (!wait_if_paused_or_cancelled("package_start")) {
            return false;
        }

        std::string staged_path;
        std::string err;
        bool downloaded = false;
        for (int attempt = 1; attempt <= download_attempts; ++attempt) {
            if (!wait_if_paused_or_cancelled("download")) {
                return false;
            }
            if (download_and_stage(url, up.task_id(), pkg, staged_path, err)) {
                downloaded = true;
                if (attempt > 1) {
                    RCLCPP_INFO(this->get_logger(), "下载重试成功 task_id=%s ecu=%s attempt=%d/%d", up.task_id().c_str(), ecu_id.c_str(), attempt, download_attempts);
                }
                break;
            }
            RCLCPP_WARN(this->get_logger(), "下载失败 task_id=%s ecu=%s attempt=%d/%d: %s", up.task_id().c_str(), ecu_id.c_str(), attempt, download_attempts, err.c_str());
            if (attempt < download_attempts) {
                std::this_thread::sleep_for(1s);
            }
        }
        if (!downloaded) {
            msg = std::string("download_failed: ") + err;
            publish_ota_ack(up.task_id(), 0, 2, 2, {{pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 1, 1, "", msg.c_str()}});
            return false;
        }
        // 文件解压 包签名认证
        {
            const std::string dest_dir = ota_tmp_file_path_ + "/ota_" + up.task_id() + "_" + pkg.ecu() + "_dir";
            if (!validate_and_extract_zip(staged_path, dest_dir, msg)) {
                RCLCPP_ERROR(this->get_logger(), "validate_and_extract_zip 错误: %s", msg.c_str());
                publish_ota_ack(up.task_id(), 0, 2, 3, {{pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 1, 2, "", msg.c_str()}});
                return false;
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "OTA下载完成");
    publish_download_progress(100.0);
    return true;
}
/**
 * @brief 刷写系统
 * @param up 升级任务消息
 * @return true
 * @return false
 */
bool ota_agent::OtaAgentNode::process_upgrade_task(const vehicle::ota::up::VehicleOtaUp& up, std::string& msg) {
    {
        std::unique_lock<std::mutex> lk(upgrade_mutex_, std::try_to_lock);
        if (!lk.owns_lock() || upgrade_in_progress_) {
            msg = "已有更新在进行，拒绝新的任务: " + up.task_id();
            RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
            return false;
        }
    }
    upgrade_in_progress_ = true;
    auto guard = std::shared_ptr<void>(this, [this](void*) {
        std::lock_guard<std::mutex> g(upgrade_mutex_);
        upgrade_in_progress_ = false;
        RCLCPP_INFO(this->get_logger(), "upgrade_in_progress_=false");
    });

    const int ts = current_task_status_.load();
    if (ts == 1) {
        msg = "任务当前为暂停状态，跳过执行: task_id=" + up.task_id();
        RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
        return false;
    }
    if (ts == 2) {
        msg = "任务已被取消，跳过执行: task_id=" + up.task_id();
        RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
        return false;
    }
    int latest_status = 0;
    if (fetch_task_status_http(up.task_id(), latest_status)) {
        current_task_status_.store(latest_status);
        if (latest_status == 1) {
            msg = "HTTP 二次确认：任务已暂停，跳过执行 task_id="+ up.task_id();
            RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
            return false;
        }
        if (latest_status == 2) {
            msg = "HTTP 二次确认：任务已取消，跳过执行 task_id=%s" + up.task_id();
            RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
            cancel_requested_.store(true);
            return false;
        }
    }
    auto pkg = up.strategy().packages(0);
    if(!prepare_upgrade_task(msg)){
        publish_ota_ack(up.task_id(), 0, 2, 7,{{pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 3, 8, "", msg.c_str()}});
        return false;
    }

    publish_system_upgrade_prepare(1);

    reset_task_ack_state(up.task_id(), up.strategy().packages_size());
    current_up_id_ = up.task_id();
    publish_upgrade_progress(0.0);
    /* 
    *@brief 升级系统 
    *   1.上控升级 -- 升级  
    *   2.下控升级 -- 
    *   3.域控升级 
    */
    std::string mainline_version;
    std::string dc_version, uc_version, mcu_version;
    for(int idx = 0; idx < up.strategy().packages_size(); ++idx){
        const auto& pkg = up.strategy().packages(idx);
        const std::string url = pkg.download_url();
        const std::string dest_dir = ota_tmp_file_path_ + "/ota_" + up.task_id() + "_" + pkg.ecu() + "_dir";
        const std::string file_path = dest_dir + "/manifest.json";
        std::vector<std::string> filelist;
        mainline_version = pkg.target_version();
        
        try{
            std::ifstream ifs(file_path);
            if (!ifs.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "无法打开 manifest.json 文件：%s", file_path.c_str());
                msg = "open_manifest_failed";
                return false;
            }
            std::string content((std::istreambuf_iterator<char>(ifs)),
                                 std::istreambuf_iterator<char>());
            ifs.close();

            json j = json::parse(content);
            if (j.contains("files") && j["files"].is_array()) {
                auto& files = j["files"];
                for(const auto& file : files){
                    filelist.push_back(file.value("file",""));
                }
            }
        }catch(const std::exception& e){
            msg = "err: manifest.json json 格式解析错误";
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            publish_ota_ack(up.task_id(), 1, 2, 7,{{pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 3, 8, "", msg.c_str()}});
            return false;
        }
        
        // fileList 内部升级排序 UC -> mc -> DC
        auto get_ecu_priority = [](const std::string& filename) -> int {
            std::string upper_filename = filename;
            std::transform(upper_filename.begin(), upper_filename.end(), 
                          upper_filename.begin(), ::toupper);
            
            // 优先级：UC(上控) = 1, mc(下控) = 2, DC(域控) = 3, 其他 = 4
            if (upper_filename.find("_UC_") != std::string::npos || 
                upper_filename.find("UC_") == 0 ||
                upper_filename.find("_UC") != std::string::npos ||
                upper_filename.find("_uc_") != std::string::npos || 
                upper_filename.find("uc_") == 0 ||
                upper_filename.find("_uc") != std::string::npos ||
                upper_filename.find("UPPER") != std::string::npos) {
                return 1;
            }
            else if (upper_filename.find("_MC_") != std::string::npos || 
                     upper_filename.find("MC_") == 0 ||
                     upper_filename.find("_MC") != std::string::npos ||
                     upper_filename.find("_mc_") != std::string::npos ||
                     upper_filename.find("_mc") != std::string::npos ||
                     upper_filename.find("mc_") != std::string::npos ||
                     upper_filename.find("LOWER") != std::string::npos) {
                return 2;
            }
            else if (upper_filename.find("_DC_") != std::string::npos || 
                     upper_filename.find("DC_") == 0 ||
                     upper_filename.find("_DC") != std::string::npos ||
                     upper_filename.find("_dc_") != std::string::npos || 
                     upper_filename.find("dc_") == 0 ||
                     upper_filename.find("_dc") != std::string::npos ||
                     upper_filename.find("DOMAIN") != std::string::npos) {
                return 3;
            }
            else {
                return 4;
            }
        };
        
        // 按 ECU 类型排序（UC → DB → DC）
        std::sort(filelist.begin(), filelist.end(), 
                  [&get_ecu_priority](const std::string& a, const std::string& b) {
                      int priority_a = get_ecu_priority(a);
                      int priority_b = get_ecu_priority(b);
                      
                      if (priority_a != priority_b) {
                          return priority_a < priority_b; // 优先级高的在前
                      }
                      return a < b; // 优先级相同则按文件名排序
        });
        double process = 0.0, value = (100.0 - 20.0) / filelist.size();
        publish_upgrade_progress(10.0);
        for(const std::string& filename: filelist){
            std::string update_file = dest_dir + "/" + filename;
            if (!std::filesystem::exists(update_file)) {
                msg = update_file + " 文件不存在";
                publish_ota_ack(up.task_id(), 1, 2, 7,{{pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 3, 5, "", msg.c_str()}});
                return false;
            }
            std::string version;
            if(filename.find("UC") != std::string::npos || filename.find("uc") != std::string::npos){
                RCLCPP_INFO(this->get_logger(), "升级上控: %s", update_file.c_str());
                uint32_t driver_rx_id = can_id_upper_recv_;  // 发送给驱动器的地址 -- 上控接收地址
                uint32_t driver_tx_id = can_id_upper_send_; // 驱动器接收的地址 -- 上控发送地址
                if(!flash_firmware_complete(driver_rx_id, driver_tx_id, update_file, msg)){
                    publish_ota_ack(up.task_id(), 1, 2, 3,{{pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 3, 5, "", msg.c_str()}});
                    return false;
                }
                if(!read_hex_get_version(update_file, version, msg) && !request_uc_version_via_service_async(version, msg)){
                    return false;
                }
                uc_version = version;
            }
            else if(filename.find("MC") != std::string::npos || filename.find("mc") != std::string::npos){
                RCLCPP_INFO(this->get_logger(), "升级下控: %s", update_file.c_str());
                uint32_t driver_rx_id = can_id_lower_recv_; // 发送给驱动器的地址 -- 下控接收地址
                uint32_t driver_tx_id = can_id_lower_send_; // 驱动器接收的地址  -- 下控发送地址
                if(!flash_firmware_complete(driver_rx_id, driver_tx_id, update_file, msg)){
                    publish_ota_ack(up.task_id(), 1, 2, 3,{{pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 3, 5, "", msg.c_str()}});
                    return false;
                }
                if(!read_hex_get_version(update_file, version, msg) && !request_cu_version_via_service_async(version, msg)){
                    return false;
                }
                mcu_version = version;
            }
            else if(filename.find("DC") != std::string::npos || filename.find("dc") != std::string::npos){
                RCLCPP_INFO(this->get_logger(), "升级域控: %s", update_file.c_str());
                dc_version = mainline_version;
                if(!falsh_domain_complete(update_file, mainline_version, msg)){
                    msg = "域控升级失败" + msg;
                    publish_ota_ack(up.task_id(), 1, 2, 7,{{pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 3, 5, "", msg.c_str()}});
                    return false;
                }
            }
            process += value;
            publish_upgrade_progress(process);
        }
    }
    // 检查关键节点是否存活
    if(!check_critical_nodes_alive(msg)){ 
        publish_ota_ack(up.task_id(), 1, 2, 7,{{pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 3, 5, "", msg.c_str()}});
        return false; 
    }
    mainline_version_ = mainline_version;
    uc_version_ = uc_version;
    mcu_version_ = mcu_version;
    dc_version_ = dc_version;

    if(!persist_mainline_config()){
        msg = "本地保存版本错误";
        publish_ota_ack(up.task_id(), 1, 2, 7,{{pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 3, 5, "", msg.c_str()}});
        return false;
    }
    // 安装包管理 (保留三个最大版本文件夹)
    {
        std::string pkg_path = ota_real_file_path_;
        retain_top_versions(pkg_path);
    }
    {
        if(!flash_firmware_success(can_id_lower_recv_, can_id_lower_send_, msg)){
            msg = "下控升级失败" + msg;
            return false;
        }
        if(!flash_firmware_success(can_id_upper_recv_, can_id_upper_send_, msg)){
            msg = "上控升级失败" + msg;
            return false;
        }
    }
    RCLCPP_INFO(this->get_logger(), "升级完成");    msg = "升级完成";
    publish_upgrade_progress(100.0);
    publish_ota_ack(up.task_id(),1, 1 ,0 ,{{pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 10, 0, "", msg.c_str()}});
    return true;
}

bool ota_agent::OtaAgentNode::process_check_task(std::string& msg){
    {
        std::unique_lock<std::mutex> lk(upgrade_mutex_, std::try_to_lock);
        if (!lk.owns_lock() || upgrade_in_progress_) {
            msg = "已有更新在进行，拒绝新的任务";
            RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
            return false;
        }
    }
    upgrade_in_progress_ = true;
    auto guard = std::shared_ptr<void>(this, [this](void*) {
        std::lock_guard<std::mutex> g(upgrade_mutex_);
        upgrade_in_progress_ = false;
        RCLCPP_INFO(this->get_logger(), "upgrade_in_progress_=false");
    });
    const std::string url = api_base_url_ + "/api/adevice/v1/ota/getUp";
    json body;
    body["sn"] = vin_;
    body["model"] = model_;
    const std::string body_str = body.dump();
    auto headers = build_headers(device_info_, body_str, true);
    std::string decoded_data;
    try {
        auto resp = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{http_timeout_ms_});
        if (resp.error) {
            msg = "network error getUp请求失败(%d): " + std::to_string(static_cast<int>(resp.error.code)) + " " + resp.error.message;
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            return false;
        }
        if (resp.status_code != 200) {
            msg = "network error getUp接口返回错误: " + std::to_string(resp.status_code);
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            return false;
        }        
        auto parsed = json::parse(resp.text, nullptr, false);
        if (parsed.is_discarded()) {
            RCLCPP_ERROR(this->get_logger(), "getUp响应解析失败");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "getUp接口返回: %s", parsed.dump().c_str());
        auto data_json = parsed["data"] ;
        std::string base64_payload = data_json["taskDataBase64"].get<std::string>();
        decoded_data.resize(((base64_payload.size() + 3) / 4) * 3);
        int len = EVP_DecodeBlock(
            reinterpret_cast<unsigned char*>(&decoded_data[0]),
            reinterpret_cast<const unsigned char*>(base64_payload.data()),
            static_cast<int>(base64_payload.size())
        );
        if (len <= 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Base64 解码失败，返回值：%d", len);
            return false;
        }
         size_t padding = 0;
        if (base64_payload.size() >= 1 && base64_payload.back() == '=') padding++;
        if (base64_payload.size() >= 2 && base64_payload[base64_payload.size()-2] == '=') padding++;
        
        decoded_data.resize(static_cast<size_t>(len - padding));
        
        RCLCPP_INFO(this->get_logger(), "✅ Base64 解码成功！");
        RCLCPP_INFO(this->get_logger(), "  📊 Base64 原始长度：%zu bytes", base64_payload.size());
        RCLCPP_INFO(this->get_logger(), "  📊 解码后二进制长度：%zu bytes", decoded_data.size());
        RCLCPP_INFO(this->get_logger(), "  🔍 Padding 字节数：%zu", padding);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "解析错误: %s", e.what());
        return false;
    }
    handle_ota_upgrade_msg(decoded_data);
    return true;
}

uint32_t ota_agent::OtaAgentNode::extract_can_id(uint32_t can_id_with_flags) {
    if (can_id_with_flags & CAN_EFF_FLAG) {
        return can_id_with_flags & CAN_EFF_MASK;
    } else {
        return can_id_with_flags & CAN_SFF_MASK;
    }
}
uint32_t ota_agent::OtaAgentNode::compute_crc32(uint32_t crc32val, const uint8_t* data, size_t len) {
    static const uint32_t crc32_tab[256] = {
        0x00000000U,0x77073096U,0xEE0E612CU,0x990951BAU,0x076DC419U,0x706AF48FU,0xE963A535U,0x9E6495A3U,
        0x0EDB8832U,0x79DCB8A4U,0xE0D5E91EU,0x97D2D988U,0x09B64C2BU,0x7EB17CBDU,0xE7B82D07U,0x90BF1D91U,
        0x1DB71064U,0x6AB020F2U,0xF3B97148U,0x84BE41DEU,0x1ADAD47DU,0x6DDDE4EBU,0xF4D4B551U,0x83D385C7U,
        0x136C9856U,0x646BA8C0U,0xFD62F97AU,0x8A65C9ECU,0x14015C4FU,0x63066CD9U,0xFA0F3D63U,0x8D080DF5U,
        0x3B6E20C8U,0x4C69105EU,0xD56041E4U,0xA2677172U,0x3C03E4D1U,0x4B04D447U,0xD20D85FDU,0xA50AB56BU,
        0x35B5A8FAU,0x42B2986CU,0xDBBBC9D6U,0xACBCF940U,0x32D86CE3U,0x45DF5C75U,0xDCD60DCFU,0xABD13D59U,
        0x26D930ACU,0x51DE003AU,0xC8D75180U,0xBFD06116U,0x21B4F4B5U,0x56B3C423U,0xCFBA9599U,0xB8BDA50FU,
        0x2802B89EU,0x5F058808U,0xC60CD9B2U,0xB10BE924U,0x2F6F7C87U,0x58684C11U,0xC1611DABU,0xB6662D3DU,
        0x76DC4190U,0x01DB7106U,0x98D220BCU,0xEFD5102AU,0x71B18589U,0x06B6B51FU,0x9FBFE4A5U,0xE8B8D433U,
        0x7807C9A2U,0x0F00F934U,0x9609A88EU,0xE10E9818U,0x7F6A0DBBU,0x086D3D2DU,0x91646C97U,0xE6635C01U,
        0x6B6B51F4U,0x1C6C6162U,0x856530D8U,0xF262004EU,0x6C0695EDU,0x1B01A57BU,0x8208F4C1U,0xF50FC457U,
        0x65B0D9C6U,0x12B7E950U,0x8BBEB8EAU,0xFCB9887CU,0x62DD1DDFU,0x15DA2D49U,0x8CD37CF3U,0xFBD44C65U,
        0x4DB26158U,0x3AB551CEU,0xA3BC0074U,0xD4BB30E2U,0x4ADFA541U,0x3DD895D7U,0xA4D1C46DU,0xD3D6F4FBU,
        0x4369E96AU,0x346ED9FCU,0xAD678846U,0xDA60B8D0U,0x44042D73U,0x33031DE5U,0xAA0A4C5FU,0xDD0D7CC9U,
        0x5005713CU,0x270241AAU,0xBE0B1010U,0xC90C2086U,0x5768B525U,0x206F85B3U,0xB966D409U,0xCE61E49FU,
        0x5EDEF90EU,0x29D9C998U,0xB0D09822U,0xC7D7A8B4U,0x59B33D17U,0x2EB40D81U,0xB7BD5C3BU,0xC0BA6CADU,
        0xEDB88320U,0x9ABFB3B6U,0x03B6E20CU,0x74B1D29AU,0xEAD54739U,0x9DD277AFU,0x04DB2615U,0x73DC1683U,
        0xE3630B12U,0x94643B84U,0x0D6D6A3EU,0x7A6A5AA8U,0xE40ECF0BU,0x9309FF9DU,0x0A00AE27U,0x7D079EB1U,
        0xF00F9344U,0x8708A3D2U,0x1E01F268U,0x6906C2FEU,0xF762575DU,0x806567CBU,0x196C3671U,0x6E6B06E7U,
        0xFED41B76U,0x89D32BE0U,0x10DA7A5AU,0x67DD4ACCU,0xF9B9DF6FU,0x8EBEEFF9U,0x17B7BE43U,0x60B08ED5U,
        0xD6D6A3E8U,0xA1D1937EU,0x38D8C2C4U,0x4FDFF252U,0xD1BB67F1U,0xA6BC5767U,0x3FB506DDU,0x48B2364BU,
        0xD80D2BDAU,0xAF0A1B4CU,0x36034AF6U,0x41047A60U,0xDF60EFC3U,0xA867DF55U,0x316E8EEFU,0x4669BE79U,
        0xCB61B38CU,0xBC66831AU,0x256FD2A0U,0x5268E236U,0xCC0C7795U,0xBB0B4703U,0x220216B9U,0x5505262FU,
        0xC5BA3BBEU,0xB2BD0B28U,0x2BB45A92U,0x5CB36A04U,0xC2D7FFA7U,0xB5D0CF31U,0x2CD99E8BU,0x5BDEAE1DU,
        0x9B64C2B0U,0xEC63F226U,0x756AA39CU,0x026D930AU,0x9C0906A9U,0xEB0E363FU,0x72076785U,0x05005713U,
        0x95BF4A82U,0xE2B87A14U,0x7BB12BAEU,0x0CB61B38U,0x92D28E9BU,0xE5D5BE0DU,0x7CDCEFB7U,0x0BDBDF21U,
        0x86D3D2D4U,0xF1D4E242U,0x68DDB3F8U,0x1FDA836EU,0x81BE16CDU,0xF6B9265BU,0x6FB077E1U,0x18B74777U,
        0x88085AE6U,0xFF0F6A70U,0x66063BCAU,0x11010B5CU,0x8F659EFFU,0xF862AE69U,0x616BFFD3U,0x166CCF45U,
        0xA00AE278U,0xD70DD2EEU,0x4E048354U,0x3903B3C2U,0xA7672661U,0xD06016F7U,0x4969474DU,0x3E6E77DBU,
        0xAED16A4AU,0xD9D65ADCU,0x40DF0B66U,0x37D83BF0U,0xA9BCAE53U,0xDEBB9EC5U,0x47B2CF7FU,0x30B5FFE9U,
        0xBDBDF21CU,0xCABAC28AU,0x53B39330U,0x24B4A3A6U,0xBAD03605U,0xCDD70693U,0x54DE5729U,0x23D967BFU,
        0xB3667A2EU,0xC4614AB8U,0x5D681B02U,0x2A6F2B94U,0xB40BBE37U,0xC30C8EA1U,0x5A05DF1BU,0x2D02EF8DU
    };
    
    if (data == nullptr || len == 0) {
        return crc32val;
    }
    
    for (size_t i = 0; i < len; i++) {
        crc32val = crc32_tab[(crc32val ^ data[i]) & 0xFFU] ^ (crc32val >> 8);
    }
    
    return crc32val;
}

bool ota_agent::OtaAgentNode::falsh_domain_complete(const std::string& filename, const std::string& mainline_version, std::string& err){
    /**
     * 1. 解压zip包
     * 2. 软连接目录
     * 3. 更新完成
     */
    std::string target_dir = ota_real_file_path_;// 真实文件目录
    std::string link_target_dir = ota_link_file_path_; // 软连接目录
    namespace fs = std::filesystem;
    try {
        if (!fs::exists(target_dir)) {
            fs::create_directories(target_dir);
        }
        std::string cmd;
        cmd = "unzip -o -qq " + filename + " -d " + target_dir;
        int rc = std::system(cmd.c_str());
        if (rc != 0) {
            err = "unzip failed rc=" + std::to_string(rc);
            return false;
        }
        std::string extracted_dir = target_dir + "/" + mainline_version;
        if (fs::exists(link_target_dir)) {
            fs::remove_all(link_target_dir);
        }
        cmd = "ln -s " + extracted_dir + " " + link_target_dir;
        rc = std::system(cmd.c_str());
        if (rc != 0) {
            err = "symlink failed rc=" + std::to_string(rc);
            return false;
        }
        cmd = "chmod -R 755 " + extracted_dir;
        rc = std::system(cmd.c_str());
        if (rc != 0) {
            err = "chmod failed rc=" + std::to_string(rc);
            return false;
        }
    } catch (const std::exception& e) {
        err = std::string("prepare dest dir failed: ") + e.what();
        return false;
    }
    return true;
}

bool ota_agent::OtaAgentNode::flash_firmware_complete(uint32_t driver_rx_id, uint32_t driver_tx_id,const std::string& filename, std::string& err){
    FileInfo file_info;
    if (!prepare_file_info(file_info, filename)) {
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "%s 开始更新", filename.c_str());
    
    size_t chunk_payload_size = Constants::ISO_TP_CONTINUOUS_FRAME_PAYLOAD;
    uint32_t crc_val = 0;
    // 步骤1：请求升级
    RCLCPP_INFO(this->get_logger(), "步骤1: 请求升级");
    {   
        std::vector<uint8_t> sf = {0x04, Constants::SID_ROUTINE_CONTROL, 0x01, 0x02, 0x03, 0x55, 0x55, 0x55};
        if(!send_can(driver_rx_id, sf, err)){
            err= "发送请求升级报文失败"; return false;
        }
        std::this_thread::sleep_for(100ms);
        if (!send_can(driver_rx_id, sf, err) || !wait_ack(driver_tx_id ,Constants::SID_POS_RESPONSE_ROUTINE, "请求升级")) {
            err= "请求升级失败"; return false;
        }
    }
    
    // 步骤2：擦除 flash
    RCLCPP_INFO(this->get_logger(), "步骤2: 擦除flash");
    {
        RCLCPP_INFO(this->get_logger(), "擦除字节数：%d 字节 (起始地址：0x%x)", file_info.data_length, file_info.start_address);
        const auto addr_bytes = u32_to_bytes(file_info.start_address);
        const auto len_bytes = u32_to_bytes(file_info.data_length);
        std::vector<uint8_t> payload = {Constants::SID_ERASE_MEMORY, 0x01, 0xFF, 0x00, 0x44, 
                                        addr_bytes[0], addr_bytes[1], addr_bytes[2], addr_bytes[3],
                                        len_bytes[0], len_bytes[1], len_bytes[2], len_bytes[3]};
        
        // 先设置回调等待响应，再发送数据
        std::vector<uint8_t> resp;
        if (!wait_for_can_response(driver_tx_id, Constants::SID_POS_RESPONSE_ROUTINE, Constants::DEFAULT_TIMEOUT_MS, resp, err, -1,
                [&](const std::function<void(uint32_t, const std::vector<uint8_t>&)>& restore_cb) {
                    return send_iso15765_payload(driver_rx_id, driver_tx_id, payload, err, Constants::ISO_TP_CONTINUOUS_FRAME_PAYLOAD, restore_cb);
                }) || !is_ack_success(resp, Constants::SID_POS_RESPONSE_ROUTINE)) {
            err= "擦除flash失败";   return false;
        }
    }
    // 步骤3：请求下载
    RCLCPP_INFO(this->get_logger(), "步骤3: 请求下载");
    {
        const auto addr_bytes = u32_to_bytes(file_info.start_address);
        const auto len_bytes = u32_to_bytes(file_info.data_length);
        std::vector<uint8_t> payload = {Constants::SID_REQUEST_DOWNLOAD, 0x00, 0x44, 
                                        addr_bytes[0], addr_bytes[1], addr_bytes[2], addr_bytes[3],
                                        len_bytes[0], len_bytes[1], len_bytes[2], len_bytes[3]};

        // 先设置回调等待响应，再发送数据
        std::vector<uint8_t> resp;
        if (!wait_for_can_response(driver_tx_id, Constants::SID_POS_RESPONSE_DOWNLOAD, Constants::DEFAULT_TIMEOUT_MS, resp, err, -1,
                [&](const std::function<void(uint32_t, const std::vector<uint8_t>&)>& restore_cb) {
                    return send_iso15765_payload(driver_rx_id, driver_tx_id, payload, err, Constants::ISO_TP_FIRST_FRAME_PAYLOAD, restore_cb);
                }) || resp.size() < 4) {
            err =  "未收到有效的请求下载响应";  RCLCPP_ERROR(this->get_logger(),"%s", err.c_str());
            return false;
        }
        uint8_t max_length = resp[3];
        if (max_length == 0) {
            err = "请求下载响应中的传输数据长度为0"; RCLCPP_ERROR(this->get_logger(),"%s", err.c_str());
            return false;
        }
        chunk_payload_size = max_length;
        RCLCPP_INFO(this->get_logger(), "请求下载响应: 每次传输 %d字节", static_cast<int>(max_length));
    }
    
    // 步骤4：传输数据
    RCLCPP_INFO(this->get_logger(), "步骤4: 传输数据（每次传输长度:%ld字节）",chunk_payload_size);
    if (!send_file_iso_tp(driver_rx_id,driver_tx_id ,filename, chunk_payload_size, file_info.is_hex_file, file_info.hex_binary_data, crc_val, err)) {
        err = "传输数据有误";  RCLCPP_ERROR(this->get_logger(), "%s", err.c_str());
        return false;
    }
    
    // 步骤5：传输结束
    RCLCPP_INFO(this->get_logger(),"步骤5: 传输结束");
    {
        std::vector<uint8_t> sf = {0x01, Constants::SID_TRANSFER_EXIT, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
        if (!send_can(driver_rx_id, sf, err) || !wait_ack(driver_tx_id, Constants::SID_POS_RESPONSE_EXIT, "结束传输")) {
            err = "等待传输结束响应失败"; RCLCPP_ERROR(this->get_logger(), "%s", err.c_str());
            return false;
        }
    }
    
    // 步骤6：CRC32 校验
    RCLCPP_INFO(this->get_logger(),"步骤6: CRC32校验");
    {
        const auto crc_bytes = u32_to_bytes(crc_val);
        std::vector<uint8_t> payload = {Constants::SID_ROUTINE_CONTROL, 0x01, 0x02, 0x02, 
                                        crc_bytes[0], crc_bytes[1], crc_bytes[2], crc_bytes[3]};
        
        // 先设置回调等待响应，再发送数据
        std::vector<uint8_t> resp;
        if (!wait_for_can_response(driver_tx_id, Constants::SID_POS_RESPONSE_ROUTINE, Constants::DEFAULT_TIMEOUT_MS, resp, err, -1,
                [&](const std::function<void(uint32_t, const std::vector<uint8_t>&)>& restore_cb) {
                    return send_iso15765_payload(driver_rx_id, driver_tx_id, payload, err, chunk_payload_size, restore_cb);
                }) || !is_ack_success(resp, Constants::SID_POS_RESPONSE_ROUTINE)) {
            err = "CRC校验失败";    RCLCPP_ERROR(this->get_logger(), "%s", err.c_str());
            return false;
        }
    }
    RCLCPP_INFO(this->get_logger(), "%s完整刷写流程成功完成", filename.c_str());
    return true;
}
bool ota_agent::OtaAgentNode::flash_firmware_success(uint32_t driver_rx_id, uint32_t driver_tx_id, std::string& err){
    std::vector<uint8_t> sf = {0x04, 0x31, 0x01, 0x0E, 0x0F, 0x55, 0x55, 0x55};
    if (!send_can(driver_rx_id, sf, err) ||
        !wait_ack(driver_tx_id, Constants::SID_POS_RESPONSE_ROUTINE, "切换版本")) {
        return false;
    }
    return true;
}
bool ota_agent::OtaAgentNode::flash_firmware_back(uint32_t driver_rx_id, uint32_t /*driver_tx_id*/, std::string& err){
    std::vector<uint8_t> sf = {0x04, 0x31, 0x01, 0x0E, 0x0E, 0x55, 0x55, 0x55};
    if (!send_can(driver_rx_id, sf, err)) {
        return false;
    }
    return true;
}

bool ota_agent::OtaAgentNode::prepare_file_info(FileInfo& info, const std::string& filename){
    info.is_hex_file = (filename.size() >= 4 && 
                           filename.substr(filename.size() - 4) == ".hex");
        
    if (info.is_hex_file) {
        std::string hex_err;
        if (!parse_hex_file(filename, info.start_address, info.data_length, hex_err)) {
            RCLCPP_ERROR(this->get_logger(),"解析HEX文件失败: %s", hex_err.c_str());
            return false;
        }
        std::string convert_err;
        if (!hex_to_binary(filename, info.hex_binary_data, convert_err)) {
            RCLCPP_ERROR(this->get_logger(),"HEX文件转换失败: %s", convert_err.c_str());
            return false;
        }
        if (info.hex_binary_data.size() != info.data_length) {
            RCLCPP_ERROR(this->get_logger(),"HEX转换后数据长度不匹配");
            return false;
        }
    } else {
        std::error_code fs_ec;
        uint64_t file_size = std::filesystem::file_size(filename, fs_ec);
        if (fs_ec) {
            RCLCPP_ERROR(this->get_logger(), "获取文件大小失败: %s" , fs_ec.message().c_str());
            return false;
        }
        info.start_address = 0;
        info.data_length = static_cast<uint32_t>(file_size);
    }
    return true;
}

bool ota_agent::OtaAgentNode::parse_hex_file(const std::string& hex_path, uint32_t& start_address, uint32_t& data_length, std::string& err){
    std::ifstream ifs(hex_path, std::ios::in);
    if (!ifs.is_open()) {
        err = "无法打开HEX文件";
        return false;
    }
    start_address = Constants::HEX_INVALID_ADDRESS;
    data_length = 0;
    uint32_t upper_address = 0;
    bool first_data_record = true;
    
    std::string line;
    int line_num = 0;
    while (std::getline(ifs, line)) {
        line_num++;
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty() || line[0] != ':') continue;
        
        if (line.size() < 11) continue;
        
        int record_len = hex_str_to_byte(line, 1);
        if (record_len < 0) continue;
        
        int addr_high = hex_str_to_byte(line, 3);
        int addr_low = hex_str_to_byte(line, 5);
        if (addr_high < 0 || addr_low < 0) continue;
        uint16_t addr = static_cast<uint16_t>((addr_high << 8) | addr_low);
        
        int record_type = hex_str_to_byte(line, 7);
        if (record_type < 0) continue;
        
        size_t expected_len = 1 + 2 + 4 + 2 + record_len * 2 + 2;
        if (line.size() < expected_len) continue;
        
        switch (record_type) {
            case 0x00: { // 数据记录
                uint32_t full_address = (upper_address << 16) | addr;
                if (first_data_record) {
                    start_address = full_address;
                    first_data_record = false;
                }
                data_length += static_cast<uint32_t>(record_len);
                break;
            }
            case 0x01: // EOF
                goto parse_complete;
            case 0x04: { // 扩展线性地址
                if (record_len != 2) continue;
                int upper_high = hex_str_to_byte(line, 9);
                int upper_low = hex_str_to_byte(line, 11);
                if (upper_high < 0 || upper_low < 0) continue;
                upper_address = static_cast<uint32_t>((upper_high << 8) | upper_low);
                break;
            }
            default:
                break;
        }
    }
parse_complete:
    if (start_address == Constants::HEX_INVALID_ADDRESS) {
        err = "未找到任何数据记录";
        return false;
    }
    if (data_length == 0) {
        err = "数据长度为0";
        return false;
    }
    return true;
}
bool ota_agent::OtaAgentNode::hex_to_binary(const std::string& hex_path, std::vector<uint8_t>& binary_data, std::string& err){
    std::ifstream ifs(hex_path, std::ios::in);
    if (!ifs.is_open()) {
        err = "无法打开HEX文件";
        return false;
    }
    binary_data.clear();
    uint32_t upper_address = 0;
    std::map<uint32_t, std::vector<uint8_t>> address_data_map;
    
    std::string line;
    while (std::getline(ifs, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty() || line[0] != ':') continue;
        if (line.size() < 11) continue;
        
        int record_len = hex_str_to_byte(line, 1);
        if (record_len < 0) continue;
        
        int addr_high = hex_str_to_byte(line, 3);
        int addr_low = hex_str_to_byte(line, 5);
        if (addr_high < 0 || addr_low < 0) continue;
        uint16_t addr = static_cast<uint16_t>((addr_high << 8) | addr_low);
        
        int record_type = hex_str_to_byte(line, 7);
        if (record_type < 0) continue;
        
        size_t expected_len = 1 + 2 + 4 + 2 + record_len * 2 + 2;
        if (line.size() < expected_len) continue;
        
        switch (record_type) {
            case 0x00: { // 数据记录
                uint32_t full_address = (upper_address << 16) | addr;
                std::vector<uint8_t> data;
                for (int i = 0; i < record_len; ++i) {
                    int byte_val = hex_str_to_byte(line, 9 + i * 2);
                    if (byte_val < 0) {
                        err = "无法解析数据";
                        return false;
                    }
                    data.push_back(static_cast<uint8_t>(byte_val));
                }
                address_data_map[full_address] = std::move(data);
                break;
            }
            case 0x01:
                goto parse_complete;
            case 0x04: {
                if (record_len != 2) continue;
                int upper_high = hex_str_to_byte(line, 9);
                int upper_low = hex_str_to_byte(line, 11);
                if (upper_high < 0 || upper_low < 0) continue;
                upper_address = static_cast<uint32_t>((upper_high << 8) | upper_low);
                break;
            }
            default:
                break;
        }
    }
parse_complete:
    if (address_data_map.empty()) {
        err = "未找到任何数据记录";
        return false;
    }
    
    uint32_t min_addr = address_data_map.begin()->first;
    uint32_t max_addr = min_addr;
    for (const auto& kv : address_data_map) {
        if (kv.first < min_addr) min_addr = kv.first;
        uint32_t end_addr = kv.first + kv.second.size();
        if (end_addr > max_addr) max_addr = end_addr;
    }
    
    size_t total_size = max_addr - min_addr;
    binary_data.resize(total_size, Constants::HEX_FILL_BYTE);
    
    for (const auto& kv : address_data_map) {
        uint32_t offset = kv.first - min_addr;
        if (offset + kv.second.size() > binary_data.size()) {
            err = "地址范围超出预期";
            return false;
        }
        std::copy(kv.second.begin(), kv.second.end(), binary_data.begin() + offset);
    }
    
    return true;
}

bool ota_agent::OtaAgentNode::wait_ack(uint32_t driver_tx_id, uint8_t expect_sid, const char* step){
    std::vector<uint8_t> resp;
    std::string err;
    if (!wait_for_can_response(driver_tx_id, expect_sid, Constants::DEFAULT_TIMEOUT_MS, resp, err)) {
        RCLCPP_ERROR(this->get_logger(), "%s 等待应答超时",step);
        return false;
    }
    if (!is_ack_success(resp, expect_sid)) {
        RCLCPP_ERROR(this->get_logger(), "%s ACK指示失败",step);
        return false;
    }
    return true;
}
void ota_agent::OtaAgentNode::handle_can_rx(uint32_t can_id, const std::vector<uint8_t>& data){
    uint32_t can_id_upper_send = extract_can_id(can_id_upper_send_);
    uint32_t can_id_lower_send = extract_can_id(can_id_lower_send_);
    // can_id 必须等于 上下控接收的一个
    if((can_id_upper_send == can_id ) == (can_id_lower_send == can_id)){
        return;
    }
    if (data.empty()) {
        return;
    }
    char cmd = static_cast<char>(data[0]);
    if (cmd == 'P' && data.size() >= 2) {
        // 进度更新
        int progress = static_cast<int>(data[1]);
        RCLCPP_INFO(this->get_logger(), "升级进度: %d%%", progress);
    } else if (cmd == 'S') {
        // 成功
        RCLCPP_INFO(this->get_logger(), "收到成功反馈");
        upgrade_success_.store(true);
    } else if (cmd == 'F') {
        // 失败
        std::string reason;
        if (data.size() > 1) {
            reason.assign(reinterpret_cast<const char*>(&data[1]), data.size() - 1);
        }
        RCLCPP_ERROR(this->get_logger(), "收到失败反馈: %s ",reason.c_str());
        upgrade_failed_.store(true);
    }
}
bool ota_agent::OtaAgentNode::send_file_iso_tp(uint32_t can_id, uint32_t driver_tx_id,const std::string& file_path, size_t chunk_payload_size, 
    bool is_hex_file, const std::vector<uint8_t>& hex_binary_data , uint32_t& crc32_val,std::string& err){
    if (chunk_payload_size == 0) {
        err = "chunk_payload_size 不能为0";
        RCLCPP_ERROR(this->get_logger(),"%s", err.c_str());
        return false;
    }
    // 初始化CRC32值
    crc32_val = 0xFFFFFFFFU;
    uintmax_t file_size = 0;
    if (is_hex_file) {
        file_size = hex_binary_data.size();
    } else {
        try {
            file_size = std::filesystem::file_size(file_path);
        } catch (const std::exception& e) {
            err = "获取文件大小失败: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(),"%s", err.c_str());
            return false;
        }
    }
    
    const uint32_t total_len = static_cast<uint32_t>(file_size);
    RCLCPP_INFO(this->get_logger(), "准备发送 %u 字节，分块大小：%zu 字节", total_len, chunk_payload_size);
    
    std::ifstream ifs;
    if (!is_hex_file) {
        ifs.open(file_path, std::ios::binary);
        if (!ifs.is_open()) {
            err = "打开文件失败";
            RCLCPP_ERROR(this->get_logger(), "%s",err.c_str());
            return false;
        }
    }
    
    // 读取数据的辅助函数
    auto read_chunk_data = [&](uint32_t offset, uint32_t count) -> std::vector<uint8_t> {
        std::vector<uint8_t> data;
        size_t read_count = 0;
        
        if (is_hex_file) {
            // 检查边界
            if (offset >= hex_binary_data.size()) {
                 err = "读取偏移超出范围：offset=" + std::to_string(offset) + ", hex_binary_data.size()=" + std::to_string(hex_binary_data.size());
                RCLCPP_ERROR(this->get_logger(), "%s", err.c_str());
                return data;
            }
            size_t available = hex_binary_data.size() - offset;
            size_t to_read = std::min<size_t>(count, available);
            if (to_read > 0) {
                data.resize(to_read);
                std::copy(hex_binary_data.begin() + offset, 
                            hex_binary_data.begin() + offset + to_read, data.begin());
                read_count = to_read;
            } else {
                RCLCPP_WARN(this->get_logger(), "可读取数据为 0: offset=%u, count=%u, available=%zu",offset, count, available);}
        } else {
            data.resize(count);
            ifs.seekg(offset, std::ios::beg);
            ifs.read(reinterpret_cast<char*>(data.data()), count);
            read_count = static_cast<size_t>(ifs.gcount());
            if (read_count < count) {
                data.resize(read_count);
            }
        }
        
        return data;
    };
    
    // 将文件分成多个chunk，每个chunk大小为chunk_payload_size
    // 使用 send_iso15765_payload 发送每个chunk
    uint32_t total_sent = 0;
    uint32_t chunk_index = 0;
    
    size_t iso_tp_chunk_size = Constants::ISO_TP_CONTINUOUS_FRAME_PAYLOAD;
    
    while (total_sent < total_len) {
        uint32_t remaining = total_len - total_sent;
        uint32_t current_chunk_size = static_cast<uint32_t>(std::min<size_t>(chunk_payload_size, remaining));
        
        RCLCPP_INFO(this->get_logger(), "发送 chunk #%u: %u 字节 (总进度：%u/%u)", chunk_index + 1, current_chunk_size, total_sent, total_len);
        
        // 读取当前chunk的数据
        std::vector<uint8_t> chunk_data = read_chunk_data(total_sent, current_chunk_size);
        if (chunk_data.size() != current_chunk_size) {
            err = "读取chunk #" + std::to_string(chunk_index + 1) + " 数据失败，期望 " + std::to_string(current_chunk_size) 
                + " 字节，实际读取 " + std::to_string(chunk_data.size()) + " 字节";
            RCLCPP_ERROR(this->get_logger(), "%s", err.c_str());
            return false;
        }
        
        // 如果是HEX文件，增量计算CRC32
        if (is_hex_file && !chunk_data.empty()) {
            crc32_val = compute_crc32(crc32_val, chunk_data.data(), chunk_data.size());
        }
        
        // 计算数据块序号：从1开始，到255后从0开始
        uint8_t block_number = static_cast<uint8_t>(((chunk_index % 256) + 1) % 256);
        
        // 在数据前面添加Transfer Data SID 和块序号
        std::vector<uint8_t> payload_with_sid;
        payload_with_sid.reserve(chunk_data.size() + 2);
        payload_with_sid.push_back(Constants::SID_TRANSFER_DATA);
        payload_with_sid.push_back(block_number);
        payload_with_sid.insert(payload_with_sid.end(), chunk_data.begin(), chunk_data.end());
        
        std::vector<uint8_t> chunk_resp;
        if (!wait_for_can_response(driver_tx_id, Constants::SID_POS_RESPONSE_TRANSFER_DATA,
                Constants::DEFAULT_TIMEOUT_MS, chunk_resp, err ,static_cast<int>(block_number),
                [&](const std::function<void(uint32_t, const std::vector<uint8_t>&)>& restore_cb) {
                    return send_iso15765_payload(can_id, driver_tx_id, payload_with_sid, err, iso_tp_chunk_size, restore_cb);
                })) {
            RCLCPP_INFO(this->get_logger(), "发送chunk %d失败或等待响应超时",(chunk_index + 1));
            return false;
        }
        // 使用 send_iso15765_payload 发送chunk（包含SID和块序号）
        if (!send_iso15765_payload(can_id, driver_tx_id, payload_with_sid, err, iso_tp_chunk_size)) {
            RCLCPP_ERROR(this->get_logger(), "发送 chunk #%d 失败", (chunk_index + 1));
            return false;
        }
        
        total_sent += current_chunk_size;
        chunk_index++;
    }
    
    // 如果是HEX文件，输出最终计算的CRC32值
    if (is_hex_file) {
        RCLCPP_INFO(this->get_logger(), "HEX文件CRC32 计算完成：0x%08X (总数据长度：%u 字节)", crc32_val, total_sent);
    }
    RCLCPP_INFO(this->get_logger(), "✅ 文件发送完成：%u 字节，共 %u 个 chunk", total_sent, chunk_index);
    return true; 
}

bool ota_agent::OtaAgentNode::wait_for_can_response(uint32_t can_id, uint8_t expect_sid, int timeout_ms, std::vector<uint8_t>& resp, std::string& /*err*/, int expect_byte_at_pos2,
        std::function<bool(const std::function<void(uint32_t, const std::vector<uint8_t>&)>&)> send_after_callback_set,
        const std::function<void(uint32_t, const std::vector<uint8_t>&)>* restore_cb_override)
{
    
    struct ResponseState {
        std::vector<uint8_t> received_data;
        std::mutex mtx;
        std::condition_variable cv;
        std::atomic<bool> received { false };
    };
    auto state = std::make_shared<ResponseState>();

    // 提取期望ID的实际值（去除标志位）
    uint32_t expected_id_actual = extract_can_id(can_id);

    // 保存原有回调（通过值捕获，确保安全）
    auto original_cb = [this](uint32_t can_id, const std::vector<uint8_t>& data) {
        handle_can_rx(can_id, data);
    };

    auto temp_cb = [this, state, expected_id_actual, expect_sid, expect_byte_at_pos2, original_cb](uint32_t can_id, const std::vector<uint8_t>& data) {
        original_cb(can_id, data);
        if (can_id == expected_id_actual && !data.empty()) {
            bool match = false;
            if (expect_sid == Constants::PCI_FLOW_CONTROL && data[0] == Constants::PCI_FLOW_CONTROL) {
                match = true;
            } else if (data.size() > 1 && data[1] == expect_sid) {
                match = true;
            }
            if (match && expect_byte_at_pos2 >= 0) {
                if (data.size() <= 2 || data[2] != static_cast<uint8_t>(expect_byte_at_pos2))
                    match = false;
            }

            if (match) {
                std::lock_guard<std::mutex> lk(state->mtx);
                state->received_data = data;
                state->received.store(true);
                state->cv.notify_one();
            }
        }
    };

    can_transport_->set_receive_handler(temp_cb);
    // 先设回调再发送，传入 temp_cb 供 send_iso15765_payload 在内部恢复，避免覆盖导致 06 76 响应丢失
    if (send_after_callback_set && !send_after_callback_set(temp_cb)) {
        can_transport_->set_receive_handler(restore_cb_override && *restore_cb_override ? *restore_cb_override : original_cb);
        return false;
    }
    bool got_response = false;
    {
        std::unique_lock<std::mutex> lk(state->mtx);
        got_response = state->cv.wait_for(lk, std::chrono::milliseconds(timeout_ms), [state]() {
            return state->received.load();
        });

        if (got_response) {
            resp = state->received_data;
        }
    }
    
    auto cb_to_restore = (restore_cb_override && *restore_cb_override) ? *restore_cb_override : original_cb;
    can_transport_->set_receive_handler(cb_to_restore);

    return got_response;
}
bool ota_agent::OtaAgentNode::is_ack_success(const std::vector<uint8_t>& resp, uint8_t expect_sid) const {
    if (resp.empty() || resp.size() < 2) return false;
    uint8_t sid = resp[1];
    if (sid == Constants::SID_POS_RESPONSE_EXIT && expect_sid == Constants::SID_POS_RESPONSE_EXIT) {
            return true;
        }
        if (sid != Constants::SID_POS_RESPONSE_ROUTINE) return false;
        
        int length = resp[0];
        if (length > 0 && length < static_cast<int>(resp.size())) {
            uint8_t crc = resp[length];
            if (crc != 0x00) {
                RCLCPP_WARN(this->get_logger(), "CRC校验失败");
                return false;
            }
        }
    return true;
}
std::string ota_agent::OtaAgentNode::hexify(const std::vector<uint8_t>& data) const {
    std::ostringstream oss;
    oss << std::hex << std::uppercase;
    for (size_t i = 0; i < data.size(); ++i) {
        oss << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]);
        if (i + 1 < data.size()) oss << ' ';
    }
    return oss.str();
}
bool ota_agent::OtaAgentNode::send_can(uint32_t can_id, const std::vector<uint8_t>& payload, std::string& err) {
    if (!can_transport_) {
        err = "can transport not ready";
        RCLCPP_ERROR(this->get_logger(), "[CAN-TX-ERROR] %s", err.c_str());
        return false;
    }
    if (payload.empty()) {
        err = "empty payload";
        RCLCPP_ERROR(this->get_logger(), "[CAN-TX-ERROR] %s", err.c_str());
        return false;
    }

    ota_agent::CanFrame frame;
    frame.id = can_id;
    frame.data = payload;
    return can_transport_->send(frame, err);
}

void ota_agent::OtaAgentNode::publish_package_info(const std::string& new_version, const std::string& new_description, int32_t new_size) {
    if (!package_info_pub_) return;
    interface::msg::OtaPackageInfo msg;
    msg.new_version = new_version;
    msg.new_description = new_description;
    msg.new_size = new_size;
    package_info_pub_->publish(msg);
}
void ota_agent::OtaAgentNode::publish_download_message(const std::string& msg) {
    if (!download_message_pub_) return;
    std_msgs::msg::String m; m.data = msg;
    download_message_pub_->publish(m);
}

void ota_agent::OtaAgentNode::publish_upgrade_message(const std::string& msg) {
    if (!upgrade_message_pub_) return;
    std_msgs::msg::String m; m.data = msg;
    upgrade_message_pub_->publish(m);
}
void ota_agent::OtaAgentNode::publish_download_progress(double progress) {
    if (!download_progress_pub_) return;
    std_msgs::msg::Float64 m; m.data = progress;
    download_progress_pub_->publish(m);
}
void ota_agent::OtaAgentNode::publish_upgrade_progress(double progress) {
    if (!upgrade_progress_pub_) return;
    std_msgs::msg::Float64 m; m.data = progress;
    upgrade_progress_pub_->publish(m);
}

void ota_agent::OtaAgentNode::publish_system_upgrade_prepare(const uint32_t& ret) {
    if (!system_upgrade_prepare_pub_) return;
    std_msgs::msg::Int32 m; m.data = ret;
    system_upgrade_prepare_pub_->publish(m);
}

bool ota_agent::OtaAgentNode::send_iso15765_payload(uint32_t can_id, uint32_t driver_tx_id,const std::vector<uint8_t>& payload, std::string& err, 
        size_t chunk_payload_size, std::function<void(uint32_t, const std::vector<uint8_t>&)> rx_cb_to_restore) {
    const uint32_t max_len = Constants::ISO_TP_MAX_LEN;
    const uint32_t total_len = static_cast<uint32_t>(std::min<size_t>(payload.size(), max_len));
    
    chunk_payload_size = std::min<size_t>(chunk_payload_size, Constants::ISO_TP_CONTINUOUS_FRAME_PAYLOAD);
    if (chunk_payload_size == 0) chunk_payload_size = Constants::ISO_TP_CONTINUOUS_FRAME_PAYLOAD;

    if (total_len <= Constants::ISO_TP_SINGLE_FRAME_MAX) {
        std::vector<uint8_t> sf;
        sf.reserve(8);
        sf.push_back(static_cast<uint8_t>(0x00 | total_len));
        sf.insert(sf.end(), payload.begin(), payload.begin() + total_len);
        while (sf.size() < 8) sf.push_back(0x55);
        return send_can(can_id, sf, err);
    }
    
    uint32_t sent = 0;
    uint8_t len_high = static_cast<uint8_t>((total_len >> 8) & 0x0F);
    uint8_t len_low  = static_cast<uint8_t>(total_len & 0xFF);
    std::vector<uint8_t> ff;
    ff.reserve(8);
    ff.push_back(static_cast<uint8_t>(0x10 | len_high));
    ff.push_back(len_low);
    size_t first_payload = std::min<size_t>(Constants::ISO_TP_FIRST_FRAME_PAYLOAD, total_len);
    ff.insert(ff.end(), payload.begin(), payload.begin() + first_payload);
    while (ff.size() < 8) ff.push_back(0x55);
    
    uint8_t fc_bs = 0;
    uint8_t fc_stmin = 0;
    
    struct FlowControlState {
        std::vector<uint8_t> resp;
        std::mutex mtx;
        std::condition_variable cv;
        std::atomic<bool> received{false};
    };
    auto fc_state = std::make_shared<FlowControlState>();
    uint32_t expected_fc_id = extract_can_id(driver_tx_id);
    
    auto original_cb = rx_cb_to_restore ? rx_cb_to_restore : std::function<void(uint32_t, const std::vector<uint8_t>&)>([this](uint32_t can_id, const std::vector<uint8_t>& data) { handle_can_rx(can_id, data); });

    auto fc_temp_cb = [this, fc_state, expected_fc_id, original_cb] (uint32_t can_id, const std::vector<uint8_t>& data) {
        original_cb(can_id, data);
        // 然后检查是否是流控帧
        if (can_id == expected_fc_id && !data.empty() && data[0] == Constants::PCI_FLOW_CONTROL) {
            std::lock_guard<std::mutex> lk(fc_state->mtx);
            fc_state->resp = data;
            fc_state->received.store(true);
            fc_state->cv.notify_one();
        }
    };
    can_transport_->set_receive_handler(fc_temp_cb);
    if (!send_can(can_id, ff, err)) {
        can_transport_->set_receive_handler(original_cb);
        return false;
    }
    sent += static_cast<uint32_t>(first_payload);
    bool got_fc = false;
    if(fc_state->received.load(std::memory_order_acquire)){
        got_fc = true;
    }
    else {
        std::unique_lock<std::mutex> lk(fc_state->mtx);
        if (!fc_state->received.load(std::memory_order_relaxed)) {
            got_fc = fc_state->cv.wait_for(lk, std::chrono::milliseconds(Constants::FLOW_CONTROL_TIMEOUT_MS), [fc_state]() {
                return fc_state->received.load(std::memory_order_relaxed);
            });
        } else {
            got_fc = true;
        }
    }
    can_transport_->set_receive_handler(original_cb);
    if (!got_fc) {
        RCLCPP_ERROR(this->get_logger(),"等待流控帧超时");
        return false;
    }
    {
        std::lock_guard<std::mutex> lk(fc_state->mtx);
        if (fc_state->resp.size() >= 3 && fc_state->resp[0] == Constants::PCI_FLOW_CONTROL) {
            fc_bs = fc_state->resp[1];
            fc_stmin = fc_state->resp[2];
            RCLCPP_INFO(this->get_logger(), "收到 FC: BS=%d, STmin=0x%02X", static_cast<int>(fc_bs), static_cast<int>(fc_stmin));
        } else {
            RCLCPP_ERROR(this->get_logger(), "收到无效的流控帧");
            return false;
        }
    }
    auto stmin_duration = [](uint8_t st) -> std::chrono::microseconds {
        if (st <= 0x7F) return std::chrono::milliseconds(static_cast<int>(st));
        if (st >= 0xF1 && st <= 0xF9) return std::chrono::microseconds((st - 0xF0) * 100);
        return std::chrono::microseconds(0);
    };
    auto st_gap = stmin_duration(fc_stmin);
    uint8_t bs_window = fc_bs;
    
    uint8_t seq = 1;
    while (sent < total_len) {
        if (fc_bs != 0 && bs_window == 0) {
            std::vector<uint8_t> fc_resp;
            if (!wait_for_can_response(driver_tx_id, Constants::PCI_FLOW_CONTROL, Constants::FLOW_CONTROL_WAIT_MS, fc_resp, 
                err, -1 , nullptr, &original_cb)) {
                RCLCPP_ERROR(this->get_logger(), "等待下一个流控帧超时");
                return false;
            }
            if (fc_resp.size() >= 3 && fc_resp[0] == Constants::PCI_FLOW_CONTROL) {
                fc_bs = fc_resp[1];
                fc_stmin = fc_resp[2];
                bs_window = fc_bs;
                st_gap = stmin_duration(fc_stmin);
            } else {
                RCLCPP_ERROR(this->get_logger(), "收到无效的流控帧");
                return false;
            }
        }
        
        uint32_t remaining = total_len - sent;
        size_t n = std::min<size_t>(chunk_payload_size, remaining);
        std::vector<uint8_t> cf;
        cf.reserve(8);
        cf.push_back(static_cast<uint8_t>(0x20 | (seq & 0x0F)));
        cf.insert(cf.end(), payload.begin() + sent, payload.begin() + sent + n);
        while (cf.size() < 8) cf.push_back(0x55);
        if (!send_can(can_id, cf, err)) return false;
        sent += static_cast<uint32_t>(n);
        seq = static_cast<uint8_t>((seq + 1) & 0x0F);
        
        if (fc_bs != 0 && bs_window > 0) {
            --bs_window;
        }
        if (st_gap.count() > 0) {
            std::this_thread::sleep_for(st_gap);
        }
    }
    return true;
}
// ===== 5.2 服务实现 =====
void ota_agent::OtaAgentNode::handle_package_download(const std::shared_ptr<interface::srv::PackageDownload::Request>/*req*/,
                            std::shared_ptr<interface::srv::PackageDownload::Response> res){
    RCLCPP_INFO(this->get_logger(), "收到包下载请求");
    
    if (is_downloading_.load()) {
        RCLCPP_WARN(this->get_logger(), "下载任务线程正在执行中,任务请求保护 忽略本次请求");
        return;
    }
    
    std::optional<vehicle::ota::up::VehicleOtaUp> up;
    {
        std::lock_guard<std::mutex> lk(cached_up_mutex_);
        if (!cached_up_task_.has_value()) {
            res->code = 1; res->message = "没有缓存的 OTA任务";
            RCLCPP_WARN(this->get_logger(), "没有缓存的 OTA任务");
            return;
        }
        up = cached_up_task_;
    }

    std::jthread([this, up]() { 
        is_downloading_ = true;
        try {
            std::string msg;
            if(!process_download_task(*up, msg)) {
                publish_download_message(msg);
            }else {
                persist_up_task(*up, current_ota_task_file_);
            }
        } catch (...) {
        }
        is_downloading_ = false;
    });
}


void ota_agent::OtaAgentNode::handle_package_upgrade(const std::shared_ptr<interface::srv::PackageUpgrade::Request> /*req*/,
                            std::shared_ptr<interface::srv::PackageUpgrade::Response> res){
    RCLCPP_INFO(this->get_logger(), "收到更新包安装请求");
    
    if (is_upgrading_.load()) {
        RCLCPP_WARN(this->get_logger(), "升级任务线程正在执行中，忽略本次升级任务请求");
        return;
    }
    
    vehicle::ota::up::VehicleOtaUp up;
    if(!load_stored_up_task(up, current_ota_task_file_)) {
        res->code = 1; res->message = "fail";
        RCLCPP_WARN(this->get_logger(), "获取缓存ota消息失败");
        return;
    }
    std::jthread([this, up]{
        is_upgrading_ = true;
        try {
            std::string msg;
            bool ret = process_upgrade_task(up, msg);
            if(!ret) {
                publish_upgrade_message(msg);
                if(msg.find("电量低于80%") != std::string::npos || msg.find("车辆速度不为0") != std::string::npos){
                    is_upgrading_ = false;
                }
                else{
                    flash_firmware_back(can_id_lower_recv_, can_id_lower_send_, msg);
                    flash_firmware_back(can_id_upper_recv_, can_id_upper_send_, msg);
                }
            }
            std::filesystem::remove_all(ota_tmp_file_path_);
            publish_system_upgrade_prepare(0);
        } catch (...) {
        }
        is_upgrading_ = false;
    });
}

void ota_agent::OtaAgentNode::handle_package_check(const std::shared_ptr<interface::srv::PackageCheck::Request> /*req*/,
                        std::shared_ptr<interface::srv::PackageCheck::Response> res){
    RCLCPP_INFO(this->get_logger(), "收到包检查请求");
    uint64_t new_size = 0;
    std::string new_version, new_description;
    res->history_version = mainline_version_;

    std::string msg;
    if(!process_check_task(msg) && msg.find("network error") != std::string::npos) {
        RCLCPP_ERROR(this->get_logger(), "网络错误 或 接口请求失败: %s", msg.c_str());
        res->code = 3; res->message = "接口请求失败:" + msg;
        return;
    }
    bool has_cache = false;
    {
        std::lock_guard<std::mutex> lk(cached_up_mutex_);
        if (cached_up_task_.has_value()) {
            has_cache = true;
            const auto& up = cached_up_task_.value();
            if (up.has_strategy() && up.strategy().packages_size() > 0) {
                new_size = up.strategy().packages(0).size();
                new_version = up.strategy().packages(0).target_version();
                new_description = up.strategy().desc();
            }
        }
    }
    // 且当前任务状态未暂停或取消，无需升级
    if (new_version == res->history_version || current_task_status_ != 0 || !has_cache){
        res->code = 1; res->message = "当前版本已更新，无需升级";
        RCLCPP_INFO(this->get_logger(), "当前版本已更新，无需升级");
        return;
    }
    res->new_size = new_size;   res->new_version = new_version;
    res->new_description = new_description;
    {
        vehicle::ota::up::VehicleOtaUp up;
        if(load_stored_up_task(up, current_ota_task_file_)){
            const auto& pkg = up.strategy().packages(0);
            const std::string file_path = ota_tmp_file_path_ + "/ota_" + up.task_id() + "_" + pkg.ecu() + "_dir/manifest.json";
            if (new_version == pkg.target_version() && std::filesystem::exists(file_path)){
                res->code = 2; res->message = "下载完成 待安装";
                RCLCPP_INFO(this->get_logger(), "下载完成 待安装");
                return;
            }
        }
    }
    res->code = 0;  res->message = "存在新版本 待下载";
    RCLCPP_INFO(this->get_logger(), "存在新版本 待下载");
}

bool ota_agent::OtaAgentNode::request_sn_via_service(uint32_t id, std::string& sn, std::string& err) {
    RCLCPP_INFO(this->get_logger(), "🔍 开始通过 Service 请求 SN id=%d", id);
    try {
        // 等待服务可用（带超时）
        auto client = this->create_client<interface::srv::IdControls>(idControlService);
        if (!client) {
            err = "创建 IdControls Service 客户端失败";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
        if (!client->wait_for_service(std::chrono::seconds(3))) {
            err = "IdControls Service 不可用：/vlago/car_id";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
        auto request = std::make_shared<interface::srv::IdControls::Request>();
        request->id = id;
        RCLCPP_INFO(this->get_logger(), "📤 发送 IdControls 请求");
        // 同步调用（带超时）
        auto future = client->async_send_request(request);
        auto result = spin_until_future_complete(this->get_node_base_interface(), future, 3s);
        if (result == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if(!response){
                err = "Service 返回失败：获取sn信息失败";
                RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
                return false;
            }
            sn = response->sn;
            uint32_t code = response->code;
            RCLCPP_INFO(this->get_logger(), "✅ IdControls 请求成功： sn='%s', code=%d", sn.c_str(), code);            
        } else {
            err = "IdControls Service 调用失败（超时或其他异常）";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
    } catch (const std::exception& e) {
        err = "IdControls Service 调用异常：" + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
        return false;
    }
    return true;
}

bool ota_agent::OtaAgentNode::request_auth_info_via_service(std::string& jwt_token, std::string& session_public_key,
    std::string& session_private_key, std::string& session_no_key, std::string& err) {
    RCLCPP_INFO(this->get_logger(), "🔐 开始通过 Service 请求认证信息");
    try {
        // 创建认证 Service 客户端
        auto client = this->create_client<interface::srv::GetToken>(cloudManagerGetAuthInfoService);
        if (!client) {
            err = "创建认证 Service 客户端失败";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
        // 等待服务可用（带超时）
        if (!client->wait_for_service(std::chrono::seconds(3))) {
            err = "认证 Service 不可用：get_token";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
        // 构建请求（GetToken 请求为空，不需要设置字段）
        auto request = std::make_shared<interface::srv::GetToken::Request>();
        request->client_id = mqtt_client_id_;
        RCLCPP_INFO(this->get_logger(), "📤 发送认证信息请求");
        // 同步调用（带超时）
        auto future = client->async_send_request(request);
        auto result = spin_until_future_complete(this->get_node_base_interface(), future, 10s);
        if (result == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            // 检查响应是否成功
            if (!response->token_success) {
                err = "Service 返回失败：获取认证信息失败";
                RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
                return false;
            }
            // 提取认证信息
            jwt_token = response->jwt_token;
            session_public_key = response->public_key;
            session_private_key = response->private_key;
            session_no_key = response->key_no;
            
            RCLCPP_INFO(this->get_logger(), 
                       "✅ 认证信息请求成功：");
            RCLCPP_INFO(this->get_logger(), 
                        "  JWT Token: %s", jwt_token.c_str());
            RCLCPP_INFO(this->get_logger(), 
                        "  会话公钥：%s", session_public_key.c_str());
            RCLCPP_INFO(this->get_logger(), 
                        "  会话私钥：%s", session_private_key.c_str());
            RCLCPP_INFO(this->get_logger(), 
                        "  会话无钥：%s", session_no_key.c_str());
        } else {
            err = "Service 调用超时（10 秒）";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
        
    } catch (const std::exception& e) {
        err = "Service 调用异常：" + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
        return false;
    }
    return true;
}

bool ota_agent::OtaAgentNode::request_auth_info_via_service_async(std::string& jwt_token, std::string& session_public_key,
    std::string& session_private_key, std::string& session_no_key, std::string& err) {
    RCLCPP_INFO(this->get_logger(), "🔐 开始通过 Service 请求认证信息");
    struct AuthInfoState {
        std::mutex mtx;
        std::condition_variable cv;
        std::atomic<bool> received{false};
        std::string jwt_token;
        std::string session_public_key;
        std::string session_private_key;
        std::string session_no_key;
        bool token_success = false;
    };
    auto auth_state = std::make_shared<AuthInfoState>();
    try {
        std::this_thread::sleep_for(2s);
        // 创建认证 Service 客户端
        auto client = this->create_client<interface::srv::GetToken>(cloudManagerGetAuthInfoService);
        if (!client) {
            err = "创建认证 Service 客户端失败";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
        // 等待服务可用（带超时）
        if (!client->wait_for_service(std::chrono::seconds(3))) {
            err = "认证 Service 不可用：get_token";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
        // 构建请求
        auto request = std::make_shared<interface::srv::GetToken::Request>();
        request->client_id = mqtt_client_id_;
        RCLCPP_INFO(this->get_logger(), "📤 发送认证信息请求");
        // 异步发送请求
        client->async_send_request(request, [auth_state](rclcpp::Client<interface::srv::GetToken>::SharedFuture future) {
            auto response = future.get();
            if(response){
                std::lock_guard<std::mutex> lk(auth_state->mtx);
                auth_state->jwt_token = response->jwt_token;
                auth_state->session_public_key = response->public_key;
                auth_state->session_private_key = response->private_key;
                auth_state->session_no_key = response->key_no;
                auth_state->token_success = response->token_success;
                auth_state->received.store(true);
                auth_state->cv.notify_one();
                RCLCPP_INFO(rclcpp::get_logger("ota_agent"), "✅ 认证信息请求成功");
            }
        });
        bool got_auth = false;
        if(auth_state->received.load(std::memory_order_acquire)){
            got_auth = true;
        }
        else {
            std::unique_lock<std::mutex> lk(auth_state->mtx);
            if (!auth_state->received.load(std::memory_order_relaxed)) {
                got_auth = auth_state->cv.wait_for(lk, std::chrono::milliseconds(10s), [auth_state]() {
                    return auth_state->received.load(std::memory_order_relaxed);
                });
            } else {
                got_auth = true;
            }
        }
        if (!got_auth) {
            err = "认证信息请求超时";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
        {
            std::lock_guard<std::mutex> lk(auth_state->mtx);
            if (!auth_state->token_success) {
                err = "Service 返回失败：获取认证信息失败";
                RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
                return false;
            }
            // 提取认证信息
            jwt_token = auth_state->jwt_token;
            session_public_key = auth_state->session_public_key;
            session_private_key = auth_state->session_private_key;
            session_no_key = auth_state->session_no_key;
            
            RCLCPP_INFO(this->get_logger(), 
                       "✅ 认证信息请求成功：");
            RCLCPP_INFO(this->get_logger(), 
                        "  JWT Token: '%s...'", jwt_token.c_str());
            RCLCPP_INFO(this->get_logger(), 
                        "  会话公钥：'%s...'", session_public_key.c_str());
            RCLCPP_INFO(this->get_logger(), 
                        "  会话私钥：'%s...'", session_private_key.c_str());
            RCLCPP_INFO(this->get_logger(), 
                        "  会话无钥：'%s...'", session_no_key.c_str());
        }
    } catch (const std::exception& e) {
        err = "Service 调用异常：" + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
        return false;
    }
    return true;
}

bool ota_agent::OtaAgentNode::request_uc_version_via_service_async(const std::string& version, std::string& err){
    RCLCPP_INFO(this->get_logger(), "🔍 开始通过 Service 获取UC版本");
    struct VersionState {
        std::mutex mtx;
        std::condition_variable cv;
        std::atomic<bool> received{false};
    };
    tmp_version_ = "";
    auto fc_state = std::make_shared<VersionState>();
    try {
        std::this_thread::sleep_for(2s);
        auto client = this->create_client<interface::srv::SystemVersion>(UCVersionService);
        if (!client) {
            err = "创建 Service 获取UC版本失败";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
        if (!client->wait_for_service(std::chrono::seconds(3))) {
            err = "SystemVersion Service 不可用：/vlago/uc/version";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
        auto request = std::make_shared<interface::srv::SystemVersion::Request>();
        RCLCPP_INFO(this->get_logger(), "📤 发送 SystemVersion 请求");
        client->async_send_request(request, [this, fc_state](rclcpp::Client<interface::srv::SystemVersion>::SharedFuture future) {
            auto response = future.get();
            if(response){
                std::lock_guard<std::mutex> lk(fc_state->mtx);
                tmp_version_ = response->software_version;
                fc_state->received.store(true);
                fc_state->cv.notify_one();
                RCLCPP_INFO(this->get_logger(),  "✅ SystemVersion 请求成功：hardware_version='%s', software_version=%s, version=%s",
                response->hardware_version.c_str(), response->software_version.c_str(), response->version.c_str());
            }
        });
        bool got_fc = false;
        if(fc_state->received.load(std::memory_order_acquire)){
            got_fc = true;
        }
        else {
            std::unique_lock<std::mutex> lk(fc_state->mtx);
            if (!fc_state->received.load(std::memory_order_relaxed)) {
                got_fc = fc_state->cv.wait_for(lk, std::chrono::milliseconds(5s), [fc_state]() {
                    return fc_state->received.load(std::memory_order_relaxed);
                });
            } else {
                got_fc = true;
            }
        }
        if (!got_fc) {
            err = "uc 请求超时";    RCLCPP_ERROR(this->get_logger(),"%s", err.c_str());
            return false;
        }
        {
            std::lock_guard<std::mutex> lk(fc_state->mtx);
            if (tmp_version_ != version) {
                err = "UC版本不匹配";
                RCLCPP_ERROR(this->get_logger(), "%s", err.c_str());
                return false;
            }
        }
    } catch (const std::exception& e) {
        err = "SystemVersion Service 调用异常：" + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
        return false;
    }
    return true;
}
bool ota_agent::OtaAgentNode::request_cu_version_via_service_async(const std::string& version, std::string& err){
    RCLCPP_INFO(this->get_logger(), "🔍 开始通过 Service 获取CU版本");
    struct VersionState {
        std::mutex mtx;
        std::condition_variable cv;
        std::atomic<bool> received{false};
    };
    tmp_version_ = "";
    auto fc_state = std::make_shared<VersionState>();
    try {
        std::this_thread::sleep_for(2s);
        auto client = this->create_client<interface::srv::SystemVersion>(DBVersionService);
        if (!client) {
            err = "创建 Service 获取CU版本失败";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
        if (!client->wait_for_service(std::chrono::seconds(3))) {
            err = "SystemVersion Service 不可用：/vlago/cu/version";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
        auto request = std::make_shared<interface::srv::SystemVersion::Request>();
        RCLCPP_INFO(this->get_logger(), "📤 发送 SystemVersion 请求");
        client->async_send_request(request, [this, fc_state](rclcpp::Client<interface::srv::SystemVersion>::SharedFuture future) {
            auto response = future.get();
            if(response){
                std::lock_guard<std::mutex> lk(fc_state->mtx);
                tmp_version_ = response->software_version;
                fc_state->received.store(true);
                fc_state->cv.notify_one();
                RCLCPP_INFO(this->get_logger(),  "✅ SystemVersion 请求成功：hardware_version='%s', software_version=%s, version=%s",
                response->hardware_version.c_str(), response->software_version.c_str(), response->version.c_str());
            }
        });
        bool got_fc = false;
        if(fc_state->received.load(std::memory_order_acquire)){
            got_fc = true;
        }
        else {
            std::unique_lock<std::mutex> lk(fc_state->mtx);
            if (!fc_state->received.load(std::memory_order_relaxed)) {
                got_fc = fc_state->cv.wait_for(lk, std::chrono::milliseconds(5s), [fc_state]() {
                    return fc_state->received.load(std::memory_order_relaxed);
                });
            } else {
                got_fc = true;
            }
        }
        if (!got_fc) {
            err = "cu 请求超时";    RCLCPP_ERROR(this->get_logger(),"%s", err.c_str());
            return false;
        }
        {
            std::lock_guard<std::mutex> lk(fc_state->mtx);
            if (tmp_version_ != version) {
                err = "CU版本不匹配";
                RCLCPP_ERROR(this->get_logger(), "%s", err.c_str());
                return false;
            }
        }
    } catch (const std::exception& e) {
        err = "SystemVersion Service 调用异常：" + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
        return false;
    }
    return true;
}

void ota_agent::OtaAgentNode::on_car_info_received(const interface::msg::CarInfo::SharedPtr msg){
    std::lock_guard<std::mutex> lock(car_info_mutex_);
    latest_car_info_ = *msg;
}
bool ota_agent::OtaAgentNode::check_critical_nodes_alive(std::string& msg) {
    std::vector<std::string> nodes_to_check = monitored_nodes_;
    // kill node
    for (const auto& node_name : nodes_to_check) {
        try {
            std::string kill_command = "pkill -f " + node_name;
            int result = std::system(kill_command.c_str());
            if (result != 0) {
                RCLCPP_WARN(this->get_logger(), "⚠️ 杀掉节点失败：%s (返回码: %d)", node_name.c_str(), result);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ 杀掉节点异常：%s - %s", node_name.c_str(), e.what());
        }
    }
    // 间隔 15s 等待节点重启
    std::this_thread::sleep_for(15s);
    int retry_count = 0;
    const int max_retries = 300; // 最多重试 300 次（300 秒）
    const std::chrono::seconds retry_interval(1); // 每次间隔 1 秒
    
    std::unordered_map<std::string, bool> node_status;
    std::unordered_map<std::string, int> node_fail_count;
    
    // 初始化节点状态
    for (const auto& node_name : nodes_to_check) {
        node_status[node_name] = false;
        node_fail_count[node_name] = 0;
    }
    
    RCLCPP_INFO(this->get_logger(), "🔍 开始检查 %zu 个关键节点的存活状态...", nodes_to_check.size());
    bool all_nodes_alive = false;
    int consecutive_success_count = 0;  // 连续成功计数器
    const int required_consecutive = 3;  // 需要连续成功的次数
    while(retry_count < max_retries){
        try {
            auto node_names = this->get_node_graph_interface()->get_node_names();
            int found_count = 0;
            // 检查每个目标节点
            for (const auto& target_node : nodes_to_check) {
                bool node_found = false;
                // 在系统节点列表中查找目标节点
                for (const auto& name : node_names) {
                    if (name == target_node || name.find(target_node) != std::string::npos) {
                        node_found = true;  break; 
                    }
                }
                if (node_found) {
                    if (!node_status[target_node]) {
                        RCLCPP_INFO(this->get_logger(), "✅ 节点已上线：%s", target_node.c_str());
                    }
                    node_status[target_node] = true;
                    node_fail_count[target_node] = 0;
                    found_count++;
                } else {
                    node_fail_count[target_node]++;
                    if (node_fail_count[target_node] >= 3 && node_status[target_node]) {
                        RCLCPP_WARN(this->get_logger(), "⚠️ 节点失联：%s (连续失败 %d 次)", target_node.c_str(), node_fail_count[target_node]);
                        node_status[target_node] = false;
                    }
                }
            }
            if (found_count == (int)nodes_to_check.size()) {
                consecutive_success_count++;
                RCLCPP_INFO(this->get_logger(), "✅ 所有 %zu 个关键节点均存活 (连续成功 %d/%d)", 
                            nodes_to_check.size(), consecutive_success_count, required_consecutive);
                if (consecutive_success_count >= required_consecutive) {
                    RCLCPP_INFO(this->get_logger(), "✅✅✅ 所有关键节点已连续 %d 次检验通过", required_consecutive);
                    all_nodes_alive = true;
                    break;
                }
            } else {
                consecutive_success_count = 0;  // 有节点未存活，重置连续计数器
            }
            if (retry_count > 0 && retry_count % 10 == 0) {
                std::string missing_nodes;
                for (const auto& node_name : nodes_to_check) {
                    if (!node_status[node_name]) {
                        if (!missing_nodes.empty()) missing_nodes += ", ";
                        missing_nodes += node_name;
                    }
                }
                RCLCPP_WARN(this->get_logger(), "⏳ 等待节点上线 (%d/%ds) - 缺失：%s", 
                            retry_count, max_retries, missing_nodes.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ 节点检查异常：%s", e.what());
            msg = std::string("节点检查异常：") + e.what();
            return false;
        }
        retry_count++;
        std::this_thread::sleep_for(retry_interval);
    }
    if (!all_nodes_alive) {
        std::stringstream ss;
        ss << "以下关键节点未找到 (" << retry_count << "s): ";
        bool first = true;
        for (const auto& node_name : nodes_to_check) {
            if (!node_status[node_name]) {
                if (!first) ss << ", ";
                ss << node_name;
                first = false;
            }
        }
        msg = ss.str();
        RCLCPP_ERROR(this->get_logger(), "❌ %s", msg.c_str());
        return false;
    }
    {   //  重新获取token session
        std::string token , pub_key , priv_key, no_key, msg;
        if(request_auth_info_via_service_async(token, pub_key, priv_key, no_key, msg)){
            token_result_.jwt_token = token;
            token_result_.success = true;
            device_info_.session_public_key = pub_key;
            device_info_.session_private_key = priv_key;
            device_info_.session_key_no = no_key;
        }
    }
    return true;
}

bool ota_agent::OtaAgentNode::read_hex_get_version(const std::string& hex_file_path,std::string& version, std::string& msg) {
    RCLCPP_INFO(this->get_logger(), "🔍 开始解析 HEX 文件：%s", hex_file_path.c_str());
    // 检查文件是否存在
    if (!std::filesystem::exists(hex_file_path)) {
        msg = "HEX 文件不存在：" + hex_file_path;
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return false;
    }
    // 打开文件
    std::ifstream file(hex_file_path);
    if (!file.is_open()) {
        msg = "无法打开 HEX 文件：" + hex_file_path;
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return false;
    }
    std::string line;
    bool found = false;
    std::string target_line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == ';') {   continue;   }
        if (line.length() > 0 && line[0] == ':') { line = line.substr(1); }
        std::string record_type = line.substr(2, 4);
        if (record_type == "0188" || record_type == "C188") { found = true;break; }
    }
    if(!found) {
        msg = "无法打开 HEX 文件：" + hex_file_path;
        RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        return false;
    }
    file.close();
    int length = hex_str_to_byte(line.substr(0, 2), 0);
    std::string data_str = line.substr(8, length * 2);
    std::string version_str = data_str.substr(2, 2) + data_str.substr(0, 2);
    
    uint8_t major = (hex_char_to_int(version_str[0]) << 4) | hex_char_to_int(version_str[1]); 
    uint8_t minor = (hex_char_to_int(version_str[2]) << 4) | hex_char_to_int(version_str[3]); 
    std::stringstream vss;
    vss << "V" << (int)major << "." << std::setfill('0') << std::setw(2) << (int)minor;
    version = vss.str();
    return true;
}


bool ota_agent::OtaAgentNode::prepare_upgrade_task(std::string& err){
    {
        std::lock_guard<std::mutex> lock(car_info_mutex_);
        if(latest_car_info_.speedkmh > 0){
            err = "升级时车辆速度不为0";
            RCLCPP_WARN(this->get_logger(), "%s", err.c_str());
            return false;
        }
        if(latest_car_info_.electricquantity < 80){
            err = "升级时电量低于80%";
            RCLCPP_WARN(this->get_logger(), "%s", err.c_str());
            return false;
        }
    }
    return true;
}
bool ota_agent::OtaAgentNode::download_and_stage(const std::string& url,
                                                 const std::string& up_id,
                                                 const vehicle::ota::up::UpgradePackage& pkg,
                                                 std::string& staged_path,
                                                 std::string& err) {
    if (!wait_if_paused_or_cancelled("download")) { err = "task_cancelled"; return false;}
    if (url.empty()) { err = "empty url"; return false;}
    staged_path = ota_tmp_file_path_ + "/ota_" + up_id + "_" + pkg.ecu() + ".zip";
    if (!oss_mgr_->download_file(url, staged_path, err)) {
        RCLCPP_WARN(this->get_logger(), "下载失败: %s", err.c_str());
        return false;
    }
    std::string actual_sha256 = compute_sha256_hex(staged_path, err);
    std::string expected_sha256 = pkg.sha256();
    if (!expected_sha256.empty()){
        std::transform(expected_sha256.begin(), expected_sha256.end(), expected_sha256.begin(),
                      [](unsigned char c) { return std::tolower(c); });
    }
    
    if (actual_sha256 != expected_sha256) {
        err = "SHA256 校验未通过";
        RCLCPP_INFO(this->get_logger(), "%s", err.c_str());
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "下载成功: %s", staged_path.c_str());
    return true;
}
std::string ota_agent::OtaAgentNode::extract_version(const std::string &version_str){
    static const std::regex version_regex(R"([vV]?(\d+(?:\.\d+)*(?:[-+]\w+)?))");
    std::smatch match;
    if (std::regex_search(version_str, match, version_regex)){
        return match[1].str();
    }
    return version_str;
}
int ota_agent::OtaAgentNode::compare_versions(const std::string &a, const std::string &b){
    auto parse_version = [](const std::string& s) -> std::vector<int> {
        std::vector<int> parts;
        std::string token;
        std::istringstream iss(s.substr(0, s.find_first_of("-+")));
        while (std::getline(iss, token, '.')) {
            size_t i = 0; int v = 0;
            while (i < token.size() && std::isdigit(token[i])) ++i;
            if (i > 0) try { v = std::stoi(token.substr(0, i)); } catch (...) {}
            parts.push_back(v);
        }
        while (parts.size() < 3) parts.push_back(0);
        return parts;
    };
    
    auto pa = parse_version(a), pb = parse_version(b);
    for (size_t i = 0; i < std::max(pa.size(), pb.size()); ++i) {
        int va = (i < pa.size()) ? pa[i] : 0;
        int vb = (i < pb.size()) ? pb[i] : 0;
        if (va != vb) return (va < vb) ? -1 : 1;
    }
    return 0;
}
bool ota_agent::OtaAgentNode::retain_top_versions(const std::string &target_dir){
    namespace fs = std::filesystem;
    size_t keep_count = 3;
    try {
        if (!fs::exists(target_dir) || !fs::is_directory(target_dir)) {
            RCLCPP_ERROR(this->get_logger(), "目录不存在：%s", target_dir.c_str());
            return false;
        }
        std::vector<std::pair<std::string, std::string>> folders;
        for (const auto& entry : fs::directory_iterator(target_dir)) {
            if (entry.is_directory()) {
                std::string name = entry.path().filename().string();
                std::string ver = extract_version(name);
                if (!ver.empty()) {
                    RCLCPP_INFO(this->get_logger(), "发现版本文件夹：%s (版本：%s)", name.c_str(), ver.c_str());
                    folders.push_back({name, ver});
                }
            }
        }
        if (folders.empty()) {
            RCLCPP_INFO(this->get_logger(), "未发现任何版本文件夹");
            return false;
        }
        
        if (folders.size() <= keep_count) {
            RCLCPP_INFO(this->get_logger(), "文件夹数量 (%zu) <= 保留数 (%zu)，无需删除", folders.size(), keep_count);
            return false;
        }
        
        // 按版本号降序排序
        std::sort(folders.begin(), folders.end(), [this](const auto& a, const auto& b) {
            return this->compare_versions(a.second, b.second) > 0;
        });
        
        RCLCPP_INFO(this->get_logger(), "\n=== 版本排序 (从高到低) ===");
        for (size_t i = 0; i < folders.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "%zu. %s (v%s)", i + 1, folders[i].first.c_str(), folders[i].second.c_str());
        }
        
        // 删除排名靠后的文件夹
        size_t deleted = 0;
        for (size_t i = keep_count; i < folders.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "删除 [%zu]: %s", i + 1, folders[i].first.c_str());
            try {
                fs::remove_all(folders[i].first);
                ++deleted;
            }
            catch (const fs::filesystem_error& e) { 
                RCLCPP_ERROR(this->get_logger(), "删除失败：%s", e.what()); 
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "\n=== 清理完成 ===\n保留：%zu 个，删除：%zu 个", keep_count, deleted);
        RCLCPP_INFO(this->get_logger(), "保留的文件夹:");
        for (size_t i = 0; i < keep_count; ++i) {
            RCLCPP_INFO(this->get_logger(), "  - %s", folders[i].first.c_str());
        }
        
        return true;
    }
    catch (const fs::filesystem_error& e) {
        RCLCPP_ERROR(this->get_logger(), "文件系统错误：%s", e.what());
        return false;
    }
}

bool ota_agent::OtaAgentNode::validate_and_extract_zip(const std::string& zip_path, const std::string& dest_dir, std::string& err) {
    namespace fs = std::filesystem;
    try {
        if (fs::exists(dest_dir)) {  fs::remove_all(dest_dir); }
        fs::create_directories(dest_dir);
        std::string cmd = "unzip -o -qq " + zip_path + " -d " + dest_dir;
        int rc = std::system(cmd.c_str());
        if (rc != 0) { err = "unzip failed rc=" + std::to_string(rc); return false;}
    } catch (const std::exception& e) {
        err = std::string("prepare dest dir failed: ") + e.what();
        return false;
    }
    const fs::path manifest_path = fs::path(dest_dir) / "manifest.json";
    const fs::path signature_path = fs::path(dest_dir) / "signature.sig";
    if (!fs::exists(manifest_path)) { err = "manifest.json missing"; return false; }
    if (!fs::exists(signature_path)) { err = "signature.sig missing"; return false; }

    std::string manifest_content;
    {
        std::ifstream ifs(manifest_path, std::ios::binary);
        if (!ifs.is_open()) { err = "open manifest failed"; return false; }
        manifest_content.assign((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    }
    std::string signature_content;
    {
        std::ifstream ifs(signature_path, std::ios::binary);
        if (!ifs.is_open()) { err = "open signature failed"; return false; }
        signature_content.assign((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    }
    if (integrity_signature_pubkey_path_.empty()) {
        err = "signature pubkey missing";
        return false;
    }
    if (!verify_signature(manifest_content, signature_content, integrity_signature_pubkey_path_, integrity_signature_algorithm_, err)) {
        err = std::string("signature verify failed: ") + err;
        return false;
    }
    json manifest_json = json::parse(manifest_content, nullptr, false);
    if (!manifest_json.is_object() || !manifest_json.contains("files") || !manifest_json["files"].is_array()) {
        err = "manifest format invalid";
        return false;
    }
    const auto files = manifest_json["files"];
    if (files.empty()) { err = "manifest files empty"; return false; }
    for (const auto& file : files) {
        std::string file_path = dest_dir + "/" + file["file"].get<std::string>();
        std::string actual_sha256 = compute_sha256_hex(file_path, err);
        std::string expected_sha256 = file["sha256"].get<std::string>();
        if (!expected_sha256.empty()){
            std::transform(expected_sha256.begin(), expected_sha256.end(), expected_sha256.begin(),
                        [](unsigned char c) { return std::tolower(c); });
        }
        if(actual_sha256 != expected_sha256){
            err = file["file"].get<std::string>() + " sha256 校验未通过";
            return false;
        }
    }
    return true;
}

std::string ota_agent::OtaAgentNode::compute_sha256_hex(const std::string& path, std::string& err) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
        err = "open file failed";
        return "";
    }

    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    if (!ctx) {
        err = "EVP_MD_CTX_new failed";
        ifs.close();
        return "";
    }
    if (EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) != 1) {
        err = "EVP_DigestInit_ex failed";
        EVP_MD_CTX_free(ctx);
        ifs.close();
        return "";
    }
    char buf[4096];
    while (ifs.good()) {
        ifs.read(buf, sizeof(buf));
        std::streamsize n = ifs.gcount();
        if (n > 0) {
            if (EVP_DigestUpdate(ctx, buf, static_cast<size_t>(n)) != 1) {
                err = "EVP_DigestUpdate failed";
                EVP_MD_CTX_free(ctx);
                ifs.close();
                return "";
            }
        }
    }
    if (ifs.bad()) {
        err = "read error";
        EVP_MD_CTX_free(ctx);
        return "";
    }

    unsigned char md[EVP_MAX_MD_SIZE];
    unsigned int md_len = 0;
    if (EVP_DigestFinal_ex(ctx, md, &md_len) != 1) {
        err = "EVP_DigestFinal_ex failed";
        EVP_MD_CTX_free(ctx);
        return "";
    }
    EVP_MD_CTX_free(ctx);

    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (unsigned int i = 0; i < md_len; ++i) {
        oss << std::setw(2) << static_cast<int>(md[i]);
    }
    return oss.str();
}


bool ota_agent::OtaAgentNode::verify_signature(const std::string& payload,
                          const std::string& base64_sig,
                          const std::string& pubkey_path,
                          const std::string& algorithm,
                          std::string& err) {
    if (pubkey_path.empty()) {
        err = "pubkey_missing";
        return false;
    }
    std::ifstream ifs(pubkey_path);
    if (!ifs.good()) {
        err = pubkey_path + ": pubkey_file_not_found";
        return false;
    }
    std::string pubkey_content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    ifs.close();
    
    std::string b64_key;
    b64_key.reserve(pubkey_content.size());
    for (char c : pubkey_content) {
        if (!std::isspace(static_cast<unsigned char>(c))) {
            b64_key.push_back(c);
        }
    }

    if (b64_key.empty()) { err = "empty_pubkey"; return false;}
    
    std::vector<unsigned char> raw_key;
    raw_key.resize(((b64_key.size() + 3) / 4) * 3);
    int decoded_len = EVP_DecodeBlock(raw_key.data(), 
                                       reinterpret_cast<const unsigned char*>(b64_key.data()), 
                                       static_cast<int>(b64_key.size()));
    
    if (decoded_len < 0) {
        err = "base64_decode_failed";
        return false;
    }
    size_t pad = 0;
    if (!b64_key.empty()) {
        if (b64_key[b64_key.size() - 1] == '=') pad++;
        if (b64_key.size() > 1 && b64_key[b64_key.size() - 2] == '=') pad++;
    }
    raw_key.resize(static_cast<size_t>(decoded_len) - pad);
    auto upper_algo = algorithm;
    std::transform(upper_algo.begin(), upper_algo.end(), upper_algo.begin(), 
                   [](unsigned char c){ return static_cast<char>(std::toupper(c)); });
    EVP_PKEY* pkey = nullptr;
    if (upper_algo == "ED25519") {
        if (raw_key.size() != 32) {
            err = "invalid_ed25519_key_length";
            return false;
        }
        pkey = EVP_PKEY_new_raw_public_key(EVP_PKEY_ED25519, nullptr, 
                                            raw_key.data(), raw_key.size());
        if (!pkey) {
            err = "create_ed25519_pubkey_failed";
            return false;
        }
    } else {
        err = "unsupported_algorithm";
        return false;
    }
    std::vector<unsigned char> sig;
    sig.resize(((base64_sig.size() + 3) / 4) * 3);
    int sig_decoded_len = EVP_DecodeBlock(sig.data(), 
                                           reinterpret_cast<const unsigned char*>(base64_sig.data()), 
                                           static_cast<int>(base64_sig.size()));
    if (sig_decoded_len < 0) {
        EVP_PKEY_free(pkey);
        err = "base64_decode_failed";
        return false;
    }
    
    size_t sig_pad = 0;
    if (!base64_sig.empty()) {
        if (base64_sig[base64_sig.size() - 1] == '=') sig_pad++;
        if (base64_sig.size() > 1 && base64_sig[base64_sig.size() - 2] == '=') sig_pad++;
    }
    sig.resize(static_cast<size_t>(sig_decoded_len) - sig_pad);
    
    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    if (!ctx) {
        EVP_PKEY_free(pkey);
        err = "EVP_MD_CTX_new_failed";
        return false;
    }
    
    if (EVP_DigestVerifyInit(ctx, nullptr, nullptr, nullptr, pkey) != 1) {
        EVP_MD_CTX_free(ctx);
        EVP_PKEY_free(pkey);
        err = "DigestVerifyInit_failed";
        return false;
    }
    
    int rc = EVP_DigestVerify(ctx, sig.data(), sig.size(),
                               reinterpret_cast<const unsigned char*>(payload.data()), 
                               payload.size());
    
    EVP_MD_CTX_free(ctx);
    EVP_PKEY_free(pkey);
    if (rc != 1) {
        err = "signature_invalid";
        return false;
    }
    return true;
}

} // namespace ota_agent

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ota_agent::OtaAgentNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}