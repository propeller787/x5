#include "ota_agent_node.hpp"
#include "ota_agent/can_transport_socketcan.hpp"
#include "ota_agent/can_transport_mock.hpp"
#include "ota_agent/can_id_policy.hpp"
#include "utils/common.h"
#include <openssl/evp.h>  // 确保添加此头文件（如果没有的话）
#include <openssl/pem.h>
#include <openssl/sha.h>
#include <mosquitto.h>
#include <cmath>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <condition_variable>
#include <random>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <cctype>
#include <openssl/md5.h>
#include <atomic>
#include <set>
#include <filesystem>
#include <cstdlib>
#include <optional>
#include <array>
namespace ota_agent {

/**
 * @brief OTA Agent Node构造函数
 * 初始化OTA升级代理节点，包括参数声明、管理器初始化、MQTT连接等
 */
OtaAgentNode::OtaAgentNode() : rclcpp::Node("ota_agent_node") {
    // 声明和加载ROS参数
    declare_ros_parameters();
    load_ros_parameters();
    // 构建车辆专用ACK主题：vehicle/ota/<model>/<sn>/<protocol>/ack
    ack_topic_ = "vehicle/ota/" + model_str_ + "/" + vin_ + "/" + protocol_version_ + "/ack";
    // 初始化MQTT和签名管理器
    initialize_managers();
    
    // 初始化本地服务主题发布器（与云端解耦）
    task_status_pub_ = this->create_publisher<std_msgs::msg::Int32>("/service/upgrade/task_status", rclcpp::QoS(30));
    upgrade_status_pub_ = this->create_publisher<std_msgs::msg::Int32>("/service/upgrade/upgrade_status", rclcpp::QoS(30));
    upgrade_message_pub_ = this->create_publisher<std_msgs::msg::String>("/service/upgrade/message", rclcpp::QoS(30));
    upgrade_progress_pub_ = this->create_publisher<std_msgs::msg::Float64>("/service/upgrade/progress", rclcpp::QoS(30));
    boot_broadcast_pub_ = this->create_publisher<std_msgs::msg::String>(boot_broadcast_topic_, rclcpp::QoS(10).best_effort());
    version_broadcast_pub_ = this->create_publisher<std_msgs::msg::String>(version_broadcast_topic_, rclcpp::QoS(10).best_effort());

    // 创建ROS服务接口（支持可插拔数据源）
    get_version_srv_ = this->create_service<interface::srv::GetVersion>(
        "/service/upgrade/current_version",
        std::bind(&OtaAgentNode::handle_get_version, this, std::placeholders::_1, std::placeholders::_2)
    );
    get_upgrade_content_srv_ = this->create_service<interface::srv::GetUpgradeContent>(
        "/service/upgrade/ecu_content",
        std::bind(&OtaAgentNode::handle_get_upgrade_content, this, std::placeholders::_1, std::placeholders::_2)
    );
    get_package_size_srv_ = this->create_service<interface::srv::GetPackageSize>(
        "/service/upgrade/package_size",
        std::bind(&OtaAgentNode::handle_get_package_size, this, std::placeholders::_1, std::placeholders::_2)
    );
    // 根据配置选择CAN传输实现（SocketCAN或Mock）
    if (can_use_socketcan_) {
        can_transport_ = std::make_shared<SocketCanTransport>(can_ifname_, this->get_logger());
    } else {
        can_transport_ = std::make_shared<MockCanTransport>(this->get_logger(), mock_can_outcome_, mock_can_step_delay_ms_, mock_can_fail_after_step_);
    }
    // 构建控制器通信层，封装CAN通信细节
    if (can_transport_) {
        ControllerCommConfig cfg;
        cfg.domain_can_id = static_cast<uint32_t>(can_id_domain_);
        cfg.upper_can_id  = static_cast<uint32_t>(can_id_upper_);
        cfg.lower_can_id  = static_cast<uint32_t>(can_id_lower_);
        cfg.frame_payload_bytes = 8;
        ctrl_comm_ = std::make_shared<CanControllerComm>(can_transport_, cfg, this->get_logger());
        // 使用按图片定义的ID策略，彻底解耦 ID 映射
        if (auto concrete = std::dynamic_pointer_cast<CanControllerComm>(ctrl_comm_)) {
            concrete->set_id_policy(std::make_shared<PictureCanIdPolicy>());
        }
        // 设置CAN反馈事件回调处理器
        ctrl_comm_->set_feedback_callback([this](const FeedbackEvent& ev){
            if (current_up_id_.empty()) return;


            // 反馈事件类型转换和日志记录
            std::string ev_type_str;
            switch(ev.type) {
                case FeedbackEvent::Type::Progress: ev_type_str = "进度更新"; break;
                case FeedbackEvent::Type::Success: ev_type_str = "刷写成功"; break;
                case FeedbackEvent::Type::Failure: ev_type_str = "刷写失败"; break;
                default: ev_type_str = "未知事件";
            }
            RCLCPP_INFO(this->get_logger(), 
                "[CAN-FEEDBACK] 事件类型=%s | 进度=%d%% | 原因=%s | 当前升级ID=%s | ECU=%s",
                ev_type_str.c_str(), ev.progress, ev.reason.c_str(),
                current_up_id_.c_str(), current_ecu_id_.c_str());
                
            const std::string pkg_id_in_ctx = current_package_id_.empty() ? current_up_id_ : current_package_id_;
            if (ev.type == FeedbackEvent::Type::Progress) {
                publish_ota_ack(current_up_id_, 1, 0, 0, {{pkg_id_in_ctx, current_ecu_id_, "", current_version_, 3, 0, "", "flashing_" + std::to_string(ev.progress) + "%"}});
                // 本地进度主题（0-100）
                publish_upgrade_progress(static_cast<double>(ev.progress));
                if (pull_watchdog_active_.load()) {
                    pull_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(pull_timeout_sec_);
                }
            } else if (ev.type == FeedbackEvent::Type::Success) {
                publish_ota_ack(current_up_id_, 1, 0, 0, {{pkg_id_in_ctx, current_ecu_id_, "", current_version_, 10, 0, "", "flash_success"}});
                publish_upgrade_progress(100.0);
                publish_upgrade_status(0); // 可升级（完成）
                pull_watchdog_active_.store(false);
                // 清理旧的临时固件，仅保留最近一版
                prune_old_staged_bins(last_staged_path_);
            } else if (ev.type == FeedbackEvent::Type::Failure) {
                publish_ota_ack(current_up_id_, 5, 1, 5, {{pkg_id_in_ctx, current_ecu_id_, "", current_version_, 11, 5, "", std::string("flash_failed:") + ev.reason}});
                publish_upgrade_message(std::string("flash_failed:") + ev.reason);
                publish_upgrade_status(1); // 不可升级（失败）
                pull_watchdog_active_.store(false);
            }
        });
        ctrl_comm_->add_raw_listener([this](const CanFrame& f){ this->on_can_frame_rx(f); });
    }
    // 创建Pull模式看门狗定时器（1秒间隔）
    pull_watchdog_timer_ = this->create_wall_timer(1000ms, [this]{ this->pull_watchdog_tick(); });
    // 创建ACK发布重试定时器
    ack_retry_timer_ = this->create_wall_timer(std::chrono::milliseconds(std::max(100, ack_retry_interval_ms_)), [this]{ this->drain_ack_queue(); });
    // 周期广播定时器（完全解耦，基于ROS主题输出，设备侧/桥接侧自行消费）
    start_boot_broadcast_timer();
    start_version_broadcast_timer();
    // 设置完整性校验回调函数
    setup_integrity_callbacks();
    
    // 连接MQTT并订阅相关主题
    connect_and_subscribe();
    // 触发模拟CAN发送测试（如果启用）
    simulate_can_file_send();
    // 设置HTTP兜底轮询：2小时间隔+随机偏移，避免服务器压力集中
    {
        std::mt19937 rng{std::random_device{}()};
        std::uniform_int_distribution<int> jitter_sec(0, 600); // 0~10min 随机抖动
        const auto period = 2h + std::chrono::seconds(jitter_sec(rng));
        RCLCPP_INFO(this->get_logger(), "HTTP 兜底轮询周期：%ld 秒", std::chrono::duration_cast<std::chrono::seconds>(period).count());
        http_poll_timer_ = this->create_wall_timer(period, [this]{
            try {
                (void)fetch_offline_upgrades();
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "HTTP 兜底拉取异常: %s", e.what());
            }
        });
    }

    // 启动后校验域控版本，如有变更则向上下控分发安装包
    check_boot_version_and_push();
}

OtaAgentNode::~OtaAgentNode() {
    if (can_transport_) {
        RCLCPP_INFO(this->get_logger(), "Shutting down CAN transport");
        can_transport_->close();
    }
}

/**
 * @brief 构建HTTP请求通用头部
 * @param body_json 请求体JSON字符串
 * @return 包含签名和协议信息的HTTP头部
 */
cpr::Header OtaAgentNode::build_common_headers(const std::string& body_json) {
    const std::string gtime = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    const std::string nonce = make_nonce();
    cpr::Header headers{
        {"Content-Type", "application/json"},
        {"Gversion", protocol_version_},
        {"Glanguage", http_language_},
        {"GClient", http_client_},
        {"GApp-Version", software_version_},
        {"Gtime", gtime},
        {"Gnonce", nonce}
    };
    // 按协议8.5规范构建签名原文
    std::string sign_source = headers["Gversion"] + headers["Glanguage"] + headers["GClient"] +
                              headers["GApp-Version"] + headers["Gtime"] + headers["Gnonce"] + body_json;
    headers["GSign"] = (signature_mgr_ ? signature_mgr_->sign_data(sign_source) : std::string(""));
    return headers;
}

/**
 * @brief 生成16位随机字符串作为nonce
 * @return 随机nonce字符串
 */
std::string OtaAgentNode::make_nonce() const {
    static const char* cs = "abcdefghijklmnopqrstuvwxyz0123456789";
    std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<int> dist(0,35);
    std::string s; s.reserve(16);
    for (int i=0;i<16;++i) s.push_back(cs[dist(rng)]);
    return s;
}

std::string OtaAgentNode::make_signature_payload(const std::string& body_json, const std::string& gtime, const std::string& nonce) const {
    return vin_ + "|" + model_str_ + "|" + gtime + "|" + nonce + "|" + body_json;
}

/**
 * @brief 构建车辆专用OTA升级主题
 * @return 格式：vehicle/ota/<model>/<sn>/<protocol>/up
 */
std::string OtaAgentNode::build_all_topic() const {
    // 平台按灰度圈选SN后逐台下发，车辆仅订阅自身定向主题
    // vehicle/ota/<model>/<sn>/<protocol>/up
    return "vehicle/ota/" + model_str_ + "/" + vin_ + "/" + protocol_version_ + "/up";
}
std::string OtaAgentNode::build_change_topic() const {
    // Spec: vehicle/ota/change/<设备型号>/<设备id>/<协议版本>/notic
    return "vehicle/ota/change/" + model_str_ + "/" + vin_ + "/" + protocol_version_ + "/notic";
}
std::string OtaAgentNode::build_tag_topic(const std::string& tag_code, const std::string& value_code) const {
    // Spec: vehicle/ota/tag/<设备型号>/<标签编号>/<标签值编号>/<协议版本>/up
    return "vehicle/ota/tag/" + model_str_ + "/" + tag_code + "/" + value_code + "/" + protocol_version_ + "/up";
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
            publish_upgrade_message("invalid_ota_payload");
            publish_upgrade_status(1);
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

        up.set_task_id(get_str("task_id"));
        up.set_task_status(get_i("task_status", 0));
        up.set_timestamp(get_ll("timestamp", 0));
        up.set_exec_time(get_ll("exec_time", 0));
        parsed_task_id = up.task_id();

        // 策略可选字段
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
                pre->set_exercise_status(p.value("exercise_status", 0));
                pre->set_min_network_quality(p.value("min_network_quality", 0));
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
                auto* m = up.add_packages();
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

        RCLCPP_INFO(this->get_logger(), "JSON兼容路径生效：映射完成，包数=%d", up.packages_size());
    }

    // 若带有 task_id，订阅任务状态变更并通过HTTP确认任务状态
    if (!up.task_id().empty()) {
        current_task_id_ = up.task_id();
        subscribe_task_status(up.task_id());
        int task_status = 0;
        if (fetch_task_status_http(up.task_id(), task_status)) {
            current_task_status_.store(task_status);
            if (task_status == 1) {
                RCLCPP_WARN(this->get_logger(), "任务被暂停，暂不执行升级，task_id=%s", up.task_id().c_str());
                return;
            } else if (task_status == 2) {
                RCLCPP_WARN(this->get_logger(), "任务被取消，跳过执行，task_id=%s", up.task_id().c_str());
                return;
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "===== OTA消息解析成功 =====");
    RCLCPP_INFO(this->get_logger(), "task_id: %s", up.task_id().c_str());
    RCLCPP_INFO(this->get_logger(), "task_status: %d", up.task_status());
    RCLCPP_INFO(this->get_logger(), "timestamp: %ld", up.timestamp());
    RCLCPP_INFO(this->get_logger(), "exec_time: %ld", up.exec_time());
    RCLCPP_INFO(this->get_logger(), "包数量: %d", up.packages_size());
    for (int i = 0; i < up.packages_size(); ++i) {
        const auto& pkg = up.packages(i);
        RCLCPP_INFO(this->get_logger(), "  包-%d:", i+1);
        RCLCPP_INFO(this->get_logger(), "    package_id: %s", pkg.package_id().c_str());
        RCLCPP_INFO(this->get_logger(), "    ecu: %s", pkg.ecu().c_str());
        RCLCPP_INFO(this->get_logger(), "    target_version: %s", pkg.target_version().c_str());
        RCLCPP_INFO(this->get_logger(), "    download_url: %s", pkg.download_url().c_str());
        RCLCPP_INFO(this->get_logger(), "    sha256: %s", pkg.sha256().c_str());
    }
    RCLCPP_INFO(this->get_logger(), "===========================");
    const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    if (up.exec_time() > now_ms) {
        const int64_t delay_ms = std::max<int64_t>(up.exec_time() - now_ms, 1);
        {
            std::lock_guard<std::mutex> guard(exec_timer_mutex_);
            pending_up_ = up;
            exec_timer_ = this->create_wall_timer(std::chrono::milliseconds(delay_ms), [this] {
                std::optional<vehicle::ota::up::VehicleOtaUp> up_copy;
                {
                    std::lock_guard<std::mutex> g(exec_timer_mutex_);
                    if (!pending_up_) return;
                    up_copy = pending_up_;
                    pending_up_.reset();
                    exec_timer_.reset();
                }
                RCLCPP_INFO(this->get_logger(), "exec_time 到达，开始执行任务 task_id=%s", up_copy->task_id().c_str());
                (void)process_upgrade_task(*up_copy);
            });
        }
        RCLCPP_INFO(this->get_logger(), "任务按 exec_time 延后执行：%ld ms 后触发", delay_ms);
        return;
    }
    if (up.exec_time() > 0 && up.exec_time() <= now_ms) {
        RCLCPP_INFO(this->get_logger(), "exec_time 已过期，立即执行 task_id=%s", up.task_id().c_str());
    }
    // 收到任务，标记为处理中
    publish_task_status(1);
    publish_upgrade_status(0);
    (void)process_upgrade_task(up);
}

void OtaAgentNode::handle_tag_change_msg(const std::string& payload) {
    vehicle::tag::VehicleTagChange msg;
    if (!msg.ParseFromString(payload)) {
        RCLCPP_ERROR(this->get_logger(), "TagChange解析失败");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "收到标签变更，刷新订阅");
    fetch_device_tags();
    subscribe_ota_topics();
}

/**
 * @brief 声明所有ROS参数及其默认值
 * 包括网络配置、CAN配置、完整性校验等参数
 */
void OtaAgentNode::declare_ros_parameters() {
    // 基础配置参数
    this->declare_parameter<std::string>("vin", "");
    this->declare_parameter<int>("model", 0);
    this->declare_parameter<std::string>("model_str", "default");
    this->declare_parameter<std::string>("api_base_url", "");
    this->declare_parameter<std::string>("session_key", "");
    this->declare_parameter<std::string>("local.bin_override", "");
    this->declare_parameter<std::string>("mqtt.broker_url", "tcp://localhost:1883");
    this->declare_parameter<std::string>("mqtt.client_id", "ota_agent");
    this->declare_parameter<std::string>("mqtt.username", "");
    this->declare_parameter<std::string>("mqtt.password", "");
    this->declare_parameter<std::string>("mqtt.ca_cert", "");
    this->declare_parameter<std::string>("mqtt.cipher", "");
    this->declare_parameter<bool>("mqtt.insecure_skip_verify", false);
    this->declare_parameter<int>("mqtt.retry_attempts", 3);
    this->declare_parameter<double>("mqtt.retry_delay_base", 2.0);
    this->declare_parameter<int>("http.timeout_ms", 5000);
    // HTTP协议相关参数
    this->declare_parameter<std::string>("protocol.version", "1.0");
    this->declare_parameter<std::string>("http.language", "zh-CN");
    this->declare_parameter<std::string>("http.client", "VehicleGateway/ota_agent");
    this->declare_parameter<std::string>("software.version", "1.0.0");
    this->declare_parameter<std::string>("hardware.version", "");
    this->declare_parameter<std::string>("domain.version_file", "/var/lib/ota_agent/domain_version.json");
    this->declare_parameter<std::string>("controller.package_path", "");
    this->declare_parameter<bool>("controller.boot_push_enabled", false);
    this->declare_parameter<std::string>("ecu.hardware_sn_map_json", "{}");
    this->declare_parameter<std::string>("ecu.model_map_json", "{}");
    this->declare_parameter<bool>("boot.broadcast_enabled", false);
    this->declare_parameter<int>("boot.broadcast_period_ms", 100);
    this->declare_parameter<std::string>("boot.broadcast_topic", "/ota/boot_broadcast");
    this->declare_parameter<std::string>("boot.broadcast_payload", "request_upgrade");
    this->declare_parameter<bool>("version.broadcast_enabled", false);
    this->declare_parameter<int>("version.broadcast_period_ms", 1000);
    this->declare_parameter<std::string>("version.broadcast_topic", "/ota/version_broadcast");
    this->declare_parameter<bool>("can.use_socketcan", false);
    this->declare_parameter<std::string>("can.ifname", "can0");
    this->declare_parameter<int>("can.send_retries", 3);
    this->declare_parameter<int>("can.retry_delay_ms", 50);
    this->declare_parameter<int>("can.id.domain", 0x3588);
    this->declare_parameter<int>("can.id.upper",  0x0510);
    this->declare_parameter<int>("can.id.lower",  0x0520);
    this->declare_parameter<bool>("flash.pull_mode", false);
    this->declare_parameter<bool>("flash.pull_to_controllers_only", false);
    this->declare_parameter<int>("flash.pull_timeout_sec", 600);
    this->declare_parameter<std::string>("flash.self_flash_cmd", "");
    this->declare_parameter<std::string>("self.reboot_cmd", "");
    this->declare_parameter<std::string>("self.verify_cmd", "");
    this->declare_parameter<std::string>("self.rollback_image_path", "");
    this->declare_parameter<std::string>("mock_can.outcome", "success");
    this->declare_parameter<int>("mock_can.fail_after_step", 2);
    this->declare_parameter<int>("mock_can.step_delay_ms", 100);
    this->declare_parameter<std::string>("signature.private_key", "");
    this->declare_parameter<std::string>("signature.public_key", "");
    // 完整性校验配置参数
    this->declare_parameter<std::string>("integrity.sha256_map_json", "{}");
    this->declare_parameter<std::string>("integrity.signature.map_json", "{}");
    this->declare_parameter<std::string>("integrity.signature.pubkey_path", "");
    this->declare_parameter<std::string>("integrity.signature.algorithm", "RSA");
    this->declare_parameter<std::string>("integrity.signature.payload_template", "{version}|{url}|{md5}|{sha256}");
    this->declare_parameter<std::string>("integrity.l3.expected_versions_json", "{}");
    this->declare_parameter<std::vector<std::string>>("integrity.l3.required_nodes", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("integrity.l4.required_nodes", std::vector<std::string>{"cloud_manager", "common", "interface", "ota_agent", "postback"});
    // 主线版本管理参数
    this->declare_parameter<std::string>("mainline.version", "");
    this->declare_parameter<std::string>("mainline.config_json", "");
    // DBC发布相关参数
    this->declare_parameter<bool>("dbc.enabled", false);
    this->declare_parameter<int>("dbc.audio_type", 1);
    this->declare_parameter<int>("dbc.publish_count", 1);
    this->declare_parameter<int>("dbc.publish_interval_ms", 200);
    this->declare_parameter<int>("isotp.chunk_size", 7); // ISO-TP 连续帧有效载荷
    // 在 CAN 发送失败时自动上报失败ACK
    this->declare_parameter<bool>("report.can_tx_fail_ack", true);

    // 模拟CAN测试参数
    this->declare_parameter<bool>("test.can.simulate_send", true);          // 是否触发模拟发送
    this->declare_parameter<std::string>("test.can.simulate_bin_path", "");  // 模拟 bin 路径
    this->declare_parameter<int>("test.can.target_id", 0x3588);              // 模拟目标 CAN ID（使用有符号整型避免ROS参数歧义）

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
    this->get_parameter("local.bin_override", local_bin_override_path_);
    this->get_parameter("mqtt.broker_url", mqtt_broker_url_);
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
    this->get_parameter("software.version", software_version_);
    this->get_parameter("hardware.version", hardware_version_);
    this->get_parameter("domain.version_file", domain_version_file_);
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
    this->get_parameter("can.id.domain", can_id_domain_);
    this->get_parameter("can.id.upper",  can_id_upper_);
    this->get_parameter("can.id.lower",  can_id_lower_);
    this->get_parameter("flash.pull_mode", pull_flash_mode_);
    this->get_parameter("flash.pull_to_controllers_only", pull_to_controllers_only_);
    this->get_parameter("flash.pull_timeout_sec", pull_timeout_sec_);
    this->get_parameter("flash.self_flash_cmd", self_flash_cmd_template_);
    this->get_parameter("self.reboot_cmd", self_reboot_cmd_template_);
    this->get_parameter("self.verify_cmd", self_verify_cmd_template_);
    this->get_parameter("self.rollback_image_path", self_rollback_image_path_);
    this->get_parameter("mock_can.outcome", mock_can_outcome_);
    this->get_parameter("mock_can.fail_after_step", mock_can_fail_after_step_);
    this->get_parameter("mock_can.step_delay_ms", mock_can_step_delay_ms_);
    this->get_parameter("integrity.sha256_map_json", integrity_sha256_json_);
    this->get_parameter("integrity.signature.map_json", integrity_signature_map_json_);
    this->get_parameter("integrity.signature.pubkey_path", integrity_signature_pubkey_path_);
    this->get_parameter("integrity.signature.algorithm", integrity_signature_algorithm_);
    this->get_parameter("integrity.signature.payload_template", integrity_signature_payload_template_);
    this->get_parameter("integrity.l3.expected_versions_json", integrity_expected_versions_json_);
    this->get_parameter("integrity.l3.required_nodes", integrity_l3_required_nodes_);
    this->get_parameter("integrity.l4.required_nodes", integrity_l4_required_nodes_);
    this->get_parameter("mainline.version", mainline_version_);
    this->get_parameter("mainline.config_json", mainline_config_json_);
    this->get_parameter("dbc.enabled", dbc_publish_enabled_);
    this->get_parameter("dbc.audio_type", dbc_audio_type_);
    this->get_parameter("dbc.publish_count", dbc_publish_count_);
    this->get_parameter("dbc.publish_interval_ms", dbc_publish_interval_ms_);
    int isotp_chunk = 7;
    this->get_parameter("isotp.chunk_size", isotp_chunk);
    iso_tp_chunk_size_ = static_cast<size_t>(std::max(1, isotp_chunk));
    this->get_parameter("report.can_tx_fail_ack", report_can_tx_fail_ack_);
    try {
        auto j = nlohmann::json::parse(ecu_hw_sn_json);
        if (j.is_object()) {
            for (auto& it : j.items()) {
                if (it.value().is_string()) {
                    ecu_hardware_sn_map_[it.key()] = it.value().get<std::string>();
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "解析 ecu.hardware_sn_map_json 失败: %s", e.what());
    }
    try {
        auto j = nlohmann::json::parse(ecu_model_json);
        if (j.is_object()) {
            for (auto& it : j.items()) {
                if (it.value().is_string()) {
                    ecu_model_map_[it.key()] = it.value().get<std::string>();
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "解析 ecu.model_map_json 失败: %s", e.what());
    }
    // 解析主线版本配置JSON
    load_mainline_config();

    // 加载模拟测试参数
    this->get_parameter("test.can.simulate_send", simulate_can_send_);
    this->get_parameter("test.can.simulate_bin_path", simulate_bin_path_);
    this->get_parameter("test.can.target_id", simulate_target_can_id_);

}


/**
 * @brief 模拟CAN文件发送测试
 * 用于开发和调试阶段验证CAN通信功能
 */
void OtaAgentNode::simulate_can_file_send() {
    if (!simulate_can_send_) return;
    if (simulate_bin_path_.empty()) {
        RCLCPP_WARN(this->get_logger(), "[CAN-TEST] 模拟发送启用，但未指定 bin 路径");
        return;
    }
    if (!std::filesystem::exists(simulate_bin_path_)) {
        RCLCPP_ERROR(this->get_logger(), "[CAN-TEST] 模拟 bin 路径不存在: %s", simulate_bin_path_.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[CAN-TEST] 开始模拟 CAN 发送 | 目标ID=0x%X | bin路径=%s",
        simulate_target_can_id_, simulate_bin_path_.c_str());
    
    // 模拟升级上下文（避免 ACK 发布失败）
    current_up_id_ = "test_up_001";
    current_ecu_id_ = "test_ecu_001";
    current_version_ = "1.0.0-test";

    // 调用 ISO-TP 分包发送
    std::string err;
    bool ret = send_file_iso_tp(simulate_target_can_id_, simulate_bin_path_, err, iso_tp_chunk_size_);
    if (ret) {
        RCLCPP_INFO(this->get_logger(), "[CAN-TEST] 模拟发送成功！");
    } else {
        RCLCPP_ERROR(this->get_logger(), "[CAN-TEST] 模拟发送失败: %s", err.c_str());
    }

    // 清理测试上下文
    current_up_id_.clear();
    current_ecu_id_.clear();
    current_version_.clear();
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
    vehicle::ota::up::VehicleOtaAck ack;
    ack.set_task_id(task_id);
    ack.set_sn(vin_);
    ack.set_model(model_str_);
    ack.set_timestamp(static_cast<int64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()));
    ack.set_task_status(task_status);
    ack.set_task_result(task_result);
    ack.set_task_error_code(task_error_code);
    ack.set_sys_error_code("");

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

    vehicle::ota::up::VehicleOtaAck ack;
    ack.set_task_id(task_id);
    ack.set_sn(vin_);
    ack.set_model(model_str_);
    ack.set_timestamp(static_cast<int64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()));
    ack.set_task_status(5);
    ack.set_task_result(summary_result);
    ack.set_task_error_code(summary_error);
    ack.set_sys_error_code("");

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

void ota_agent::OtaAgentNode::send_ack_payload(const vehicle::ota::up::VehicleOtaAck& ack, const std::string& label) {
    std::string data;
    if (!ack.SerializeToString(&data)) {
        RCLCPP_ERROR(this->get_logger(), "ACK序列化失败(%s)", label.c_str());
        return;
    }

    {
        std::lock_guard<std::mutex> lk(ack_mutex_);
        ack_queue_.push_back(PendingAck{ack.task_id(), data, label, 0});
    }
    drain_ack_queue();
}

void ota_agent::OtaAgentNode::drain_ack_queue() {
    std::lock_guard<std::mutex> lk(ack_mutex_);
    if (!mqtt_manager_) return;

    while (!ack_queue_.empty()) {
        auto& item = ack_queue_.front();
        if (!mqtt_manager_->is_connected()) {
            break;
        }
        int rc = mqtt_manager_->publish(ack_topic_, item.payload, 1);
        if (rc == MOSQ_ERR_SUCCESS) {
            ack_queue_.pop_front();
            continue;
        }
        ++item.attempts;
        RCLCPP_WARN(this->get_logger(), "ACK发布失败 rc=%d label=%s task=%s attempt=%d", rc, item.label.c_str(), item.task_id.c_str(), item.attempts);
        if (item.attempts >= ack_retry_max_attempts_) {
            RCLCPP_ERROR(this->get_logger(), "ACK重试次数耗尽，丢弃 label=%s task=%s", item.label.c_str(), item.task_id.c_str());
            ack_queue_.pop_front();
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

void ota_agent::OtaAgentNode::remove_staged_file(const std::string& path) {
    if (path.empty()) return;
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
    if (!ec) {
        RCLCPP_INFO(this->get_logger(), "已清理临时文件/目录: %s", path.c_str());
    }
}

bool ota_agent::OtaAgentNode::handle_cancel_and_cleanup(const vehicle::ota::up::VehicleOtaUp& up, const std::string& staged_path, const std::string& reason) {
    cancel_requested_.store(false);
    remove_staged_file(staged_path);
    remove_staged_file(last_staged_path_);
    current_up_id_.clear();
    current_package_id_.clear();
    current_ecu_id_.clear();
    current_version_.clear();

    std::vector<OtaStatusEntry> snapshot;
    {
        std::lock_guard<std::mutex> lk(ack_mutex_);
        for (const auto& kv : task_ack_state_.latest) {
            snapshot.push_back(kv.second);
        }
    }
    if (snapshot.empty()) {
        snapshot.reserve(static_cast<size_t>(up.packages_size()));
        for (const auto& pkg : up.packages()) {
            snapshot.push_back({pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 11, 9, "", reason});
        }
    } else {
        for (auto& s : snapshot) {
            s.status = 11;
            if (s.error_code == 0) s.error_code = 9;
            if (s.message.empty()) s.message = reason;
        }
    }

    publish_ota_ack(up.task_id(), 5, 1, 9, snapshot);
    {
        std::lock_guard<std::mutex> lk(ack_mutex_);
        task_ack_state_.summary_sent = true;
    }
    publish_upgrade_message(reason);
    publish_upgrade_status(1);
    publish_task_status(0);
    return false;
}

void ota_agent::OtaAgentNode::initialize_managers() {
    mqtt_module::SignatureConfig sign_config;
    std::string private_key = this->get_parameter("signature.private_key").as_string();
    std::string public_key = this->get_parameter("signature.public_key").as_string();
    if (!private_key.empty()) {
        sign_config.private_key = private_key;
    } else {
        sign_config.private_key = mqtt_module::generate_seed_hex();
    }
    if (!public_key.empty()) {
        sign_config.public_key = public_key;
    }
    sign_config.sign_separator = "|";
    signature_mgr_ = std::make_shared<mqtt_module::SignatureManager>(sign_config, this->get_logger());

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
    mqtt_manager_ = std::make_shared<mqtt_module::MqttConnectionManager>(mqtt_config, this->get_logger());
    mqtt_manager_->set_message_callback([this](const std::string& topic, const std::string& payload) {
        this->on_mqtt_message(topic, payload);
    });

    // DBC 发布通道：与 ota_dbc_demo_node 相同的主题
    dbc_audio_pub_ = this->create_publisher<std_msgs::msg::Int32>("/vlago/audio/play", 10);
}

// 比较版本字符串（语义化 x.y.z 或纯数字字符串）。返回：<0 a<b, 0 相等, >0 a>b
int ota_agent::OtaAgentNode::compare_versions(const std::string& a, const std::string& b) const {
    auto trim_build = [](const std::string& s) {
        const auto pos = s.find_first_of("-+");
        return (pos == std::string::npos) ? s : s.substr(0, pos);
    };
    auto split = [&](const std::string& raw){
        const std::string s = trim_build(raw);
        std::vector<int> parts; parts.reserve(4);
        std::string token; std::istringstream iss(s);
        while (std::getline(iss, token, '.')) {
            // 提取数字前缀，非数字则按0处理
            int v = 0;
            if (!token.empty()) {
                size_t i = 0; while (i < token.size() && std::isdigit(static_cast<unsigned char>(token[i]))) ++i;
                if (i > 0) { try { v = std::stoi(token.substr(0, i)); } catch (...) { v = 0; } }
            }
            parts.push_back(v);
        }
        // 规范化长度到3段
        while (parts.size() < 3) parts.push_back(0);
        return parts;
    };
    auto pa = split(a), pb = split(b);
    for (size_t i = 0; i < std::max(pa.size(), pb.size()); ++i) {
        int va = (i < pa.size()) ? pa[i] : 0;
        int vb = (i < pb.size()) ? pb[i] : 0;
        if (va != vb) return (va < vb) ? -1 : 1;
    }
    return 0;
}

void ota_agent::OtaAgentNode::load_mainline_config() {
    // 如果提供 JSON，支持结构：{"version":"1.2.3","ecu_versions":{"3588":"1.2.3","upper":"1.2.3"}}
    if (!mainline_config_json_.empty()) {
        try {
            auto j = nlohmann::json::parse(mainline_config_json_);
            if (j.contains("version") && j["version"].is_string()) {
                mainline_version_ = j["version"].get<std::string>();
            }
            if (j.contains("ecu_versions") && j["ecu_versions"].is_object()) {
                for (auto& it : j["ecu_versions"].items()) {
                    if (it.value().is_string()) {
                        mainline_ecu_versions_[it.key()] = it.value().get<std::string>();
                    }
                }
            }
            RCLCPP_INFO(this->get_logger(), "已加载主线版本配置：global=%s ecu_count=%zu",
                        mainline_version_.c_str(), mainline_ecu_versions_.size());
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "主线版本配置JSON解析失败: %s", e.what());
        }
    }
}

bool ota_agent::OtaAgentNode::load_stored_domain_version(std::string& out_version) {
    try {
        std::filesystem::path p(domain_version_file_);
        if (!std::filesystem::exists(p)) {
            return false;
        }
        std::ifstream ifs(p);
        if (!ifs.is_open()) {
            RCLCPP_WARN(this->get_logger(), "无法打开域控版本文件: %s", domain_version_file_.c_str());
            return false;
        }
        nlohmann::json j = nlohmann::json::parse(ifs, nullptr, false);
        if (j.is_object() && j.contains("version") && j["version"].is_string()) {
            out_version = j["version"].get<std::string>();
            return true;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "读取域控版本文件失败: %s", e.what());
    }
    return false;
}

void ota_agent::OtaAgentNode::persist_domain_version(const std::string& version) {
    try {
        std::filesystem::path p(domain_version_file_);
        if (!p.parent_path().empty()) {
            std::filesystem::create_directories(p.parent_path());
        }
        nlohmann::json j;
        j["version"] = version;
        std::ofstream ofs(p, std::ios::trunc);
        if (!ofs.is_open()) {
            RCLCPP_WARN(this->get_logger(), "写入域控版本文件失败: %s", domain_version_file_.c_str());
            return;
        }
        ofs << j.dump();
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "保存域控版本文件异常: %s", e.what());
    }
}

void OtaAgentNode::start_boot_broadcast_timer() {
    if (!boot_broadcast_enabled_) {
        return;
    }
    const int period_ms = std::max(10, boot_broadcast_period_ms_);
    boot_broadcast_timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), [this]{
        if (!boot_broadcast_pub_) {
            return;
        }
        std_msgs::msg::String msg;
        msg.data = boot_broadcast_payload_;
        boot_broadcast_pub_->publish(msg);
    });
    RCLCPP_INFO(this->get_logger(), "启动boot广播：topic=%s period=%dms payload=%s", boot_broadcast_topic_.c_str(), period_ms, boot_broadcast_payload_.c_str());
}

void OtaAgentNode::start_version_broadcast_timer() {
    if (!version_broadcast_enabled_) {
        return;
    }
    const int period_ms = std::max(100, version_broadcast_period_ms_);
    version_broadcast_timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), [this]{
        if (!version_broadcast_pub_) {
            return;
        }
        std::string sw = software_version_;
        if (info_provider_) {
            int code = 0;
            std::string msg;
            std::string current;
            if (info_provider_->get_current_version(current, code, msg) && !current.empty()) {
                sw = current;
            }
        }
        json payload;
        payload["software_version"] = sw;
        if (!hardware_version_.empty()) {
            payload["hardware_version"] = hardware_version_;
        }
        if (!vin_.empty()) {
            payload["vin"] = vin_;
        }
        if (!model_str_.empty()) {
            payload["model"] = model_str_;
        }
        if (!ecu_hardware_sn_map_.empty()) {
            nlohmann::json arr = nlohmann::json::array();
            for (const auto& kv : ecu_hardware_sn_map_) {
                nlohmann::json one;
                one["ecu"] = kv.first;
                one["hardware_sn"] = kv.second;
                one["hardware_code"] = kv.second; // 为兼容命名，使用相同字段
                // 版本暂以域控软件版本作为缺省，如后续可从数据源获取可替换
                one["version"] = software_version_;
                arr.push_back(one);
            }
            payload["ecu_report"] = arr;
        }
        const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::system_clock::now().time_since_epoch()).count();
        payload["timestamp_ms"] = now_ms;
        std_msgs::msg::String out;
        out.data = payload.dump();
        version_broadcast_pub_->publish(out);
    });
    RCLCPP_INFO(this->get_logger(), "启动版本广播：topic=%s period=%dms", version_broadcast_topic_.c_str(), period_ms);
}

bool OtaAgentNode::validate_package_metadata(const vehicle::ota::up::UpgradePackage& pkg, std::string& reason, int& pkg_error_code) const {
    const std::string ecu_id = pkg.ecu();
    // 车型校验：如配置了 ecu_model_map_ 对应 ECU 的车型，则要求与当前 model_str_ 一致
    auto it_model = ecu_model_map_.find(ecu_id);
    if (it_model != ecu_model_map_.end()) {
        if (!model_str_.empty() && model_str_ != it_model->second) {
            reason = "model_mismatch:" + model_str_ + " vs " + it_model->second;
            pkg_error_code = 3;
            return false;
        }
    }
    // 硬件编号校验：如配置了 ecu_hardware_sn_map_ 对应 ECU 的编号，则要求包中声明一致
    auto it_hw = ecu_hardware_sn_map_.find(ecu_id);
    if (it_hw != ecu_hardware_sn_map_.end()) {
        const std::string pkg_hw = pkg.ecu_hardware_code();
        if (pkg_hw.empty()) {
            reason = "missing_hw_code";
            pkg_error_code = 4;
            return false;
        }
        if (pkg_hw != it_hw->second) {
            reason = "hw_code_mismatch:" + pkg_hw + " vs " + it_hw->second;
            pkg_error_code = 4;
            return false;
        }
    }
    return true;
}

void ota_agent::OtaAgentNode::check_boot_version_and_push() {
    if (!controller_boot_push_enabled_) {
        return;
    }
    if (controller_package_path_.empty()) {
        RCLCPP_WARN(this->get_logger(), "已启用开机推送，但未配置 controller.package_path，跳过");
        return;
    }
    if (!std::filesystem::exists(controller_package_path_)) {
        RCLCPP_WARN(this->get_logger(), "开机推送包不存在: %s", controller_package_path_.c_str());
        return;
    }

    std::string detected_version = software_version_;
    if (info_provider_) {
        int code = 0; std::string msg;
        if (!info_provider_->get_current_version(detected_version, code, msg)) {
            RCLCPP_WARN(this->get_logger(), "读取当前域控版本失败，使用软件版本: %s", software_version_.c_str());
            detected_version = software_version_;
        }
    }

    std::string stored_version;
    const bool has_stored = load_stored_domain_version(stored_version);
    if (has_stored && stored_version == detected_version) {
        RCLCPP_INFO(this->get_logger(), "域控版本未变化(%s)，跳过开机推送", detected_version.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "检测到域控版本变化: stored=%s current=%s，向上下控推送安装包", stored_version.c_str(), detected_version.c_str());
    std::string err;
    bool upper_ok = flash_file_via_can(static_cast<uint32_t>(can_id_upper_), detected_version, controller_package_path_, err);
    if (!upper_ok) {
        RCLCPP_WARN(this->get_logger(), "上控推送失败: %s", err.c_str());
    }
    err.clear();
    bool lower_ok = flash_file_via_can(static_cast<uint32_t>(can_id_lower_), detected_version, controller_package_path_, err);
    if (!lower_ok) {
        RCLCPP_WARN(this->get_logger(), "下控推送失败: %s", err.c_str());
    }

    if (upper_ok && lower_ok) {
        persist_domain_version(detected_version);
        RCLCPP_INFO(this->get_logger(), "开机推送完成并更新域控版本记录: %s", detected_version.c_str());
    }
}

bool ota_agent::OtaAgentNode::fetch_device_tags() {
    const std::string url = api_base_url_ + "/api/device/v1/device/getTags";
    json body;
    body["vin"] = vin_;
    body["model"] = model_;
    const std::string body_str = body.dump();
    auto headers = build_common_headers(body_str);
    try {
        auto resp = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{http_timeout_ms_});
        if (resp.error) {
            RCLCPP_ERROR(this->get_logger(), "标签请求失败(%d): %s", static_cast<int>(resp.error.code), resp.error.message.c_str());
            return false;
        }
        if (resp.status_code != 200) {
            RCLCPP_ERROR(this->get_logger(), "标签接口返回 %ld", resp.status_code);
            return false;
        }
        auto parsed = json::parse(resp.text, nullptr, false);
        if (parsed.is_discarded()) {
            RCLCPP_ERROR(this->get_logger(), "标签接口响应解析失败");
            return false;
        }
        std::vector<DeviceTag> parsed_tags;
        auto append_candidates = [&](const json& candidate) {
            if (!candidate.is_array()) {
                return false;
            }
            for (const auto& node : candidate) {
                DeviceTag tag;
                tag.tag_name = node.value("tagName", node.value("tag_name", std::string("")));
                tag.tag_code = node.value("tagCode", node.value("tag_code", std::string("")));
                const auto& values = node.contains("tagValues") ? node["tagValues"] :
                    (node.contains("tag_values") ? node["tag_values"] : json::array());
                if (values.is_array()) {
                    for (const auto& value : values) {
                        TagValue tv;
                        tv.value_code = value.value("valueCode", value.value("value_code", std::string("")));
                        tv.value_name = value.value("valueName", value.value("value_name", std::string("")));
                        tag.tag_values.push_back(tv);
                    }
                }
                parsed_tags.push_back(std::move(tag));
            }
            return !parsed_tags.empty();
        };
        std::vector<json> candidates;
        candidates.push_back(parsed);
        if (parsed.contains("data")) {
            candidates.push_back(parsed["data"]);
        }
        if (parsed.contains("deviceTags")) {
            candidates.push_back(parsed["deviceTags"]);
        }
        if (parsed.contains("tags")) {
            candidates.push_back(parsed["tags"]);
        }
        if (parsed.contains("data") && parsed["data"].is_object()) {
            const auto& data_node = parsed["data"];
            if (data_node.contains("deviceTags")) {
                candidates.push_back(data_node["deviceTags"]);
            }
            if (data_node.contains("tags")) {
                candidates.push_back(data_node["tags"]);
            }
        }
        bool found = false;
        for (const auto& candidate : candidates) {
            if (append_candidates(candidate)) {
                found = true;
                break;
            }
        }
        if (!found) {
            RCLCPP_WARN(this->get_logger(), "标签接口没有返回有效标签，保持上次结果");
            return !device_tags_.empty();
        }
        {
            std::lock_guard<std::mutex> guard(tag_mutex_);
            device_tags_ = std::move(parsed_tags);
            tags_ready_ = !device_tags_.empty();
        }
        RCLCPP_INFO(this->get_logger(), "加载标签成功，共 %zu 个标签", device_tags_.size());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "标签接口异常: %s", e.what());
        return false;
    }
}

bool ota_agent::OtaAgentNode::fetch_offline_upgrades() {
    const std::string url = api_base_url_ + "/api/device/v1/ota/getUp";
    json body;
    body["sn"] = vin_;
    body["model"] = model_str_;
    const std::string body_str = body.dump();
    auto headers = build_common_headers(body_str);

    try {
        auto resp = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{http_timeout_ms_});
        if (resp.error) {
            RCLCPP_ERROR(this->get_logger(), "getUp请求失败(%d): %s", static_cast<int>(resp.error.code), resp.error.message.c_str());
            return false;
        }
        if (resp.status_code != 200) {
            RCLCPP_ERROR(this->get_logger(), "getUp接口返回 %ld", resp.status_code);
            return false;
        }

        auto parsed = json::parse(resp.text, nullptr, false);
        if (parsed.is_discarded()) {
            RCLCPP_ERROR(this->get_logger(), "getUp响应解析失败");
            return false;
        }

        size_t processed = 0;
        auto try_item = [&](const json& item) {
            // 优先 taskDataBase64
            if (item.contains("taskDataBase64") && item["taskDataBase64"].is_string()) {
                const auto base64 = item["taskDataBase64"].get<std::string>();
                std::string decoded;
                decoded.resize(((base64.size() + 3) / 4) * 3);
                int len = EVP_DecodeBlock(reinterpret_cast<unsigned char*>(&decoded[0]), reinterpret_cast<const unsigned char*>(base64.data()), static_cast<int>(base64.size()));
                if (len > 0) {
                    decoded.resize(static_cast<size_t>(len));
                    handle_ota_upgrade_msg(decoded);
                    ++processed;
                    return true;
                }
            }
            // fallback: JSON -> VehicleOtaUp
            vehicle::ota::up::VehicleOtaUp up;
            up.set_task_id(item.value("taskId", std::string("")));
            up.set_task_status(item.value("taskStatus", 0));
            up.set_timestamp(item.value("timestamp", static_cast<int64_t>(0)));
            up.set_exec_time(item.value("execTime", static_cast<int64_t>(0)));
            const auto& pkg_arr = item.contains("packages") ? item["packages"] : json::array();
            if (pkg_arr.is_array()) {
                for (const auto& p : pkg_arr) {
                    if (!p.is_object()) continue;
                    auto* m = up.add_packages();
                    m->set_package_id(p.value("package_id", std::string("")));
                    m->set_ecu(p.value("ecu", std::string("")));
                    m->set_ecu_hardware_code(p.value("ecu_hardware_code", std::string("")));
                    m->set_target_version(p.value("target_version", std::string("")));
                    m->set_from_version(p.value("from_version", std::string("")));
                    m->set_from_max_version(p.value("from_max_version", std::string("")));
                    m->set_download_url(p.value("download_url", std::string("")));
                    m->set_sha256(p.value("sha256", std::string("")));
                    m->set_size(p.value("size", static_cast<int64_t>(0)));
                }
            }
            std::string payload;
            if (up.SerializeToString(&payload)) {
                handle_ota_upgrade_msg(payload);
                ++processed;
                return true;
            }
            return false;
        };

        if (parsed.is_array()) {
            for (const auto& item : parsed) try_item(item);
        } else if (parsed.contains("data")) {
            const auto& d = parsed["data"];
            if (d.is_array()) {
                for (const auto& item : d) try_item(item);
            } else if (d.is_object()) {
                try_item(d);
            }
        } else {
            try_item(parsed);
        }

        RCLCPP_INFO(this->get_logger(), "离线补获处理完成：%zu 条", processed);
        return processed > 0;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "getUp异常: %s", e.what());
        return false;
    }
}

// void ota_agent::OtaAgentNode::connect_and_subscribe() {
//     if (!mqtt_manager_) {
//         RCLCPP_FATAL(this->get_logger(), "MQTT管理器未初始化");
//         throw std::runtime_error("mqtt manager null");
//     }
//     mqtt_module::TokenResult empty_token;
//     for (int attempt = 0; attempt < mqtt_retry_attempts_; ++attempt) {
//         if (mqtt_manager_->connect(empty_token)) {
//             subscribe_ota_topics();
//             return;
//         }
//         if (attempt < mqtt_retry_attempts_ - 1) {
//             double delay = mqtt_retry_delay_base_ * std::pow(2.0, attempt);
//             RCLCPP_WARN(this->get_logger(), "MQTT连接失败，重试 %d/%d, 等待 %.1f 秒", attempt + 1, mqtt_retry_attempts_, delay);
//             std::this_thread::sleep_for(std::chrono::duration<double>(delay));
//         }
//     }
//     RCLCPP_FATAL(this->get_logger(), "无法建立MQTT连接");
//     throw std::runtime_error("MQTT连接失败");
// }

void ota_agent::OtaAgentNode::connect_and_subscribe() {
    if (!mqtt_manager_) {
        RCLCPP_FATAL(this->get_logger(), "MQTT管理器未初始化");
        throw std::runtime_error("mqtt manager null");
    }
    RCLCPP_INFO(this->get_logger(), "开始连接MQTT Broker: %s (client_id: %s)", 
                mqtt_broker_url_.c_str(), mqtt_client_id_.c_str());
    mqtt_module::TokenResult empty_token;
    for (int attempt = 0; attempt < mqtt_retry_attempts_; ++attempt) {
        RCLCPP_INFO(this->get_logger(), "MQTT连接尝试 %d/%d", attempt + 1, mqtt_retry_attempts_);
        if (mqtt_manager_->connect(empty_token)) {
            RCLCPP_INFO(this->get_logger(), "MQTT连接成功！Broker: %s", mqtt_broker_url_.c_str());
            subscribe_ota_topics();
            return;
        }
        if (attempt < mqtt_retry_attempts_ - 1) {
            double delay = mqtt_retry_delay_base_ * std::pow(2.0, attempt);
            RCLCPP_WARN(this->get_logger(), "MQTT连接失败，重试 %d/%d, 等待 %.1f 秒", 
                        attempt + 1, mqtt_retry_attempts_, delay);
            std::this_thread::sleep_for(std::chrono::duration<double>(delay));
        }
    }
    RCLCPP_FATAL(this->get_logger(), "MQTT连接失败！已尝试 %d 次，Broker: %s", 
                mqtt_retry_attempts_, mqtt_broker_url_.c_str());
    throw std::runtime_error("MQTT连接失败");
}


// 反馈处理已在 ctrl_comm_ 中统一回调

void ota_agent::OtaAgentNode::subscribe_ota_topics() {
    if (!mqtt_manager_ || !mqtt_manager_->is_connected()) {
        RCLCPP_WARN(this->get_logger(), "MQTT未连接，无法订阅OTA主题");
        return;
    }
    subscribe_topic(build_all_topic());
    subscribe_topic(build_change_topic());
    std::vector<std::string> next_topics;
    {
        std::lock_guard<std::mutex> guard(tag_mutex_);
        if (!tags_ready_) {
            RCLCPP_DEBUG(this->get_logger(), "标签尚未准备好，跳过标签灰度订阅");
        } else {
            for (const auto& tag : device_tags_) {
                for (const auto& value : tag.tag_values) {
                    next_topics.push_back(build_tag_topic(tag.tag_code, value.value_code));
                }
            }
        }
    }
    unsubscribe_topics(subscribed_tag_topics_);
    subscribed_tag_topics_.clear();
    for (const auto& topic : next_topics) {
        if (subscribe_topic(topic)) {
            subscribed_tag_topics_.push_back(topic);
        }
    }
}

bool ota_agent::OtaAgentNode::subscribe_topic(const std::string& topic, int qos) {
    if (topic.empty()) {
        return false;
    }
    if (!mqtt_manager_ || !mqtt_manager_->is_connected()) {
        return false;
    }
    int rc = mqtt_manager_->subscribe(topic, qos);
    if (rc == MOSQ_ERR_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "已订阅: %s", topic.c_str());
        return true;
    }
    RCLCPP_WARN(this->get_logger(), "订阅失败: %s (%s)", topic.c_str(), mosquitto_strerror(rc));
    return false;
}

// void ota_agent::OtaAgentNode::on_mqtt_message(const std::string& topic, const std::string& payload) {
//     RCLCPP_DEBUG(this->get_logger(), "收到MQTT消息 %s", topic.c_str());
//     if (topic.find("vehicle/ota/all") != std::string::npos || topic.find("vehicle/ota/tag") != std::string::npos) {
//         handle_ota_upgrade_msg(payload);
//     } else if (topic.find("vehicle/ota/change") != std::string::npos) {
//         handle_tag_change_msg(payload);
//     }
// }


void ota_agent::OtaAgentNode::on_mqtt_message(const std::string& topic, const std::string& payload) {
    // 打印接收到的原始消息（注意：payload是ProtoBuf二进制，需解析后看内容）
    RCLCPP_INFO(this->get_logger(), "===== 收到MQTT消息 =====");
    RCLCPP_INFO(this->get_logger(), "主题: %s", topic.c_str());
    RCLCPP_INFO(this->get_logger(), "消息长度: %zu 字节", payload.size());
    RCLCPP_INFO(this->get_logger(), "=======================");

    // 新规范：仅处理发往本车SN的定向主题，兼容保留 tag/change
    const std::string my_up_topic = build_all_topic();
    if (topic == my_up_topic) {
        RCLCPP_INFO(this->get_logger(), "开始处理OTA升级消息");
        handle_ota_upgrade_msg(payload);
    } else if (topic.find("vehicle/ota/tag") != std::string::npos) {
        RCLCPP_INFO(this->get_logger(), "开始处理OTA升级消息(tag)");
        handle_ota_upgrade_msg(payload);
    } else if (topic.find("vehicle/ota/change") != std::string::npos) {
        RCLCPP_INFO(this->get_logger(), "开始处理标签变更消息");
        handle_tag_change_msg(payload);
    } else if (!current_task_id_.empty() && topic == build_task_status_topic(current_task_id_)) {
        RCLCPP_INFO(this->get_logger(), "收到任务状态变更，task_id=%s", current_task_id_.c_str());
        handle_task_status_change(payload);
    } else {
        RCLCPP_WARN(this->get_logger(), "收到未知主题消息: %s", topic.c_str());
    }
}

void ota_agent::OtaAgentNode::subscribe_task_status(const std::string& task_id) {
    if (!mqtt_manager_ || task_id.empty()) return;
    const std::string t = build_task_status_topic(task_id);
    int rc = mqtt_manager_->subscribe(t, 1);
    if (rc == MOSQ_ERR_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "已订阅任务状态变更: %s", t.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "订阅任务状态变更失败: %s (%s)", t.c_str(), mosquitto_strerror(rc));
    }
}

void ota_agent::OtaAgentNode::handle_task_status_change(const std::string& payload) {
    // 兼容：优先尝试JSON {"task_id":"","task_status":0,...}
    auto j = nlohmann::json::parse(payload, nullptr, false);
    if (!j.is_discarded()) {
        int st = 0;
        if (j.contains("task_status")) {
            if (j["task_status"].is_number_integer()) st = j["task_status"].get<int>();
            else if (j["task_status"].is_string()) { try { st = std::stoi(j["task_status"].get<std::string>()); } catch (...) {} }
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
    // 其它格式忽略但记录
    RCLCPP_WARN(this->get_logger(), "无法解析任务状态变更负载，忽略");
}

bool ota_agent::OtaAgentNode::fetch_task_status_http(const std::string& task_id, int& task_status) {
    if (api_base_url_.empty() || task_id.empty()) return false;
    const std::string url = api_base_url_ + "/api/device/v1/ota/getTaskStatus";
    nlohmann::json body; body["taskId"] = task_id;
    const std::string body_str = body.dump();
    auto headers = build_common_headers(body_str);
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
    if (!mqtt_manager_ || topics.empty()) return;
    for (const auto& topic : topics) {
        int rc = mqtt_manager_->unsubscribe(topic);
        if (rc == MOSQ_ERR_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "退订: %s", topic.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "退订失败: %s (%s)", topic.c_str(), mosquitto_strerror(rc));
        }
    }
}

/**
 * @brief 处理升级任务的主要流程
 * 包括下载、完整性校验、刷写等步骤
 * @param up 升级任务消息
 * @return 是否成功完成所有升级包
 */
bool ota_agent::OtaAgentNode::process_upgrade_task(const vehicle::ota::up::VehicleOtaUp& up) {
    // 检查是否已有升级任务在进行
    std::unique_lock<std::mutex> lk(upgrade_mutex_, std::try_to_lock);
    if (!lk.owns_lock() || upgrade_in_progress_) {
        RCLCPP_WARN(this->get_logger(), "已有升级在进行，拒绝新的任务：%s", up.task_id().c_str());
        return false;
    }
    const int ts = current_task_status_.load();
    if (ts == 1) {
        RCLCPP_WARN(this->get_logger(), "任务当前为暂停状态，跳过执行: task_id=%s", up.task_id().c_str());
        return false;
    }
    if (ts == 2) {
        RCLCPP_WARN(this->get_logger(), "任务已被取消，跳过执行: task_id=%s", up.task_id().c_str());
        return false;
    }
    int latest_status = 0;
    if (fetch_task_status_http(up.task_id(), latest_status)) {
        current_task_status_.store(latest_status);
        if (latest_status == 1) {
            RCLCPP_WARN(this->get_logger(), "HTTP 二次确认：任务已暂停，跳过执行 task_id=%s", up.task_id().c_str());
            return false;
        }
        if (latest_status == 2) {
            RCLCPP_WARN(this->get_logger(), "HTTP 二次确认：任务已取消，跳过执行 task_id=%s", up.task_id().c_str());
            cancel_requested_.store(true);
            return handle_cancel_and_cleanup(up, "", "task_cancelled_before_start");
        }
    }
    current_task_id_ = up.task_id();
    reset_task_ack_state(up.task_id(), static_cast<size_t>(up.packages_size()));
    upgrade_in_progress_ = true;
    auto guard = std::unique_ptr<void, std::function<void(void*)>>(nullptr, [this](void*) {
        std::lock_guard<std::mutex> g(upgrade_mutex_);
        upgrade_in_progress_ = false;
    });
    auto summary_guard = std::unique_ptr<void, std::function<void(void*)>>(nullptr, [this, tid = up.task_id()](void*) {
        this->maybe_publish_task_summary(tid, true);
    });
    auto ctx_guard = std::unique_ptr<void, std::function<void(void*)>>(nullptr, [this](void*) {
        current_up_id_.clear();
        current_package_id_.clear();
        current_ecu_id_.clear();
        current_version_.clear();
    });

    if (up.packages_size() == 0) {
        RCLCPP_WARN(this->get_logger(), "OTA消息无升级包：%s", up.task_id().c_str());
        publish_ota_ack(up.task_id(), 5, 1, 8, {});
        publish_task_status(0); // 有升级包待下载（为空时视为待准备）
        publish_upgrade_message("no_packages");
        publish_upgrade_status(1);
        return false;
    }

    bool any_below = false;
    for (const auto& pkg : up.packages()) {
        std::string ml = mainline_version_;
        auto it_ml = mainline_ecu_versions_.find(pkg.ecu());
        if (it_ml != mainline_ecu_versions_.end()) ml = it_ml->second;
        if (!ml.empty() && !pkg.target_version().empty()) {
            if (compare_versions(pkg.target_version(), ml) < 0) { any_below = true; break; }
        }
    }
    if (any_below) {
        std::vector<OtaStatusEntry> snapshot;
        snapshot.reserve(static_cast<size_t>(up.packages_size()));
        for (const auto& pkg : up.packages()) {
            snapshot.push_back({pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 11, 3, "", "below_mainline"});
        }
        publish_ota_ack(up.task_id(), 5, 1, 4, snapshot);
        publish_upgrade_message("below_mainline");
        publish_upgrade_status(1);
        return false;
    }

    std::vector<OtaStatusEntry> pending_snapshot;
    pending_snapshot.reserve(static_cast<size_t>(up.packages_size()));
    for (const auto& pkg : up.packages()) {
        std::string meta_reason;
        int meta_err = 4;
        if (!validate_package_metadata(pkg, meta_reason, meta_err)) {
            std::vector<OtaStatusEntry> snapshot;
            snapshot.reserve(static_cast<size_t>(up.packages_size()));
            snapshot.push_back({pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 11, meta_err, "", meta_reason});
            publish_ota_ack(up.task_id(), 5, 1, meta_err, snapshot);
            publish_upgrade_message(meta_reason);
            publish_upgrade_status(1);
            return false;
        }
        pending_snapshot.push_back({pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 1, 0, "", "pending"});
    }
    publish_ota_ack(up.task_id(), 1, 0, 0, pending_snapshot);

    std::vector<OtaStatusEntry> completed_entries;
    const int download_attempts = std::max(1, 1 + (up.has_strategy() ? up.strategy().download_retry_count() : 0));
    const int flash_attempts = std::max(1, 1 + (up.has_strategy() ? up.strategy().flash_retry_count() : 0));
    RCLCPP_INFO(this->get_logger(), "任务级重试：download_attempts=%d flash_attempts=%d", download_attempts, flash_attempts);
    auto publish_failure = [&](int fail_idx, const vehicle::ota::up::UpgradePackage& pkg, const std::string& msg, int pkg_error_code) {
        std::vector<OtaStatusEntry> snapshot;
        snapshot.reserve(static_cast<size_t>(up.packages_size()));
        snapshot.insert(snapshot.end(), completed_entries.begin(), completed_entries.end());
        snapshot.push_back({pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 11, pkg_error_code, "", msg});
        for (int j = fail_idx + 1; j < up.packages_size(); ++j) {
            const auto& rest = up.packages(j);
            snapshot.push_back({rest.package_id(), rest.ecu(), rest.ecu_hardware_code(), rest.target_version(), 0, 0, "", "pending"});
        }
        publish_ota_ack(up.task_id(), 5, 1, pkg_error_code, snapshot);
    };

    for (int idx = 0; idx < up.packages_size(); ++idx) {
        const auto& pkg = up.packages(idx);
        const std::string ecu_id = pkg.ecu();
        const std::string pkg_id = pkg.package_id();
        const std::string version = pkg.target_version();
        const std::string url = pkg.download_url();

        if (!wait_if_paused_or_cancelled("package_start")) {
            return handle_cancel_and_cleanup(up, "", "task_cancelled");
        }

        current_up_id_ = up.task_id();
        current_package_id_ = pkg_id;
        current_ecu_id_ = ecu_id;
        current_version_ = version;
        publish_ota_ack(up.task_id(), 1, 0, 0, {{pkg_id, ecu_id, pkg.ecu_hardware_code(), version, 1, 0, "", pull_flash_mode_ ? std::string("instructing_pull") : std::string("downloading")}});

        std::string staged_path;
        std::string err;
        const bool need_local_image = (!pull_flash_mode_ || pull_to_controllers_only_ || ecu_id == "3588");

        if (need_local_image) {
            bool downloaded = false;
            for (int attempt = 1; attempt <= download_attempts; ++attempt) {
                if (!wait_if_paused_or_cancelled("download")) {
                    return handle_cancel_and_cleanup(up, staged_path, "task_cancelled");
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
                publish_failure(idx, pkg, std::string("download_failed: ") + err, 1);
                return false;
            }
            publish_bin_via_dbc(staged_path);
        }

        if (need_local_image) {
            IntegrityContext ctx;
            ctx.up_id = up.task_id();
            ctx.ecu_id = ecu_id;
            ctx.version = version;
            ctx.url = url;
            ctx.staged_path = staged_path;
            ctx.expected_md5 = std::string();
            ctx.expected_sha256 = pkg.sha256();
            ctx.expected_version = version_expectations_.count(ecu_id) ? version_expectations_[ecu_id] : std::string();
            ctx.signature = std::string();
            ctx.signature_algorithm = integrity_signature_algorithm_;
            ctx.signature_pubkey_path = integrity_signature_pubkey_path_;
            ctx.md5_calculator = [this](const std::string& path, std::string& e) { return this->compute_md5_hex(path, e); };
            ctx.sha256_calculator = [this](const std::string& path, std::string& e) { return this->compute_sha256_hex(path, e); };
            std::error_code sz_ec;
            ctx.file_size = std::filesystem::file_size(staged_path, sz_ec);
            if (sz_ec) ctx.file_size = 0;
            ctx.required_nodes_l3 = integrity_l3_required_nodes_;
            ctx.required_nodes_l4 = integrity_l4_required_nodes_;

            IntegrityValidator::ValidationResult vr;
            if (!wait_if_paused_or_cancelled("integrity")) {
                return handle_cancel_and_cleanup(up, staged_path, "task_cancelled");
            }
            if (!run_integrity_pipeline(ctx, vr)) {
                auto layer_to_str = [](IntegrityValidator::Layer l) {
                    switch (l) {
                        case IntegrityValidator::Layer::L1: return std::string("L1");
                        case IntegrityValidator::Layer::L2: return std::string("L2");
                        case IntegrityValidator::Layer::L3: return std::string("L3");
                        case IntegrityValidator::Layer::L4: return std::string("L4");
                    }
                    return std::string("unknown");
                };
                RCLCPP_ERROR(this->get_logger(), "完整性校验失败 task_id=%s ecu=%s layer=%s reason=%s", up.task_id().c_str(), ecu_id.c_str(), layer_to_str(vr.layer).c_str(), vr.message.c_str());
                rollback_on_integrity_failure(staged_path, vr.message);
                publish_failure(idx, pkg, std::string("integrity_failed:") + vr.message, 2);
                return false;
            }
        }

        const bool is_self_3588 = (ecu_id == "3588");
        if (is_self_3588) {
            RCLCPP_INFO(this->get_logger(), "[3588] 开始自身升级 task_id=%s version=%s", up.task_id().c_str(), version.c_str());
            publish_ota_ack(up.task_id(), 1, 0, 0, {{pkg_id, ecu_id, pkg.ecu_hardware_code(), version, 3, 0, "", "flashing_30%"}});
            std::string self_err;
            bool self_flushed = false;
            for (int attempt = 1; attempt <= flash_attempts; ++attempt) {
                if (!wait_if_paused_or_cancelled("flash_self")) {
                    return handle_cancel_and_cleanup(up, staged_path, "task_cancelled");
                }
                if (flash_self_via_bootloader(staged_path, self_err)) {
                    self_flushed = true;
                    if (attempt > 1) {
                        RCLCPP_INFO(this->get_logger(), "3588 刷写重试成功 task_id=%s attempt=%d/%d", up.task_id().c_str(), attempt, flash_attempts);
                    }
                    break;
                }
                RCLCPP_WARN(this->get_logger(), "3588 刷写失败 task_id=%s attempt=%d/%d: %s", up.task_id().c_str(), attempt, flash_attempts, self_err.c_str());
                if (attempt < flash_attempts) {
                    std::this_thread::sleep_for(1s);
                }
            }
            if (!self_flushed) {
                publish_failure(idx, pkg, std::string("self_flash_failed: ") + self_err, 5);
                return false;
            }
            publish_ota_ack(up.task_id(), 1, 0, 0, {{pkg_id, ecu_id, pkg.ecu_hardware_code(), version, 3, 0, "", "flashing_70%"}});
            publish_ota_ack(up.task_id(), 1, 0, 0, {{pkg_id, ecu_id, pkg.ecu_hardware_code(), version, 10, 0, "", "upgrade_done"}});
            completed_entries.push_back({pkg_id, ecu_id, pkg.ecu_hardware_code(), version, 10, 0, "", "upgrade_done"});
        } else {
            std::string flash_err;
            publish_ota_ack(up.task_id(), 1, 0, 0, {{pkg_id, ecu_id, pkg.ecu_hardware_code(), version, 3, 0, "", "flashing_30%"}});
            bool flashed = false;
            for (int attempt = 1; attempt <= flash_attempts; ++attempt) {
                if (!wait_if_paused_or_cancelled("flash_can")) {
                    return handle_cancel_and_cleanup(up, staged_path, "task_cancelled");
                }
                if (flash_ecu_via_can(pkg, staged_path, flash_err)) {
                    flashed = true;
                    if (attempt > 1) {
                        RCLCPP_INFO(this->get_logger(), "刷写重试成功 task_id=%s ecu=%s attempt=%d/%d", up.task_id().c_str(), ecu_id.c_str(), attempt, flash_attempts);
                    }
                    break;
                }
                RCLCPP_WARN(this->get_logger(), "刷写失败 task_id=%s ecu=%s attempt=%d/%d: %s", up.task_id().c_str(), ecu_id.c_str(), attempt, flash_attempts, flash_err.c_str());
                if (attempt < flash_attempts) {
                    std::this_thread::sleep_for(1s);
                }
            }
            if (!flashed) {
                publish_failure(idx, pkg, std::string("flash_failed: ") + flash_err, 5);
                return false;
            }
            publish_ota_ack(up.task_id(), 1, 0, 0, {{pkg_id, ecu_id, pkg.ecu_hardware_code(), version, 3, 0, "", "flashing_70%"}});
            publish_ota_ack(up.task_id(), 1, 0, 0, {{pkg_id, ecu_id, pkg.ecu_hardware_code(), version, 10, 0, "", "upgrade_done"}});
            completed_entries.push_back({pkg_id, ecu_id, pkg.ecu_hardware_code(), version, 10, 0, "", "upgrade_done"});
        }
    }

    current_up_id_.clear();
    current_package_id_.clear();
    current_ecu_id_.clear();
    current_version_.clear();
    publish_ota_ack(up.task_id(), 5, 0, 0, completed_entries);
    return true;
}
// ===== 本地服务主题发布（5.1） =====
void ota_agent::OtaAgentNode::publish_task_status(int value) {
    if (!task_status_pub_) return;
    std_msgs::msg::Int32 m; m.data = value;
    task_status_pub_->publish(m);
}

void ota_agent::OtaAgentNode::publish_upgrade_status(int value) {
    if (!upgrade_status_pub_) return;
    std_msgs::msg::Int32 m; m.data = value;
    upgrade_status_pub_->publish(m);
}

void ota_agent::OtaAgentNode::publish_upgrade_message(const std::string& msg) {
    if (!upgrade_message_pub_) return;
    std_msgs::msg::String m; m.data = msg;
    upgrade_message_pub_->publish(m);
}

void ota_agent::OtaAgentNode::publish_upgrade_progress(double progress) {
    if (!upgrade_progress_pub_) return;
    std_msgs::msg::Float64 m; m.data = progress;
    upgrade_progress_pub_->publish(m);
}

// ===== 5.2 服务实现（调用可插拔数据源，若无则回退本地状态） =====
void ota_agent::OtaAgentNode::handle_get_version(
    const std::shared_ptr<interface::srv::GetVersion::Request> /*req*/,
    std::shared_ptr<interface::srv::GetVersion::Response> res) {
    std::string ver;
    int code = 0; std::string msg;
    if (info_provider_ && info_provider_->get_current_version(ver, code, msg)) {
        res->current_version = ver; res->code = code; res->message = msg; return;
    }
    // Fallback: use software_version_ parameter as current version
    res->current_version = software_version_;
    res->code = 0;
    res->message = "ok";
}

void ota_agent::OtaAgentNode::handle_get_upgrade_content(
    const std::shared_ptr<interface::srv::GetUpgradeContent::Request> /*req*/,
    std::shared_ptr<interface::srv::GetUpgradeContent::Response> res) {
    std::string ecu; int code = 0; std::string msg;
    if (info_provider_ && info_provider_->get_upgrade_content(ecu, code, msg)) {
        res->ecu = ecu; res->code = code; res->message = msg; return;
    }
    // Fallback: summarize last known ECU and version
    if (!current_ecu_id_.empty()) {
        res->ecu = std::string("ECU=") + current_ecu_id_ + ", version=" + current_version_;
        res->code = 0; res->message = "ok";
    } else {
        res->ecu = ""; res->code = 1; res->message = "no_ecu_info";
    }
}

void ota_agent::OtaAgentNode::handle_get_package_size(
    const std::shared_ptr<interface::srv::GetPackageSize::Request> /*req*/,
    std::shared_ptr<interface::srv::GetPackageSize::Response> res) {
    double sz = 0.0; int code = 0; std::string msg;
    if (info_provider_ && info_provider_->get_package_size(sz, code, msg)) {
        res->package_size = sz; res->code = code; res->message = msg; return;
    }
    // Fallback: use last staged path
    try {
        if (!last_staged_path_.empty() && std::filesystem::exists(last_staged_path_)) {
            sz = static_cast<double>(std::filesystem::file_size(last_staged_path_));
            res->package_size = sz; res->code = 0; res->message = "ok";
        } else {
            res->package_size = 0.0; res->code = 1; res->message = "no_staged_file";
        }
    } catch (const std::exception& e) {
        res->package_size = 0.0; res->code = 1; res->message = std::string("error:") + e.what();
    }
}

bool ota_agent::OtaAgentNode::send_pull_metadata_can(uint32_t can_id,
                                const std::string& version,
                                const std::string& url,
                                const std::string& md5,
                                uint64_t size,
                                std::string& err) {
    RCLCPP_INFO(this->get_logger(), "[CAN-META] 下发元数据 | can_id=0x%X | version=%s | url=%s | md5=%s | size=%llu",
                can_id, version.c_str(), url.c_str(), md5.c_str(), static_cast<unsigned long long>(size));
    // 采用简易 TLV over CAN：每帧8字节，第1字节为类型，后续7字节为数据分片。
    auto send_tlv = [&, this](char type, const std::string& data) -> bool {
        RCLCPP_INFO(this->get_logger(), "[CAN-META] TLV 发送 | type=%c | len=%zu | can_id=0x%X", type, data.size(), can_id);
        size_t off = 0;
        while (off < data.size()) {
            std::vector<uint8_t> frame(1, static_cast<uint8_t>(type));
            size_t n = std::min<size_t>(7, data.size() - off);
            frame.insert(frame.end(), data.begin() + off, data.begin() + off + n);
            std::string e;
            if (!send_can(can_id, frame, e)) { err = e; return false; }
            off += n;
        }
        // 空字段也至少发一帧指示存在
        if (data.empty()) {
            std::vector<uint8_t> frame(1, static_cast<uint8_t>(type));
            if (!send_can(can_id, frame, err)) return false;
        }
        return true;
    };
    {
        std::vector<uint8_t> f(1, static_cast<uint8_t>('B'));
        if (!send_can(can_id, f, err)) return false;
    }
    // VERSION / URL / MD5 / SIZE(ascii)
    if (!send_tlv('V', version)) return false;
    if (!send_tlv('U', url)) return false;
    if (!send_tlv('M', md5)) return false;
    std::string size_str = (size == 0) ? std::string("") : std::to_string(size);
    if (!send_tlv('S', size_str)) return false;
    // END
    {
        std::vector<uint8_t> f(1, static_cast<uint8_t>('E'));
        if (!send_can(can_id, f, err)) return false;
    }
    return true;
}

bool ota_agent::OtaAgentNode::download_and_stage(const std::string& url,
                                                 const std::string& up_id,
                                                 const vehicle::ota::up::UpgradePackage& pkg,
                                                 std::string& staged_path,
                                                 std::string& err) {
    if (!wait_if_paused_or_cancelled("download")) {
        err = "task_cancelled";
        return false;
    }
    // 优先使用本地覆盖文件（便于无网/调试）
    if (!local_bin_override_path_.empty()) {
        if (std::filesystem::exists(local_bin_override_path_)) {
            staged_path = local_bin_override_path_;
            RCLCPP_INFO(this->get_logger(), "使用本地固件覆盖路径: %s", staged_path.c_str());
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "本地固件覆盖路径不存在: %s，回退到 HTTP 下载", local_bin_override_path_.c_str());
        }
    }

    if (url.empty()) {
        err = "empty url";
        return false;
    }
    auto resp = cpr::Get(cpr::Url{url}, cpr::Timeout{http_timeout_ms_});
    if (resp.error) {
        err = resp.error.message;
        return false;
    }
    if (resp.status_code != 200) {
        err = "http status " + std::to_string(resp.status_code);
        return false;
    }
    const bool is_zip = url.size() >= 4 && url.substr(url.size() - 4) == ".zip";
    std::string download_path = is_zip ? ("/tmp/ota_" + up_id + "_" + pkg.ecu() + ".zip")
                                       : ("/tmp/ota_" + up_id + "_" + pkg.ecu() + ".bin");

    std::ofstream ofs(download_path, std::ios::binary | std::ios::trunc);
    if (!ofs.is_open()) {
        err = "open file failed";
        return false;
    }
    ofs.write(resp.text.data(), resp.text.size());
    if (!ofs.good()) {
        err = "write file failed";
        return false;
    }
    ofs.close();

    if (is_zip) {
        const std::string dest_dir = "/tmp/ota_" + up_id + "_" + pkg.ecu() + "_dir";
        if (!validate_and_extract_zip(download_path, dest_dir, pkg, staged_path, err)) {
            return false;
        }
    } else {
        staged_path = download_path;
    }
    last_staged_path_ = staged_path;
    return true;
}

bool ota_agent::OtaAgentNode::run_integrity_pipeline(const IntegrityContext& ctx, IntegrityValidator::ValidationResult& fail) {
    return integrity_validator_.run_all(ctx, fail);
}

bool ota_agent::OtaAgentNode::validate_and_extract_zip(const std::string& zip_path,
                                                      const std::string& dest_dir,
                                                      const vehicle::ota::up::UpgradePackage& pkg,
                                                      std::string& main_file_path,
                                                      std::string& err) {
    namespace fs = std::filesystem;
    try {
        if (fs::exists(dest_dir)) {
            fs::remove_all(dest_dir);
        }
        fs::create_directories(dest_dir);
    } catch (const std::exception& e) {
        err = std::string("prepare dest dir failed: ") + e.what();
        return false;
    }

    // 解压 zip（依赖 unzip 工具）
    std::string cmd = "unzip -o -qq " + zip_path + " -d " + dest_dir;
    int rc = std::system(cmd.c_str());
    if (rc != 0) {
        err = "unzip failed rc=" + std::to_string(rc);
        return false;
    }

    const fs::path manifest_path = fs::path(dest_dir) / "manifest.json";
    const fs::path signature_path = fs::path(dest_dir) / "signature.sig";
    if (!fs::exists(manifest_path)) { err = "manifest.json missing"; return false; }
    if (!fs::exists(signature_path)) { err = "signature.sig missing"; return false; }

    // 读取 manifest 与签名
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
    std::string sig_err;
    if (!verify_signature(manifest_content, signature_content, integrity_signature_pubkey_path_, integrity_signature_algorithm_, sig_err)) {
        err = std::string("signature verify failed: ") + sig_err;
        return false;
    }

    // 校验 manifest 列出的文件 SHA256 与元数据约束
    json manifest_json = json::parse(manifest_content, nullptr, false);
    if (!manifest_json.is_object() || !manifest_json.contains("files") || !manifest_json["files"].is_array()) {
        err = "manifest format invalid";
        return false;
    }
    const auto files = manifest_json["files"];
    if (files.empty()) { err = "manifest files empty"; return false; }

    const std::string manifest_ecu = manifest_json.value("ecu", std::string(""));
    if (!manifest_ecu.empty() && manifest_ecu != pkg.ecu()) { err = "manifest ecu mismatch"; return false; }
    const std::string manifest_ecu_sn = manifest_json.value("ecuSn", std::string(""));
    if (!manifest_ecu_sn.empty() && manifest_ecu_sn != vin_) { err = "manifest ecuSn mismatch"; return false; }
    const std::string manifest_target = manifest_json.value("targetVersion", std::string(""));
    if (!manifest_target.empty() && manifest_target != pkg.target_version()) { err = "manifest targetVersion mismatch"; return false; }
    const std::string manifest_from = manifest_json.value("fromVersion", std::string(""));
    if (!manifest_from.empty() && !pkg.from_version().empty() && manifest_from != pkg.from_version()) { err = "manifest fromVersion mismatch"; return false; }
    const std::string manifest_from_max = manifest_json.value("fromMaxVersion", std::string(""));
    if (!manifest_from_max.empty() && !pkg.from_max_version().empty() && manifest_from_max != pkg.from_max_version()) { err = "manifest fromMaxVersion mismatch"; return false; }
    const std::string manifest_type = manifest_json.value("type", std::string(""));
    const std::string manifest_up_type = manifest_json.value("upType", std::string(""));
    bool patch_declared = (manifest_type == "patch" || manifest_type == "diff" || manifest_up_type == "patch" || manifest_up_type == "diff");

    auto compute_file_sha = [this](const fs::path& p, std::string& out_err)->std::string {
        return this->compute_sha256_hex(p.string(), out_err);
    };

    fs::path selected_main;
    fs::path patch_file;
    std::string patch_base_rel;

    for (const auto& f : files) {
        if (!f.is_object()) { err = "manifest file entry invalid"; return false; }
        std::string fname = f.value("file", std::string(""));
        std::string sha = f.value("sha256", std::string(""));
        if (fname.empty() || sha.empty()) { err = "manifest file/sha missing"; return false; }
        fs::path fpath = fs::path(dest_dir) / fname;
        if (!fs::exists(fpath)) { err = "file missing: " + fname; return false; }

        std::error_code size_ec;
        const uintmax_t actual_size = fs::file_size(fpath, size_ec);
        if (f.contains("size") && f["size"].is_number_unsigned()) {
            const uint64_t expect_size = f["size"].get<uint64_t>();
            if (!size_ec && expect_size != actual_size) { err = "size mismatch for " + fname; return false; }
        }

        std::string sha_err;
        auto actual_sha = compute_file_sha(fpath, sha_err);
        if (!sha_err.empty()) { err = "sha calc failed: " + sha_err; return false; }
        if (actual_sha != sha) { err = std::string("sha mismatch for ") + fname; return false; }

        const bool is_main = f.value("isMain", false) || f.value("main", false);
        const bool is_patch = f.value("isPatch", false) || f.value("patch", false) || (fname.size() >= 6 && fname.substr(fname.size() - 6) == ".patch");
        if (is_main && selected_main.empty()) {
            selected_main = fpath;
        }
        if (is_patch) {
            patch_file = fpath;
            patch_base_rel = f.value("base", manifest_json.value("base", std::string("")));
            patch_declared = true;
        }
        if (selected_main.empty()) {
            selected_main = fpath;
        }
    }

    if (patch_declared) {
        if (patch_file.empty()) { err = "patch declared but patch file missing"; return false; }
        if (patch_base_rel.empty()) { err = "patch base missing"; return false; }
        fs::path base_path = fs::path(dest_dir) / patch_base_rel;
        if (!fs::exists(base_path)) { err = "patch base not found"; return false; }
        fs::path backup_path = base_path; backup_path += ".bak";
        std::error_code cp_ec;
        fs::copy_file(base_path, backup_path, fs::copy_options::overwrite_existing, cp_ec);
        if (cp_ec) { err = std::string("backup base failed: ") + cp_ec.message(); return false; }

        std::string patch_cmd = "patch -u " + base_path.string() + " < " + patch_file.string();
        int prc = std::system(patch_cmd.c_str());
        if (prc != 0) { err = "apply patch failed rc=" + std::to_string(prc); return false; }
        RCLCPP_INFO(this->get_logger(), "差分包已应用，目标=%s 备份=%s", base_path.c_str(), backup_path.c_str());
        selected_main = base_path;
    }

    if (selected_main.empty()) {
        err = "no file selected";
        return false;
    }

    // 额外比对主文件与包元数据
    std::string sha_err;
    auto final_sha = compute_file_sha(selected_main, sha_err);
    if (!sha_err.empty()) { err = "sha calc failed: " + sha_err; return false; }
    if (!pkg.sha256().empty() && final_sha != pkg.sha256()) { err = "pkg sha256 mismatch"; return false; }
    std::error_code final_size_ec;
    const uintmax_t final_size = fs::file_size(selected_main, final_size_ec);
    if (!final_size_ec && pkg.size() > 0 && final_size != static_cast<uintmax_t>(pkg.size())) { err = "pkg size mismatch"; return false; }

    main_file_path = selected_main.string();
    return true;
}

void ota_agent::OtaAgentNode::rollback_on_integrity_failure(const std::string& staged_path, const std::string& reason) {
    if (!staged_path.empty()) {
        std::error_code ec;
        std::filesystem::remove(staged_path, ec);
        if (ec) {
            RCLCPP_WARN(this->get_logger(), "回滚删除临时固件失败: %s 错误=%s", staged_path.c_str(), ec.message().c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "已删除临时固件以防重复使用: %s", staged_path.c_str());
        }
    }
    RCLCPP_ERROR(this->get_logger(), "OTA完整性校验失败，已停机: %s", reason.c_str());
}

/**
 * @brief 计算文件SHA256哈希值
 * @param path 文件路径
 * @param err 错误信息输出
 * @return 十六进制SHA256字符串
 */
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
    auto upper_algo = algorithm;
    std::transform(upper_algo.begin(), upper_algo.end(), upper_algo.begin(), [](unsigned char c){ return static_cast<char>(std::toupper(c)); });
    if (pubkey_path.empty()) {
        err = "pubkey_missing";
        return false;
    }

    BIO* bio = BIO_new_file(pubkey_path.c_str(), "r");
    if (!bio) {
        err = "open_pubkey_failed";
        return false;
    }
    EVP_PKEY* pkey = PEM_read_bio_PUBKEY(bio, nullptr, nullptr, nullptr);
    BIO_free(bio);
    if (!pkey) {
        err = "parse_pubkey_failed";
        return false;
    }

    int key_type = EVP_PKEY_base_id(pkey);
    const bool is_ed25519 = (upper_algo == "ED25519") || (key_type == EVP_PKEY_ED25519);
    if (upper_algo == "RSA" && key_type != EVP_PKEY_RSA) {
        EVP_PKEY_free(pkey);
        err = "pubkey_not_rsa";
        return false;
    }
    if ((upper_algo == "ECDSA" || upper_algo == "EC") && key_type != EVP_PKEY_EC) {
        EVP_PKEY_free(pkey);
        err = "pubkey_not_ecdsa";
        return false;
    }
    if (upper_algo == "ED25519" && key_type != EVP_PKEY_ED25519) {
        EVP_PKEY_free(pkey);
        err = "pubkey_not_ed25519";
        return false;
    }

    std::vector<unsigned char> sig;
    sig.resize(((base64_sig.size() + 3) / 4) * 3);
    int decoded_len = EVP_DecodeBlock(sig.data(), reinterpret_cast<const unsigned char*>(base64_sig.data()), static_cast<int>(base64_sig.size()));
    if (decoded_len < 0) {
        EVP_PKEY_free(pkey);
        err = "base64_decode_failed";
        return false;
    }
    // remove padding bytes
    size_t pad = 0;
    if (!base64_sig.empty()) {
        if (base64_sig[base64_sig.size() - 1] == '=') pad++;
        if (base64_sig.size() > 1 && base64_sig[base64_sig.size() - 2] == '=') pad++;
    }
    sig.resize(static_cast<size_t>(decoded_len) - pad);

    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    if (!ctx) {
        EVP_PKEY_free(pkey);
        err = "EVP_MD_CTX_new failed";
        return false;
    }

    int rc = 0;
    if (is_ed25519) {
        if (EVP_DigestVerifyInit(ctx, nullptr, nullptr, nullptr, pkey) != 1) {
            EVP_MD_CTX_free(ctx);
            EVP_PKEY_free(pkey);
            err = "DigestVerifyInit ed25519 failed";
            return false;
        }
        rc = EVP_DigestVerify(ctx,
                              sig.data(), sig.size(),
                              reinterpret_cast<const unsigned char*>(payload.data()), payload.size());
    } else {
        const EVP_MD* md = EVP_sha256();
        if (EVP_DigestVerifyInit(ctx, nullptr, md, nullptr, pkey) != 1) {
            EVP_MD_CTX_free(ctx);
            EVP_PKEY_free(pkey);
            err = "DigestVerifyInit failed";
            return false;
        }
        if (EVP_DigestVerifyUpdate(ctx, payload.data(), payload.size()) != 1) {
            EVP_MD_CTX_free(ctx);
            EVP_PKEY_free(pkey);
            err = "DigestVerifyUpdate failed";
            return false;
        }
        rc = EVP_DigestVerifyFinal(ctx, sig.data(), sig.size());
    }

    EVP_MD_CTX_free(ctx);
    EVP_PKEY_free(pkey);
    if (rc != 1) {
        err = "signature_invalid";
        return false;
    }
    return true;
}

void ota_agent::OtaAgentNode::setup_integrity_callbacks() {
    auto parse_map = [this](const std::string& raw, std::unordered_map<std::string, std::string>& out, const char* tag) {
        out.clear();
        try {
            auto j = json::parse(raw);
            if (!j.is_object()) {
                RCLCPP_WARN(this->get_logger(), "integrity %s 不是对象，忽略", tag);
                return;
            }
            for (auto it = j.begin(); it != j.end(); ++it) {
                if (it.value().is_string()) {
                    out[it.key()] = it.value().get<std::string>();
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "解析 integrity %s 失败: %s", tag, e.what());
            out.clear();
        }
    };

    parse_map(integrity_sha256_json_, sha256_expectations_, "sha256_map_json");
    parse_map(integrity_signature_map_json_, signature_expectations_, "signature_map_json");
    parse_map(integrity_expected_versions_json_, version_expectations_, "expected_versions_json");

    // 节点列表去重工具函数
    auto dedup_nodes = [](std::vector<std::string>& nodes) {
        std::sort(nodes.begin(), nodes.end());
        nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
    };
    dedup_nodes(integrity_l3_required_nodes_);
    dedup_nodes(integrity_l4_required_nodes_);

    // L2：在默认元信息校验基础上追加签名验签
    integrity_validator_.set_layer_callback(IntegrityValidator::Layer::L2, [this](const IntegrityContext& ctx) {
        IntegrityValidator::ValidationResult r;
        r.layer = IntegrityValidator::Layer::L2;
        if (ctx.version.empty()) { r.ok = false; r.message = "version_missing"; return r; }
        if (ctx.url.empty()) { r.ok = false; r.message = "url_missing"; return r; }
        auto lower_url = ctx.url;
        std::transform(lower_url.begin(), lower_url.end(), lower_url.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
        if (!(lower_url.rfind("http://", 0) == 0 || lower_url.rfind("https://", 0) == 0)) {
            r.ok = false; r.message = "url_scheme_invalid"; return r;
        }
        if (!ctx.expected_md5.empty()) {
            bool hex_only = ctx.expected_md5.size() == 32 && std::all_of(ctx.expected_md5.begin(), ctx.expected_md5.end(), [](unsigned char c){ return std::isxdigit(c) != 0; });
            if (!hex_only) { r.ok = false; r.message = "md5_not_hex"; return r; }
        }
        // 签名校验（可选）
        if (!ctx.signature.empty()) {
            if (ctx.signature_pubkey_path.empty()) {
                r.ok = false; r.message = "signature_pubkey_missing"; return r; }

            std::string md5_err;
            std::string sha_err;
            auto actual_md5 = ctx.md5_calculator ? ctx.md5_calculator(ctx.staged_path, md5_err) : std::string();
            if (!md5_err.empty()) { r.ok = false; r.message = std::string("md5_calc_failed:") + md5_err; return r; }
            auto actual_sha = ctx.sha256_calculator ? ctx.sha256_calculator(ctx.staged_path, sha_err) : std::string();
            if (!sha_err.empty()) { r.ok = false; r.message = std::string("sha256_calc_failed:") + sha_err; return r; }

            auto payload = integrity_signature_payload_template_;
            auto replace_all = [](std::string& s, const std::string& from, const std::string& to) {
                size_t pos = 0; while ((pos = s.find(from, pos)) != std::string::npos) { s.replace(pos, from.size(), to); pos += to.size(); }
            };
            replace_all(payload, "{version}", ctx.version);
            replace_all(payload, "{url}", ctx.url);
            replace_all(payload, "{md5}", actual_md5);
            replace_all(payload, "{sha256}", actual_sha);

            std::string sig_err;
            if (!verify_signature(payload, ctx.signature, ctx.signature_pubkey_path, ctx.signature_algorithm, sig_err)) {
                r.ok = false; r.message = std::string("signature_verify_failed:") + sig_err; return r;
            }
        }
        r.ok = true;
        return r;
    });

    // L3：在内容一致性基础上，叠加版本与必需节点检查
    integrity_validator_.set_layer_callback(IntegrityValidator::Layer::L3, [this](const IntegrityContext& ctx) {
        IntegrityValidator::ValidationResult r;
        r.layer = IntegrityValidator::Layer::L3;
        if (!ctx.expected_md5.empty()) {
            if (!ctx.md5_calculator) { r.ok = false; r.message = "md5_calculator_missing"; return r; }
            std::string err;
            auto md5 = ctx.md5_calculator(ctx.staged_path, err);
            if (!err.empty()) { r.ok = false; r.message = std::string("md5_calc_failed:") + err; return r; }
            if (md5 != ctx.expected_md5) { r.ok = false; r.message = std::string("md5_mismatch:got=") + md5; return r; }
        }
        if (!ctx.expected_version.empty() && ctx.version != ctx.expected_version) {
            r.ok = false; r.message = std::string("version_mismatch:expect=") + ctx.expected_version + " got=" + ctx.version; return r;
        }
        if (!ctx.required_nodes_l3.empty()) {
            std::vector<std::string> nodes = this->get_node_graph_interface()->get_node_names();
            auto has_node = [&nodes](const std::string& name){
                return std::find(nodes.begin(), nodes.end(), name) != nodes.end() ||
                       std::find(nodes.begin(), nodes.end(), "/" + name) != nodes.end();
            };
            for (const auto& n : ctx.required_nodes_l3) {
                if (!has_node(n)) { r.ok = false; r.message = std::string("node_missing:") + n; return r; }
            }
        }
        r.ok = true; return r;
    });

    // L4：运行态健康，叠加节点存活检查 + 磁盘空间
    integrity_validator_.set_layer_callback(IntegrityValidator::Layer::L4, [this](const IntegrityContext& ctx) {
        IntegrityValidator::ValidationResult r; r.layer = IntegrityValidator::Layer::L4;
        if (ctx.staged_path.empty()) { r.ok = true; r.message = "skip_no_path"; return r; }
        namespace fs = std::filesystem;
        std::error_code ec;
        auto space_info = fs::space(fs::path(ctx.staged_path).parent_path(), ec);
        if (ec) { r.ok = false; r.message = "space_query_failed"; return r; }
        uint64_t size = ctx.file_size;
        if (size == 0) {
            size = fs::file_size(ctx.staged_path, ec);
            if (ec) { r.ok = false; r.message = "stat_failed"; return r; }
        }
        if (space_info.available < size * 2) { r.ok = false; r.message = "insufficient_space"; return r; }

        if (!ctx.required_nodes_l4.empty()) {
            std::vector<std::string> nodes = this->get_node_graph_interface()->get_node_names();
            auto has_node = [&nodes](const std::string& name){
                return std::find(nodes.begin(), nodes.end(), name) != nodes.end() ||
                       std::find(nodes.begin(), nodes.end(), "/" + name) != nodes.end();
            };
            for (const auto& n : ctx.required_nodes_l4) {
                if (!has_node(n)) { r.ok = false; r.message = std::string("node_not_running:") + n; return r; }
            }
        }
        r.ok = true; return r;
    });
}

void ota_agent::OtaAgentNode::publish_bin_via_dbc(const std::string& staged_path) {
    if (!dbc_publish_enabled_) {
        return;
    }
    if (!dbc_audio_pub_) {
        RCLCPP_WARN(this->get_logger(), "DBC发布未初始化，跳过");
        return;
    }

    // 读取文件大小仅用于日志，模拟 ota_dbc_demo_node 的“收到 bin”行为
    std::ifstream ifs(staged_path, std::ios::binary | std::ios::ate);
    if (ifs.is_open()) {
        auto size = static_cast<long>(ifs.tellg());
        RCLCPP_INFO(this->get_logger(), "[DBC] 本地 bin 已就绪: %s (%ld bytes)", staged_path.c_str(), size);
    } else {
        RCLCPP_WARN(this->get_logger(), "[DBC] 打开 bin 失败: %s (仍尝试发布指令)", staged_path.c_str());
    }

    std_msgs::msg::Int32 msg;
    msg.data = dbc_audio_type_;
    int count = std::max(1, dbc_publish_count_);
    int interval_ms = std::max(1, dbc_publish_interval_ms_);
    rclcpp::WallRate rate{std::chrono::milliseconds(interval_ms)};
    for (int i = 0; i < count; ++i) {
        dbc_audio_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "[DBC] 第 %d/%d 次发布 audio_type=%d 到 /vlago/audio/play", i + 1, count, msg.data);
        rate.sleep();
    }
}

bool ota_agent::OtaAgentNode::flash_ecu_via_can(const vehicle::ota::up::UpgradePackage& pkg,
                                                const std::string& staged_path,
                                                std::string& err) {
    if (!wait_if_paused_or_cancelled("flash_can")) {
        err = "task_cancelled";
        return false;
    }
    const std::string ecu = pkg.ecu();
    const uint32_t target_can_id = (ecu == "upper") ? static_cast<uint32_t>(can_id_upper_)
                                 : (ecu == "lower") ? static_cast<uint32_t>(can_id_lower_)
                                 : static_cast<uint32_t>(can_id_domain_);
    const char* target_label = (ecu == "upper") ? "上控" : (ecu == "lower" ? "下控" : "域控");
    RCLCPP_INFO(this->get_logger(), "[FLASH] 准备刷写 %s 控制器 | ecu=%s | can_id=0x%X | file=%s",
                target_label, ecu.c_str(), target_can_id, staged_path.c_str());

    auto wait_ack = [&](uint8_t expect_sid, const char* step)->bool {
        std::vector<uint8_t> resp;
        if (!wait_for_can_response(target_can_id, expect_sid, 2000, resp, err)) {
            RCLCPP_ERROR(this->get_logger(), "[%s] 等待应答超时/失败: %s", step, err.c_str());
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "[%s] 收到应答 sid=0x%02X len=%zu data=[%s]", step, expect_sid, resp.size(), hexify(resp).c_str());
        if (!is_ack_success(resp)) {
            err = std::string(step) + " ack indicates failure";
            return false;
        }
        return true;
    };

    // 步骤1：请求升级（单帧，SID=0x31，载荷 31 01 02 03）
    {
        std::vector<uint8_t> sf = {0x04, 0x31, 0x01, 0x02, 0x03, 0x55, 0x55, 0x55};
        if (!send_can(target_can_id, sf, err)) return false;
        if (!wait_ack(0x71, "请求升级")) return false; // 0x71 = 0x31 正响应
    }

    // 准备文件尺寸与 CRC32
    std::error_code fs_ec;
    const uint64_t file_size = std::filesystem::file_size(staged_path, fs_ec);
    if (fs_ec) { err = std::string("stat failed: ") + fs_ec.message(); return false; }
    const uint32_t crc_val = compute_crc32_file(staged_path, err);
    if (!err.empty()) return false;
    RCLCPP_INFO(this->get_logger(), "[FLASH] 元信息 | 目标=%s | can_id=0x%X | size=%ju | crc32=0x%08X",
                target_label, target_can_id, static_cast<uintmax_t>(file_size), crc_val);

    auto u32_to_bytes = [](uint32_t v){ return std::array<uint8_t,4>{
        static_cast<uint8_t>((v >> 24) & 0xFF),
        static_cast<uint8_t>((v >> 16) & 0xFF),
        static_cast<uint8_t>((v >> 8) & 0xFF),
        static_cast<uint8_t>(v & 0xFF)
    }; };

    // 步骤2：擦除 flash（SID=0x31 01 FF 00 + addr + len）
    {
        const auto len_bytes = u32_to_bytes(static_cast<uint32_t>(file_size));
        std::vector<uint8_t> payload = {0x31, 0x01, 0xFF, 0x00, 0x44, 0x00, 0x00, 0x00, len_bytes[0], len_bytes[1], len_bytes[2], len_bytes[3]};
        if (!send_iso15765_payload(target_can_id, payload, err, iso_tp_chunk_size_)) return false;
        if (!wait_ack(0x71, "擦除flash")) return false;
    }

    // 步骤3：请求下载（SID=0x34 00 44 + addr + len）
    {
        const auto len_bytes = u32_to_bytes(static_cast<uint32_t>(file_size));
        std::vector<uint8_t> payload = {0x34, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, len_bytes[0], len_bytes[1], len_bytes[2], len_bytes[3]};
        if (!send_iso15765_payload(target_can_id, payload, err, iso_tp_chunk_size_)) return false;
        if (!wait_ack(0x74, "请求下载")) return false; // 0x74 = 0x34 正响应
    }

    // 步骤4：传输数据（多帧）
    if (!send_file_iso_tp(target_can_id, staged_path, err, iso_tp_chunk_size_)) {
        return false;
    }

    // 步骤5：传输结束（SID=0x37）
    {
        std::vector<uint8_t> sf = {0x01, 0x37, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
        if (!send_can(target_can_id, sf, err)) return false;
        if (!wait_ack(0x77, "结束传输")) return false; // 0x77 = 0x37 正响应
    }

    // 步骤6：CRC32 校验（SID=0x31 01 02 02 + CRC32）
    {
        const auto crc_bytes = u32_to_bytes(crc_val);
        std::vector<uint8_t> payload = {0x31, 0x01, 0x02, 0x02, crc_bytes[0], crc_bytes[1], crc_bytes[2], crc_bytes[3]};
        if (!send_iso15765_payload(target_can_id, payload, err, iso_tp_chunk_size_)) return false;
        if (!wait_ack(0x71, "CRC校验")) return false;
    }

    publish_ota_ack(current_up_id_, 1, 0, 0, {{pkg.package_id(), pkg.ecu(), pkg.ecu_hardware_code(), pkg.target_version(), 3, 0, "", "flashing_90%"}});
    prune_old_staged_bins(staged_path);
    return true;
}

bool ota_agent::OtaAgentNode::notify_controllers(const std::string& ecu_id, const std::string& version, std::string& err) {
    std::vector<uint8_t> payload;
    payload.insert(payload.end(), ecu_id.begin(), ecu_id.end());
    payload.push_back(0);
    payload.insert(payload.end(), version.begin(), version.end());

    const std::string payload_hex = hexify(payload);
    RCLCPP_INFO(this->get_logger(), "[CTRL-NOTIFY] 准备通知上下控 | ecu=%s | version=%s | payload_len=%zu | hex=[%s]",
                ecu_id.c_str(), version.c_str(), payload.size(), payload_hex.c_str());

    if (!send_can(0x0510, payload, err)) { // 上控
        RCLCPP_ERROR(this->get_logger(), "[CTRL-NOTIFY] 上控通知发送失败 can_id=0x0510: %s", err.c_str());
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "[CTRL-NOTIFY] 上控通知已发送 can_id=0x0510");
    if (!send_can(0x0520, payload, err)) { // 下控
        RCLCPP_ERROR(this->get_logger(), "[CTRL-NOTIFY] 下控通知发送失败 can_id=0x0520: %s", err.c_str());
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "[CTRL-NOTIFY] 下控通知已发送 can_id=0x0520");
    // 控制器通知完成后，同步清理旧的临时固件，仅保留最近一版
    prune_old_staged_bins(last_staged_path_);
    return true;
}

// 通用：将文件通过 CAN 推送到指定控制器（含版本与大小元数据），与 3588 路径一致
bool ota_agent::OtaAgentNode::flash_file_via_can(uint32_t target_can_id,
                            const std::string& version,
                            const std::string& staged_path,
                            std::string& err) {
    // 同样采用 ISO-TP 风格分段发送
    if (!send_file_iso_tp(target_can_id, staged_path, err, iso_tp_chunk_size_)) {
        return false;
    }
    return true;
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
    frame.dlc = static_cast<uint8_t>(std::min<size_t>(payload.size(), 8));
    frame.is_extended = (can_id > 0x7FF);

    // ===== 强化打印：帧完整信息 =====
    std::ostringstream oss_data;
    oss_data << std::hex << std::uppercase;
    for (size_t i=0; i<payload.size(); i++) {
        oss_data << std::setw(2) << std::setfill('0') << static_cast<int>(payload[i]);
        if (i < payload.size()-1) oss_data << " ";
    }
    RCLCPP_INFO(this->get_logger(), 
        "[CAN-TX] 帧信息 | ID=0x%X (扩展帧:%d) | DLC=%d | 数据=[%s] | 总长度=%zu",
        can_id, frame.is_extended ? 1 : 0, frame.dlc, oss_data.str().c_str(), payload.size());

    int attempts = std::max(1, can_send_retries_);
    for (int i = 0; i < attempts; ++i) {
        if (can_transport_->send(frame, err)) {
            RCLCPP_INFO(this->get_logger(), "[CAN-TX-SUCCESS] ID=0x%X | 尝试次数=%d/%d", can_id, i+1, attempts);
            return true;
        }
        if (i < attempts - 1) {
            RCLCPP_WARN(this->get_logger(), 
                "[CAN-TX-RETRY] ID=0x%X | 尝试%d/%d失败 | 原因: %s | 重试延迟=%dms",
                can_id, i+1, attempts, err.c_str(), can_retry_delay_ms_);
            std::this_thread::sleep_for(std::chrono::milliseconds(std::max(0, can_retry_delay_ms_)));
        }
    }
    RCLCPP_ERROR(this->get_logger(), 
        "[CAN-TX-FAIL] ID=0x%X | 所有%d次尝试失败 | 最终错误: %s",
        can_id, attempts, err.c_str());
    
    // 上报失败ACK（保持原有逻辑）
    if (report_can_tx_fail_ack_ && !current_up_id_.empty()) {
        std::ostringstream msg;
        msg << "can_tx_failed id=0x" << std::hex << std::uppercase << can_id << ": " << err;
        publish_ota_ack(current_up_id_, 5, 1, 5, {{current_up_id_, current_ecu_id_, "", current_version_, 11, 5, "", msg.str()}});
    }
    return false;
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

bool ota_agent::OtaAgentNode::is_ack_success(const std::vector<uint8_t>& resp) const {
    if (resp.empty()) return false;
    auto decode_negative = [](uint8_t code) -> const char* {
        switch (code) {
            case 0x10: return "general_reject";
            case 0x11: return "service_not_supported";
            case 0x12: return "subfunction_not_supported";
            case 0x13: return "incorrect_message_length";
            case 0x22: return "conditions_not_correct";
            case 0x31: return "request_out_of_range";
            case 0x33: return "security_access_denied";
            case 0x35: return "invalid_key";
            case 0x70: return "upload_download_not_accepted";
            case 0x71: return "transfer_data_suspended";
            case 0x72: return "general_programming_failure";
            case 0x73: return "wrong_block_sequence";
            case 0x78: return "request_correctly_received_response_pending";
            default:   return "uds_negative";
        }
    };

    // UDS negative response: 0x7F <sid> <reason>
    if (!resp.empty() && resp[0] == 0x7F) {
        uint8_t reason = resp.size() > 2 ? resp[2] : 0xFF;
        RCLCPP_ERROR(this->get_logger(), "UDS否定应答 reason=0x%02X (%s) raw=[%s]", reason, decode_negative(reason), hexify(resp).c_str());
        return false;
    }
    // 常规成功：最后一个字节为0x00
    const bool ok = resp.back() == 0x00;
    if (!ok) {
        RCLCPP_WARN(this->get_logger(), "应答非成功码 raw=[%s]", hexify(resp).c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "应答成功 raw=[%s]", hexify(resp).c_str());
    }
    return ok;
}

void ota_agent::OtaAgentNode::on_can_frame_rx(const CanFrame& f) {
    if (f.data.empty()) return;
    RCLCPP_INFO(this->get_logger(), "[CAN-RX] 收到帧 | id=0x%X | len=%zu | data=[%s]",
                f.id, f.data.size(), hexify(f.data).c_str());
    uint8_t sid_primary = f.data[0];
    uint8_t sid_secondary = f.data.size() > 1 ? f.data[1] : 0x00;
    std::unique_lock<std::mutex> lk(can_resp_mutex_);
    for (auto& pending : pending_can_resps_) {
        bool match = (!pending.fulfilled && pending.can_id == f.id && pending.expect_sid == sid_secondary);
        // 支持流控帧(0x30)匹配第一字节
        if (!match && pending.expect_sid == 0x30 && sid_primary == 0x30 && pending.can_id == f.id && !pending.fulfilled) {
            match = true;
        }
        if (match) {
            pending.data.assign(f.data.begin(), f.data.end());
            pending.fulfilled = true;
            can_resp_cv_.notify_all();
            break;
        }
    }
}

bool ota_agent::OtaAgentNode::wait_for_can_response(uint32_t can_id, uint8_t expect_sid, int timeout_ms, std::vector<uint8_t>& resp, std::string& err) {
    PendingCanResp pending{can_id, expect_sid, {}, false};
    {
        std::unique_lock<std::mutex> lk(can_resp_mutex_);
        pending_can_resps_.push_back(pending);
    }

    auto cleanup = [this, can_id, expect_sid]() {
        std::unique_lock<std::mutex> lk(can_resp_mutex_);
        pending_can_resps_.erase(std::remove_if(pending_can_resps_.begin(), pending_can_resps_.end(),
            [can_id, expect_sid](const PendingCanResp& p){ return p.can_id == can_id && p.expect_sid == expect_sid; }),
            pending_can_resps_.end());
    };

    std::unique_lock<std::mutex> lk(can_resp_mutex_);
    auto pred = [&](){
        for (auto& p : pending_can_resps_) {
            if (p.can_id == can_id && p.expect_sid == expect_sid && p.fulfilled) {
                resp = p.data;
                return true;
            }
        }
        return false;
    };

    if (!can_resp_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), pred)) {
        err = "wait can response timeout";
        RCLCPP_ERROR(this->get_logger(), "[CAN-RX-TIMEOUT] 等待应答超时 | can_id=0x%X | expect_sid=0x%02X | timeout_ms=%d | pending=%zu",
                     can_id, expect_sid, timeout_ms, pending_can_resps_.size());
        cleanup();
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "[CAN-RX] 收到匹配应答 | can_id=0x%X | expect_sid=0x%02X | len=%zu | data=[%s]",
                can_id, expect_sid, resp.size(), hexify(resp).c_str());
    cleanup();
    return true;
}

bool ota_agent::OtaAgentNode::send_file_iso_tp(uint32_t can_id,
                          const std::string& staged_path,
                          std::string& err,
                          size_t chunk_payload_size) {
    uintmax_t file_size = 0;
    try {
        file_size = std::filesystem::file_size(staged_path);
    } catch (const std::exception& e) {
        err = std::string("stat file failed: ") + e.what();
        return false;
    }

    chunk_payload_size = std::min<size_t>(chunk_payload_size, 7); // ISO-TP 连续帧最大7字节
    if (chunk_payload_size == 0) chunk_payload_size = 7;

    RCLCPP_INFO(this->get_logger(), "[ISO-TP] 准备分包: path=%s size=%ju chunk=%zu bytes can_id=0x%X", staged_path.c_str(), static_cast<uintmax_t>(file_size), chunk_payload_size, can_id);

    std::ifstream ifs(staged_path, std::ios::binary);
    if (!ifs.is_open()) {
        err = "open staged file failed";
        return false;
    }

    const uint32_t max_len = 0xFFF; // ISO-TP 12bit 长度
    if (file_size > max_len) {
        RCLCPP_WARN(this->get_logger(), "[ISO-TP] 文件长度 %ju 超过12bit上限，将仅发送前 %u 字节", static_cast<uintmax_t>(file_size), max_len);
    }
    const uint32_t total_len = static_cast<uint32_t>(std::min<uintmax_t>(file_size, max_len));


    RCLCPP_INFO(this->get_logger(), 
    "[ISO-TP] 分包初始化 | 文件路径=%s | 文件大小=%ju | 单帧有效载荷=%zu | 目标CAN ID=0x%X | 最大帧长度=%u",
    staged_path.c_str(), static_cast<uintmax_t>(file_size), chunk_payload_size, can_id, max_len);


    uint8_t seq = 1;
    uint32_t sent = 0;


        // 新增：进度计数器
    uint32_t total_chunks = (total_len + chunk_payload_size - 1) / chunk_payload_size;
    uint32_t current_chunk = 0;

    auto send_frame = [&](const std::vector<uint8_t>& bytes) -> bool {
        std::string e;
        if (!send_can(can_id, bytes, e)) {
            err = e;
            return false;
        }
        return true;
    };

    if (total_len <= 7) {
        std::vector<uint8_t> sf;
        sf.reserve(1 + total_len);
        sf.push_back(static_cast<uint8_t>(0x00 | total_len));
        std::array<char, 7> buf{};
        ifs.read(buf.data(), total_len);
        sf.insert(sf.end(), buf.data(), buf.data() + total_len);
        return send_frame(sf);
    }

    std::vector<uint8_t> ff;
    ff.reserve(8);
    uint8_t len_high = static_cast<uint8_t>((total_len >> 8) & 0x0F);
    uint8_t len_low  = static_cast<uint8_t>(total_len & 0xFF);
    ff.push_back(static_cast<uint8_t>(0x10 | len_high));
    ff.push_back(len_low);
    size_t first_payload = std::min<size_t>(chunk_payload_size, total_len);
    std::vector<char> buf(first_payload);
    ifs.read(buf.data(), first_payload);
    ff.insert(ff.end(), buf.begin(), buf.end());
    if (!send_frame(ff)) return false;
    sent += static_cast<uint32_t>(first_payload);

    // 等待流控帧(FC)以获取块大小与帧间隔
    uint8_t fc_bs = 0;   // 0 表示无限制
    uint8_t fc_stmin = 0;
    {
        std::vector<uint8_t> fc_resp;
        if (!wait_for_can_response(can_id, 0x30, 1000, fc_resp, err)) {
            err = "wait fc timeout";
            return false;
        }
        if (fc_resp.size() >= 3 && fc_resp[0] == 0x30) {
            fc_bs = fc_resp[1];
            fc_stmin = fc_resp[2];
            RCLCPP_INFO(this->get_logger(), "[ISO-TP] 收到FC: BS=%u STmin=0x%02X", fc_bs, fc_stmin);
        }
    }

    auto stmin_duration = [this](uint8_t st)->std::chrono::microseconds {
        if (st <= 0x7F) return std::chrono::milliseconds(static_cast<int>(st));
        if (st >= 0xF1 && st <= 0xF9) {
            int us = (st - 0xF0) * 100;
            return std::chrono::microseconds(us);
        }
        return std::chrono::microseconds(0);
    };

    const auto base_stmin = stmin_duration(fc_stmin);
    uint8_t bs_window = fc_bs;

    while (sent < total_len) {
        if (fc_bs != 0 && bs_window == 0) {
            std::vector<uint8_t> fc_resp;
            if (!wait_for_can_response(can_id, 0x30, 1000, fc_resp, err)) {
                err = "wait next fc timeout";
                return false;
            }
            if (fc_resp.size() >= 3 && fc_resp[0] == 0x30) {
                fc_bs = fc_resp[1];
                fc_stmin = fc_resp[2];
                bs_window = fc_bs;
                RCLCPP_INFO(this->get_logger(), "[ISO-TP] 续传FC: BS=%u STmin=0x%02X", fc_bs, fc_stmin);
            } else {
                err = "invalid fc frame";
                return false;
            }
        }

        size_t n = std::min<size_t>(chunk_payload_size, total_len - sent);
        std::vector<char> cbuf(n);
        ifs.read(cbuf.data(), n);
        if (ifs.gcount() != static_cast<std::streamsize>(n)) {
            err = "read truncated";
            return false;
        }
        std::vector<uint8_t> cf;
        cf.reserve(1 + n);
        cf.push_back(static_cast<uint8_t>(0x20 | (seq & 0x0F)));
        cf.insert(cf.end(), cbuf.begin(), cbuf.end());
        if (!send_frame(cf)) return false;
        ++current_chunk;
        sent += static_cast<uint32_t>(n);
        seq = static_cast<uint8_t>((seq + 1) & 0x0F);
        if (seq == 0) seq = 1; // 保持 1..15 轮转

        if (fc_bs != 0) {
            if (bs_window > 0) --bs_window;
        }

        // STmin 节流
        if (base_stmin.count() > 0) {
            std::this_thread::sleep_for(base_stmin);
        }
    }

        // 分包完成打印
    RCLCPP_INFO(this->get_logger(), 
        "[ISO-TP] 分包完成 | 总块数=%u | 实际发送=%u字节 | 目标CAN ID=0x%X",
        current_chunk, sent, can_id);

    return true;
}

bool ota_agent::OtaAgentNode::send_iso15765_payload(uint32_t can_id,
                          const std::vector<uint8_t>& payload,
                          std::string& err,
                          size_t chunk_payload_size) {
    const uint32_t max_len = 0xFFF; // 12-bit length per ISO15765
    const uint32_t total_len = static_cast<uint32_t>(std::min<size_t>(payload.size(), max_len));
    if (total_len <= 7) {
        std::vector<uint8_t> sf; sf.reserve(8);
        sf.push_back(static_cast<uint8_t>(0x00 | total_len));
        sf.insert(sf.end(), payload.begin(), payload.begin() + total_len);
        while (sf.size() < 8) sf.push_back(0x55);
        return send_can(can_id, sf, err);
    }

    chunk_payload_size = std::min<size_t>(chunk_payload_size, 7);
    if (chunk_payload_size == 0) chunk_payload_size = 7;

    uint32_t sent = 0;
    uint8_t len_high = static_cast<uint8_t>((total_len >> 8) & 0x0F);
    uint8_t len_low  = static_cast<uint8_t>(total_len & 0xFF);
    std::vector<uint8_t> ff;
    ff.reserve(8);
    ff.push_back(static_cast<uint8_t>(0x10 | len_high));
    ff.push_back(len_low);
    size_t first_payload = std::min<size_t>(chunk_payload_size, total_len);
    ff.insert(ff.end(), payload.begin(), payload.begin() + first_payload);
    while (ff.size() < 8) ff.push_back(0x55);
    if (!send_can(can_id, ff, err)) return false;
    sent += static_cast<uint32_t>(first_payload);

    // 等待流控帧，遵守 BS / STmin
    uint8_t fc_bs = 0;
    uint8_t fc_stmin = 0;
    {
        std::vector<uint8_t> fc_resp;
        if (!wait_for_can_response(can_id, 0x30, 1000, fc_resp, err)) {
            err = "wait fc timeout";
            return false;
        }
        if (fc_resp.size() >= 3 && fc_resp[0] == 0x30) {
            fc_bs = fc_resp[1];
            fc_stmin = fc_resp[2];
            RCLCPP_INFO(this->get_logger(), "[ISO-TP] 收到FC: BS=%u STmin=0x%02X", fc_bs, fc_stmin);
        }
    }
    auto stmin_duration = [](uint8_t st)->std::chrono::microseconds {
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
            if (!wait_for_can_response(can_id, 0x30, 1000, fc_resp, err)) {
                err = "wait next fc timeout";
                return false;
            }
            if (fc_resp.size() >= 3 && fc_resp[0] == 0x30) {
                fc_bs = fc_resp[1];
                fc_stmin = fc_resp[2];
                bs_window = fc_bs;
                st_gap = stmin_duration(fc_stmin);
                RCLCPP_INFO(this->get_logger(), "[ISO-TP] 续传FC: BS=%u STmin=0x%02X", fc_bs, fc_stmin);
            } else {
                err = "invalid fc frame";
                return false;
            }
        }

        size_t n = std::min<size_t>(chunk_payload_size, total_len - sent);
        std::vector<uint8_t> cf; cf.reserve(8);
        cf.push_back(static_cast<uint8_t>(0x20 | (seq & 0x0F)));
        cf.insert(cf.end(), payload.begin() + sent, payload.begin() + sent + n);
        while (cf.size() < 8) cf.push_back(0x55);
        if (!send_can(can_id, cf, err)) return false;
        sent += static_cast<uint32_t>(n);
        seq = static_cast<uint8_t>((seq + 1) & 0x0F);
        if (seq == 0) seq = 1;

        if (fc_bs != 0 && bs_window > 0) {
            --bs_window;
        }
        if (st_gap.count() > 0) {
            std::this_thread::sleep_for(st_gap);
        }
    }
    return true;
}

uint32_t ota_agent::OtaAgentNode::compute_crc32_file(const std::string& path, std::string& err) {
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

    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
        err = "open file failed";
        return 0;
    }
    uint32_t crc = 0xFFFFFFFFU;
    std::array<unsigned char, 4096> buf{};
    while (ifs.good()) {
        ifs.read(reinterpret_cast<char*>(buf.data()), buf.size());
        std::streamsize n = ifs.gcount();
        for (std::streamsize i = 0; i < n; ++i) {
            crc = crc32_tab[(crc ^ buf[static_cast<size_t>(i)]) & 0xFFU] ^ (crc >> 8);
        }
    }
    if (ifs.bad()) {
        err = "read error";
        return 0;
    }
    return crc;
}
// bool ota_agent::OtaAgentNode::send_can(uint32_t can_id, const std::vector<uint8_t>& payload, std::string& err) {
//     if (!can_transport_) {
//         err = "can transport not ready";
//         return false;
//     }
//     if (payload.empty()) {
//         err = "empty payload";
//         return false;
//     }
//     // 调试日志：打印发送帧的基本信息与数据十六进制（改为 DEBUG 以降低磁盘写入量）
//     {
//         std::ostringstream oss;
//         oss << std::hex << std::uppercase;
//         for (auto b : payload) {
//             oss << std::setw(2) << std::setfill('0') << static_cast<int>(b) << ' ';
//         }
//         RCLCPP_DEBUG(this->get_logger(), "[CAN-TX] id=0x%X ext=%d dlc=%zu data=[%s]",
//                      can_id, (can_id > 0x7FF) ? 1 : 0, std::min<size_t>(payload.size(), 8), oss.str().c_str());
//     }
//     ota_agent::CanFrame frame;
//     frame.id = can_id;
//     frame.data = payload;
//     frame.dlc = static_cast<uint8_t>(std::min<size_t>(payload.size(), 8));
//     frame.is_extended = (can_id > 0x7FF);
//     int attempts = std::max(1, can_send_retries_);
//     for (int i = 0; i < attempts; ++i) {
//         if (can_transport_->send(frame, err)) {
//             return true;
//         }
//         if (i < attempts - 1) {
//             RCLCPP_WARN(this->get_logger(), "CAN 发送失败，重试 %d/%d: %s", i + 1, attempts, err.c_str());
//             std::this_thread::sleep_for(std::chrono::milliseconds(std::max(0, can_retry_delay_ms_)));
//         }
//     }
//     return false;
// }

#include <openssl/evp.h>  // 确保添加此头文件（如果没有的话）

/**
 * @brief 计算文件MD5哈希值（使用OpenSSL EVP接口）
 * @param path 文件路径
 * @param err 错误信息输出
 * @return 十六进制MD5字符串
 */
std::string ota_agent::OtaAgentNode::compute_md5_hex(const std::string& path, std::string& err) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
        err = "open file failed";
        return "";
    }

    // 使用OpenSSL 3.0兼容的EVP接口替代已废弃的MD5接口
    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    if (!ctx) {
        err = "EVP_MD_CTX_new failed";
        ifs.close();
        return "";
    }

    // 初始化 MD5 算法（EVP_md5() 兼容所有版本）
    if (EVP_DigestInit_ex(ctx, EVP_md5(), nullptr) != 1) {
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
            // 替代 MD5_Update
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
    // 替代 MD5_Final
    if (EVP_DigestFinal_ex(ctx, md, &md_len) != 1) {
        err = "EVP_DigestFinal_ex failed";
        EVP_MD_CTX_free(ctx);
        return "";
    }

    EVP_MD_CTX_free(ctx);  // 释放上下文

    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (unsigned int i = 0; i < md_len; ++i) {
        oss << std::setw(2) << static_cast<int>(md[i]);
    }
    return oss.str();
}


bool ota_agent::OtaAgentNode::reboot_self_and_wait(std::string& err) {
    std::string cmd = self_reboot_cmd_template_;
    if (cmd.empty()) {
        // Fallback to systemctl reboot when not configured
        cmd = "systemctl reboot";
    }
    RCLCPP_INFO(this->get_logger(), "触发自重启: %s", cmd.c_str());
    int rc = std::system(cmd.c_str());
    if (rc != 0) {
        err = std::string("reboot cmd failed rc=") + std::to_string(rc);
        return false;
    }

    // Give the system some time to restart; real deployments should use watchdog/heartbeat
    const int sleep_secs = 15;
    std::this_thread::sleep_for(std::chrono::seconds(sleep_secs));
    return true;
}

bool ota_agent::OtaAgentNode::verify_self_after_reboot(const std::string& expected_version, std::string& err) {
    if (self_verify_cmd_template_.empty()) {
        return true;
    }
    std::string cmd = self_verify_cmd_template_;
    const std::string placeholder = "{version}";
    auto pos = cmd.find(placeholder);
    if (pos != std::string::npos) {
        cmd.replace(pos, placeholder.size(), expected_version);
    }

    RCLCPP_INFO(this->get_logger(), "自检命令: %s", cmd.c_str());
    int rc = std::system(cmd.c_str());
    if (rc != 0) {
        err = std::string("verify cmd failed rc=") + std::to_string(rc);
        return false;
    }
    return true;
}

bool ota_agent::OtaAgentNode::rollback_self_image(std::string& err) {
    if (self_rollback_image_path_.empty()) {
        err = "rollback image path not set";
        return false;
    }
    if (!std::filesystem::exists(self_rollback_image_path_)) {
        err = "rollback image not found";
        return false;
    }

    std::string inner_err;
    if (!flash_self_via_bootloader(self_rollback_image_path_, inner_err)) {
        err = std::string("rollback flash failed: ") + inner_err;
        return false;
    }
    return true;
}


bool ota_agent::OtaAgentNode::flash_self_via_bootloader(const std::string& staged_path, std::string& err) {
    if (!wait_if_paused_or_cancelled("flash_self")) {
        err = "task_cancelled";
        return false;
    }
    // 优先使用 CAN/ISO15765 本地 Bootloader 流程（域控自身，使用 domain CAN ID）
    const uint32_t target_can_id = static_cast<uint32_t>(can_id_domain_);
    std::error_code fs_ec;
    const uint64_t file_size = std::filesystem::file_size(staged_path, fs_ec);
    if (fs_ec) { err = std::string("stat failed: ") + fs_ec.message(); return false; }
    const uint32_t crc_val = compute_crc32_file(staged_path, err);
    if (!err.empty()) return false;

    auto u32_to_bytes = [](uint32_t v){ return std::array<uint8_t,4>{
        static_cast<uint8_t>((v >> 24) & 0xFF),
        static_cast<uint8_t>((v >> 16) & 0xFF),
        static_cast<uint8_t>((v >> 8) & 0xFF),
        static_cast<uint8_t>(v & 0xFF)
    }; };

    // 步骤1：请求升级（单帧）
    {
        std::vector<uint8_t> sf = {0x04, 0x31, 0x01, 0x02, 0x03, 0x55, 0x55, 0x55};
        if (!send_can(target_can_id, sf, err)) {
            RCLCPP_WARN(this->get_logger(), "域控自升级：请求升级失败，回退到外部脚本");
            goto fallback_cmd;
        }
    }

    // 步骤2：擦除
    {
        const auto len_bytes = u32_to_bytes(static_cast<uint32_t>(file_size));
        std::vector<uint8_t> payload = {0x31, 0x01, 0xFF, 0x00, 0x44, 0x00, 0x00, 0x00, len_bytes[0], len_bytes[1], len_bytes[2], len_bytes[3]};
        if (!send_iso15765_payload(target_can_id, payload, err, iso_tp_chunk_size_)) goto fallback_cmd;
    }

    // 步骤3：请求下载
    {
        const auto len_bytes = u32_to_bytes(static_cast<uint32_t>(file_size));
        std::vector<uint8_t> payload = {0x34, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, len_bytes[0], len_bytes[1], len_bytes[2], len_bytes[3]};
        if (!send_iso15765_payload(target_can_id, payload, err, iso_tp_chunk_size_)) goto fallback_cmd;
    }

    // 步骤4：传输数据
    if (!send_file_iso_tp(target_can_id, staged_path, err, iso_tp_chunk_size_)) goto fallback_cmd;

    // 步骤5：结束
    {
        std::vector<uint8_t> sf = {0x01, 0x37, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
        if (!send_can(target_can_id, sf, err)) goto fallback_cmd;
    }

    // 步骤6：CRC32 校验
    {
        const auto crc_bytes = u32_to_bytes(crc_val);
        std::vector<uint8_t> payload = {0x31, 0x01, 0x02, 0x02, crc_bytes[0], crc_bytes[1], crc_bytes[2], crc_bytes[3]};
        if (!send_iso15765_payload(target_can_id, payload, err, iso_tp_chunk_size_)) goto fallback_cmd;
    }

    // 自升级完成后，尝试重启并验证
    {
        std::string reboot_err;
        if (!reboot_self_and_wait(reboot_err)) {
            RCLCPP_WARN(this->get_logger(), "自升级后重启触发失败: %s", reboot_err.c_str());
        }
        std::string verify_err;
        if (!verify_self_after_reboot(current_version_, verify_err)) {
            RCLCPP_ERROR(this->get_logger(), "自升级后版本校验失败，尝试回滚: %s", verify_err.c_str());
            if (staged_path == self_rollback_image_path_) {
                RCLCPP_ERROR(this->get_logger(), "回滚镜像校验仍失败，停止递归回滚");
            } else {
                rollback_self_image(verify_err);
            }
        }
    }
    return true;

fallback_cmd:
    if (self_flash_cmd_template_.empty()) {
        // 无外部脚本可用时，返回失败以便上层处理
        if (err.empty()) err = "self flash via CAN failed";
        return false;
    }
    {
        // 模板支持 {path} 占位符
        std::string cmd = self_flash_cmd_template_;
        const std::string placeholder = "{path}";
        auto pos = cmd.find(placeholder);
        if (pos != std::string::npos) {
            cmd.replace(pos, placeholder.size(), staged_path);
        } else {
            cmd += " " + staged_path;
        }
        RCLCPP_INFO(this->get_logger(), "执行自身刷写命令(回退): %s", cmd.c_str());
        int rc = std::system(cmd.c_str());
        if (rc != 0) {
            err = std::string("self flash cmd failed rc=") + std::to_string(rc);
            return false;
        }
    }
    return true;
}

void ota_agent::OtaAgentNode::pull_watchdog_tick() {
    if (!pull_watchdog_active_.load()) return;
    if (current_up_id_.empty()) return; // 无任务
    auto now = std::chrono::steady_clock::now();
    if (now >= pull_deadline_) {
        // 超时处理：上报失败并释放占用
        RCLCPP_ERROR(this->get_logger(), "Pull模式超时：up_id=%s ecu=%s version=%s 超过 %d 秒无反馈",
                     current_up_id_.c_str(), current_ecu_id_.c_str(), current_version_.c_str(), pull_timeout_sec_);
        publish_ota_ack(current_up_id_, 5, 1, 8, {{current_up_id_, current_ecu_id_, "", current_version_, 11, 8, "", "pull_timeout"}});
        {
            std::lock_guard<std::mutex> g(upgrade_mutex_);
            upgrade_in_progress_ = false;
        }
        current_up_id_.clear();
        current_ecu_id_.clear();
        current_version_.clear();
        pull_watchdog_active_.store(false);
    }
}

void ota_agent::OtaAgentNode::prune_old_staged_bins(const std::string& keep_path) {
    try {
        const std::string dir = "/tmp";
        struct Item { std::filesystem::path path; std::filesystem::file_time_type mtime; };
        std::vector<Item> items;
        for (const auto& entry : std::filesystem::directory_iterator(dir)) {
            if (!entry.is_regular_file()) continue;
            const auto p = entry.path();
            const auto name = p.filename().string();
            if (name.rfind("ota_", 0) == 0 && name.size() > 7 && name.find(".bin") == name.size() - 4) {
                std::error_code tec;
                auto ft = std::filesystem::last_write_time(p, tec);
                if (tec) {
                    // 如果获取时间失败，保守加入列表（按最旧处理）
                    ft = std::filesystem::file_time_type::min();
                }
                items.push_back({p, ft});
            }
        }

        if (items.empty()) return;
        std::stable_sort(items.begin(), items.end(), [](const Item& a, const Item& b){ return a.mtime > b.mtime; });

        // 需要保留的集合：最新两份 + 显式 keep_path（如果存在且不同）
        std::set<std::string> keep;
        keep.insert(items[0].path.string());
        if (items.size() > 1) keep.insert(items[1].path.string());
        if (!keep_path.empty()) keep.insert(keep_path);

        size_t removed = 0;
        for (const auto& it : items) {
            const auto ps = it.path.string();
            if (keep.count(ps)) continue;
            std::error_code ec;
            std::filesystem::remove(it.path, ec);
            if (ec) {
                RCLCPP_WARN(this->get_logger(), "清理旧固件失败: %s (%s)", ps.c_str(), ec.message().c_str());
            } else {
                ++removed;
                RCLCPP_INFO(this->get_logger(), "已清理旧固件: %s", ps.c_str());
            }
        }
        RCLCPP_INFO(this->get_logger(), "固件清理完成：保留 %zu 份，删除 %zu 份", keep.size(), removed);
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "清理旧固件异常: %s", e.what());
    }
}

} // namespace ota_agent

/**
 * @brief OTA Agent节点主函数
 * 初始化ROS2环境并启动OTA代理节点
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ota_agent::OtaAgentNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}