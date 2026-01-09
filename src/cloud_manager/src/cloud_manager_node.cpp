#include <rclcpp/rclcpp.hpp>
// #include <mqtt/async_client.h> // 移除Paho
#include <mosquitto.h> // 添加mosquitto
#include <chrono>
#include <functional>
#include <cstdint>
#include <unordered_map>
#include <map>
#include <string>
#include <vector>
#include <filesystem>
#include <memory>
#include <sstream>
#include <iomanip>
#include <random>
#include <fstream>
#include <stdexcept>
#include <thread>
#include <cmath>
#include <future>
#include <atomic>
#include <clocale>
#include <codecvt>
#include <nlohmann/json.hpp>
#include <cpr/cpr.h>

// 自定义消息
#include "interface/msg/car_info.hpp"
#include "interface/msg/recharge_status.hpp"
#include "interface/msg/encoder.hpp"
#include "interface/msg/hex_data.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "interface/srv/id_controls.hpp"
#include "utils/topic.h"

// Service
#include "interface/srv/summon_status.hpp"
#include "interface/srv/follow_status.hpp"

// Protobuf
#include "car_status.pb.h"
// GPS proto
#include "gps_data.pb.h"
#include "domain_controller.pb.h"
#include "event_config.pb.h"
#include "vehicle_event.pb.h"
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// 新增：解耦的MQTT连接管理器 + 签名工具
#include "cloud_manager/mqtt_connection_manager.hpp"
#include "cloud_manager/mileage_calculator.h"
#include "cloud_manager/token_manager.hpp"
#include "cloud_manager/signature_manager.hpp"
#include "cloud_manager/domain_controller_monitor.hpp"
#include "utils/trigger_logger.h"

using namespace std::chrono_literals;
namespace cloud_manager_node {

const std::unordered_map<std::string, std::string> MODE_TO_CHINESE = {
    {"auxiliary", "辅助"}, {"cruise", "巡航"}, {"narrow", "狭窄"}, {"standard", "标准"},
    {"recharge", "回充"}, {"followme", "自动跟随"}, {"dance", "跳舞"}, {"autoindoor", "自动驾驶"}
};

constexpr int32_t MODEL_X5_CODE = 0;

struct VehicleStatus {
    float speed_kmh = 0.0f;
    int32_t remaining_mileage = 0;
    int32_t gear = 0;
    int32_t charging_status = 0;
    int32_t parking_status = 0;
    int32_t assist_mode = 0;
    int32_t battery = 0;
    int32_t left_brakests = 0;
    int32_t right_brakests = 0;
    float total_mileage = 0.0f;
    float current_interval_mileage = 0.0f;
    int32_t recharge_status = 0;
    int32_t summon_status = 0;
    int32_t follow_me_status = 0;
    std::string current_mode_chinese = "";
    int32_t alarmstatus = 0;
};

struct HistoryCache {
    int32_t recharge_status = -1;
    int32_t summon_status = -1;
    int32_t follow_me_status = -1;
    std::string current_mode_chinese = "__INIT__";
    int32_t alarmstatus = -1;
};

class CloudManagerNode : public rclcpp::Node {
public:
    CloudManagerNode() : Node("cloud_manager_node"), current_status_(), history_cache_() {
        this->get_logger().set_level(rclcpp::Logger::Level::Info);

        // 确保运行时使用 UTF-8 编码，避免签名时出现跨平台编码差异
        const char* locale_res = std::setlocale(LC_ALL, "C.UTF-8");
        if (!locale_res) {
            locale_res = std::setlocale(LC_ALL, "en_US.UTF-8");
        }
        if (!locale_res) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 未能设置 UTF-8 locale，字符串编码需要外部保证为 UTF-8");
        }

        // ========== 新增：声明ROS参数 ==========
        this->declare_parameter("mqtt.broker", "mqtts://39.106.140.222:28883");
        this->declare_parameter("mqtt.client_id", "");
        this->declare_parameter("mqtt.retry_attempts", 3);
        this->declare_parameter("mqtt.retry_delay_base", 2.0); // 指数退避基数（秒）
        // 默认为空，走JWT token认证路径
        this->declare_parameter<std::string>("mqtt.username", "");
        this->declare_parameter<std::string>("mqtt.password", "");
        this->declare_parameter<std::string>("mqtt.ca_cert", "");
        this->declare_parameter<bool>("mqtt.insecure_skip_verify", true);
        this->declare_parameter<std::string>("mqtt.cipher", "ECDHE-RSA-AES256-GCM-SHA384");
        this->declare_parameter<int>("mqtt.version", 5);  // 5 或 4(=3.1.1)
        this->declare_parameter<std::string>("mqtt.token_username", "token_user");
        this->declare_parameter<std::string>("mqtt.jwt_token", "");
        this->declare_parameter<std::string>("mqtt.base_token", "");
        this->declare_parameter<bool>("mqtt.use_param_token", false);
        this->declare_parameter<double>("mqtt.reconnect_interval_sec", 5.0);
        this->declare_parameter<int64_t>("mqtt.token_refresh_margin_ms", 300000);
        // 会话密钥缓存参数
        this->declare_parameter<bool>("token.session_cache_enable", true);
        this->declare_parameter<std::string>("token.session_cache_path", "/tmp/cloud_manager_session_keys.json");
        // 根密钥与key-no（用于获取会话密钥/Challenge等接口签名）
        this->declare_parameter<std::string>("token.root_key_no", "");            // 例如：2;ZQKJ001;3
        this->declare_parameter<std::string>("token.root_private_key", "");       // Base64或hex，优先于内置默认
        this->declare_parameter<std::string>("token.root_public_key", "");        // 可选
        // 声明签名相关参数（可通过命令行或参数文件注入 hex 格式私钥/公钥）
        this->declare_parameter<std::string>("signature.private_key", "");
        this->declare_parameter<std::string>("signature.public_key", "");
        // 日志开关：是否打印“即将上报”明细，默认开启便于联调
        this->declare_parameter<bool>("log.pre_publish", true);
        // 是否打印完整JWT令牌（默认关闭，避免泄露）
        this->declare_parameter<bool>("log.token_full", false);
        // 默认关闭里程计算器打印，如需调试可通过参数开启
        this->declare_parameter<bool>("log.mileage", false);
        // GPS模拟参数（无真机时上报用）
        this->declare_parameter<bool>("gps.simulate", false);
        this->declare_parameter<double>("gps.sim_rate_hz", 1.0);
        this->declare_parameter<double>("gps.sim_lat", 39.9042);   // 默认北京坐标
        this->declare_parameter<double>("gps.sim_lon", 116.4074);
        this->declare_parameter<double>("gps.sim_alt", 50.0);
        // 设备与事件配置参数
        this->declare_parameter<std::string>("device.model", "x5");
        this->declare_parameter<std::string>("device.id", "ZQKJ001");
        this->declare_parameter<std::string>("vin", "");
        this->declare_parameter<std::string>("model_str", "");
        // 主题中的车型段与业务中的模型名解耦：主题固定用简写（默认 x5）
        this->declare_parameter<std::string>("topic.model", "x5");
        this->declare_parameter<std::string>("protocol.version", "1");
        this->declare_parameter<std::string>("software.version", "1.0.0");
        this->declare_parameter<std::string>("event.config_path", "/tmp/event_config.bin");
        this->declare_parameter<std::string>("event.config_json_path", "/tmp/event_config.json");
        this->declare_parameter<bool>("event.http_enable", true);
        this->declare_parameter<double>("event.http_interval_sec", 600.0);
        // API基础地址（用于令牌/配置等接口）
        this->declare_parameter<std::string>("api_base_url", device_info_.api_base_url);
        this->declare_parameter<std::string>(
            "event.http_url", device_info_.api_base_url + "/api/device/v1/config/getUp");
        // 域控制器监控来源与文件路径参数
        this->declare_parameter<std::string>("domain_monitor.status_file", "/tmp/domain_controller_status.txt");
        this->declare_parameter<std::string>("metrics.source", "system"); // file|system|auto

        // ========== 1. 初始化令牌管理器（原有逻辑） ==========
        device_model_ = this->get_parameter("device.model").as_string();
        device_id_ = this->get_parameter("device.id").as_string();
        // 兼容旧参数：若提供 vin/model_str 则覆盖
        {
            const std::string vin_alias = this->get_parameter("vin").as_string();
            if (!vin_alias.empty()) {
                device_id_ = vin_alias;
            }
            const std::string model_alias = this->get_parameter("model_str").as_string();
            if (!model_alias.empty()) {
                device_model_ = model_alias;
            }
        }
        protocol_version_ = this->get_parameter("protocol.version").as_string();
        software_version_ = this->get_parameter("software.version").as_string();
        event_config_path_ = this->get_parameter("event.config_path").as_string();
        event_config_json_path_ = this->get_parameter("event.config_json_path").as_string();
        event_http_enabled_ = this->get_parameter("event.http_enable").as_bool();
        event_http_interval_sec_ = this->get_parameter("event.http_interval_sec").as_double();
        // 覆盖API基础地址
        device_info_.api_base_url = this->get_parameter("api_base_url").as_string();
        event_http_url_ = this->get_parameter("event.http_url").as_string();
        device_info_.vin = device_id_;
        device_info_.model = device_model_;
        // 主题路径中的车型段（独立于业务 model 字段）
        topic_model_ = this->get_parameter("topic.model").as_string();
        if (topic_model_.empty()) topic_model_ = "x5";
        // 覆盖根密钥参数与 gkey-no（用于获取会话密钥/Challenge等接口）
        {
            const std::string root_key_no = this->get_parameter("token.root_key_no").as_string();
            const std::string root_priv = this->get_parameter("token.root_private_key").as_string();
            const std::string root_pub  = this->get_parameter("token.root_public_key").as_string();
            if (!root_key_no.empty()) {
                device_info_.key_no = root_key_no;
                RCLCPP_INFO(this->get_logger(), "🔐 使用外部提供的 gkey-no（前8）：%s", root_key_no.substr(0,8).c_str());
            }
            if (!root_priv.empty()) {
                device_info_.root_private_key = root_priv;
                RCLCPP_INFO(this->get_logger(), "🔐 已注入根私钥（Base64/hex，长度=%zu）", root_priv.size());
            }
            if (!root_pub.empty()) {
                device_info_.root_public_key = root_pub;
            }
        }
        // 优先使用参数中注入的私钥（hex），否则保持原有 mock 值以兼容旧逻辑
        std::string param_sk = this->get_parameter("signature.private_key").as_string();
        if (!param_sk.empty()) {
            device_info_.private_key = param_sk;
            RCLCPP_INFO(this->get_logger(), "🔑 从参数加载私钥（hex）");
        } else {
            device_info_.private_key = "MOCK_ED25519_PRIVATE_KEY";
        }

        // 阶段1：先尝试加载会话密钥缓存，失败则通过接口获取并缓存
        cloud_manager::SessionKeyResult session_res;
        const bool cache_enable = this->get_parameter("token.session_cache_enable").as_bool();
        const std::string cache_path = this->get_parameter("token.session_cache_path").as_string();

        auto load_session_cache = [&](cloud_manager::SessionKeyResult &out)->bool {
            try {
                if (!cache_enable) return false;
                if (!std::filesystem::exists(cache_path)) return false;
                std::ifstream in(cache_path);
                if (!in.is_open()) return false;
                nlohmann::json j;
                in >> j;
                if (!j.contains("private_key") || !j.contains("public_key")) return false;
                out.private_key = j.value("private_key", "");
                out.public_key = j.value("public_key", "");
                out.key_no = j.value("key_no", "");
                if (out.private_key.empty() || out.public_key.empty()) return false;
                out.success = true;
                return true;
            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "会话密钥缓存读取失败：%s", e.what());
                return false;
            }
        };

        auto save_session_cache = [&](const cloud_manager::SessionKeyResult &res){
            try {
                if (!cache_enable || !res.success) return;
                std::filesystem::path p(cache_path);
                std::error_code ec;
                std::filesystem::create_directories(p.parent_path(), ec);
                nlohmann::json j{
                    {"private_key", res.private_key},
                    {"public_key",  res.public_key},
                    {"key_no",      res.key_no}
                };
                std::ofstream out(cache_path);
                out << j.dump(2);
                RCLCPP_INFO(this->get_logger(), "✅ 会话密钥已缓存到: %s", cache_path.c_str());
            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "会话密钥缓存写入失败：%s", e.what());
            }
        };

        if (load_session_cache(session_res)) {
            RCLCPP_INFO(this->get_logger(), "📦 使用缓存会话密钥（公钥前20：%s）",
                        session_res.public_key.substr(0, 20).c_str());
        } else {
            session_res = token_mgr_.get_session_key(device_info_);
            if (session_res.success) {
                RCLCPP_INFO(this->get_logger(), "✅ 会话密钥获取成功（公钥前20：%s）",
                            session_res.public_key.substr(0, 20).c_str());
                save_session_cache(session_res);
            } else {
                RCLCPP_WARN(this->get_logger(), "⚠️ 会话密钥获取失败：%s", session_res.err_msg.c_str());
            }
        }

        if (session_res.success) {
            // 更新设备会话密钥（供后续请求/加签/令牌申请使用）
            device_info_.session_private_key = session_res.private_key;
            device_info_.session_public_key = session_res.public_key;
            device_info_.key_no = session_res.key_no;
        } else {
            // 保持现有私钥兜底
            RCLCPP_WARN(this->get_logger(), "继续使用本地私钥兜底");
        }

        cloud_manager::TokenResult jwt_res;
        log_token_full_ = this->get_parameter("log.token_full").as_bool();
        auto log_token = [&](const cloud_manager::TokenResult& token, const std::string& label) {
            if (!token.success) return;
            if (log_token_full_) {
                RCLCPP_WARN(this->get_logger(), "%s JWT（完整）：%s", label.c_str(), token.jwt_token.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "%s JWT（前20）：%s", label.c_str(), token.jwt_token.substr(0, 20).c_str());
            }
        };
        const bool use_param_token = this->get_parameter("mqtt.use_param_token").as_bool();
        const std::string jwt_param = this->get_parameter("mqtt.jwt_token").as_string();
        const std::string base_token_param = this->get_parameter("mqtt.base_token").as_string();
        if (use_param_token && !base_token_param.empty()) {
            jwt_res = token_mgr_.get_mqtt_token_v1(device_info_, base_token_param);
            if (jwt_res.success) {
                log_token(jwt_res, "✅ 使用base_token获取MQTT令牌");
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ 基于base_token获取MQTT令牌失败：%s", jwt_res.err_msg.c_str());
            }
        } else if (use_param_token && !jwt_param.empty()) {
            jwt_res.jwt_token = jwt_param;
            jwt_res.exp = 3600000;
            jwt_res.success = true;
            log_token(jwt_res, "✅ 使用参数JWT令牌");
        }

        if (!jwt_res.success) {
            // 使用接口真实获取令牌（基于设备信息）
            jwt_res = token_mgr_.get_trusted_token(device_info_);
            if (jwt_res.success) {
                log_token(jwt_res, "✅ 令牌获取成功");
                RCLCPP_INFO(this->get_logger(), "有效期%ldms", jwt_res.exp);
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ 令牌获取失败：%s", jwt_res.err_msg.c_str());
            }
        }

        if (jwt_res.success) {
            current_token_ = jwt_res;
            update_token_expiry(jwt_res);
        }

        // ========== 新增：初始化签名工具（解耦核心） ==========
        cloud_manager::SignatureConfig sign_config;
        if (session_res.success) {
            // 公钥用于验签，私钥用于加签
            sign_config.private_key = session_res.private_key;  // 会话私钥，加签
            sign_config.public_key = session_res.public_key;    // 会话公钥，验签
        } else {
            // 测试环境：若未提供真实私钥，自动生成 seed 并由 SignatureManager 派生 keypair
            if (device_info_.private_key.empty() || device_info_.private_key.find("MOCK") != std::string::npos) {
                sign_config.private_key = cloud_manager::generate_seed_hex();
            } else {
                sign_config.private_key = device_info_.private_key; // 复用设备私钥
            }
        }
        sign_config.sign_separator = "|";               // 数据-签名分隔符
        signature_mgr_ = std::make_shared<cloud_manager::SignatureManager>(
            sign_config, this->get_logger()
        );

        // 固定协议命名的 topic 基础路径（主题使用简写车型段，如 x5）
        // 实时 info   => vehicle/up/<topic_model>/<id>/<ver>/info
        // 状态 status => vehicle/up/<topic_model>/<id>/<ver>/status
        base_topic_ = "vehicle/up/" + topic_model_ + "/" + device_id_ + "/" + protocol_version_;
        // 配置与事件主题也使用 topic_model_
        event_config_topic_ = "vehicle/config/" + topic_model_ + "/" + protocol_version_ + "/up";
        event_config_ack_topic_ = "vehicle/config/" + topic_model_ + "/" + protocol_version_ + "/ack";
        event_push_topic_ = "vehicle/event/" + topic_model_ + "/" + device_id_ + "/" + protocol_version_ + "/push";
        event_ack_topic_ = "vehicle/event/" + topic_model_ + "/" + device_id_ + "/" + protocol_version_ + "/ack";
        
        // ========== 2. 初始化MQTT连接管理器（解耦核心） ==========
        cloud_manager::MqttConnConfig mqtt_config;
        mqtt_config.broker = this->get_parameter("mqtt.broker").as_string();
        mqtt_config.client_id = this->get_parameter("mqtt.client_id").as_string();
        if (mqtt_config.client_id.empty()) {
            mqtt_config.client_id = device_id_; // 默认使用设备SN作为ClientID
        }
        mqtt_config.username = this->get_parameter("mqtt.username").as_string();
        mqtt_config.password = this->get_parameter("mqtt.password").as_string();
        mqtt_config.ca_cert_path = this->get_parameter("mqtt.ca_cert").as_string();
        mqtt_config.insecure_skip_verify = this->get_parameter("mqtt.insecure_skip_verify").as_bool();
        mqtt_config.tls_cipher = this->get_parameter("mqtt.cipher").as_string();
        mqtt_config.mqtt_version = this->get_parameter("mqtt.version").as_int();
        if (mqtt_config.mqtt_version != 5 && mqtt_config.mqtt_version != 4) {
            RCLCPP_WARN(this->get_logger(), "MQTT version %d 非法，强制使用5", mqtt_config.mqtt_version);
            mqtt_config.mqtt_version = 5;
        }
        // token认证用户名（用于JWT认证）
        if (this->has_parameter("mqtt.token_username")) {
            mqtt_config.token_username = this->get_parameter("mqtt.token_username").as_string();
        } else {
            // 默认值
            mqtt_config.token_username = "token_user";
        }
        if (mqtt_config.username.empty() && mqtt_config.password.empty()) {
            mqtt_config.use_token_auth = true; // 保持缺省回退 token
        }
        mqtt_username_ = mqtt_config.username;
        mqtt_ca_cert_ = mqtt_config.ca_cert_path;
        mqtt_insecure_skip_verify_ = mqtt_config.insecure_skip_verify;
        mqtt_reconnect_interval_sec_ = this->get_parameter("mqtt.reconnect_interval_sec").as_double();
        if (mqtt_reconnect_interval_sec_ < 1.0) mqtt_reconnect_interval_sec_ = 1.0;
        token_refresh_margin_ms_ = this->get_parameter("mqtt.token_refresh_margin_ms").as_int();
        if (token_refresh_margin_ms_ <= 0) token_refresh_margin_ms_ = 300000;
        using_token_auth_ = mqtt_config.use_token_auth && mqtt_config.username.empty() && mqtt_config.password.empty();
        log_pre_publish_ = this->get_parameter("log.pre_publish").as_bool();
        log_mileage_ = this->get_parameter("log.mileage").as_bool();
        int retry_attempts = this->get_parameter("mqtt.retry_attempts").as_int();
        double retry_delay_base = this->get_parameter("mqtt.retry_delay_base").as_double();
        mqtt_manager_ = std::make_shared<cloud_manager::MqttConnectionManager>(
            mqtt_config, this->get_logger()
        );
        mqtt_manager_->set_disconnect_callback([this](int rc) { this->on_mqtt_disconnected(rc); });
        mqtt_manager_->set_message_callback([this](const std::string& topic, const std::string& payload) {
            this->handle_mqtt_message(topic, payload);
        });

        // 建立MQTT连接（传入令牌，当前仅日志打印），带重试
        bool connected = false;
        cloud_manager::TokenResult empty_token;
        for (int attempt = 0; attempt < retry_attempts; ++attempt) {
            cloud_manager::TokenResult token_to_use = empty_token;
            if (using_token_auth_) {
                token_to_use = current_token_;
                if (!token_to_use.success) {
                    auto refreshed = token_mgr_.get_trusted_token(device_info_);
                    if (refreshed.success) {
                        current_token_ = refreshed;
                        update_token_expiry(refreshed);
                        token_to_use = refreshed;
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "❌ 获取令牌失败，跳过本次连接：%s", refreshed.err_msg.c_str());
                        token_to_use = empty_token;
                    }
                }
            }
            // 避免使用空令牌触发 MQTT 未授权错误
            if (using_token_auth_ && !token_to_use.success) {
                if (attempt < retry_attempts - 1) {
                    double delay = retry_delay_base * std::pow(2.0, attempt);
                    RCLCPP_WARN(this->get_logger(), "🔄 令牌不可用，等待 %.1f 秒后重试（%d/%d）...", delay, attempt + 1, retry_attempts);
                    std::this_thread::sleep_for(std::chrono::duration<double>(delay));
                    continue;
                } else {
                    RCLCPP_FATAL(this->get_logger(), "❌ 令牌不可用，已达最大重试次数，放弃连接");
                    break;
                }
            }

            if (mqtt_manager_->connect(token_to_use)) {
                connected = true;
                if (utils::TriggerLogger::getInstance().isInitialized()) {
                    auto log_data = std::map<std::string, std::string>{
                        {"broker", mqtt_config.broker},
                        {"client_id", mqtt_config.client_id},
                        {"use_token_auth", mqtt_config.use_token_auth ? "1" : "0"}
                    };
                    TRIGGER_LOG_WITH_NODE("mqtt_connect_ok", log_data, this->get_name());
                }
                break;
            }
            if (attempt < retry_attempts - 1) {
                double delay = retry_delay_base * std::pow(2.0, attempt);
                RCLCPP_WARN(this->get_logger(), "🔄 MQTT连接失败，重试 %d/%d，等待 %.1f 秒...", 
                           attempt + 1, retry_attempts, delay);
                std::this_thread::sleep_for(std::chrono::duration<double>(delay));
            }
        }
        if (!connected) {
            RCLCPP_FATAL(this->get_logger(), "❌ MQTT连接失败（已重试 %d 次），节点初始化失败", retry_attempts);
            if (utils::TriggerLogger::getInstance().isInitialized()) {
                auto log_data = std::map<std::string, std::string>{
                    {"broker", mqtt_config.broker},
                    {"client_id", mqtt_config.client_id},
                    {"retry_attempts", std::to_string(retry_attempts)}
                };
                TRIGGER_LOG_WITH_NODE("mqtt_connect_fail", log_data, this->get_name());
            }
            throw std::runtime_error("MQTT连接失败");
        }
        int cfg_sub_rc = mqtt_manager_->subscribe(event_config_topic_, 1);
        if (cfg_sub_rc == MOSQ_ERR_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "✅ 订阅事件配置主题成功：%s", event_config_topic_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ 订阅事件配置主题失败：%s", mosquitto_strerror(cfg_sub_rc));
        }
        int ack_sub_rc = mqtt_manager_->subscribe(event_ack_topic_, 0);
        if (ack_sub_rc == MOSQ_ERR_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "✅ 订阅事件回执主题成功：%s", event_ack_topic_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ 订阅事件回执主题失败：%s", mosquitto_strerror(ack_sub_rc));
        }
        // 获取客户端供上报逻辑使用
        // mqtt_client_ = mqtt_manager_->get_client(); // 移除，使用mqtt_manager_直接调用

        // 定期重连/令牌刷新
        mqtt_reconnect_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(mqtt_reconnect_interval_sec_),
            [this]() { this->ensure_mqtt_connected(); }
        );
        if (using_token_auth_) {
            token_refresh_timer_ = this->create_wall_timer(
                30s,
                [this]() { this->maybe_refresh_token(); }
            );
        }

        // ========== 原有逻辑保留（仅替换mqtt_client_来源） ==========
        init_mileage_calculator();
        subscribe_ros_topics();
        create_service_clients();
        schedule_vehicle_id_request();
        init_gps_simulator();
        init_timers();
        // 设置监控文件路径并立即刷新
        domain_monitor_.set_status_file(this->get_parameter("domain_monitor.status_file").as_string());
        domain_monitor_.refresh();
        check_and_report_passive_status();

        event_push_sub_ = this->create_subscription<std_msgs::msg::String>(
            "cloud/event/push", 10, std::bind(&CloudManagerNode::on_event_push_request, this, std::placeholders::_1)
        );

        if (event_http_interval_sec_ < 1.0) event_http_interval_sec_ = 60.0;
        if (event_http_enabled_) {
            event_http_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(event_http_interval_sec_),
                [this]() { this->maybe_fetch_event_config_http(); }
            );
            maybe_fetch_event_config_http();
        }

        // 错误事件发布者（十六进制错误码）
        error_event_pub_ = this->create_publisher<interface::msg::HexData>(errorEventTopic, 10);
    }

    ~CloudManagerNode() {
        if (mqtt_manager_ && mqtt_manager_->is_connected()) {
            mqtt_manager_->disconnect();
            RCLCPP_INFO(this->get_logger(), "🔌 MQTT连接已断开");
        }
    }

private:
    // 原有函数完全保留（仅修改日志，不改动逻辑）
    void init_mileage_calculator() {
        const std::string config_path = "src/cloud_manager/config/mileage_config.ini";
        std::filesystem::path config_file(config_path);
        if (!std::filesystem::exists(config_file.parent_path())) {
            std::filesystem::create_directories(config_file.parent_path());
        }
        mileage_calculator_ = std::make_unique<MileageCalculator>(0.45f, config_path, log_mileage_);
        current_status_.total_mileage = mileage_calculator_->get_total_mileage();
        current_status_.current_interval_mileage = mileage_calculator_->get_current_mileage();
        RCLCPP_INFO(this->get_logger(), "📊 里程初始化完成：总里程=%.2fm，当次里程=%.2fm",
                    current_status_.total_mileage, current_status_.current_interval_mileage);
    }

    void subscribe_ros_topics() {
        car_info_sub_ = this->create_subscription<interface::msg::CarInfo>(
            carInfoTopic, 10, std::bind(&CloudManagerNode::car_info_callback, this, std::placeholders::_1)
        );
        recharge_sub_ = this->create_subscription<interface::msg::RechargeStatus>(
            rechargeStatusTopic, 10, std::bind(&CloudManagerNode::recharge_callback, this, std::placeholders::_1)
        );
        scene_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "sceneServiceModeTopic", 10, std::bind(&CloudManagerNode::scene_mode_callback, this, std::placeholders::_1)
        );
        encoder_sub_ = this->create_subscription<interface::msg::Encoder>(
            carEncoderTopic, 10, std::bind(&CloudManagerNode::encoder_callback, this, std::placeholders::_1)
        );
        // GPS 订阅
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gpsTopic, 10, std::bind(&CloudManagerNode::gps_callback, this, std::placeholders::_1)
        );
        vehicle_id_sub_ = this->create_subscription<std_msgs::msg::String>(
            idControlService, 10, std::bind(&CloudManagerNode::vehicle_id_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "📥 ROS2主题订阅完成");
    }

    void create_service_clients() {
        summon_client_ = this->create_client<interface::srv::SummonStatus>(summonStatusService);
        follow_client_ = this->create_client<interface::srv::FollowStatus>(followControlStatusService);
        id_client_ = this->create_client<interface::srv::IdControls>(idControlService);
        RCLCPP_INFO(this->get_logger(), "📞 Service客户端创建完成");
    }

    // 将某个 key=value 写入状态文件（替换已有 key 或追加），用于解耦与外部守护脚本通信
    void write_status_key(const std::string &key, const std::string &value) {
        std::string status_file = "/tmp/domain_controller_status.txt";
        std::ifstream ifs(status_file);
        std::string content;
        if (ifs.is_open()) {
            std::string line;
            while (std::getline(ifs, line)) content += line + "\n";
            ifs.close();
        }
        std::istringstream iss(content);
        std::string out;
        std::string line;
        bool replaced = false;
        while (std::getline(iss, line)) {
            if (line.find(key + "=") == 0) {
                out += key + "=" + value + "\n";
                replaced = true;
            } else {
                out += line + "\n";
            }
        }
        if (!replaced) out += key + "=" + value + "\n";
        std::ofstream ofs(status_file, std::ios::trunc);
        if (ofs.is_open()) {
            ofs << out;
            ofs.close();
        }
    }

    void vehicle_id_callback(const std_msgs::msg::String::SharedPtr msg) {
        // 使用统一 helper 将 vehicle_id 写入状态文件并刷新监控器
        write_status_key("vehicle_id", msg->data);
        domain_monitor_.refresh();
    }

    void init_timers() {
        realtime_pub_timer_ = this->create_wall_timer(
            60s, std::bind(&CloudManagerNode::publish_realtime_msg, this)
        );
        //暂时应云端联调要求 暂时改为1min上传一次 后续根据项目实际需求进行调整
        service_query_timer_ = this->create_wall_timer(
            60s, std::bind(&CloudManagerNode::query_service_status, this)
        );
        // 域控制器状态定时上报（仅实时上报）
        domain_ctrl_timer_ = this->create_wall_timer(
            60s, std::bind(&CloudManagerNode::publish_domain_controller, this)
        );
        // GPS数据定时上报（每分钟一次），与驱动发布解耦
        gps_pub_timer_ = this->create_wall_timer(
            60s, std::bind(&CloudManagerNode::publish_gps_tick, this)
        );
        RCLCPP_INFO(this->get_logger(), "⏱️ 定时器初始化完成");
    }

    // 无GPS源时模拟上报 NavSatFix，方便联调
    void init_gps_simulator() {
        simulate_gps_ = this->get_parameter("gps.simulate").as_bool();
        if (!simulate_gps_) return;

        // 模拟GPS固定为每分钟一次
        double rate_hz = 1.0 / 60.0;
        sim_lat_ = this->get_parameter("gps.sim_lat").as_double();
        sim_lon_ = this->get_parameter("gps.sim_lon").as_double();
        sim_alt_ = this->get_parameter("gps.sim_alt").as_double();

        gps_sim_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gpsTopic, 10);
        auto period_ms = std::chrono::duration_cast<std::chrono::milliseconds>(60s);
        gps_sim_timer_ = this->create_wall_timer(period_ms, [this]() {
            sensor_msgs::msg::NavSatFix msg;
            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = "gps";
            msg.latitude = sim_lat_;
            msg.longitude = sim_lon_;
            msg.altitude = sim_alt_;
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
            msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
            // 记录模拟GPS消息的时间戳，便于区分来源
            last_sim_pub_stamp_sec_ = static_cast<int32_t>(msg.header.stamp.sec);
            last_sim_pub_stamp_nsec_ = static_cast<uint32_t>(msg.header.stamp.nanosec);
            gps_sim_pub_->publish(msg);
        });

        RCLCPP_WARN(this->get_logger(), "⚠️ 启用GPS模拟，上报固定坐标(%.6f,%.6f,%.2f) 每60s一次",
                    sim_lat_, sim_lon_, sim_alt_);
    }

    // 启动后主动请求车辆ID（服务 /vlago/car_id），成功后写入状态文件供监控使用
    void schedule_vehicle_id_request() {
        if (vehicle_id_timer_) return;
        vehicle_id_timer_ = this->create_wall_timer(
            1s,
            [this]() {
                if (vehicle_id_timer_) vehicle_id_timer_->cancel();
                request_vehicle_id();
            }
        );
    }

    void request_vehicle_id() {
        auto logger = this->get_logger();
        if (!id_client_) {
            RCLCPP_WARN(logger, "车辆ID服务客户端未初始化");
            return;
        }
        if (!id_client_->wait_for_service(3s)) {
            RCLCPP_WARN(logger, "等待车辆ID服务超时");
            return;
        }

        auto req = std::make_shared<interface::srv::IdControls::Request>();
        req->id = 0; // 请求体暂未使用，发送占位
        auto future = id_client_->async_send_request(req);
        if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
            RCLCPP_WARN(logger, "车辆ID请求响应超时");
            return;
        }

        try {
            auto resp = future.get();
            vehicle_id_ = std::to_string(resp->carid);
            write_status_key("vehicle_id", vehicle_id_);
            domain_monitor_.refresh();
            RCLCPP_INFO(logger, "✅ 获取车辆ID成功：%s", vehicle_id_.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(logger, "车辆ID请求异常：%s", e.what());
        }
    }

    // 生成16字节hex随机数作为nonce
    std::string make_nonce() const {
        static thread_local std::mt19937_64 gen(std::random_device{}());
        std::uniform_int_distribution<uint64_t> dis;
        uint64_t a = dis(gen), b = dis(gen);
        std::ostringstream ss;
        ss << std::hex << std::setw(16) << std::setfill('0') << a
           << std::hex << std::setw(16) << std::setfill('0') << b;
        return ss.str();
    }

    // 将输入字符串规范化为 UTF-8；转换失败时返回空串以避免非 UTF-8 参与签名
    std::string ensure_utf8(const std::string& input) const {
        std::wstring_convert<std::codecvt_utf8<wchar_t>> conv;
        try {
            auto wide = conv.from_bytes(input);
            return conv.to_bytes(wide);
        } catch (const std::range_error&) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 检测到非 UTF-8 字符串，已清空该字段避免验签失败");
            return "";
        }
    }

    int64_t now_ms() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    void update_token_expiry(const cloud_manager::TokenResult& token) {
        int64_t fallback = 3600000; // 1h 默认
        token_expire_at_ms_ = now_ms() + (token.exp > 0 ? token.exp : fallback);
    }

    bool ensure_mqtt_connected(bool force_refresh_token = false) {
        if (!mqtt_manager_) return false;
        if (mqtt_manager_->is_connected() && !force_refresh_token) return true;

        if (reconnecting_.exchange(true)) {
            return mqtt_manager_->is_connected();
        }

        cloud_manager::TokenResult token_to_use;
        if (using_token_auth_) {
            if (force_refresh_token || !current_token_.success ||
                (token_expire_at_ms_ > 0 && (token_expire_at_ms_ - now_ms()) < token_refresh_margin_ms_)) {
                auto refreshed = token_mgr_.get_trusted_token(device_info_);
                if (!refreshed.success) {
                    RCLCPP_ERROR(this->get_logger(), "❌ 令牌刷新失败：%s", refreshed.err_msg.c_str());
                    reconnecting_ = false;
                    return false;
                }
                current_token_ = refreshed;
                update_token_expiry(refreshed);
                RCLCPP_INFO(this->get_logger(), "🔄 令牌已刷新，距到期(ms)：%ld", refreshed.exp);
            }
            token_to_use = current_token_;
        }

        bool ok = mqtt_manager_->force_reconnect(token_to_use);
        reconnecting_ = false;
        if (!ok) {
            RCLCPP_WARN(this->get_logger(), "⚠️ MQTT重连失败");
        }
        return ok;
    }

    void maybe_refresh_token() {
        if (!using_token_auth_) return;
        if (refreshing_token_.exchange(true)) return;

        if (token_expire_at_ms_ <= 0) {
            refreshing_token_ = false;
            return;
        }
        int64_t remain = token_expire_at_ms_ - now_ms();
        if (remain > token_refresh_margin_ms_) {
            refreshing_token_ = false;
            return;
        }

        auto refreshed = token_mgr_.get_trusted_token(device_info_);
        if (refreshed.success) {
            current_token_ = refreshed;
            update_token_expiry(refreshed);
            RCLCPP_INFO(this->get_logger(), "🔄 令牌自动刷新成功，重新连接MQTT");
            ensure_mqtt_connected(true);
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ 令牌自动刷新失败：%s", refreshed.err_msg.c_str());
        }
        refreshing_token_ = false;
    }

    void on_mqtt_disconnected(int rc) {
        RCLCPP_WARN(this->get_logger(), "⚠️ MQTT断开，rc=%d，计划自动重连", rc);
    }

    std::string base64_decode(const std::string& in) const {
        static const std::string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::vector<int> T(256, -1);
        for (int i = 0; i < 64; ++i) T[static_cast<size_t>(chars[i])] = i;
        std::string out;
        int val = 0, valb = -8;
        for (unsigned char c : in) {
            if (c == '=') break;
            if (T[c] == -1) continue;
            val = (val << 6) + T[c];
            valb += 6;
            if (valb >= 0) {
                out.push_back(static_cast<char>((val >> valb) & 0xFF));
                valb -= 8;
            }
        }
        return out;
    }

    std::string get_vehicle_sn() const {
        return !vehicle_id_.empty() ? vehicle_id_ : device_id_;
    }

    void handle_mqtt_message(const std::string& topic, const std::string& payload) {
        if (topic == event_config_topic_) {
            handle_event_config_payload(payload, "mqtt");
        } else if (topic == event_ack_topic_) {
            on_event_ack_message(payload);
        }
    }

    bool handle_event_config_payload(const std::string& payload, const std::string& source) {
        vehicle::config::EventConfigPackage pkg;
        if (!pkg.ParseFromString(payload)) {
            RCLCPP_ERROR(this->get_logger(), "❌ 事件配置解析失败（来源=%s）", source.c_str());
            return false;
        }
        if (pkg.config_id() == current_config_id_ && pkg.version() == current_config_version_str_) {
            RCLCPP_INFO(this->get_logger(), "ℹ️ 事件配置未变化，跳过处理（source=%s id=%s version=%s）",
                        source.c_str(), pkg.config_id().c_str(), pkg.version().c_str());
            return true;
        }
        persist_event_config(pkg, payload, source);
        send_event_config_ack(pkg);
        return true;
    }

    void persist_event_config(const vehicle::config::EventConfigPackage& pkg,
                              const std::string& raw,
                              const std::string& source) {
        try {
            std::filesystem::path bin_path(event_config_path_);
            if (!bin_path.parent_path().empty()) {
                std::filesystem::create_directories(bin_path.parent_path());
            }
            std::ofstream ofs(bin_path, std::ios::binary | std::ios::trunc);
            ofs.write(raw.data(), static_cast<std::streamsize>(raw.size()));
            ofs.close();

            nlohmann::json meta;
            meta["config_id"] = pkg.config_id();
            meta["timestamp"] = pkg.timestamp();
            meta["version"] = pkg.version();
            meta["configs_count"] = pkg.configs_size();
            meta["source"] = source;
            for (const auto& kv : pkg.extras()) {
                meta["extras"][kv.first] = kv.second;
            }
            std::filesystem::path json_path(event_config_json_path_);
            if (!json_path.parent_path().empty()) {
                std::filesystem::create_directories(json_path.parent_path());
            }
            std::ofstream jofs(json_path, std::ios::trunc);
            jofs << meta.dump(2);
            jofs.close();

            current_config_id_ = pkg.config_id();
            current_config_version_str_ = pkg.version();
            try {
                current_config_version_num_ = std::stoll(pkg.version());
            } catch (...) {
                current_config_version_num_ = 0;
            }

            RCLCPP_INFO(this->get_logger(), "✅ 事件配置已保存：id=%s version=%s source=%s configs=%d",
                        pkg.config_id().c_str(), pkg.version().c_str(), source.c_str(), pkg.configs_size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ 事件配置持久化失败：%s", e.what());
        }
    }

    void send_event_config_ack(const vehicle::config::EventConfigPackage& pkg) {
        vehicle::config::EventConfigAck ack;
        ack.set_config_id(pkg.config_id());
        ack.set_timestamp(now_ms());
        ack.set_version(pkg.version());
        ack.set_sn(get_vehicle_sn());
        publish_passive_msg(event_config_ack_topic_, "事件配置回执", ack);
    }

    bool maybe_fetch_event_config_http() {
        if (!event_http_enabled_) return false;
        try {
            nlohmann::json body;
            body["sn"] = get_vehicle_sn();
            body["model"] = device_model_;
            body["configVersion"] = current_config_version_str_;

            auto ts = std::to_string(now_ms());
            std::string nonce = make_nonce();
            cpr::Header headers{
                {"Content-Type", "application/json; charset=UTF-8"},
                {"Gversion", device_info_.version},
                {"Glanguage", device_info_.language},
                {"GClient", device_info_.client},
                {"GApp-Version", device_info_.app_version},
                {"Gtime", ts},
                {"Gnonce", nonce}
            };
            std::string sign_source = headers["Gversion"] + headers["Glanguage"] + headers["GClient"] +
                                      headers["GApp-Version"] + headers["Gtime"] + headers["Gnonce"] + body.dump();
            headers["GSign"] = signature_mgr_ ? signature_mgr_->sign_data(sign_source) : "";

            auto resp = cpr::Post(cpr::Url{event_http_url_}, headers, cpr::Body{body.dump()}, cpr::Timeout{5000});
            if (resp.error) {
                RCLCPP_WARN(this->get_logger(), "⚠️ HTTP获取事件配置失败：%s", resp.error.message.c_str());
                return false;
            }
            if (resp.status_code != 200) {
                RCLCPP_WARN(this->get_logger(), "⚠️ HTTP获取事件配置返回码：%ld", resp.status_code);
                return false;
            }
            auto json_resp = nlohmann::json::parse(resp.text, nullptr, false);
            if (json_resp.is_discarded()) {
                RCLCPP_ERROR(this->get_logger(), "❌ 事件配置HTTP响应解析失败");
                return false;
            }
            nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
            std::string proto_b64 = data.value("proto", std::string(""));
            if (proto_b64.empty()) {
                RCLCPP_WARN(this->get_logger(), "⚠️ 事件配置响应缺少proto字段");
                return false;
            }
            std::string raw = base64_decode(proto_b64);
            if (raw.empty()) {
                RCLCPP_ERROR(this->get_logger(), "❌ 事件配置Base64解码失败");
                return false;
            }
            return handle_event_config_payload(raw, "http");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ HTTP获取事件配置异常：%s", e.what());
            return false;
        }
    }

    void on_event_ack_message(const std::string& payload) {
        vehicle::event::VehicleEventAck ack;
        if (!ack.ParseFromString(payload)) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 事件回执解析失败");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "📩 收到事件回执：id=%s status=%d up_index=%ld",
                    ack.event_id().c_str(), ack.status(), static_cast<long>(ack.up_index()));
    }

    void on_event_push_request(const std_msgs::msg::String::SharedPtr msg) {
        nlohmann::json j = nlohmann::json::parse(msg->data, nullptr, false);
        if (j.is_discarded()) {
            RCLCPP_ERROR(this->get_logger(), "❌ 事件上报JSON解析失败");
            return;
        }
        if (!j.contains("event_type") || !j.contains("status")) {
            RCLCPP_ERROR(this->get_logger(), "❌ 事件上报缺少必填字段 event_type/status");
            return;
        }

        vehicle::event::VehicleEventInfo info;
        info.set_sn(get_vehicle_sn());
        info.set_device_timestamp(now_ms());
        info.set_model(device_model_);
        info.set_version(software_version_);
        info.set_config_version(current_config_version_num_);
        info.set_event_id(j.value("event_id", make_nonce()));
        info.set_event_type(j.value("event_type", 0));
        info.set_duration_type(j.value("duration_type", 0));
        info.set_status(j.value("status", 0));
        info.set_up_index(j.value("up_index", static_cast<int64_t>(0)));

        if (j.contains("source")) {
            auto src = j["source"];
            if (src.contains("device_type")) info.mutable_source()->set_device_type(src.value("device_type", ""));
            if (src.contains("device_sn")) info.mutable_source()->set_device_sn(src.value("device_sn", ""));
            if (src.contains("firmware_version")) info.mutable_source()->set_firmware_version(src.value("firmware_version", ""));
        }

        if (j.contains("safety_info")) {
            auto s = j["safety_info"];
            auto p = info.mutable_safety_info();
            if (s.contains("code")) p->set_code(s.value("code", ""));
            if (s.contains("level")) p->set_level(s.value("level", 0));
            if (s.contains("trigger_value")) p->set_trigger_value(s.value("trigger_value", 0.0));
            if (s.contains("threshold")) p->set_threshold(s.value("threshold", 0.0));
            if (s.contains("first_ms")) p->set_first_ms(s.value("first_ms", static_cast<int64_t>(0)));
            if (s.contains("last_ms")) p->set_last_ms(s.value("last_ms", static_cast<int64_t>(0)));
            if (s.contains("desc")) p->set_desc(s.value("desc", ""));
            if (s.contains("extras")) p->set_extras(s.value("extras", ""));
        }

        if (j.contains("alarm_info")) {
            auto a = j["alarm_info"];
            auto p = info.mutable_alarm_info();
            if (a.contains("code")) p->set_code(a.value("code", ""));
            if (a.contains("code_name")) p->set_code_name(a.value("code_name", ""));
            if (a.contains("desc")) p->set_desc(a.value("desc", ""));
            if (a.contains("level")) p->set_level(a.value("level", 0));
            if (a.contains("info")) p->set_info(a.value("info", ""));
            if (a.contains("first_ms")) p->set_first_ms(a.value("first_ms", static_cast<int64_t>(0)));
            if (a.contains("last_ms")) p->set_last_ms(a.value("last_ms", static_cast<int64_t>(0)));
        }

        if (j.contains("dtc_info")) {
            auto d = j["dtc_info"];
            auto p = info.mutable_dtc_info();
            if (d.contains("code")) p->set_code(d.value("code", ""));
            if (d.contains("desc")) p->set_desc(d.value("desc", ""));
            if (d.contains("info")) p->set_info(d.value("info", ""));
            if (d.contains("first_ms")) p->set_first_ms(d.value("first_ms", static_cast<int64_t>(0)));
            if (d.contains("last_ms")) p->set_last_ms(d.value("last_ms", static_cast<int64_t>(0)));
        }

        if (j.contains("telemetry")) {
            auto t = j["telemetry"];
            auto p = info.mutable_telemetry();
            if (t.contains("speed")) p->set_speed(t.value("speed", 0.0));
            if (t.contains("steering")) p->set_steering(t.value("steering", 0.0));
            if (t.contains("brake")) p->set_brake(t.value("brake", 0));
            if (t.contains("b_force")) p->set_b_force(t.value("b_force", 0));
            if (t.contains("battery")) p->set_battery(t.value("battery", 0.0));
            if (t.contains("battery_voltage_v")) p->set_battery_voltage_v(t.value("battery_voltage_v", 0.0));
            if (t.contains("battery_current_a")) p->set_battery_current_a(t.value("battery_current_a", 0.0));
            if (t.contains("gear")) p->set_gear(t.value("gear", 0));
            if (t.contains("mode")) p->set_mode(t.value("mode", 0));
            if (t.contains("latitude")) p->set_latitude(t.value("latitude", 0.0));
            if (t.contains("longitude")) p->set_longitude(t.value("longitude", 0.0));
            if (t.contains("altitude")) p->set_altitude(t.value("altitude", 0.0));
            if (t.contains("extras")) p->set_extras(t.value("extras", ""));
        }

        if (j.contains("log_file") && j["log_file"].is_array()) {
            for (const auto& lf : j["log_file"]) {
                if (lf.is_string()) info.add_log_file(lf.get<std::string>());
            }
        }
        if (j.contains("video_file") && j["video_file"].is_array()) {
            for (const auto& vf : j["video_file"]) {
                if (vf.is_string()) info.add_video_file(vf.get<std::string>());
            }
        }
        if (j.contains("extras")) {
            if (j["extras"].is_string()) info.set_extras(j.value("extras", ""));
            else info.set_extras(j["extras"].dump());
        }

        publish_passive_msg(event_push_topic_, "事件上报", info);
    }

    

  void car_info_callback(const interface::msg::CarInfo::SharedPtr msg) {
    // 更新车辆状态（原有逻辑不变）
    current_status_.speed_kmh = msg->speedkmh;
    current_status_.remaining_mileage = static_cast<int32_t>(msg->remainingmileage);
    current_status_.gear = static_cast<int32_t>(msg->gear);
    current_status_.charging_status = static_cast<int32_t>(msg->getcharging);
    current_status_.parking_status = static_cast<int32_t>(msg->lockswitch);
    current_status_.assist_mode = static_cast<int32_t>(msg->helpcar);
    current_status_.battery = static_cast<int32_t>(msg->electricquantity);
    current_status_.left_brakests = static_cast<int32_t>(msg->left_brakests);
    current_status_.right_brakests = static_cast<int32_t>(msg->right_brakests);

    // 打印所有参数（中文标注+原始值，格式对齐）
    RCLCPP_INFO(this->get_logger(), 
        "🚗 车辆信息更新：\n"
        "  - 速度：%.2f km/h\n"
        "  - 剩余里程：%d\n"
        "  - 档位：%d\n"
        "  - 充电状态：%d（0=未充电，1=充电中，2=充满）\n"
        "  - 驻车状态：%d（0=未驻车，1=已驻车）\n"
        "  - 辅助驾驶模式：%d\n"
        "  - 电池电量：%d %%\n"
        "  - 左刹车状态：%d\n"
        "  - 右刹车状态：%d",
        current_status_.speed_kmh,
        current_status_.remaining_mileage,
        current_status_.gear,
        current_status_.charging_status,
        current_status_.parking_status,
        current_status_.assist_mode,
        current_status_.battery,
        current_status_.left_brakests,
        current_status_.right_brakests
    );
}

    void recharge_callback(const interface::msg::RechargeStatus::SharedPtr msg) {
        current_status_.recharge_status = static_cast<int32_t>(msg->status);
        check_and_report_passive_status();
    }

    void scene_mode_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string mode = msg->data;
        if (mode.find(':') != std::string::npos) {
            mode = mode.substr(0, mode.find(':'));
        }
        auto iter = MODE_TO_CHINESE.find(mode);
        current_status_.current_mode_chinese = ensure_utf8(iter != MODE_TO_CHINESE.end() ? iter->second : "");
        check_and_report_passive_status();
        // 将 robot_mode 写入状态文件，供 DomainControllerMonitor 读取（解耦采集/上报）
        write_status_key("robot_mode", mode);
        // 立即刷新监控器以便下次 domain 上报能看到最新 mode
        domain_monitor_.refresh();
    }

    void encoder_callback(const interface::msg::Encoder::SharedPtr msg) {
        if (!mileage_calculator_) return;
        mileage_calculator_->update_real_time_mileage(msg->left_wheel_count, msg->right_wheel_count);
        current_status_.current_interval_mileage = mileage_calculator_->get_current_mileage();
        current_status_.total_mileage = mileage_calculator_->get_total_mileage();
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        // 仅缓存最新的GPS消息，不直接上报；由定时器每分钟上报一次
        last_gps_ = *msg;
        // 判断是否来自模拟器（通过时间戳与上次模拟发布时间对比）
        last_gps_simulated_ = (static_cast<int32_t>(msg->header.stamp.sec) == last_sim_pub_stamp_sec_
                                && static_cast<uint32_t>(msg->header.stamp.nanosec) == last_sim_pub_stamp_nsec_);
        has_gps_ = true;
    }

    void publish_gps_tick() {
        try {
            if (!ensure_mqtt_connected()) return;
            if (!has_gps_) {
                if (log_pre_publish_) {
                    RCLCPP_WARN(this->get_logger(), "GPS暂无可上报数据，等待下一次采样...");
                }
                return;
            }

            const auto &msg = last_gps_;
            gps::GPSData pb_msg;
            auto base = pb_msg.mutable_base();
            base->set_model(MODEL_X5_CODE);
            base->set_vin("x001");
            double ts = static_cast<double>(msg.header.stamp.sec) + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
            const auto device_ts_ms = static_cast<int64_t>(std::llround(ts * 1000.0));
            base->set_device_timestamp(device_ts_ms);
            pb_msg.set_timestamp(ts);
            pb_msg.set_latitude(msg.latitude);
            pb_msg.set_longitude(msg.longitude);
            pb_msg.set_altitude(msg.altitude);
            pb_msg.set_velocity_n(0.0f);
            pb_msg.set_velocity_e(0.0f);
            pb_msg.set_velocity_u(0.0f);
            pb_msg.set_fix_type(static_cast<uint32_t>(msg.status.status));
            pb_msg.set_num_satellites(0);
            pb_msg.set_hdop(0.0f);

            if (!pb_msg.IsInitialized()) {
                RCLCPP_ERROR(this->get_logger(), "❌ GPS数据未初始化完整：%s", pb_msg.InitializationErrorString().c_str());
                return;
            }

            if (log_pre_publish_) {
                const std::string gps_topic = base_topic_ + "/gps";
                const char* src = last_gps_simulated_ ? "simulated" : "driver";
                RCLCPP_INFO(this->get_logger(),
                            "➡️ 即将上报GPS topic=%s source=%s lat=%.6f lon=%.6f alt=%.2f ts=%.3f fix=%d",
                            gps_topic.c_str(), src,
                            msg.latitude,
                            msg.longitude,
                            msg.altitude,
                            ts,
                            msg.status.status);
            }

            std::string raw_data;
            if (!pb_msg.SerializeToString(&raw_data)) {
                RCLCPP_ERROR(this->get_logger(), "❌ GPS数据序列化失败");
                return;
            }

            auto now_ms = std::to_string(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count()
            );
            std::string nonce = make_nonce();
            std::string sign_source = now_ms + nonce + raw_data;
            std::string signature = signature_mgr_->sign_data(sign_source);

            int ret = mqtt_manager_->publish_with_properties(
                base_topic_ + "/gps",
                raw_data,
                0,
                {
                    {"gtime", now_ms},
                    {"gnonce", nonce},
                    {"gsign", signature}
                }
            );
            if (ret == MOSQ_ERR_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "📤 GPS上报成功（payload:%zu bytes）", raw_data.size());
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ GPS上报失败：%s", mosquitto_strerror(ret));
                if (utils::TriggerLogger::getInstance().isInitialized()) {
                    auto log_data = std::map<std::string, std::string>{
                        {"rc", std::to_string(ret)},
                        {"err", mosquitto_strerror(ret)},
                        {"payload_bytes", std::to_string(raw_data.size())}
                    };
                    TRIGGER_LOG_WITH_NODE("publish_gps_fail", log_data, this->get_name());
                }
            }

        } catch (const google::protobuf::FatalException& e) {
            RCLCPP_FATAL(this->get_logger(), "GPS Protobuf致命错误：%s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "GPS上报异常：%s", e.what());
        }
    }

    void query_service_status() {
        if (summon_client_->service_is_ready()) {
            auto req = std::make_shared<interface::srv::SummonStatus::Request>();
            req->id = 1;
            summon_client_->async_send_request(req, std::bind(&CloudManagerNode::summon_response_cb, this, std::placeholders::_1));
        }
        if (follow_client_->service_is_ready()) {
            auto req = std::make_shared<interface::srv::FollowStatus::Request>();
            req->id = 1;
            follow_client_->async_send_request(req, std::bind(&CloudManagerNode::follow_response_cb, this, std::placeholders::_1));
        }
    }

    void publish_domain_controller() {
        try {
            if (!ensure_mqtt_connected()) return;

            // 刷新监控数据 + 采样系统指标（/proc）
            domain_monitor_.refresh();
            domain_monitor_.sample_system();

            domain::DomainControllerData pb;
            auto base = pb.mutable_base();
            base->set_model(MODEL_X5_CODE);
            base->set_vin("x001");
            int64_t ts = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            base->set_device_timestamp(ts);

            // 选择指标来源
            std::string source = this->get_parameter("metrics.source").as_string();
            float cpu_file = domain_monitor_.cpu_usage();
            float mem_file = domain_monitor_.mem_usage();
            uint32_t net_file = domain_monitor_.network_status();
            float cpu_sys = domain_monitor_.cpu_usage_system();
            float mem_sys = domain_monitor_.mem_usage_system();
            uint32_t net_sys = domain_monitor_.network_status_system();
            float cpu_to_pub = cpu_file;
            float mem_to_pub = mem_file;
            uint32_t net_to_pub = net_file;
            std::string chosen = "file";
            if (source == "system") {
                cpu_to_pub = cpu_sys;
                mem_to_pub = mem_sys;
                net_to_pub = net_sys;
                chosen = "system";
            } else if (source == "auto") {
                // 简单策略：优先使用文件值，若文件无效(均为0)且系统有值，则用系统
                bool file_invalid = (cpu_file <= 0.0f && mem_file <= 0.0f && net_file == 0);
                bool sys_valid = (cpu_sys >= 0.0f && mem_sys >= 0.0f);
                if (file_invalid && sys_valid) {
                    cpu_to_pub = cpu_sys;
                    mem_to_pub = mem_sys;
                    net_to_pub = net_sys;
                    chosen = "system";
                }
            }
            pb.set_cpu_usage(cpu_to_pub);
            pb.set_mem_usage(mem_to_pub);
            pb.set_network_status(net_to_pub);
            pb.set_error_code(domain_monitor_.error_code());

            // 如错误码变化且非0，发布到 /error_events 供本地订阅处理
            uint32_t cur_err = static_cast<uint32_t>(domain_monitor_.error_code());
            if (error_event_pub_ && cur_err != 0 && cur_err != last_error_code_) {
                interface::msg::HexData h;
                h.hex_value = cur_err;
                error_event_pub_->publish(h);
                last_error_code_ = cur_err;
                RCLCPP_WARN(this->get_logger(), "发布错误事件，code=0x%X", cur_err);
            }
            // robot_mode 取自监控文件（优先）或 scene_mode
            if (!domain_monitor_.robot_mode().empty()) pb.set_robot_mode(ensure_utf8(domain_monitor_.robot_mode()));
            if (!domain_monitor_.vehicle_id().empty()) pb.set_vehicle_id(ensure_utf8(domain_monitor_.vehicle_id()));
            if (log_pre_publish_) {
                // 打印本地读取与系统采样的原始值，以及最终上报值
                RCLCPP_INFO(this->get_logger(),
                            "🧪 指标来源对比: file(cpu=%.2f mem=%.2f net=%u), system(cpu=%.2f mem=%.2f net=%u), chosen=%s",
                            cpu_file, mem_file, net_file, cpu_sys, mem_sys, net_sys, chosen.c_str());
                const std::string dc_topic = base_topic_ + "/dc";
                RCLCPP_INFO(this->get_logger(),
                            "➡️ 即将上报域控制 topic=%s cpu=%.2f mem=%.2f net=%u err=%d robot_mode=%s vehicle_id=%s",
                            dc_topic.c_str(),
                            pb.cpu_usage(),
                            pb.mem_usage(),
                            pb.network_status(),
                            pb.error_code(),
                            pb.robot_mode().c_str(),
                            pb.vehicle_id().c_str());
            }

            // 序列化前检查必填字段
            if (!pb.IsInitialized()) {
                RCLCPP_ERROR(this->get_logger(), "❌ 域控制器数据未初始化完整：%s", pb.InitializationErrorString().c_str());
                return;
            }
            std::string raw;
            if (!pb.SerializeToString(&raw)) {
                RCLCPP_ERROR(this->get_logger(), "❌ DomainController 序列化失败");
                return;
            }
            // 取消“仅在变化时上报”的策略。
            // 实时上报始终发送（与之前行为一致），以确保上报频率与协议一致。

            auto now_ms = std::to_string(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count()
            );
            std::string nonce = make_nonce();
            std::string signature = signature_mgr_->sign_data(raw);

            int ret = mqtt_manager_->publish_with_properties(
                base_topic_ + "/dc",
                raw,
                0,
                {
                    {"gtime", now_ms},
                    {"gnonce", nonce},
                    {"gsign", signature}
                }
            );
            if (ret == MOSQ_ERR_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "📤 域控制器上报成功（payload:%zu bytes）", raw.size());
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ 域控制器上报失败：%s", mosquitto_strerror(ret));
                if (utils::TriggerLogger::getInstance().isInitialized()) {
                    auto log_data = std::map<std::string, std::string>{
                        {"rc", std::to_string(ret)},
                        {"err", mosquitto_strerror(ret)},
                        {"payload_bytes", std::to_string(raw.size())}
                    };
                    TRIGGER_LOG_WITH_NODE("publish_domain_fail", log_data, this->get_name());
                }
            }

        
        } catch (const google::protobuf::FatalException& e) {
            RCLCPP_FATAL(this->get_logger(), "Domain Protobuf致命错误：%s", e.what());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "域控制器上报异常：%s", e.what());
        }
    }

    void summon_response_cb(const rclcpp::Client<interface::srv::SummonStatus>::SharedFuture future) {
        try {
            auto res = future.get();
            if (res->code == 0) {
                current_status_.summon_status = static_cast<int32_t>(res->status);
                check_and_report_passive_status();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "召唤状态查询异常：%s", e.what());
        }
    }

    void follow_response_cb(const rclcpp::Client<interface::srv::FollowStatus>::SharedFuture future) {
        try {
            auto res = future.get();
            if (res->code == 0) {
                current_status_.follow_me_status = static_cast<int32_t>(res->status);
                check_and_report_passive_status();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "跟我走状态查询异常：%s", e.what());
        }
    }

    // ========== 核心修改：实时上报加签 ==========
    void publish_realtime_msg() {
        try {
            if (!ensure_mqtt_connected()) return;

            // 1. 序列化原始Protobuf数据（VehicleInfo）并填充 base
            car_status::VehicleInfo pb_msg;
            // 填充 BaseInfo（写死为 x5/x001，与 topic 对齐）
            auto base = pb_msg.mutable_base();
            base->set_model(MODEL_X5_CODE);
            base->set_vin("x001");
            // 时间戳（毫秒）
            int64_t ts = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            base->set_device_timestamp(ts);

            pb_msg.set_speed_kmh(current_status_.speed_kmh);
            pb_msg.set_remaining_mileage(current_status_.remaining_mileage);
            pb_msg.set_gear(current_status_.gear);
            pb_msg.set_charging_status(current_status_.charging_status);
            pb_msg.set_parking_status(current_status_.parking_status);
            pb_msg.set_assist_mode(current_status_.assist_mode);
            pb_msg.set_battery(current_status_.battery);
            pb_msg.set_left_brakests(current_status_.left_brakests);
            pb_msg.set_right_brakests(current_status_.right_brakests);
            pb_msg.set_total_mileage(current_status_.total_mileage);
            pb_msg.set_current_interval_mileage(current_status_.current_interval_mileage);
            if (log_pre_publish_) {
                RCLCPP_INFO(this->get_logger(),
                            "➡️ 即将上报实时: speed=%.2fkm/h batt=%d%% total=%.2fm interval=%.2fm gear=%d charge=%d park=%d assist=%d",
                            current_status_.speed_kmh,
                            current_status_.battery,
                            current_status_.total_mileage,
                            current_status_.current_interval_mileage,
                            current_status_.gear,
                            current_status_.charging_status,
                            current_status_.parking_status,
                            current_status_.assist_mode);
            }

            // 序列化前检查必填字段
            if (!pb_msg.IsInitialized()) {
                RCLCPP_ERROR(this->get_logger(), "❌ 实时数据未初始化完整：%s", pb_msg.InitializationErrorString().c_str());
                return;
            }
            std::string raw_data;
            if (!pb_msg.SerializeToString(&raw_data)) {
                RCLCPP_ERROR(this->get_logger(), "❌ 原始数据序列化失败");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "📝 原始数据序列化完成（长度：%zu 字节）", raw_data.size());

            // 2. 生成 gtime/gnonce 并签名（签名仅对原始二进制）
            auto now_ms = std::to_string(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count()
            );
            std::string nonce = make_nonce();
            std::string sign_source = now_ms + nonce + raw_data;
            std::string signature = signature_mgr_->sign_data(sign_source);
            RCLCPP_DEBUG(this->get_logger(), "实时上报签名前20：%s", signature.substr(0,20).c_str());

            int ret = mqtt_manager_->publish_with_properties(
                base_topic_ + "/info",
                raw_data,
                0,
                {
                    {"gtime", now_ms},
                    {"gnonce", nonce},
                    {"gsign", signature}
                }
            );
            if (ret == MOSQ_ERR_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "📤 实时上报成功（payload:%zu gtime=%s nonce=%s）", raw_data.size(), now_ms.c_str(), nonce.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ 实时上报失败：%s", mosquitto_strerror(ret));
                if (utils::TriggerLogger::getInstance().isInitialized()) {
                    auto log_data = std::map<std::string, std::string>{
                        {"rc", std::to_string(ret)},
                        {"err", mosquitto_strerror(ret)},
                        {"payload_bytes", std::to_string(raw_data.size())}
                    };
                    TRIGGER_LOG_WITH_NODE("publish_realtime_fail", log_data, this->get_name());
                }
            }

        } catch (const google::protobuf::FatalException& e) {
            RCLCPP_FATAL(this->get_logger(), "实时上报 Protobuf致命错误：%s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "实时上报异常：%s", e.what());
        }
    }

    // ========== 核心修改：被动上报加签 ==========
    template <typename T>
    void publish_passive_msg(const std::string& topic, const std::string& type_name, const T& msg) {
        try {
            if (!ensure_mqtt_connected()) return;

            // 1. 序列化原始数据
            // 序列化前检查必填字段
            if (!msg.IsInitialized()) {
                RCLCPP_ERROR(this->get_logger(), "❌ %s未初始化完整：%s", type_name.c_str(), msg.InitializationErrorString().c_str());
                return;
            }
            std::string raw_data;
            if (!msg.SerializeToString(&raw_data)) {
                RCLCPP_ERROR(this->get_logger(), "❌ %s原始数据序列化失败", type_name.c_str());
                return;
            }

            // 2. 生成 gtime/gnonce 并签名
            auto now_ms = std::to_string(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count()
            );
            std::string nonce = make_nonce();
            std::string sign_source = now_ms + nonce + raw_data;
            std::string signature = signature_mgr_->sign_data(sign_source);
            RCLCPP_DEBUG(this->get_logger(), "被动上报签名前20：%s", signature.substr(0,20).c_str());

            int ret = mqtt_manager_->publish_with_properties(
                topic,
                raw_data,
                0,
                {
                    {"gtime", now_ms},
                    {"gnonce", nonce},
                    {"gsign", signature}
                }
            );
            if (ret == MOSQ_ERR_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "📤 带签名%s上报成功（payload:%zu gtime=%s nonce=%s）", type_name.c_str(), raw_data.size(), now_ms.c_str(), nonce.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ %s上报失败：%s", type_name.c_str(), mosquitto_strerror(ret));
                if (utils::TriggerLogger::getInstance().isInitialized()) {
                    auto log_data = std::map<std::string, std::string>{
                        {"type", type_name},
                        {"rc", std::to_string(ret)},
                        {"err", mosquitto_strerror(ret)},
                        {"payload_bytes", std::to_string(raw_data.size())}
                    };
                    TRIGGER_LOG_WITH_NODE("publish_passive_fail", log_data, this->get_name());
                }
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "%s上报异常：%s", type_name.c_str(), e.what());
        }
    }

    void check_and_report_passive_status() {
        // 使用 VehicleStatus 合并被动字段，仅在字段变更时设置对应可选字段
        car_status::VehicleStatus vs;
        auto base = vs.mutable_base();
        base->set_model(MODEL_X5_CODE);
        base->set_vin("x001");
        int64_t ts = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        base->set_device_timestamp(ts);

        bool need_send = false;
        // 首次开机必须上报一次
        if (!first_status_publish_done_) {
            need_send = true;
        }
        if (current_status_.recharge_status != history_cache_.recharge_status) {
            history_cache_.recharge_status = current_status_.recharge_status;
            need_send = true;
        }
        if (current_status_.summon_status != history_cache_.summon_status) {
            history_cache_.summon_status = current_status_.summon_status;
            need_send = true;
        }
        if (current_status_.follow_me_status != history_cache_.follow_me_status) {
            history_cache_.follow_me_status = current_status_.follow_me_status;
            need_send = true;
        }
        if (current_status_.current_mode_chinese != history_cache_.current_mode_chinese) {
            history_cache_.current_mode_chinese = current_status_.current_mode_chinese;
            need_send = true;
        }
        if (current_status_.alarmstatus != history_cache_.alarmstatus) {
            history_cache_.alarmstatus = current_status_.alarmstatus;
            need_send = true;
        }

        if (need_send) {
            // 当任一字段发生变化时，整包上报：所有字段均填充当前值（不空）
            vs.mutable_recharge_status()->set_value(current_status_.recharge_status);
            vs.mutable_summon_status()->set_value(current_status_.summon_status);
            vs.mutable_follow_me_status()->set_value(current_status_.follow_me_status);
            vs.mutable_current_mode_chinese()->set_value(current_status_.current_mode_chinese);
            vs.mutable_alarmstatus()->set_value(current_status_.alarmstatus);

            if (utils::TriggerLogger::getInstance().isInitialized()) {
                std::string changed_fields;
                if (vs.has_recharge_status()) changed_fields += "recharge_status|";
                if (vs.has_summon_status()) changed_fields += "summon_status|";
                if (vs.has_follow_me_status()) changed_fields += "follow_me_status|";
                if (vs.has_current_mode_chinese()) changed_fields += "mode|";
                if (vs.has_alarmstatus()) changed_fields += "alarmstatus|";
                if (!changed_fields.empty() && changed_fields.back() == '|') changed_fields.pop_back();
                auto log_data = std::map<std::string, std::string>{
                    {"changed", changed_fields},
                    {"recharge_status", vs.has_recharge_status() ? std::to_string(vs.recharge_status().value()) : ""},
                    {"summon_status", vs.has_summon_status() ? std::to_string(vs.summon_status().value()) : ""},
                    {"follow_me_status", vs.has_follow_me_status() ? std::to_string(vs.follow_me_status().value()) : ""},
                    {"mode", vs.has_current_mode_chinese() ? vs.current_mode_chinese().value() : ""},
                    {"alarmstatus", vs.has_alarmstatus() ? std::to_string(vs.alarmstatus().value()) : ""}
                };
                TRIGGER_LOG_WITH_NODE("status_change", log_data, this->get_name());
            }
            if (log_pre_publish_) {
                const std::string status_topic = base_topic_ + "/status";
                RCLCPP_INFO(this->get_logger(),
                            "➡️ 即将上报状态 topic=%s recharge=%d summon=%d follow=%d mode=%s alarm=%d",
                            status_topic.c_str(),
                            current_status_.recharge_status,
                            current_status_.summon_status,
                            current_status_.follow_me_status,
                            current_status_.current_mode_chinese.c_str(),
                            current_status_.alarmstatus);
            }
            publish_passive_msg(base_topic_ + "/status", "车辆状态", vs);
            first_status_publish_done_ = true;
        }
    }

    // 成员变量
    // std::shared_ptr<mqtt::async_client> mqtt_client_; // 移除，使用mqtt_manager_
    std::shared_ptr<cloud_manager::MqttConnectionManager> mqtt_manager_;
    std::shared_ptr<cloud_manager::SignatureManager> signature_mgr_; // 新增：签名工具
    // 协议基础 topic（例如 vehicle/up/x5/x001/1）
    std::string base_topic_;
    std::string device_model_ = "x5";
    // 主题路径中的车型段（与 device_model_ 解耦，固定简写）
    std::string topic_model_ = "x5";
    std::string device_id_ = "x001";
    std::string protocol_version_ = "1";
    std::string software_version_ = "1.0.0";
    std::string event_config_topic_;
    std::string event_config_ack_topic_;
    std::string event_push_topic_;
    std::string event_ack_topic_;
    std::string event_config_path_;
    std::string event_config_json_path_;
    std::string event_http_url_;
    bool event_http_enabled_ = true;
    double event_http_interval_sec_ = 600.0;
    std::string current_config_id_;
    std::string current_config_version_str_ = "0";
    int64_t current_config_version_num_ = 0;
    rclcpp::TimerBase::SharedPtr event_http_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_push_sub_;
    // MQTT 参数缓存（便于调试输出或后续扩展）
    std::string mqtt_username_;
    std::string mqtt_ca_cert_;
    bool mqtt_insecure_skip_verify_ = false;
    cloud_manager::TokenManager token_mgr_;
    cloud_manager::DeviceInfo device_info_;
    cloud_manager::TokenResult current_token_;
    int64_t token_expire_at_ms_ = 0;
    int64_t token_refresh_margin_ms_ = 300000;
    std::atomic<bool> reconnecting_{false};
    std::atomic<bool> refreshing_token_{false};
    bool using_token_auth_ = false;
    double mqtt_reconnect_interval_sec_ = 5.0;
    rclcpp::TimerBase::SharedPtr token_refresh_timer_;
    rclcpp::TimerBase::SharedPtr mqtt_reconnect_timer_;
    bool log_mileage_ = false; //暂时不打印 有需要时再打开打印开关
    bool log_token_full_ = false;
    bool log_pre_publish_ = true;
    rclcpp::Subscription<interface::msg::CarInfo>::SharedPtr car_info_sub_;
    rclcpp::Subscription<interface::msg::RechargeStatus>::SharedPtr recharge_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scene_mode_sub_;
    rclcpp::Subscription<interface::msg::Encoder>::SharedPtr encoder_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    // GPS定时上报
    rclcpp::TimerBase::SharedPtr gps_pub_timer_;
    sensor_msgs::msg::NavSatFix last_gps_;
    bool has_gps_ = false;
    bool last_gps_simulated_ = false;
    int32_t last_sim_pub_stamp_sec_ = -1;
    uint32_t last_sim_pub_stamp_nsec_ = 0;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vehicle_id_sub_;
    rclcpp::Client<interface::srv::SummonStatus>::SharedPtr summon_client_;
    rclcpp::Client<interface::srv::FollowStatus>::SharedPtr follow_client_;
    rclcpp::Client<interface::srv::IdControls>::SharedPtr id_client_;
    rclcpp::TimerBase::SharedPtr realtime_pub_timer_;
    rclcpp::TimerBase::SharedPtr service_query_timer_;
    rclcpp::TimerBase::SharedPtr vehicle_id_timer_;
    // GPS模拟
    bool simulate_gps_ = true; //暂时室内没有数据，联调阶段先模拟数据打通数据通路。
    double sim_lat_ = 0.0;
    double sim_lon_ = 0.0;
    double sim_alt_ = 0.0;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sim_pub_;
    rclcpp::TimerBase::SharedPtr gps_sim_timer_;
    VehicleStatus current_status_;
    HistoryCache history_cache_;
    std::unique_ptr<MileageCalculator> mileage_calculator_;
    cloud_manager::DomainControllerMonitor domain_monitor_;
    rclcpp::TimerBase::SharedPtr domain_ctrl_timer_;
    // 错误事件发布（十六进制错误码）
    rclcpp::Publisher<interface::msg::HexData>::SharedPtr error_event_pub_;
    uint32_t last_error_code_ = 0;
    // 缓存上次上报的序列化结果，用于仅在变化时上报以节省带宽
    std::string last_domain_raw_;
    std::string vehicle_id_;
    // 首次开机状态上报标志
    bool first_status_publish_done_ = false;
};

} // namespace cloud_manager_node

// int main(int argc, char* argv[]) {
//     GOOGLE_PROTOBUF_VERIFY_VERSION;
//     rclcpp::init(argc, argv);
//     try {
//         auto node = std::make_shared<cloud_manager_node::CloudManagerNode>();
//         rclcpp::spin(node);
//     } catch (const std::exception &e) {
//         RCLCPP_FATAL(rclcpp::get_logger("cloud_manager_node"), "节点初始化失败：%s", e.what());
//     }

    
//     rclcpp::shutdown();
//     google::protobuf::ShutdownProtobufLibrary();
//     return 0;
// }


int main(int argc, char* argv[]) {
    // 1. 初始化Protobuf并检查版本
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    // 2. 初始化ROS2（单独捕获初始化失败的异常）
    try {
        rclcpp::init(argc, argv);
    } catch (const std::exception& e) {
        // 直接打印到标准输出（此时ROS2日志可能未初始化）
        std::cerr << "ROS2初始化失败：" << e.what() << std::endl;
        google::protobuf::ShutdownProtobufLibrary();
        return -1;
    }

    // 3. 节点创建与运行（覆盖初始化+spin全阶段）
    try {
        auto node = std::make_shared<cloud_manager_node::CloudManagerNode>();
        RCLCPP_INFO(node->get_logger(), "节点初始化成功，开始运行...");
        
        // 捕获spin过程中的异常（节点运行时的所有操作）
        rclcpp::spin(node);
    } catch (const google::protobuf::FatalException& e) {
        // 专门捕获Protobuf相关异常（如序列化失败）
        RCLCPP_FATAL(rclcpp::get_logger("cloud_manager_node"), "Protobuf致命错误：%s", e.what());
    } catch (const std::invalid_argument& e) {
        // 捕获参数错误（如OSS密钥为空、MQTT地址无效）
        RCLCPP_FATAL(rclcpp::get_logger("cloud_manager_node"), "参数错误：%s", e.what());
    } catch (const std::runtime_error& e) {
        // 捕获运行时错误（如网络连接失败、文件操作失败）
        RCLCPP_FATAL(rclcpp::get_logger("cloud_manager_node"), "运行时错误：%s", e.what());
    } catch (const std::exception& e) {
        // 捕获所有其他标准异常
        RCLCPP_FATAL(rclcpp::get_logger("cloud_manager_node"), "未知异常：%s", e.what());
    } catch (...) {
        // 捕获非标准异常（如自定义异常）
        RCLCPP_FATAL(rclcpp::get_logger("cloud_manager_node"), "捕获到未识别的异常");
    }

    // 4. 资源清理
    rclcpp::shutdown();
    google::protobuf::ShutdownProtobufLibrary();
    RCLCPP_INFO(rclcpp::get_logger("cloud_manager_node"), "节点已退出并清理资源");
    return 0;
}