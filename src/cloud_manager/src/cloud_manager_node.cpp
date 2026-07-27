#include <rclcpp/rclcpp.hpp>
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
#include "utils/topic.h"

// Service
#include "interface/srv/id_controls.hpp"
#include "interface/srv/bind_status.hpp"
#include "interface/srv/summon_status.hpp"
#include "interface/srv/follow_status.hpp"
#include "interface/srv/get_token.hpp"

// mqtt module
#include "mqtt_module/signature_manager.hpp"
#include "mqtt_module/mqtt_connection.hpp"
#include "mqtt_module/mqtt_client.hpp"
#include "mqtt_module/token_manager.hpp"

// Protobuf
#include "car_status.pb.h"
// GPS proto
#include "gps_data.pb.h"
#include "domain_controller.pb.h"
#include "event_config.pb.h"
#include "vehicle_event.pb.h"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
// device bind msg 
#include "event_device_bind.pb.h"
#include "interface/msg/vehicle_bind_status.hpp"

// 新增：解耦的MQTT连接管理器 + 签名工具
// #include "cloud_manager/domain_controller_monitor.hpp"
#include "cloud_manager/http_singleton.hpp"
#include "cloud_manager/mileage_calculator.hpp"

#include "utils/trigger_logger.h"
#include "error/freego_error_code.hpp"
#include "error/freego_error_code_helper.hpp"

using namespace std::chrono_literals;
using namespace freego_error_code;
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

        declare_ros_parameters();

        device_bind_pub_ = this->create_publisher<interface::msg::VehicleBindStatus>(cloudManagerDeviceBindPubTopic, 10);
        device_bind_status_service_ = this->create_service<interface::srv::BindStatus>(cloudManagerDeviceBindService, 
                                            std::bind(&CloudManagerNode::rec_device_bind_callback, this, std::placeholders::_1, std::placeholders::_2));
        get_token_service_ = this->create_service<interface::srv::GetToken>(cloudManagerGetAuthInfoService, 
                                            std::bind(&CloudManagerNode::rec_auth_info_callback, this, std::placeholders::_1, std::placeholders::_2));
                                
        // ========== 1. 初始化令牌管理器（原有逻辑） ==========
        std::string device_model_http = this->get_parameter("device.model_http").as_string();
        device_model_ = this->get_parameter("device.model").as_string();
        device_id_ = this->get_parameter("device.id").as_string();
        protocol_version_ = this->get_parameter("protocol.version").as_string();
        software_version_ = this->get_parameter("software.version").as_string();
        event_config_path_ = this->get_parameter("event.config_path").as_string();
        event_config_json_path_ = this->get_parameter("event.config_json_path").as_string();
        event_http_enabled_ = this->get_parameter("event.http_enable").as_bool();
        event_http_interval_sec_ = this->get_parameter("event.http_interval_sec").as_double();
        event_http_url_ = this->get_parameter("event.http_url").as_string();
        
        // device_info_.vin = device_id_;
        device_info_.model = device_model_http;
        mqtt_device_info_.model = device_model_;

        // 设置车云接口协议的根密钥信息
        device_info_.gkey_no = "2;ZQKJ001;3";
        device_info_.root_private_key = "ydSKNkOGdKx0ITtDp2GtTN9MChiDo6SQXSSRRusli0U=";
        device_info_.root_public_key = "p1c0dmOw6enDGkks5nwuwmr7mMpQm3xfDjrSDl7R0dg=";
        device_info_.api_base_url = "http://39.107.107.204:22000";
        device_info_.client = "vehicle";
        device_info_.app_version = "1.0";
        device_info_.language = "zh-cn";
        device_info_.version = "1";

        // 设置车云接口协议的根密钥信息
        mqtt_device_info_.gkey_no = "2;ZQKJ001;3";
        mqtt_device_info_.root_private_key = "ydSKNkOGdKx0ITtDp2GtTN9MChiDo6SQXSSRRusli0U=";
        mqtt_device_info_.root_public_key = "p1c0dmOw6enDGkks5nwuwmr7mMpQm3xfDjrSDl7R0dg=";
        mqtt_device_info_.api_base_url = "http://39.107.107.204:22000";
        mqtt_device_info_.client = "vehicle";
        mqtt_device_info_.app_version = "1.0";
        mqtt_device_info_.language = "zh-cn";
        mqtt_device_info_.version = "1";
        
        // 优先使用参数中注入的私钥（hex），否则保持原有 mock 值以兼容旧逻辑
        std::string param_sk = this->get_parameter("signature.private_key").as_string();
        if (!param_sk.empty()) {
            device_info_.private_key = param_sk;
            mqtt_device_info_.private_key = param_sk;
            RCLCPP_INFO(this->get_logger(), "🔑 从参数加载私钥（hex）");
        } else {
            device_info_.private_key = "MOCK_ED25519_PRIVATE_KEY";
            mqtt_device_info_.private_key = "MOCK_ED25519_PRIVATE_KEY";
        }

        // 固定协议命名的 topic 基础路径（可根据协议写死为 x5/x001/1）
        // 按你的要求：实时 info => vehicle/up/<model>/<id>/<ver>/info
        //          状态 status => vehicle/up/<model>/<id>/<ver>/status
        // 优先获取SN号
        init_mileage_calculator();
        create_service_clients();
        init_gps_simulator();
        subscribe_ros_topics();
        schedule_vehicle_id_request();
        /** device 后续更新 */
        base_topic_ = "vehicle/up/" + mqtt_device_info_.model + "/" + mqtt_device_info_.vin + "/" + protocol_version_;
        event_config_topic_ = "vehicle/config/" + mqtt_device_info_.model + "/" + protocol_version_ + "/up";
        event_config_ack_topic_ = "vehicle/config/" + mqtt_device_info_.model + "/" + protocol_version_ + "/ack";
        event_push_topic_ = "vehicle/event/" + mqtt_device_info_.model + "/" + mqtt_device_info_.vin + "/" + protocol_version_ + "/push";
        event_ack_topic_ = "vehicle/event/" + mqtt_device_info_.model + "/" + mqtt_device_info_.vin + "/" + protocol_version_ + "/ack";

        
        // ========== 2. 初始化MQTT连接管理器（解耦核心） ==========
        load_ros_parameters();
        init_manager();
        // 这里会获取token和key
        connect_and_subscribe();
        // 先连接
        // make_mqtt_manager();
        // 再订阅
        mqtt_sub_manager(); // 将这个放到connect_and_subscribe里面
        // 注册MQTT signature
        make_signature_mqtt_manager();


        init_timers();
        /* 监控车辆状态 */
        check_and_report_passive_status();

        // 设备绑定，开机时发送一次，并广播至车端域
        device_bind_http_send("/api/adevice/v1/bind/status");
        // 错误码上报 - 订阅 /error_events 获取本地错误事件
        event_push_sub_ = this->create_subscription<interface::msg::HexData>(
            errorEventTopic, 10,
            [this](const interface::msg::HexData::SharedPtr msg) {
                this->on_error_event_callback(msg);
            });
        // 错误事件发布者（发布到云端）
        error_event_pub_ = this->create_publisher<interface::msg::HexData>(eventTopic, 10);
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
            sceneServiceModeTopic, 10, std::bind(&CloudManagerNode::scene_mode_callback, this, std::placeholders::_1)
        );
        // encoder_sub_ = this->create_subscription<interface::msg::Encoder>(
        //     carEncoderTopic, 10, std::bind(&CloudManagerNode::encoder_callback, this, std::placeholders::_1)
        // );
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
        // domain_monitor_.refresh();
        
        // 设置设备信息中的 VIN 为从服务获取的 vehicle_id
        device_info_.vin = msg->data;
        mqtt_device_info_.vin = msg->data;
        RCLCPP_INFO(this->get_logger(), "🚗 从服务获取车辆ID：%s，已设置为设备 VIN", msg->data.c_str());
    }

    void init_timers() {
        realtime_pub_timer_ = this->create_wall_timer(5s, std::bind(&CloudManagerNode::publish_realtime_msg, this));
        service_query_timer_ = this->create_wall_timer(1s, std::bind(&CloudManagerNode::query_service_status, this));
        RCLCPP_INFO(this->get_logger(), "⏱️ 定时器初始化完成");
    }

    // 无GPS源时模拟上报 NavSatFix，方便联调
    void init_gps_simulator() {
        simulate_gps_ = this->get_parameter("gps.simulate").as_bool();
        if (!simulate_gps_) return;

        double rate_hz = this->get_parameter("gps.sim_rate_hz").as_double();
        if (rate_hz <= 0.0) rate_hz = 1.0;
        sim_lat_ = this->get_parameter("gps.sim_lat").as_double();
        sim_lon_ = this->get_parameter("gps.sim_lon").as_double();
        sim_alt_ = this->get_parameter("gps.sim_alt").as_double();

        gps_sim_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gpsTopic, 10);
        auto period_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / rate_hz)
        );
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
            gps_sim_pub_->publish(msg);
        });

        RCLCPP_WARN(this->get_logger(), "⚠️ 启用GPS模拟，上报固定坐标(%f,%f,%f) 频率 %.2f Hz",
                    sim_lat_, sim_lon_, sim_alt_, rate_hz);
    }

    void init_manager() {
        init_mqtt_manager();

    }

    // 启动后主动请求车辆ID（服务 /vlago/car_id），成功后写入状态文件供监控使用
    void schedule_vehicle_id_request()
    {
        try
        {
            auto logger = this->get_logger();
            if (!id_client_)
            {
                RCLCPP_WARN(logger, "车辆ID服务客户端未初始化");
                return;
            }
            if (!id_client_->wait_for_service(3s))
            {
                RCLCPP_WARN(logger, "等待车辆ID服务超时");
                return;
            }

            auto request = std::make_shared<interface::srv::IdControls::Request>();
            request->id = 0; // 请求体暂未使用，发送占位
            RCLCPP_INFO(this->get_logger(), "📤 发送 IdControls 请求");

            auto future = id_client_->async_send_request(request);
            auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 10s);
            if (result == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = future.get();
                if (response)
                {
                    vehicle_id_ = response->sn;
                    device_info_.vin = response->sn;
                    mqtt_device_info_.vin = response->sn;
                    RCLCPP_INFO(this->get_logger(), "✅ 车辆ID 请求成功：sn='%s'", response->sn.c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "❌ 车辆ID 请求失败!");
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "车辆ID请求异常：%s", e.what());
        }
    }

    void mqtt_sub_manager() {
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

    // void update_token_expiry(const mqtt_module::TokenResult& token) {
    //     int64_t fallback = 3600000; // 1h 默认
    //     token_expire_at_ms_ = now_ms() + (token.exp > 0 ? token.exp : fallback);
    // }

    // bool ensure_mqtt_connected(bool force_refresh_token = false) {
    //     return false;
    // }

    // void on_mqtt_disconnected(int rc) {
    //     RCLCPP_WARN(this->get_logger(), "⚠️ MQTT断开，rc=%d，计划自动重连", rc);
    // }

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

    // 设备绑定 使用此接口完成MQTT的消息回复 
    bool handle_event_device_bind_send(const std::string& payload, const std::string& source) {
        vehicle::device::bind::VehicleBindStatus pkg;
        if (!pkg.ParseFromString(payload)) {
            RCLCPP_ERROR(this->get_logger(), "❌ 事件配置解析失败（来源=%s）", source.c_str());
            return false;
        }
        
        if(pkg.status() < 0 || pkg.status() > 2){
            RCLCPP_WARN(this->get_logger(), "⚠️ Service 返回的状态不为有效值，status:%d", pkg.status());
        }
        // 消息回复
        send_event_device_bind_ack(pkg);
        return true;
    }
    // 设备绑定service端
    void rec_device_bind_callback(const std::shared_ptr<interface::srv::BindStatus::Request> /*request*/,
                                     std::shared_ptr<interface::srv::BindStatus::Response> response) {
        try {
            vehicle::device::bind::VehicleBindStatus pkg{};
            std::string err_msg = "";
            // 收到请求后，下发HTTP消息，更新绑定信息
            bool req_flag = HttpClientManager::getInstance()->device_bind_msg("/api/adevice/v1/bind/status", device_info_, pkg, err_msg);
            
            if (req_flag) {
                // 绑定成功
                response->status = pkg.status();  // 已绑定状态
                response->privacy_policy = pkg.privacy_policy();
                response->user_id = pkg.user_id();
                response->success = true;
                
                RCLCPP_INFO(this->get_logger(), "✅ 设备绑定成功: user_id=%s, status=%d, privacy_policy=%s", 
                response->user_id.c_str(), response->status, response->privacy_policy.c_str());
            } else {
                // 绑定失败
                response->status = -1;  // 未绑定状态
                response->privacy_policy = "";
                response->user_id = "";
                response->success = false;
                
                RCLCPP_ERROR(this->get_logger(), "❌ 设备绑定失败：%s", err_msg.c_str());
            }
            
        } catch (const std::exception& e) {
            // 异常处理
            RCLCPP_ERROR(this->get_logger(), "处理设备绑定请求异常: %s", e.what());
            response->status = -1;
            response->privacy_policy = "";
            response->user_id = "";
            response->success = false;
        }
    }

    void rec_auth_info_callback(const std::shared_ptr<interface::srv::GetToken::Request> request,
                                     std::shared_ptr<interface::srv::GetToken::Response> response) {
        try
        {
            if (request->client_id.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "❌ 设备ID为空");
                response->jwt_token = "";
                response->token_success = false;
                response->exp = -1;
                response->public_key = "";
                response->private_key = "";
                response->key_no = "";
                response->key_success = false;
                response->key_err_msg = "client_id为空";
                response->result = "";
                return;
            }

            response->jwt_token = token_result_.jwt_token;
            response->token_success = true;
            response->exp = token_result_.exp;
            response->public_key =  mqtt_device_info_.session_public_key;
            response->private_key =  mqtt_device_info_.session_private_key;
            response->key_no = mqtt_device_info_.session_key_no;
            response->key_success = true;
            response->key_err_msg = "";
            response->result = "";

            RCLCPP_INFO(this->get_logger(), "✅ 获取密钥公钥成功: jwt_token=%s, public_key=%s, private_key=%s, key_no=%s", 
                response->jwt_token.c_str(), response->public_key.c_str(), response->private_key.c_str(), response->key_no.c_str());
        }
        catch (const std::exception &e)
        {
            // 异常处理
            RCLCPP_ERROR(this->get_logger(), "获取密钥公钥请求异常: %s", e.what());
            response->jwt_token = "";
            response->token_success = false;
            response->public_key = "";
            response->private_key = "";
            response->key_no = "";
            response->key_success = false;
            response->key_err_msg = "获取密钥公钥请求异常";
            response->result = "";
        }
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

    // 设备绑定MQTT回复
    void send_event_device_bind_ack(const vehicle::device::bind::VehicleBindStatus& pkg) {
        interface::msg::VehicleBindStatus vehicle_bind_msg;
        vehicle_bind_msg.sn = get_vehicle_sn();
        vehicle_bind_msg.timestamp = now_ms();
        vehicle_bind_msg.model = pkg.model();
        vehicle_bind_msg.user_id = pkg.user_id();
        vehicle_bind_msg.status = pkg.status();
        vehicle_bind_msg.privacy_policy = pkg.privacy_policy();

        device_bind_pub_->publish(vehicle_bind_msg);
        RCLCPP_INFO(this->get_logger(), "✅ 指令已发布到ROS2主题：%s，user_id：%s，status：%d，privacy_policy：%s",
                    cloudManagerDeviceBindPubTopic, vehicle_bind_msg.user_id.c_str(), vehicle_bind_msg.status, vehicle_bind_msg.privacy_policy.c_str());
        // 更新一下绑定状态，作为成员存储
        if(pkg.status() >= 0 && pkg.status() <= 2) {
            device_bind_status_ = pkg.status();
            RCLCPP_INFO(this->get_logger(), "✅ 数据有效更新数据status：%d", device_bind_status_);
        }

        if(!pkg.privacy_policy().empty()) {
            device_bind_privacy_policy_ = pkg.privacy_policy();
            if(device_bind_privacy_policy_.find("Location") != std::string::npos) {
                user_gps_agree_ = true;
                RCLCPP_INFO(this->get_logger(), "用户同意上传GPS数据：%d", user_gps_agree_);
            } else {
                user_gps_agree_ = false;
                RCLCPP_INFO(this->get_logger(), "用户不同意上传GPS数据：%d", user_gps_agree_);
            }
            RCLCPP_INFO(this->get_logger(), "✅ 数据有效更新数据privacy_policy：%s", device_bind_privacy_policy_.c_str());
        } else {
            user_gps_agree_ = false;
            RCLCPP_INFO(this->get_logger(), "隐私协议为空，不上传GPS数据：%d", user_gps_agree_);
        }                
    }

    // 设备绑定与解绑，HTTP验证 
    bool device_bind_http_send(const std::string &str_url) {
        vehicle::device::bind::VehicleBindStatus pkg{};
        std::string err_msg = "";
        bool req_flag = HttpClientManager::getInstance()->device_bind_msg(str_url, device_info_, pkg, err_msg);

        if(!req_flag){
            RCLCPP_ERROR(this->get_logger(), "❌ 设备绑定HTTP验证失败：%s", err_msg.c_str());
            return false;
        } else {
            send_event_device_bind_ack(pkg);
        }
        return true;
    }

    void on_error_event_callback(const interface::msg::HexData::SharedPtr msg) {
        uint32_t error_code = msg->hex_value;
        int64_t now = now_ms();
        if (error_code != (error_code & 0xFFFFFFFF)) {
            RCLCPP_ERROR(this->get_logger(), "错误码无效错误码 %d", msg->hex_value);
            return;
        }

        uint32_t error_type = get_error_type(error_code); // 提取错误码
        uint8_t status = get_error_status(error_code); //提取当前错误码是否生效或解除
        // 核心逻辑：获取的错误码存在map中，没有该错误码则添加，如果检测到5次，进行错误码处理
        // 如果是解除错误码，则将错误码从map中删除
        {
            auto it = error_event_map_.find(error_type);
            if (it != error_event_map_.end()) {
                if (status == 0x01) {
                    if (it->second.count < 5) {
                        it->second.status = 1;
                        it->second.count++;
                        RCLCPP_INFO(this->get_logger(), "数据累加：0x%08X，错误码生效状态：%d", error_type, status);
                        return;
                    }
                } else if (status == 0x02) {
                    if (!error_event_map_[error_type].reported) {
                        error_event_map_.erase(error_type);
                        RCLCPP_INFO(this->get_logger(), "收到解除消息但数据未上报清除map：0x%08X，错误码生效状态：%d", error_type, status);
                        return;
                    }
                    it->second.status = 2;
                }
            } else {
                if (status == 0x01) {
                    error_event_map_[error_type].status = 1;
                    error_event_map_[error_type].count = 1;
                    error_event_map_[error_type].reported = false;
                }
                return; // 排除直接发“解除”的情况
            }
        }

        // 如果已经上报过了，则不再继续上报
        if (status == 0x01 && error_event_map_[error_type].reported) {
            RCLCPP_INFO(this->get_logger(), "再次触发上报但已上报，不再重复上报：0x%08X，错误码生效状态：%d", error_type, status);
            return;
        }

        // 错误码未分类 现在统一处理为故障事件
        char hex_buf[10];
        snprintf(hex_buf, sizeof(hex_buf), "%08X", error_code);
        std::string error_code_str = std::string(hex_buf);
        RCLCPP_INFO(this->get_logger(), "收到错误事件上报，code=%s", error_code_str.c_str());
        // 使用错误码枚举获取描述和模块
        freego_error_code::ErrorCode error_code_enum;
        try {
            // 使用stoul将十六进制字符串转换为无符号长整型
            uint32_t error_code_value = static_cast<uint32_t>(std::stoul(error_code_str, nullptr, 16));
            error_code_enum = static_cast<freego_error_code::ErrorCode>(error_code_value);
            RCLCPP_INFO(this->get_logger(), "错误码枚举值：0x%08X", static_cast<uint32_t>(error_code_enum));
        } catch (const std::invalid_argument& e) {
            error_code_enum = freego_error_code::ErrorCode::UNKNOWN_ERROR;
            RCLCPP_ERROR(this->get_logger(), "无效的错误码 %s", error_code_str.c_str());
        } catch (const std::out_of_range& e) {
            error_code_enum = freego_error_code::ErrorCode::UNKNOWN_ERROR;
            RCLCPP_ERROR(this->get_logger(), "错误码超出范围 %s", error_code_str.c_str());
        }
        std::string error_desc = freego_error_code::get_error_description(error_code_enum);
        std::string error_module = freego_error_code::get_error_module(error_code_enum);

        vehicle::event::VehicleEventInfo info;
        // 基本信息
        info.set_sn(get_vehicle_sn());
        info.set_device_timestamp(now);
        info.set_model(device_model_);
        info.set_version(software_version_);
        info.set_config_version(current_config_version_num_);

        // 事件信息
        info.set_event_id(make_nonce());
        info.set_event_type(2);  // 2=故障事件
        info.set_duration_type(0);  // 0=瞬时事件
        info.set_status(2);  // 2=结束（瞬时事件直接结束）
        info.set_up_index(0);

        // DTC故障信息
        auto* dtc = info.mutable_dtc_info();
        dtc->set_code("0x" + error_code_str);
        dtc->set_desc(error_module + ":" + error_desc);
        dtc->set_first_ms(now);
        dtc->set_last_ms(now);

        publish_passive_msg(event_push_topic_, "错误事件上报", info);

        if(status == 0x01) {
            error_event_map_[error_type].reported = true; 
        } else if(status == 0x02) {
            error_event_map_.erase(error_type);
        }
           
    }

    void car_info_callback(const interface::msg::CarInfo::SharedPtr msg) {
        current_status_.speed_kmh = msg->speedkmh;
        current_status_.remaining_mileage = static_cast<int32_t>(msg->remainingmileage);
        current_status_.gear = static_cast<int32_t>(msg->gear);
        current_status_.charging_status = static_cast<int32_t>(msg->getcharging);
        current_status_.parking_status = static_cast<int32_t>(msg->lockswitch);
        current_status_.assist_mode = static_cast<int32_t>(msg->helpcar);
        current_status_.battery = static_cast<int32_t>(msg->electricquantity);
        current_status_.left_brakests = static_cast<int32_t>(msg->left_brakests);
        current_status_.right_brakests = static_cast<int32_t>(msg->right_brakests);
        // RCLCPP_INFO(this->get_logger(), "🚗 车辆信息更新：速度=%.2fkm/h，电量=%d%%",
        //             current_status_.speed_kmh, current_status_.battery);
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
        write_status_key("robot_mode", mode);
    }

    // void encoder_callback(const interface::msg::Encoder::SharedPtr msg) {
    //     return;
    // }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if(!user_gps_agree_) {
            RCLCPP_INFO(this->get_logger(), "用户不同意上传GPS数据，cloud_manager不会主动上传数据");
            return;
        }
        try {
            if (!mqtt_manager_ || !mqtt_manager_->is_connected()) {
                RCLCPP_ERROR(this->get_logger(), "❌ MQTT未连接");
                return;
            }

            gps::GPSData pb_msg;
            auto base = pb_msg.mutable_base();
            base->set_model(MODEL_X5_CODE);
            base->set_vin(get_vehicle_sn());
            // 使用 message header 时间（秒为 double）作为采样时间
            double ts = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
            const auto device_ts_ms = static_cast<int64_t>(std::llround(ts * 1000.0));
            base->set_device_timestamp(device_ts_ms);
            pb_msg.set_timestamp(ts);
            pb_msg.set_latitude(msg->latitude);
            pb_msg.set_longitude(msg->longitude);
            pb_msg.set_altitude(msg->altitude);
            pb_msg.set_velocity_n(0.0f);
            pb_msg.set_velocity_e(0.0f);
            pb_msg.set_velocity_u(0.0f);
            pb_msg.set_fix_type(static_cast<uint32_t>(msg->status.status));
            pb_msg.set_num_satellites(0);
            pb_msg.set_hdop(0.0f);
            if (log_pre_publish_) {
                RCLCPP_INFO(this->get_logger(),
                            "➡️ 即将上报GPS: lat=%.6f lon=%.6f alt=%.2f ts=%.3f fix=%d",
                            msg->latitude,
                            msg->longitude,
                            msg->altitude,
                            ts,
                            msg->status.status);
            }

            // 序列化前检查必填字段，避免Protobuf FatalException
            if (!pb_msg.IsInitialized()) {
                RCLCPP_ERROR(this->get_logger(), "❌ GPS数据未初始化完整：%s", pb_msg.InitializationErrorString().c_str());
                return;
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
            std::string signature = signature_mqtt_mgr_->sign_data(sign_source);
            RCLCPP_DEBUG(this->get_logger(), "GPS签名前20：%s", signature.substr(0,20).c_str());

            std::string gps_topic = build_gps_topic();

            int ret = mqtt_manager_->publish_with_properties(
                gps_topic,
                raw_data,
                0,
                {
                    {"gtime", now_ms},
                    {"gnonce", nonce},
                    {"gsign", signature}
                }
            );

            if (ret == MOSQ_ERR_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "📤 GPS上报成功（payload:%zu bytes gtime=%s nonce=%s）", raw_data.size(), now_ms.c_str(), nonce.c_str());
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

    void publish_realtime_msg() {
        if (device_bind_status_ != 1) {
            RCLCPP_INFO(this->get_logger(), "设备未绑定，不发送车辆状态数据: device_bind_status:%d", device_bind_status_);
            return;
        }
        try {
            if (!mqtt_manager_ || !mqtt_manager_->is_connected()) {
                RCLCPP_ERROR(this->get_logger(), "❌ MQTT未连接");
                return;
            }

            // 1. 序列化原始Protobuf数据（VehicleInfo）并填充 base
            car_status::VehicleInfo pb_msg;
            // 填充 BaseInfo（写死为 x5/x001，与 topic 对齐）
            auto base = pb_msg.mutable_base();
            base->set_model(MODEL_X5_CODE);
            base->set_vin(get_vehicle_sn());
            // 时间戳（毫秒）
            int64_t ts = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            base->set_device_timestamp(ts);

            pb_msg.set_speed_kmh(current_status_.speed_kmh);
            pb_msg.set_remaining_mileage(current_status_.remaining_mileage);
            pb_msg.set_gear(current_status_.gear);
            int line_charging_status = (current_status_.charging_status == 1) ? 1 : 0;
            int return_charging_status = (current_status_.charging_status == 2) ? 1 : 0;
            pb_msg.set_charging_status(current_status_.charging_status);
            pb_msg.set_line_charging(line_charging_status);
            pb_msg.set_return_charging(return_charging_status);
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
            std::string signature = signature_mqtt_mgr_->sign_data(sign_source);

            RCLCPP_DEBUG(this->get_logger(), "实时上报签名前20：%s", signature.substr(0,20).c_str());
            std::string vehicle_info_topic = build_vehicle_info_topic();
            RCLCPP_INFO(this->get_logger(), "获取vehicle_info_topic：%s", vehicle_info_topic.c_str());
            
            int ret = mqtt_manager_->publish_with_properties(
                vehicle_info_topic,
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

    template <typename T>
    void publish_passive_msg(const std::string& topic, const std::string& type_name, const T& msg) {
        if (device_bind_status_ != 1) {
            RCLCPP_INFO(this->get_logger(), "设备未绑定，不发送数据: device_bind_status:%d", device_bind_status_);
            return;
        }

        try {
            if (!mqtt_manager_ || !mqtt_manager_->is_connected()) {
                RCLCPP_ERROR(this->get_logger(), "❌ MQTT未连接");
                return;
            }

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
            std::string signature = signature_mqtt_mgr_->sign_data(sign_source);
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

    void check_and_report_passive_status()
    {
        if (device_bind_status_ != 1) {
            RCLCPP_INFO(this->get_logger(), "设备未绑定，不发送整车应用数据: device_bind_status:%d", device_bind_status_);
            return;
        }
        // 使用 VehicleStatus 合并被动字段，仅在字段变更时设置对应可选字段
        car_status::VehicleStatus vs;
        auto base = vs.mutable_base();
        base->set_model(MODEL_X5_CODE);
        base->set_vin(get_vehicle_sn());
        int64_t ts = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
        base->set_device_timestamp(ts);

        bool need_send = false;
        if (current_status_.recharge_status != history_cache_.recharge_status)
        {
            vs.mutable_recharge_status()->set_value(current_status_.recharge_status);
            history_cache_.recharge_status = current_status_.recharge_status;
            need_send = true;
        }
        if (current_status_.summon_status != history_cache_.summon_status)
        {
            vs.mutable_summon_status()->set_value(current_status_.summon_status);
            history_cache_.summon_status = current_status_.summon_status;
            need_send = true;
        }
        if (current_status_.follow_me_status != history_cache_.follow_me_status)
        {
            vs.mutable_follow_me_status()->set_value(current_status_.follow_me_status);
            history_cache_.follow_me_status = current_status_.follow_me_status;
            need_send = true;
        }
        if (current_status_.current_mode_chinese != history_cache_.current_mode_chinese)
        {
            vs.mutable_current_mode_chinese()->set_value(current_status_.current_mode_chinese);
            history_cache_.current_mode_chinese = current_status_.current_mode_chinese;
            need_send = true;
        }
        if (current_status_.alarmstatus != history_cache_.alarmstatus)
        {
            vs.mutable_alarmstatus()->set_value(current_status_.alarmstatus);
            history_cache_.alarmstatus = current_status_.alarmstatus;
            need_send = true;
        }

        if (need_send)
        {
            if (utils::TriggerLogger::getInstance().isInitialized())
            {
                std::string changed_fields;
                if (vs.has_recharge_status())
                    changed_fields += "recharge_status|";
                if (vs.has_summon_status())
                    changed_fields += "summon_status|";
                if (vs.has_follow_me_status())
                    changed_fields += "follow_me_status|";
                if (vs.has_current_mode_chinese())
                    changed_fields += "mode|";
                if (vs.has_alarmstatus())
                    changed_fields += "alarmstatus|";
                if (!changed_fields.empty() && changed_fields.back() == '|')
                    changed_fields.pop_back();
                auto log_data = std::map<std::string, std::string>{
                    {"changed", changed_fields},
                    {"recharge_status", vs.has_recharge_status() ? std::to_string(vs.recharge_status().value()) : ""},
                    {"summon_status", vs.has_summon_status() ? std::to_string(vs.summon_status().value()) : ""},
                    {"follow_me_status", vs.has_follow_me_status() ? std::to_string(vs.follow_me_status().value()) : ""},
                    {"mode", vs.has_current_mode_chinese() ? vs.current_mode_chinese().value() : ""},
                    {"alarmstatus", vs.has_alarmstatus() ? std::to_string(vs.alarmstatus().value()) : ""}};
                TRIGGER_LOG_WITH_NODE("status_change", log_data, this->get_name());
            }
            if (log_pre_publish_)
            {
                RCLCPP_INFO(this->get_logger(),
                            "➡️ 即将上报被动: recharge=%s summon=%s follow=%s mode=%s alarm=%s",
                            vs.has_recharge_status() ? std::to_string(vs.recharge_status().value()).c_str() : "",
                            vs.has_summon_status() ? std::to_string(vs.summon_status().value()).c_str() : "",
                            vs.has_follow_me_status() ? std::to_string(vs.follow_me_status().value()).c_str() : "",
                            vs.has_current_mode_chinese() ? vs.current_mode_chinese().value().c_str() : "",
                            vs.has_alarmstatus() ? std::to_string(vs.alarmstatus().value()).c_str() : "");
            }

            std::string vehicle_status_topic = build_vehicle_status_topic();
            publish_passive_msg(vehicle_status_topic, "车辆状态", vs);
        }
    }

    void declare_ros_parameters()
    {
        // ========== 新增：声明ROS参数 ==========
        this->declare_parameter("mqtt.broker", "mqtts://39.106.140.222:28883");
        this->declare_parameter("mqtt.client_id", "ZQKJ001_cloud");
        this->declare_parameter("mqtt.retry_attempts", 3);
        this->declare_parameter("mqtt.retry_delay_base", 2.0); // 指数退避基数（秒）
        this->declare_parameter<std::string>("mqtt.username", "");
        this->declare_parameter<std::string>("mqtt.password", "");
        this->declare_parameter<std::string>("mqtt.ca_cert", "");
        this->declare_parameter<bool>("mqtt.insecure_skip_verify", true);
        this->declare_parameter<std::string>("mqtt.cipher", "ECDHE-RSA-AES256-GCM-SHA384");
        this->declare_parameter<std::string>("mqtt.jwt_token", "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpYXQiOjE3NjcxOTY4MDB9.jZHEpHOzI__buvnuLyA0RkHiOp3ThtKrngdknIc0wYo");
        this->declare_parameter<double>("mqtt.reconnect_interval_sec", 5.0);
        this->declare_parameter<int64_t>("mqtt.token_refresh_margin_ms", 300000);
        // 声明签名相关参数（可通过命令行或参数文件注入 hex 格式私钥/公钥）
        this->declare_parameter<std::string>("signature.private_key", "");
        this->declare_parameter<std::string>("signature.public_key", "");
        // 日志开关：是否打印“即将上报”明细，默认开启便于联调
        this->declare_parameter<bool>("log.pre_publish", true);
        this->declare_parameter<bool>("log.mileage", true);
        // GPS模拟参数（无真机时上报用）
        this->declare_parameter<bool>("gps.simulate", false);
        this->declare_parameter<double>("gps.sim_rate_hz", 1.0);
        this->declare_parameter<double>("gps.sim_lat", 39.9042); // 默认北京坐标
        this->declare_parameter<double>("gps.sim_lon", 116.4074);
        this->declare_parameter<double>("gps.sim_alt", 50.0);
        // 设备与事件配置参数
        this->declare_parameter<std::string>("device.model", "FREEGO X5 Air");
        this->declare_parameter<std::string>("device.model_http", "FREEGO X5 Air");
        this->declare_parameter<std::string>("device.id", "x001");
        this->declare_parameter<std::string>("protocol.version", "1");
        this->declare_parameter<std::string>("software.version", "1.0.0");
        this->declare_parameter<std::string>("event.config_path", "/tmp/event_config.bin");
        this->declare_parameter<std::string>("event.config_json_path", "/tmp/event_config.json");
        this->declare_parameter<bool>("event.http_enable", true);
        this->declare_parameter<double>("event.http_interval_sec", 600.0);
        this->declare_parameter<std::string>(
            "event.http_url", device_info_.api_base_url + "/api/device/v1/config/getUp");
    }

    void load_ros_parameters()
    {
        // 加载配置参数
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

        mqtt_client_id_ = get_vehicle_sn();
    }

    void init_mqtt_manager()
    {
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
        if (private_key.empty())
        {
            private_key = default_root_private_key;
        }
        if (public_key.empty())
        {
            public_key = default_root_public_key;
        }
        if (gkey_no.empty()) {
            gkey_no = default_root_gkey_no;
        }

        sign_config.private_key = private_key;
        sign_config.public_key = public_key;
        sign_config.sign_separator = "|";
        // MQTT连接参数
        mqtt_module::MqttConnConfig mqtt_config;
        mqtt_config.broker = mqtt_broker_url_;
        mqtt_config.client_id = mqtt_client_id_;
        mqtt_config.username = mqtt_username_;
        mqtt_config.password = mqtt_password_;
        mqtt_config.ca_cert_path = mqtt_ca_cert_;
        mqtt_config.insecure_skip_verify = mqtt_insecure_skip_verify_;
        mqtt_config.tls_cipher = mqtt_cipher_;

        RCLCPP_INFO(this->get_logger(),
                    "MQTT连接参数打印: broker=%s client_id=%s username=%s password=%s ca_cert_path=%s tls_cipher=%s",
                    mqtt_config.broker.c_str(),
                    mqtt_config.client_id.c_str(),
                    mqtt_config.username.c_str(),
                    mqtt_config.password.c_str(),
                    mqtt_config.ca_cert_path.c_str(),
                    mqtt_config.tls_cipher.c_str());

        // 兼容 mqtts:// 前缀
        if (mqtt_config.broker.rfind("mqtts://", 0) == 0)
        {
            mqtt_config.broker = "ssl://" + mqtt_config.broker.substr(8);
        }
        else if (mqtt_config.broker.rfind("mqtt://", 0) == 0)
        {
            mqtt_config.broker = "tcp://" + mqtt_config.broker.substr(7);
        }
        if (mqtt_config.username.empty() && mqtt_config.password.empty())
        {
            mqtt_config.use_token_auth = true; // 如无账号密码，则可回退 token 方式
        }

        // 初始化 MQTT 客户端配置
        mqtt_module::MqttClientConfig client_config;
        client_config.device = mqtt_device_info_;
        client_config.mqtt = mqtt_config;
        client_config.signature = sign_config;

        // 创建 MQTT 客户端
        mqtt_client_ = std::make_shared<mqtt_module::MqttClient>(client_config, this->get_logger());
        // 设置 MQTT 回调
        mqtt_manager_ = mqtt_client_->connection();
        mqtt_manager_->set_disconnect_callback([this](int rc)
                                      {
        RCLCPP_WARN(this->get_logger(), "MQTT连接断开: %s", mosquitto_strerror(rc));
        this->on_connection_lost(std::string(mosquitto_strerror(rc))); });
        mqtt_manager_->set_message_callback([this](const std::string &topic, const std::string &payload, const std::vector<std::pair<std::string, std::string>> &properties)
                                   { this->on_mqtt_message(topic, payload, properties); });

        return;
    }

    void connect_and_subscribe() {
        if (!mqtt_client_) {
            RCLCPP_FATAL(this->get_logger(), "MQTT客户端未初始化");
            throw std::runtime_error("mqtt client null");
        }

        RCLCPP_INFO(this->get_logger(), "开始连接MQTT Broker: %s (client_id: %s)", mqtt_broker_url_.c_str(), mqtt_client_id_.c_str());
        for (int attempt = 0; attempt < mqtt_retry_attempts_; ++attempt) {      
            RCLCPP_INFO(this->get_logger(), "MQTT连接尝试 %d/%d，使用完整认证流程", attempt + 1, mqtt_retry_attempts_);
            if (mqtt_client_->connect_with_token_flow())
            {
                RCLCPP_INFO(this->get_logger(), "MQTT连接成功！Broker: %s", mqtt_broker_url_.c_str());
                // 获取token和key
                mqtt_client_->get_session_key(mqtt_device_info_.session_public_key, mqtt_device_info_.session_private_key, mqtt_device_info_.session_key_no);
                token_result_ = mqtt_client_->token();

                RCLCPP_INFO(this->get_logger(), "获取token成功，jwt_token：%s，exp：%ld", token_result_.jwt_token.c_str(), token_result_.exp);

                mqtt_module::SignatureConfig cfg;
                cfg.public_key = mqtt_device_info_.session_public_key;
                cfg.private_key = mqtt_device_info_.session_private_key;
                signature_mqtt_mgr_ = std::make_shared<mqtt_module::SignatureManager>(cfg, this->get_logger());

                subscribe_device_bind_topics(); //
                return;
            }
            if (attempt < mqtt_retry_attempts_ - 1)
            {
                double delay = mqtt_retry_delay_base_ * std::pow(2.0, attempt);
                RCLCPP_WARN(this->get_logger(), "MQTT连接失败，重试 %d/%d, 等待 %.1f 秒", attempt + 1, mqtt_retry_attempts_, delay);
                std::this_thread::sleep_for(std::chrono::duration<double>(delay));
            }
        }
        RCLCPP_FATAL(this->get_logger(), "MQTT连接失败！已尝试 %d 次，Broker: %s", mqtt_retry_attempts_, mqtt_broker_url_.c_str());
        throw std::runtime_error("MQTT连接失败");
    }

    void subscribe_device_bind_topics()
    {
        if (!mqtt_client_ || !mqtt_manager_ || !mqtt_manager_->is_connected())
        {
            RCLCPP_WARN(this->get_logger(), "MQTT未连接，无法订阅设备绑定主题");
            return;
        }
        subscribe_topic(build_device_bind_topic());
    }
    bool subscribe_topic(const std::string &topic, int qos = 0)
    {
        if (topic.empty())
        {
            return false;
        }
        if (!mqtt_client_ || !mqtt_manager_ || !mqtt_manager_->is_connected())
        {
            return false;
        }
        int rc = mqtt_manager_->subscribe_v5(topic, qos);
        if (rc == MOSQ_ERR_SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "已订阅: %s", topic.c_str());
            return true;
        }
        RCLCPP_WARN(this->get_logger(), "订阅失败: %s (%s)", topic.c_str(), mosquitto_strerror(rc));
        return false;
    }
    void on_connection_lost(const std::string &cause)
    {
        RCLCPP_WARN(this->get_logger(), "MQTT连接丢失: %s", cause.c_str());
    }

    void on_mqtt_message(const std::string &topic, const std::string &payload, const std::vector<std::pair<std::string, std::string>> &properties) {
        RCLCPP_INFO(this->get_logger(), "===== 收到MQTT消息 =====");
        RCLCPP_INFO(this->get_logger(), "主题: %s", topic.c_str());
        RCLCPP_INFO(this->get_logger(), "消息长度: %zu 字节", payload.size());

        RCLCPP_INFO(this->get_logger(), "MQTT v5属性详情:");
        if (properties.empty())
        {
            RCLCPP_INFO(this->get_logger(), "  [无属性]");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "  属性数量: %zu", properties.size());
            for (size_t i = 0; i < properties.size(); ++i)
            {
                const auto &[k, v] = properties[i];
                RCLCPP_INFO(this->get_logger(), "  属性[%zu]: 键='%s', 值='%s', 键长度=%zu, 值长度=%zu",
                            i + 1, k.c_str(), v.c_str(), k.size(), v.size());
            }
        }

        std::map<std::string, std::string> all_properties;
        std::string gsign, gtime, gnonce, gkeyno;
        RCLCPP_INFO(this->get_logger(), "🔍 开始解析Properties...");
        for (const auto &[k, v] : properties)
        {
            std::string key_lower = k;
            std::transform(key_lower.begin(), key_lower.end(), key_lower.begin(), ::tolower);
            all_properties[key_lower] = v;
            RCLCPP_INFO(this->get_logger(), "  属性解析: key='%s', key='%s', value='%s'", k.c_str(), key_lower.c_str(), v.c_str());
            if (key_lower == "gsign")
            {
                gsign = v;
                RCLCPP_INFO(this->get_logger(), "  ✅ 找到签名属性: gsign='%s'", v.substr(0, 20).c_str());
            }
            else if (key_lower == "gtime")
            {
                gtime = v;
                RCLCPP_INFO(this->get_logger(), "  ✅ 找到时间属性: gtime='%s'", v.c_str());
            }
            else if (key_lower == "gnonce")
            {
                gnonce = v;
                RCLCPP_INFO(this->get_logger(), "  ✅ 找到随机数属性: gnonce='%s'", v.c_str());
            }
        }

        // 验证签名
        bool signature_valid = true;
        std::string raw_payload = payload;
        if (!gsign.empty())
        {
            RCLCPP_INFO(this->get_logger(), "🔍 开始验证消息签名...");

            if (gtime.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "❌ 指令缺少gtime属性，验签失败");
                signature_valid = false;
            }
            if (gnonce.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "❌ 指令缺少gnonce随机数，验签失败");
                signature_valid = false;
            }

            if (signature_valid)
            {
                // 构造签名源：gtime + gnonce + payload
                std::string sign_source = gtime + gnonce + raw_payload;
                RCLCPP_INFO(this->get_logger(), "🔍 车端拼接待验签数据：gtime='%s', gnonce='%s', payload_len=%zu, total_len=%zu",
                            gtime.c_str(), gnonce.c_str(), raw_payload.size(), sign_source.size());

                // 检查是否为任务状态变更消息（使用根公钥验签）
                // bool is_task_status_msg = (!current_task_id_.empty() && topic == build_task_status_topic(current_task_id_));
                bool is_task_status_msg = false;
                if (is_task_status_msg)
                {
                    RCLCPP_INFO(this->get_logger(), "📋 任务状态变更消息，使用根公钥验签");
                    const std::string root_public_key_b64 = mqtt_device_info_.session_public_key;

                    // 使用直接的libsodium函数进行根公钥验证
                    std::vector<unsigned char> root_public_key;
                    if (mqtt_module::b64_to_bytes(root_public_key_b64, root_public_key))
                    {
                        if (root_public_key.size() != crypto_sign_PUBLICKEYBYTES)
                        {
                            RCLCPP_ERROR(this->get_logger(), "❌ 根公钥长度无效: %zu, 期望: %d", root_public_key.size(), crypto_sign_PUBLICKEYBYTES);
                            signature_valid = false;
                        }
                        else
                        {
                            // 转换签名为字节数组
                            std::vector<unsigned char> sig_bytes;
                            if (mqtt_module::b64_to_bytes(gsign, sig_bytes))
                            {
                                if (sig_bytes.size() != crypto_sign_BYTES)
                                {
                                    RCLCPP_ERROR(this->get_logger(), "❌ 根公钥签名长度无效: %zu, 期望: %d", sig_bytes.size(), crypto_sign_BYTES);
                                    signature_valid = false;
                                }
                                else
                                {
                                    // 执行实际的验签
                                    int rc = crypto_sign_verify_detached(
                                        sig_bytes.data(),
                                        reinterpret_cast<const unsigned char *>(sign_source.data()),
                                        sign_source.size(),
                                        root_public_key.data());

                                    if (rc == 0)
                                    {
                                        RCLCPP_INFO(this->get_logger(), "✅ 任务状态消息根公钥验签通过");
                                        signature_valid = true;
                                    }
                                    else
                                    {
                                        RCLCPP_ERROR(this->get_logger(), "❌ 任务状态消息根公钥验签失败，错误码: %d", rc);
                                        signature_valid = false;
                                    }
                                }
                            }
                            else
                            {
                                RCLCPP_ERROR(this->get_logger(), "❌ 根公钥签名Base64解码失败");
                                signature_valid = false;
                            }
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "❌ 根公钥Base64解码失败");
                        signature_valid = false;
                    }
                }
                else
                {
                    // 普通消息使用原有签名管理器验签
                    RCLCPP_INFO(this->get_logger(), "🔍 普通消息或OTA升级消息，使用默认公钥验签");
                    if (signature_mqtt_mgr_ && signature_mqtt_mgr_->verify_data(sign_source, gsign))
                    {
                        RCLCPP_INFO(this->get_logger(), "✅ 消息签名验证通过");
                        signature_valid = true;
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "❌ 消息签名验证失败");
                        signature_valid = false;
                    }
                }
            }
        }
        else {
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

        // 使用回调进行多种topic的接收判断 TODO

        // build_device_bind_topic
        const std::string device_bind_topic = build_device_bind_topic();
        if (topic == device_bind_topic) {
            RCLCPP_INFO(this->get_logger(), "===== 收到车辆绑定信息 =====");
            RCLCPP_INFO(this->get_logger(), "主题: %s", topic.c_str());
            RCLCPP_INFO(this->get_logger(), "===== 处理车辆绑定消息 =====");
            RCLCPP_INFO(this->get_logger(), "车辆绑定消息属性详情:");
            for (const auto &[key, value] : all_properties) {
                RCLCPP_INFO(this->get_logger(), "  %s: %s", key.c_str(), value.c_str());
            }
            // 处理并转发消息
            handle_event_device_bind_send(payload, "mqtt");
        }
    }

    std::string build_device_bind_topic() const {
        // vehicle/bind//<model>/<sn>/<protocol>/status
        std::string model_for_topic = mqtt_device_info_.model;
        model_for_topic.erase(std::remove(model_for_topic.begin(), model_for_topic.end(), ' '), model_for_topic.end());
        return "vehicle/bind/" + model_for_topic + "/" + mqtt_device_info_.vin + "/" + protocol_version_ + "/status";
    }

    std::string build_vehicle_info_topic() const {
        // vehicle/up//<model>/<sn>/<protocol>/info  model通过配置文件获取
        return "vehicle/up/" + mqtt_device_info_.model + "/" + mqtt_device_info_.vin + "/" + protocol_version_ + "/info";
    }


    std::string build_vehicle_status_topic() const {
        // vehicle/up//<model>/<sn>/<protocol>/status  model通过配置文件获取
        return "vehicle/up/" + mqtt_device_info_.model + "/" + mqtt_device_info_.vin + "/" + protocol_version_ + "/status";
    }

    std::string build_gps_topic() const {
        // vehicle/up//<model>/<sn>/<protocol>/gps  model通过配置文件获取
        return "vehicle/up/" + mqtt_device_info_.model + "/" + mqtt_device_info_.vin + "/" + protocol_version_ + "/gps";
    }

    void make_signature_mqtt_manager() {
        mqtt_module::SignatureConfig sign_config;
        if (!mqtt_device_info_.session_public_key.empty()) {
            // 公钥用于验签，私钥用于加签
            sign_config.private_key = mqtt_device_info_.session_private_key; // 会话私钥，加签
            sign_config.public_key = mqtt_device_info_.session_public_key;   // 会话公钥，验签
        }
        else {
            // 测试环境：若未提供真实私钥，自动生成 seed 并由 SignatureManager 派生 keypair
            if (mqtt_device_info_.private_key.empty() || mqtt_device_info_.private_key.find("MOCK") != std::string::npos) {
                sign_config.private_key = mqtt_module::generate_seed_hex();
            }
            else {
                sign_config.private_key = mqtt_device_info_.private_key; // 复用设备私钥
            }
        }
        sign_config.sign_separator = "|"; // 数据-签名分隔符
        signature_mqtt_mgr_ = std::make_shared<mqtt_module::SignatureManager>(
            sign_config, this->get_logger());
        if(signature_mqtt_mgr_){
            RCLCPP_INFO(this->get_logger(), "注册signature_mqtt_mgr_完成");
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ 册signature_mqtt_mgr_失败!");
        }
    }

    // 使用内部库实现MQTT连接
    std::shared_ptr<mqtt_module::MqttClient> mqtt_client_;
    // MQTT发送
    std::shared_ptr<mqtt_module::MqttConnectionManager> mqtt_manager_; 
    std::shared_ptr<mqtt_module::SignatureManager> signature_mqtt_mgr_;
    // 协议基础 topic（例如 vehicle/up/x5/x001/1）
    std::string base_topic_;
    std::string device_model_ = "FREEGO X5 Air";
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
    rclcpp::Subscription<interface::msg::HexData>::SharedPtr event_push_sub_;
    mqtt_module::DeviceInfo device_info_;
    mqtt_module::DeviceInfo mqtt_device_info_;
    int64_t token_expire_at_ms_ = 0;
    int64_t token_refresh_margin_ms_ = 300000;
    std::atomic<bool> reconnecting_{false};
    std::atomic<bool> refreshing_token_{false};
    bool using_token_auth_ = false;
    double mqtt_reconnect_interval_sec_ = 5.0;
    rclcpp::TimerBase::SharedPtr token_refresh_timer_;
    
    // 错误事件限流
    std::unordered_map<uint32_t, freego_error_code::ErrorInfo> error_event_map_;  // 错误码 -> 过滤错误码使用
    const int64_t ERROR_EVENT_RATE_LIMIT_MS = 3600000;  // 1小时错误事件限流窗口
    rclcpp::TimerBase::SharedPtr mqtt_reconnect_timer_;
    bool log_mileage_ = true;
    bool log_pre_publish_ = true;
    rclcpp::Subscription<interface::msg::CarInfo>::SharedPtr car_info_sub_;
    rclcpp::Subscription<interface::msg::RechargeStatus>::SharedPtr recharge_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scene_mode_sub_;
    rclcpp::Subscription<interface::msg::Encoder>::SharedPtr encoder_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vehicle_id_sub_;
    rclcpp::Client<interface::srv::SummonStatus>::SharedPtr summon_client_;
    rclcpp::Client<interface::srv::FollowStatus>::SharedPtr follow_client_;
    rclcpp::Client<interface::srv::IdControls>::SharedPtr id_client_;
    rclcpp::TimerBase::SharedPtr realtime_pub_timer_;
    rclcpp::TimerBase::SharedPtr service_query_timer_;
    rclcpp::TimerBase::SharedPtr vehicle_id_timer_;
    // GPS模拟
    bool simulate_gps_ = false;
    double sim_lat_ = 0.0;
    double sim_lon_ = 0.0;
    double sim_alt_ = 0.0;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sim_pub_;
    rclcpp::TimerBase::SharedPtr gps_sim_timer_;
    VehicleStatus current_status_;
    HistoryCache history_cache_;
    std::unique_ptr<MileageCalculator> mileage_calculator_;
    // mqtt_module::DomainControllerMonitor domain_monitor_;
    // rclcpp::TimerBase::SharedPtr domain_ctrl_timer_;
    // 错误事件发布（十六进制错误码）
    rclcpp::Publisher<interface::msg::HexData>::SharedPtr error_event_pub_;
    uint32_t last_error_code_ = 0;
    // 缓存上次上报的序列化结果，用于仅在变化时上报以节省带宽
    std::string last_domain_raw_;
    std::string vehicle_id_;

    // MQTT参数
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

    // 设备绑定
    rclcpp::Publisher<interface::msg::VehicleBindStatus>::SharedPtr device_bind_pub_;
    rclcpp::Service<interface::srv::BindStatus>::SharedPtr device_bind_status_service_ { nullptr };
    rclcpp::Service<interface::srv::GetToken>::SharedPtr get_token_service_ { nullptr };
    int device_bind_status_ = -1; // 0：未绑定，1：已绑定，2：变更（暂未启用）
    std::string device_bind_privacy_policy_ = "";
    // 用户是否同意上传GPS数据
    bool user_gps_agree_ = false;
};

}


int main(int argc, char* argv[]) {
    // 1. 初始化Protobuf并检查版本
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    try {
        rclcpp::init(argc, argv);
    } catch (const std::exception& e) {
        // 直接打印到标准输出（此时ROS2日志可能未初始化）
        std::cerr << "ROS2初始化失败：" << e.what() << std::endl;
        google::protobuf::ShutdownProtobufLibrary();
        return -1;
    }

    try {
        auto node = std::make_shared<cloud_manager_node::CloudManagerNode>();
        RCLCPP_INFO(node->get_logger(), "节点初始化成功，开始运行...");
        rclcpp::spin(node);
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("cloud_manager_node"), "未知异常");
    }

    rclcpp::shutdown();
    google::protobuf::ShutdownProtobufLibrary();
    RCLCPP_INFO(rclcpp::get_logger("cloud_manager_node"), "节点已退出并清理资源");
    return 0;
}


// 暂时没有用到的功能，暂时注释
#if 0
    // bool maybe_fetch_event_config_http() {
    //     if (!event_http_enabled_) return false;
    //     try {
    //         nlohmann::json body;
    //         body["sn"] = get_vehicle_sn();
    //         body["model"] = device_model_;
    //         body["configVersion"] = current_config_version_str_;

    //         auto ts = std::to_string(now_ms());
    //         std::string nonce = make_nonce();
    //         cpr::Header headers{
    //             {"Content-Type", "application/json; charset=UTF-8"},
    //             {"Gversion", device_info_.version},
    //             {"Glanguage", device_info_.language},
    //             {"GClient", device_info_.client},
    //             {"GApp-Version", device_info_.app_version},
    //             {"Gtime", ts},
    //             {"Gnonce", nonce}
    //         };
    //         std::string sign_source = headers["Gversion"] + headers["Glanguage"] + headers["GClient"] +
    //                                   headers["GApp-Version"] + headers["Gtime"] + headers["Gnonce"] + body.dump();
    //         headers["GSign"] = signature_mqtt_mgr_ ? signature_mqtt_mgr_->sign_data(sign_source) : "";

    //         auto resp = cpr::Post(cpr::Url{event_http_url_}, headers, cpr::Body{body.dump()}, cpr::Timeout{5000});
    //         if (resp.error) {
    //             RCLCPP_WARN(this->get_logger(), "⚠️ HTTP获取事件配置失败：%s", resp.error.message.c_str());
    //             return false;
    //         }
    //         if (resp.status_code != 200) {
    //             RCLCPP_WARN(this->get_logger(), "⚠️ HTTP获取事件配置返回码：%ld", resp.status_code);
    //             return false;
    //         }
    //         auto json_resp = nlohmann::json::parse(resp.text, nullptr, false);
    //         if (json_resp.is_discarded()) {
    //             RCLCPP_ERROR(this->get_logger(), "❌ 事件配置HTTP响应解析失败");
    //             return false;
    //         }
    //         nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
    //         std::string proto_b64 = data.value("proto", std::string(""));
    //         if (proto_b64.empty()) {
    //             RCLCPP_WARN(this->get_logger(), "⚠️ 事件配置响应缺少proto字段");
    //             return false;
    //         }
    //         std::string raw = base64_decode(proto_b64);
    //         if (raw.empty()) {
    //             RCLCPP_ERROR(this->get_logger(), "❌ 事件配置Base64解码失败");
    //             return false;
    //         }
    //         return handle_event_config_payload(raw, "http");
    //     } catch (const std::exception& e) {
    //         RCLCPP_ERROR(this->get_logger(), "❌ HTTP获取事件配置异常：%s", e.what());
    //         return false;
    //     }
    // }

    // void on_event_ack_message(const std::string& payload) {
    //     vehicle::event::VehicleEventAck ack;
    //     if (!ack.ParseFromString(payload)) {
    //         RCLCPP_WARN(this->get_logger(), "⚠️ 事件回执解析失败");
    //         return;
    //     }
    //     RCLCPP_INFO(this->get_logger(), "📩 收到事件回执：id=%s status=%d up_index=%ld",
    //                 ack.event_id().c_str(), ack.status(), static_cast<long>(ack.up_index()));
    // }

    // void on_event_push_request(const std_msgs::msg::String::SharedPtr msg) {
    //     nlohmann::json j = nlohmann::json::parse(msg->data, nullptr, false);
    //     if (j.is_discarded()) {
    //         RCLCPP_ERROR(this->get_logger(), "❌ 事件上报JSON解析失败");
    //         return;
    //     }
    //     if (!j.contains("event_type") || !j.contains("status")) {
    //         RCLCPP_ERROR(this->get_logger(), "❌ 事件上报缺少必填字段 event_type/status");
    //         return;
    //     }

    //     vehicle::event::VehicleEventInfo info;
    //     info.set_sn(get_vehicle_sn());
    //     info.set_device_timestamp(now_ms());
    //     info.set_model(device_model_);
    //     info.set_version(software_version_);
    //     info.set_config_version(current_config_version_num_);
    //     info.set_event_id(j.value("event_id", make_nonce()));
    //     info.set_event_type(j.value("event_type", 0));
    //     info.set_duration_type(j.value("duration_type", 0));
    //     info.set_status(j.value("status", 0));
    //     info.set_up_index(j.value("up_index", static_cast<int64_t>(0)));

    //     if (j.contains("source")) {
    //         auto src = j["source"];
    //         if (src.contains("device_type")) info.mutable_source()->set_device_type(src.value("device_type", ""));
    //         if (src.contains("device_sn")) info.mutable_source()->set_device_sn(src.value("device_sn", ""));
    //         if (src.contains("firmware_version")) info.mutable_source()->set_firmware_version(src.value("firmware_version", ""));
    //     }

    //     if (j.contains("safety_info")) {
    //         auto s = j["safety_info"];
    //         auto p = info.mutable_safety_info();
    //         if (s.contains("code")) p->set_code(s.value("code", ""));
    //         if (s.contains("level")) p->set_level(s.value("level", 0));
    //         if (s.contains("trigger_value")) p->set_trigger_value(s.value("trigger_value", 0.0));
    //         if (s.contains("threshold")) p->set_threshold(s.value("threshold", 0.0));
    //         if (s.contains("first_ms")) p->set_first_ms(s.value("first_ms", static_cast<int64_t>(0)));
    //         if (s.contains("last_ms")) p->set_last_ms(s.value("last_ms", static_cast<int64_t>(0)));
    //         if (s.contains("desc")) p->set_desc(s.value("desc", ""));
    //         if (s.contains("extras")) p->set_extras(s.value("extras", ""));
    //     }

    //     if (j.contains("alarm_info")) {
    //         auto a = j["alarm_info"];
    //         auto p = info.mutable_alarm_info();
    //         if (a.contains("code")) p->set_code(a.value("code", ""));
    //         if (a.contains("code_name")) p->set_code_name(a.value("code_name", ""));
    //         if (a.contains("desc")) p->set_desc(a.value("desc", ""));
    //         if (a.contains("level")) p->set_level(a.value("level", 0));
    //         if (a.contains("info")) p->set_info(a.value("info", ""));
    //         if (a.contains("first_ms")) p->set_first_ms(a.value("first_ms", static_cast<int64_t>(0)));
    //         if (a.contains("last_ms")) p->set_last_ms(a.value("last_ms", static_cast<int64_t>(0)));
    //     }

    //     if (j.contains("dtc_info")) {
    //         auto d = j["dtc_info"];
    //         auto p = info.mutable_dtc_info();
    //         if (d.contains("code")) p->set_code(d.value("code", ""));
    //         if (d.contains("desc")) p->set_desc(d.value("desc", ""));
    //         if (d.contains("info")) p->set_info(d.value("info", ""));
    //         if (d.contains("first_ms")) p->set_first_ms(d.value("first_ms", static_cast<int64_t>(0)));
    //         if (d.contains("last_ms")) p->set_last_ms(d.value("last_ms", static_cast<int64_t>(0)));
    //     }

    //     if (j.contains("telemetry")) {
    //         auto t = j["telemetry"];
    //         auto p = info.mutable_telemetry();
    //         if (t.contains("speed")) p->set_speed(t.value("speed", 0.0));
    //         if (t.contains("steering")) p->set_steering(t.value("steering", 0.0));
    //         if (t.contains("brake")) p->set_brake(t.value("brake", 0));
    //         if (t.contains("b_force")) p->set_b_force(t.value("b_force", 0));
    //         if (t.contains("battery")) p->set_battery(t.value("battery", 0.0));
    //         if (t.contains("battery_voltage_v")) p->set_battery_voltage_v(t.value("battery_voltage_v", 0.0));
    //         if (t.contains("battery_current_a")) p->set_battery_current_a(t.value("battery_current_a", 0.0));
    //         if (t.contains("gear")) p->set_gear(t.value("gear", 0));
    //         if (t.contains("mode")) p->set_mode(t.value("mode", 0));
    //         if (t.contains("latitude")) p->set_latitude(t.value("latitude", 0.0));
    //         if (t.contains("longitude")) p->set_longitude(t.value("longitude", 0.0));
    //         if (t.contains("altitude")) p->set_altitude(t.value("altitude", 0.0));
    //         if (t.contains("extras")) p->set_extras(t.value("extras", ""));
    //     }

    //     if (j.contains("log_file") && j["log_file"].is_array()) {
    //         for (const auto& lf : j["log_file"]) {
    //             if (lf.is_string()) info.add_log_file(lf.get<std::string>());
    //         }
    //     }
    //     if (j.contains("video_file") && j["video_file"].is_array()) {
    //         for (const auto& vf : j["video_file"]) {
    //             if (vf.is_string()) info.add_video_file(vf.get<std::string>());
    //         }
    //     }
    //     if (j.contains("extras")) {
    //         if (j["extras"].is_string()) info.set_extras(j.value("extras", ""));
    //         else info.set_extras(j["extras"].dump());
    //     }

    //     publish_passive_msg(event_push_topic_, "事件上报", info);
    // }

        // void handle_mqtt_message(const std::string& topic, const std::string& payload, const std::map<std::string, std::string>& properties) {
    //     if (topic == event_config_topic_) {
    //         handle_event_config_payload(payload, "mqtt");
    //     } else if (topic == event_ack_topic_) {
    //         on_event_ack_message(payload);
    //     } 
    // }

    // bool handle_event_config_payload(const std::string& payload, const std::string& source) {
    //     vehicle::config::EventConfigPackage pkg;
	//     if (!pkg.ParseFromString(payload)) {
    //         RCLCPP_ERROR(this->get_logger(), "❌ 事件配置解析失败（来源=%s）", source.c_str());
    //         return false;
    //     }
    //     if (pkg.config_id() == current_config_id_ && pkg.version() == current_config_version_str_) {
    //         RCLCPP_INFO(this->get_logger(), "ℹ️ 事件配置未变化，跳过处理（source=%s id=%s version=%s）",
    //                     source.c_str(), pkg.config_id().c_str(), pkg.version().c_str());
    //         return true;
    //     }
    //     persist_event_config(pkg, payload, source);
    //     send_event_config_ack(pkg);
    //     return true;
    // }

#endif