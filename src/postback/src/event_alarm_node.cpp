#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <cpr/cpr.h>
#include <chrono>
#include <string>
#include <sstream>
#include <random>
#include <filesystem>
#include <vector>
#include <map>
#include <set>
#include <mutex>
#include <condition_variable>
#include <future>
#include <thread>
#include <cstdlib>
#include <fstream>
#include <optional>

#include "event_config.pb.h"
#include "vehicle_event.pb.h"
#include <google/protobuf/util/json_util.h>

#include "postback/oss_uploader.hpp"
#include "mqtt_module/mqtt_connection.hpp"
#include <std_msgs/msg/string.hpp>

using json = nlohmann::json;

class EventAlarmNode : public rclcpp::Node {
public:
  EventAlarmNode() : Node("event_alarm_node") {
    declare_parameters();
    load_parameters();
    init_mqtt();

    // Subscribe to config update topic
    subscribe_config_topics();

    // Offline HTTP fallback timer (every 30s)
    int poll_sec = this->get_parameter("config.poll_interval_seconds").as_int();
    config_poll_timer_ = this->create_wall_timer(
        std::chrono::seconds(std::max(1, poll_sec)),
        [this]() { this->poll_config_http(); });

    RCLCPP_INFO(this->get_logger(), "EventAlarm initialized. model=%s vin=%s proto=%s",
                model_str_.c_str(), vin_.c_str(), protocol_version_.c_str());

    // subscribe trigger topic
    subscribe_trigger_ = this->create_subscription<std_msgs::msg::String>(
        trigger_topic_, 10,
        [this](const std_msgs::msg::String::SharedPtr msg){ this->on_trigger(msg->data); });

    // start cache maintenance timer
    cache_timer_ = this->create_wall_timer(
        std::chrono::seconds(std::max(10, cache_maint_interval_sec_)),
        [this](){ this->maintain_cache(); });

    // Optional: fire a test alarm at startup if enabled
    if (this->get_parameter("test_alarm.enabled").as_bool()) {
      this->create_wall_timer(std::chrono::seconds(2), [this]() {
        this->emit_test_alarm();
      });
    }
  }

private:
  // Parameters
  std::string vin_;
  std::string model_str_;
  std::string protocol_version_ = "1"; // spec examples use 1
  std::string api_base_url_;
  std::string session_key_;
  int64_t config_version_ = 0; // local known config version

  // MQTT
  std::shared_ptr<mqtt_module::MqttConnectionManager> mqtt_;

  // OSS uploader
  std::shared_ptr<postback::OSSUploader> oss_;

  // Timers
  rclcpp::TimerBase::SharedPtr config_poll_timer_;
  rclcpp::TimerBase::SharedPtr cache_timer_;

  // Trigger subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscribe_trigger_;
  std::string trigger_topic_ = "event_trigger"; // expect JSON payload

  // Cache
  std::string cache_base_dir_ = "/tmp/vehicle_cache";
  int cache_retention_minutes_ = 10;
  int cache_maint_interval_sec_ = 30;
  std::vector<std::string> cache_scan_dirs_;

  // Ack tracking
  std::mutex ack_mutex_;
  std::condition_variable ack_cv_;
  std::set<std::string> acked_event_ids_;

  // Retry parameters
  int publish_retry_max_ = 3;
  int publish_retry_interval_ms_ = 5000;
  int ack_timeout_ms_ = 4000;
  int upload_retry_max_ = 3;
  int upload_retry_interval_ms_ = 5000;
  bool use_protobuf_ = true;

  void declare_parameters() {
    this->declare_parameter<std::string>("vin", "X001");
    this->declare_parameter<std::string>("model_str", "x5");
    this->declare_parameter<std::string>("protocol.version", "1");
    this->declare_parameter<std::string>("api_base_url", "");
    this->declare_parameter<std::string>("session_key", "");
    this->declare_parameter<int>("config.poll_interval_seconds", 30);
    // MQTT minimal params
    this->declare_parameter<std::string>("mqtt.broker_url", "tcp://localhost:1883");
    this->declare_parameter<std::string>("mqtt.client_id", "event_alarm");
    this->declare_parameter<std::string>("mqtt.username", "");
    this->declare_parameter<std::string>("mqtt.password", "");
    this->declare_parameter<std::string>("mqtt.ca_cert", "");
    this->declare_parameter<std::string>("mqtt.cipher", "");
    this->declare_parameter<bool>("mqtt.insecure_skip_verify", false);
    // OSS base
    this->declare_parameter<std::string>("oss.endpoint", "oss-cn-beijing.aliyuncs.com");
    this->declare_parameter<std::string>("oss.bucket", "test-oss-log-2025");
    this->declare_parameter<std::string>("oss.access_key", "");
    this->declare_parameter<std::string>("oss.secret_key", "");
    this->declare_parameter<bool>("oss.use_env", true);
    this->declare_parameter<std::string>("oss.config_file", "");
    // Test alarm
    this->declare_parameter<bool>("test_alarm.enabled", false);
    this->declare_parameter<std::string>("test_alarm.code", "ALM_TEST");
    this->declare_parameter<int>("test_alarm.level", 3);
    this->declare_parameter<std::string>("test_alarm.info", "demo");

    // Cache and workflow params
    this->declare_parameter<std::string>("cache.base_dir", "/tmp/vehicle_cache");
    this->declare_parameter<int>("cache.retention_minutes", 10);
    this->declare_parameter<int>("cache.maint_interval_sec", 30);
    this->declare_parameter<std::string>("cache.scan_dirs", ""); // comma-separated list
    this->declare_parameter<std::string>("trigger.topic", "event_trigger");
    this->declare_parameter<int>("retry.publish.max", 3);
    this->declare_parameter<int>("retry.publish.interval_ms", 5000);
    this->declare_parameter<int>("retry.ack.timeout_ms", 4000);
    this->declare_parameter<int>("retry.upload.max", 3);
    this->declare_parameter<int>("retry.upload.interval_ms", 5000);
    this->declare_parameter<bool>("protocol.use_protobuf", true);
  }

  void load_parameters() {
    this->get_parameter("vin", vin_);
    this->get_parameter("model_str", model_str_);
    this->get_parameter("protocol.version", protocol_version_);
    this->get_parameter("api_base_url", api_base_url_);
    this->get_parameter("session_key", session_key_);

    this->get_parameter("cache.base_dir", cache_base_dir_);
    this->get_parameter("cache.retention_minutes", cache_retention_minutes_);
    this->get_parameter("cache.maint_interval_sec", cache_maint_interval_sec_);
    this->get_parameter("trigger.topic", trigger_topic_);
    publish_retry_max_ = this->get_parameter("retry.publish.max").as_int();
    publish_retry_interval_ms_ = this->get_parameter("retry.publish.interval_ms").as_int();
    ack_timeout_ms_ = this->get_parameter("retry.ack.timeout_ms").as_int();
    upload_retry_max_ = this->get_parameter("retry.upload.max").as_int();
    upload_retry_interval_ms_ = this->get_parameter("retry.upload.interval_ms").as_int();
    use_protobuf_ = this->get_parameter("protocol.use_protobuf").as_bool();

    std::string scan_dirs;
    this->get_parameter("cache.scan_dirs", scan_dirs);
    cache_scan_dirs_.clear();
    if (!scan_dirs.empty()) {
      std::stringstream ss(scan_dirs);
      std::string item;
      while (std::getline(ss, item, ',')) {
        if (!item.empty()) cache_scan_dirs_.push_back(item);
      }
    }
    if (cache_scan_dirs_.empty()) {
      cache_scan_dirs_.push_back(cache_base_dir_);
    }
    std::error_code ec; std::filesystem::create_directories(cache_base_dir_, ec);
  }

  void init_mqtt() {
    mqtt_module::MqttConnConfig cfg;
    cfg.broker = this->get_parameter("mqtt.broker_url").as_string();
    cfg.client_id = this->get_parameter("mqtt.client_id").as_string();
    cfg.username = this->get_parameter("mqtt.username").as_string();
    cfg.password = this->get_parameter("mqtt.password").as_string();
    cfg.ca_cert_path = this->get_parameter("mqtt.ca_cert").as_string();
    cfg.tls_cipher = this->get_parameter("mqtt.cipher").as_string();
    cfg.insecure_skip_verify = this->get_parameter("mqtt.insecure_skip_verify").as_bool();
    mqtt_ = std::make_shared<mqtt_module::MqttConnectionManager>(cfg, this->get_logger());
    mqtt_->set_message_callback([this](const std::string& topic, const std::string& payload){
      this->on_mqtt_message(topic, payload);
    });
    mqtt_module::TokenResult empty;
    if (!mqtt_->connect(empty)) {
      RCLCPP_ERROR(this->get_logger(), "MQTT连接失败: %s", cfg.broker.c_str());
    }

    // OSS uploader
    std::string ep = this->get_parameter("oss.endpoint").as_string();
    std::string bucket = this->get_parameter("oss.bucket").as_string();
    std::string ak = this->get_parameter("oss.access_key").as_string();
    std::string sk = this->get_parameter("oss.secret_key").as_string();
    bool use_env = this->get_parameter("oss.use_env").as_bool();
    std::string cfg_file = this->get_parameter("oss.config_file").as_string();

    if (use_env) {
      std::string env_ak, env_sk;
      if (get_aliyun_credentials(env_ak, env_sk)) {
        ak = env_ak; sk = env_sk;
        RCLCPP_INFO(this->get_logger(), "已从环境变量读取OSS密钥");
      } else {
        RCLCPP_WARN(this->get_logger(), "未检测到OSS环境密钥，回退使用参数值");
      }
    } else if (!cfg_file.empty()) {
      std::map<std::string,std::string> cfg;
      if (read_simple_config(cfg_file, cfg)) {
        // Accept both Aliyun env-like keys and generic keys
        if (cfg.count("ALIBABA_CLOUD_ACCESS_KEY_ID")) ak = cfg["ALIBABA_CLOUD_ACCESS_KEY_ID"];
        if (cfg.count("ALIBABA_CLOUD_ACCESS_KEY_SECRET")) sk = cfg["ALIBABA_CLOUD_ACCESS_KEY_SECRET"];
        if (cfg.count("access_key")) ak = cfg["access_key"];
        if (cfg.count("secret_key")) sk = cfg["secret_key"];
        RCLCPP_INFO(this->get_logger(), "已从配置文件读取OSS密钥: %s", cfg_file.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "读取OSS配置文件失败: %s", cfg_file.c_str());
      }
    }

    oss_ = std::make_shared<postback::OSSUploader>(ep, bucket, ak, sk);
  }

  std::string build_config_up_topic() const {
    return "vehicle/config/" + model_str_ + "/" + protocol_version_ + "/up";
  }
  std::string build_config_ack_topic() const {
    return "vehicle/config/" + model_str_ + "/" + protocol_version_ + "/ack";
  }
  std::string build_event_push_topic() const {
    return "vehicle/event/" + model_str_ + "/" + vin_ + "/" + protocol_version_ + "/push";
  }
  std::string build_event_ack_topic() const {
    return "vehicle/event/" + model_str_ + "/" + vin_ + "/" + protocol_version_ + "/ack";
  }

  void subscribe_config_topics() {
    if (!mqtt_ || !mqtt_->is_connected()) return;
    mqtt_->subscribe(build_config_up_topic(), 1);
    mqtt_->subscribe(build_event_ack_topic(), 1);
  }

  void ack_config_received(const std::string& config_id, int64_t ts, const std::string& version) {
    std::string data;
    serialize_config_ack(config_id, ts, version, data);
    if (mqtt_ && mqtt_->is_connected()) {
      mqtt_->publish(build_config_ack_topic(), data, 1);
    }
  }

  void on_mqtt_message(const std::string& topic, const std::string& payload) {
    if (topic == build_config_up_topic()) {
      // Extra logging for received config topic
      RCLCPP_INFO(this->get_logger(), "收到配置主题: %s", topic.c_str());
      const size_t max_preview = 1024;
      std::string preview = payload.substr(0, std::min(max_preview, payload.size()));
      RCLCPP_INFO(this->get_logger(), "配置内容预览(前%zu字节): %s", preview.size(), preview.c_str());

      std::string cfg_id = "auto";
      std::string version = "";
      int64_t ts_ms = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      bool parsed = false;
      if (use_protobuf_) {
        vehicle::config::EventConfigPackage pkg;
        if (pkg.ParseFromString(payload)) {
          cfg_id = pkg.config_id();
          version = pkg.version();
          ts_ms = pkg.timestamp();
          parsed = true;
        }
      }
      if (!parsed) {
        try {
          auto j = json::parse(payload);
          cfg_id = j.value("config_id", cfg_id);
          version = j.value("version", version);
          ts_ms = j.value("timestamp", ts_ms);
        } catch (...) {}
      }

      config_version_ = ts_ms / 1000; // keep monotonic for demo
      ack_config_received(cfg_id, ts_ms, version);
      RCLCPP_INFO(this->get_logger(), "收到配置更新 (len=%zu) config_id=%s version=%s 并已ACK", payload.size(), cfg_id.c_str(), version.c_str());
    } else if (topic == build_event_ack_topic()) {
      RCLCPP_INFO(this->get_logger(), "收到事件ACK (len=%zu)", payload.size());
      auto evt = parse_event_ack_event_id(payload);
      if (evt && !evt->empty()) {
        {
          std::lock_guard<std::mutex> lk(ack_mutex_);
          acked_event_ids_.insert(*evt);
        }
        ack_cv_.notify_all();
      }
    }
  }

  void poll_config_http() {
    if (api_base_url_.empty()) return;
    json body;
    body["sn"] = vin_;
    body["model"] = model_str_;
    body["configVersion"] = std::to_string(config_version_);
    auto resp = cpr::Post(cpr::Url{api_base_url_ + "/api/device/v1/config/getUp"},
                          cpr::Body{body.dump()},
                          cpr::Header{{"Content-Type","application/json"}});
    if (resp.error || resp.status_code != 200) {
      RCLCPP_WARN(this->get_logger(), "配置HTTP补获失败: %s code=%ld", resp.error.message.c_str(), (long)resp.status_code);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "配置HTTP补获成功 (len=%zu)", resp.text.size());
    // naive parse for demo: assume JSON with fields configId, version, timestamp
    try {
      auto j = json::parse(resp.text);
      std::string cfg_id = j.value("configId", "http");
      std::string version = j.value("version", "");
      int64_t ts = j.value("timestamp", (int64_t)0);
      ack_config_received(cfg_id, ts, version);
    } catch (...) {
      // ignore
    }
  }

  void emit_test_alarm() {
    // Build VehicleEventInfo-like JSON for demo
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    std::string event_id = gen_uuid_like();
    std::string code = this->get_parameter("test_alarm.code").as_string();
    int level = this->get_parameter("test_alarm.level").as_int();
    std::string info = this->get_parameter("test_alarm.info").as_string();

    json telemetry;
    telemetry["speed"] = 12.3;
    telemetry["brake"] = 1;
    telemetry["latitude"] = 39.9;
    telemetry["longitude"] = 116.4;

    json ev;
    ev["sn"] = vin_;
    ev["device_timestamp"] = now_ms;
    ev["model"] = model_str_;
    ev["version"] = "1.0.0";
    ev["config_version"] = config_version_;
    ev["event_id"] = event_id;
    ev["event_type"] = 1; // 报警
    ev["duration_type"] = 0; // 瞬时
    ev["status"] = 0; // 开始
    ev["up_index"] = 0;

    json alarm;
    alarm["code"] = code;
    alarm["code_name"] = code;
    alarm["desc"] = info;
    alarm["level"] = level;
    alarm["first_ms"] = now_ms;
    alarm["last_ms"] = now_ms;
    ev["alarm_info"] = alarm;
    ev["telemetry"] = telemetry;

    auto data = ev.dump();
    // Launch full workflow for this event
    launch_event_workflow(event_id, code, ev);
  }

  static std::string build_oss_event_log_path(const std::string& model, const std::string& event_id, const std::string& code, bool pre) {
    auto t = std::time(nullptr);
    struct tm tm = *gmtime(&t);
    char date_buf[32] = {0};
    strftime(date_buf, sizeof(date_buf), "%Y%m%d", &tm);
    std::string date_str = date_buf;
    std::stringstream ss;
    ss << "events/logs/" << model << "/" << date_str << "/" << (pre?"0":"1") << "_" << event_id << "_" << code << ".zip";
    return ss.str();
  }

  static std::string build_oss_event_video_path(const std::string& model, const std::string& event_id, int seq, const std::string& type, bool pre) {
    auto t = std::time(nullptr);
    struct tm tm = *gmtime(&t);
    char date_buf[32] = {0};
    strftime(date_buf, sizeof(date_buf), "%Y%m%d", &tm);
    std::string date_str = date_buf;
    std::stringstream ss;
    ss << "events/videos/" << model << "/" << date_str << "/" << (pre?"0":"1") << "_" << event_id << "_" << seq << "/" << type << ".mp4";
    return ss.str();
  }

  static std::string gen_uuid_like() {
    std::mt19937_64 rng{std::random_device{}()};
    std::uniform_int_distribution<uint64_t> d;
    std::stringstream ss; ss << std::hex << d(rng) << d(rng);
    return ss.str();
  }

  // ====== Helpers to read env/config for OSS credentials (mirroring oss_log_uploader_node) ======
  static inline std::string trim(const std::string &s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
  }

  static bool read_simple_config(const std::string &path, std::map<std::string,std::string> &out) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
      return false;
    }
    std::string line;
    while (std::getline(ifs, line)) {
      std::string s = trim(line);
      if (s.empty() || s[0] == '#') continue;
      auto pos = s.find('=');
      if (pos == std::string::npos) continue;
      std::string key = trim(s.substr(0,pos));
      std::string val = trim(s.substr(pos+1));
      if (!key.empty()) out[key] = val;
    }
    return true;
  }

  bool get_aliyun_credentials(std::string& access_key, std::string& secret_key) {
    const char* ak_env = std::getenv("ALIBABA_CLOUD_ACCESS_KEY_ID");
    const char* sk_env = std::getenv("ALIBABA_CLOUD_ACCESS_KEY_SECRET");
    if (ak_env == nullptr || sk_env == nullptr) {
      return false;
    }
    access_key = std::string(ak_env);
    secret_key = std::string(sk_env);
    return !access_key.empty() && !secret_key.empty();
  }

  // ========== Cache maintenance ==========
  void maintain_cache() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto cutoff = now - minutes(cache_retention_minutes_);
    for (const auto& dir : cache_scan_dirs_) {
      std::error_code ec;
      if (!std::filesystem::exists(dir)) continue;
      for (auto& p : std::filesystem::recursive_directory_iterator(dir, ec)) {
        if (ec) break;
        if (!p.is_regular_file()) continue;
        auto ftime = std::filesystem::last_write_time(p, ec);
        if (ec) continue;
        // compare by converting to system_clock time point
        auto sctp = time_point_cast<system_clock::duration>(ftime - decltype(ftime)::clock::now() + system_clock::now());
        if (sctp < cutoff) {
          std::filesystem::remove(p.path(), ec);
        }
      }
    }
  }

  std::vector<std::string> collect_files_for_window_minutes(int minutes_window) {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto since = now - minutes(minutes_window);
    std::vector<std::string> files;
    std::error_code ec;
    for (const auto& dir : cache_scan_dirs_) {
      if (!std::filesystem::exists(dir)) continue;
      for (auto& p : std::filesystem::recursive_directory_iterator(dir, ec)) {
        if (ec) break;
        if (!p.is_regular_file()) continue;
        auto ftime = std::filesystem::last_write_time(p, ec);
        if (ec) continue;
        auto sctp = time_point_cast<system_clock::duration>(ftime - decltype(ftime)::clock::now() + system_clock::now());
        if (sctp >= since) {
          files.push_back(p.path().string());
        }
      }
    }
    return files;
  }

  // ========== Trigger handling ==========
  void on_trigger(const std::string& payload) {
    // expected JSON: {"code":"ALM_X","level":3,"info":"..."}
    std::string code = "ALM_TRIGGER";
    int level = 3;
    std::string info = "";
    try {
      auto j = json::parse(payload);
      code = j.value("code", code);
      level = j.value("level", level);
      info = j.value("info", info);
    } catch (...) {}

    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    std::string event_id = gen_uuid_like();

    json ev;
    ev["sn"] = vin_;
    ev["device_timestamp"] = now_ms;
    ev["model"] = model_str_;
    ev["version"] = "1.0.0";
    ev["config_version"] = config_version_;
    ev["event_id"] = event_id;
    ev["event_type"] = 1;
    ev["duration_type"] = 0;
    ev["status"] = 0;
    ev["up_index"] = 0;
    json alarm; alarm["code"]=code; alarm["code_name"]=code; alarm["desc"]=info; alarm["level"]=level;
    alarm["first_ms"]=now_ms; alarm["last_ms"]=now_ms; ev["alarm_info"]=alarm;
    json telemetry; telemetry["speed"]=0; telemetry["brake"]=0; ev["telemetry"]=telemetry;

    launch_event_workflow(event_id, code, ev);
  }

  void launch_event_workflow(const std::string& event_id,
                             const std::string& code,
                             json event_json) {
    std::string log_oss_path = build_oss_event_log_path(model_str_, event_id, code, /*pre=*/true);
    std::string video_oss_path = build_oss_event_video_path(model_str_, event_id, /*seq=*/0, code, /*pre=*/true);
    event_json["log_file"] = json::array({log_oss_path});
    event_json["video_file"] = json::array({video_oss_path});
    event_json["source"] = "event_alarm_node";

    std::thread([this, event_id, code, event_json, log_oss_path]() mutable {
      auto pub_future = std::async(std::launch::async, [this, event_json, event_id]() {
        std::string payload; bool used_proto = false;
        serialize_vehicle_event(event_json, payload, used_proto);
        return this->publish_with_ack(payload, event_id, used_proto);
      });

      std::string pkg_path = this->package_cached_files(event_id, code);

      if (!pkg_path.empty()) {
        bool uploaded = upload_with_retry(pkg_path, log_oss_path);
        if (uploaded) {
          RCLCPP_INFO(this->get_logger(), "事件[%s] 日志上传成功 (OSS)", event_id.c_str());
          std::error_code ec; std::filesystem::remove(pkg_path, ec);
        } else {
          RCLCPP_WARN(this->get_logger(), "事件[%s] 日志上传失败 (OSS) 已重试上限", event_id.c_str());
        }
      }

      (void)pub_future.get();
    }).detach();
  }

  bool publish_with_ack(const std::string& payload, const std::string& event_id, bool payload_is_proto) {
    // retry publish and wait ack
    // Precompute topic and extract code for logging
    const std::string topic = build_event_push_topic();
    std::string alarm_code;
    int alarm_level = -1;
    try {
      auto j = json::parse(payload);
      if (j.contains("alarm_info")) {
        const auto &a = j["alarm_info"];
        alarm_code = a.value("code", std::string{});
        alarm_level = a.value("level", -1);
      }
    } catch (...) {
      // ignore parse errors, logging will omit extras
    }
    for (int attempt = 1; attempt <= std::max(1, publish_retry_max_); ++attempt) {
      if (mqtt_ && mqtt_->is_connected()) {
        // Log detailed publish info
        if (!alarm_code.empty()) {
          RCLCPP_INFO(this->get_logger(), "上报事件: topic=%s event_id=%s code=%s level=%d 尝试=%d/%d 长度=%zu fmt=%s",
                      topic.c_str(), event_id.c_str(), alarm_code.c_str(), alarm_level,
                      attempt, publish_retry_max_, payload.size(), payload_is_proto?"pb":"json");
        } else {
          RCLCPP_INFO(this->get_logger(), "上报事件: topic=%s event_id=%s 尝试=%d/%d 长度=%zu fmt=%s",
                      topic.c_str(), event_id.c_str(), attempt, publish_retry_max_, payload.size(), payload_is_proto?"pb":"json");
        }
        mqtt_->publish(topic, payload, 1);
      }
      std::unique_lock<std::mutex> lk(ack_mutex_);
      bool got = ack_cv_.wait_for(lk, std::chrono::milliseconds(std::max(100, ack_timeout_ms_)), [this,&event_id](){
        return acked_event_ids_.count(event_id) > 0; });
      if (got) {
        RCLCPP_INFO(this->get_logger(), "事件[%s] 收到ACK", event_id.c_str());
        return true;
      }
      if (attempt < publish_retry_max_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(std::max(100, publish_retry_interval_ms_)));
      }
    }
    RCLCPP_WARN(this->get_logger(), "事件[%s] 发布或ACK等待失败", event_id.c_str());
    return false;
  }

  std::string package_cached_files(const std::string& event_id, const std::string& code) {
    // gather files from last N minutes
    auto files = collect_files_for_window_minutes(cache_retention_minutes_);
    std::error_code ec; std::filesystem::create_directories("/tmp/events", ec);
    std::string out_zip = std::string("/tmp/events/") + event_id + "_" + code + ".zip";
    if (files.empty()) {
      // create an empty placeholder file
      std::ofstream ofs(out_zip); ofs << ""; ofs.close();
      return out_zip;
    }
    // try zip, fallback tar.gz
    std::stringstream cmd;
    cmd << "/usr/bin/zip -j -q " << out_zip << "";
    for (auto &f : files) cmd << " " << f;
    int rc = std::system(cmd.str().c_str());
    if (rc != 0) {
      std::string out_tgz = std::string("/tmp/events/") + event_id + "_" + code + ".tar.gz";
      std::stringstream tcmd; tcmd << "/bin/tar -czf " << out_tgz;
      for (auto &f : files) tcmd << " -C / " << f; // may not work for arbitrary paths
      int rc2 = std::system(tcmd.str().c_str());
      if (rc2 == 0) return out_tgz;
    }
    return out_zip;
  }

  bool upload_with_retry(const std::string& file_path, const std::string& oss_path) {
    if (!oss_) return false;
    for (int attempt = 1; attempt <= std::max(1, upload_retry_max_); ++attempt) {
      bool ok = oss_->upload_file(file_path, oss_path);
      if (ok) return true;
      if (attempt < upload_retry_max_) std::this_thread::sleep_for(std::chrono::milliseconds(std::max(100, upload_retry_interval_ms_)));
    }
    return false;
  }

  bool serialize_vehicle_event(const json& event_json, std::string& out, bool& used_proto) {
    used_proto = false;
    vehicle::event::VehicleEventInfo msg;
    if (fill_vehicle_event_from_json(event_json, msg)) {
      if (use_protobuf_) {
        if (msg.SerializeToString(&out)) { used_proto = true; return true; }
        RCLCPP_WARN(this->get_logger(), "事件PB序列化失败，回退JSON");
      }
      std::string json_out;
      auto status = google::protobuf::util::MessageToJsonString(msg, &json_out);
      if (status.ok()) { out = json_out; return true; }
    }
    out = event_json.dump();
    return true;
  }

  bool fill_vehicle_event_from_json(const json& j, vehicle::event::VehicleEventInfo& msg) {
    try {
      msg.set_sn(j.value("sn", ""));
      msg.set_device_timestamp(j.value<int64_t>("device_timestamp", 0));
      msg.set_model(j.value("model", ""));
      msg.set_version(j.value("version", ""));
      msg.set_config_version(j.value<int64_t>("config_version", 0));
      msg.set_event_id(j.value("event_id", ""));
      msg.set_event_type(j.value<int>("event_type", 0));
      msg.set_duration_type(j.value<int>("duration_type", 0));
      msg.set_status(j.value<int>("status", 0));
      msg.set_up_index(j.value<int64_t>("up_index", 0));

      if (j.contains("alarm_info")) {
        auto* a = msg.mutable_alarm_info();
        const auto &aj = j.at("alarm_info");
        a->set_code(aj.value("code", ""));
        a->set_code_name(aj.value("code_name", ""));
        a->set_desc(aj.value("desc", ""));
        a->set_level(aj.value<int>("level", 0));
        a->set_info(aj.value("info", ""));
        a->set_first_ms(aj.value<int64_t>("first_ms", 0));
        a->set_last_ms(aj.value<int64_t>("last_ms", 0));
      }
      if (j.contains("telemetry")) {
        auto* t = msg.mutable_telemetry();
        const auto &tj = j.at("telemetry");
        t->set_speed(tj.value<double>("speed", 0));
        t->set_brake(tj.value<int>("brake", 0));
        t->set_latitude(tj.value<double>("latitude", 0));
        t->set_longitude(tj.value<double>("longitude", 0));
      }
      if (j.contains("source")) {
        auto* s = msg.mutable_source();
        s->set_device_type(j.value("source", ""));
      }
      if (j.contains("log_file")) {
        for (const auto& lf : j.at("log_file")) {
          msg.add_log_file(lf.get<std::string>());
        }
      }
      if (j.contains("video_file")) {
        for (const auto& vf : j.at("video_file")) {
          msg.add_video_file(vf.get<std::string>());
        }
      }
      if (j.contains("extras")) {
        msg.set_extras(j.value("extras", ""));
      }
      return true;
    } catch (...) {
      return false;
    }
  }

  bool serialize_config_ack(const std::string& config_id, int64_t ts, const std::string& version, std::string& out) {
    if (use_protobuf_) {
      vehicle::config::EventConfigAck ack;
      ack.set_config_id(config_id);
      ack.set_timestamp(ts);
      ack.set_version(version);
      ack.set_sn(vin_);
      if (ack.SerializeToString(&out)) return true;
      RCLCPP_WARN(this->get_logger(), "配置ACK PB序列化失败，回退JSON");
    }
    json ack;
    ack["config_id"] = config_id;
    ack["timestamp"] = ts;
    ack["version"] = version;
    ack["sn"] = vin_;
    out = ack.dump();
    return true;
  }

  std::optional<std::string> parse_event_ack_event_id(const std::string& payload) {
    vehicle::event::VehicleEventAck ack;
    if (ack.ParseFromString(payload)) {
      return ack.event_id();
    }
    try {
      auto j = json::parse(payload);
      return j.value("event_id", std::string{});
    } catch (...) {
      return std::nullopt;
    }
  }

  static std::string ext_of(const std::string& path) {
    auto pos = path.find_last_of('.');
    if (pos == std::string::npos) return "log";
    return path.substr(pos+1);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EventAlarmNode>());
  rclcpp::shutdown();
  return 0;
}
