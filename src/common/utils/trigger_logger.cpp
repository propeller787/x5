#include "trigger_logger.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cstdio>

// WebSocket客户端实现 (复用cloud_service中的实现)
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

namespace utils {
static std::string escapeJsonString(const std::string& str);
// WebSocket客户端实现
class TriggerLogger::WebSocketClient {
public:
    explicit WebSocketClient(const std::string& url)
        : url_(url), ioc_(), resolver_(ioc_), ws_(ioc_), 
          work_guard_(net::make_work_guard(ioc_)), 
          reconnect_timer_(ioc_), run_thread_(nullptr),
          should_stop_(false) {
        parseUrl();
        run_thread_ = std::make_unique<std::thread>([this] { this->run(); });
    }

    ~WebSocketClient() {
        should_stop_.store(true);
        net::post(ioc_, [this] {
            beast::error_code ec;
            ws_.close(websocket::close_code::normal, ec);
        });
        if (run_thread_ && run_thread_->joinable()) {
            run_thread_->join();
        }
    }

    void send(const std::string& text) {
        net::post(ioc_, [this, text] {
            outbound_.push(text);
            if (ws_.is_open()) {
                this->doWrite();
            }
        });
    }

private:
    void parseUrl() {
        const std::string prefix = "ws://";
        if (url_.rfind(prefix, 0) == 0) {
            host_port_path_ = url_.substr(prefix.size());
        } else {
            host_port_path_ = url_;
        }
        auto slash_pos = host_port_path_.find('/');
        if (slash_pos == std::string::npos) {
            host_port_ = host_port_path_;
            target_ = "/";
        } else {
            host_port_ = host_port_path_.substr(0, slash_pos);
            target_ = host_port_path_.substr(slash_pos);
        }
        auto colon_pos = host_port_.find(':');
        host_ = colon_pos == std::string::npos ? host_port_ : host_port_.substr(0, colon_pos);
        port_ = colon_pos == std::string::npos ? "80" : host_port_.substr(colon_pos + 1);
    }

    void doWrite() {
        if (outbound_.empty())
            return;
        auto msg = outbound_.front();
        outbound_.pop();
        ws_.async_write(net::buffer(msg), [this](beast::error_code ec, std::size_t) {
            if (ec) {
                this->reconnectSoon();
                return;
            }
            if (!outbound_.empty())
                doWrite();
        });
    }

    void reconnectSoon() {
        reconnect_timer_.expires_after(std::chrono::seconds(2));
        reconnect_timer_.async_wait([this](beast::error_code) { this->connect(); });
    }

    void connect() {
        resolver_.async_resolve(host_, port_, [this](beast::error_code ec, tcp::resolver::results_type results) {
            if (ec) {
                reconnectSoon();
                return;
            }
            net::async_connect(ws_.next_layer(), results, [this](beast::error_code ec, const tcp::endpoint&) {
                if (ec) {
                    reconnectSoon();
                    return;
                }
                ws_.async_handshake(host_, target_, [this](beast::error_code ec) {
                    if (ec) {
                        reconnectSoon();
                        return;
                    }
                    if (!outbound_.empty())
                        doWrite();
                });
            });
        });
    }

    void run() {
        connect();
        while (!should_stop_.load()) {
            ioc_.run_one();
        }
    }

    std::string url_;
    std::string host_port_path_;
    std::string host_port_;
    std::string host_;
    std::string port_;
    std::string target_;
    net::io_context ioc_;
    tcp::resolver resolver_;
    websocket::stream<tcp::socket> ws_;
    net::executor_work_guard<net::io_context::executor_type> work_guard_;
    net::steady_timer reconnect_timer_;
    std::unique_ptr<std::thread> run_thread_;
    std::atomic<bool> should_stop_;
    std::queue<std::string> outbound_;
};

// TriggerLogger 实现

TriggerLogger& TriggerLogger::getInstance() {
    static TriggerLogger instance;
    return instance;
}

TriggerLogger::~TriggerLogger() {
    shutdown();
}

void TriggerLogger::initialize(const std::string& websocket_url,
                              const std::string& vehicle_id,
                              const std::string& version) {
    if (initialized_.load()) {
        return;
    }
    
    websocket_url_ = websocket_url;
    vehicle_id_ = vehicle_id;
    version_ = version;
    
    // 创建WebSocket客户端
    ws_client_ = std::make_unique<WebSocketClient>(websocket_url_);
    
    // 启动上传线程
    should_stop_.store(false);
    upload_thread_ = std::thread(&TriggerLogger::uploadThread, this);
    
    initialized_.store(true);
}

void TriggerLogger::log(const std::string& event_name,
                       const std::map<std::string, std::string>& data,
                       const std::string& file_path,
                       int line_number,
                       const std::string& function_name,
                       const std::string& node_name) {
    if (!initialized_.load()) {
        // 如果未初始化，静默失败或记录到本地日志
        return;
    }
    
    TriggerData trigger_data;
    trigger_data.from = "vlago";
    trigger_data.type = "trigger";
    trigger_data.event_name = event_name;
    trigger_data.timestamp = getCurrentTimestamp();
    trigger_data.timestamp_str = formatTimestamp(trigger_data.timestamp);
    trigger_data.node_name = node_name;
    trigger_data.file_path = file_path;
    trigger_data.line_number = line_number;
    trigger_data.function_name = function_name;
    trigger_data.data = data;
    trigger_data.vehicle_id = vehicle_id_;
    trigger_data.version = version_;
    
    // 添加到队列
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (data_queue_.size() >= MAX_QUEUE_SIZE) {
            // 队列满时，丢弃最旧的数据
            data_queue_.pop();
        }
        data_queue_.push(trigger_data);
    }
}

void TriggerLogger::setVehicleId(const std::string& vehicle_id) {
    vehicle_id_ = vehicle_id;
}

void TriggerLogger::setVersion(const std::string& version) {
    version_ = version;
}

void TriggerLogger::shutdown() {
    if (!initialized_.load()) {
        return;
    }
    
    should_stop_.store(true);
    
    if (upload_thread_.joinable()) {
        upload_thread_.join();
    }
    
    ws_client_.reset();
    initialized_.store(false);
}

// JSON字符串转义辅助函数
static std::string escapeJsonString(const std::string& str) {
    std::string escaped;
    escaped.reserve(str.length());
    for (char c : str) {
        switch (c) {
            case '"':  escaped += "\\\""; break;
            case '\\': escaped += "\\\\"; break;
            case '\b': escaped += "\\b"; break;
            case '\f': escaped += "\\f"; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default:
                if (c >= 0 && c < 0x20) {
                    // 控制字符转义为Unicode
                    char buf[7];
                    snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(c));
                    escaped += buf;
                } else {
                    escaped += c;
                }
                break;
        }
    }
    return escaped;
}

std::string TriggerLogger::toJson(const TriggerData& trigger_data) {
    std::ostringstream oss;
    oss << "{";
    
    // 基础字段
    oss << "\"from\":\"" << escapeJsonString(trigger_data.from) << "\"";
    oss << ",\"type\":\"" << escapeJsonString(trigger_data.type) << "\"";
    oss << ",\"event\":\"" << escapeJsonString(trigger_data.event_name) << "\"";
    oss << ",\"timestamp\":" << trigger_data.timestamp;
    oss << ",\"timestamp_str\":\"" << escapeJsonString(trigger_data.timestamp_str) << "\"";
    
    // 位置信息
    if (!trigger_data.node_name.empty()) {
        oss << ",\"node_name\":\"" << escapeJsonString(trigger_data.node_name) << "\"";
    }
    if (!trigger_data.file_path.empty()) {
        oss << ",\"file\":\"" << escapeJsonString(trigger_data.file_path) << "\"";
    }
    if (trigger_data.line_number > 0) {
        oss << ",\"line\":" << trigger_data.line_number;
    }
    if (!trigger_data.function_name.empty()) {
        oss << ",\"function\":\"" << escapeJsonString(trigger_data.function_name) << "\"";
    }
    
    // 业务数据
    if (!trigger_data.data.empty()) {
        oss << ",\"data\":{";
        bool first = true;
        for (const auto& [key, value] : trigger_data.data) {
            if (!first) oss << ",";
            first = false;
            // 转义JSON特殊字符
            std::string escaped_key = escapeJsonString(key);
            std::string escaped_value = escapeJsonString(value);
            oss << "\"" << escaped_key << "\":\"" << escaped_value << "\"";
        }
        oss << "}";
    }
    
    // 系统信息
    if (!trigger_data.vehicle_id.empty()) {
        oss << ",\"vehicle_id\":\"" << escapeJsonString(trigger_data.vehicle_id) << "\"";
    }
    if (!trigger_data.version.empty()) {
        oss << ",\"version\":\"" << escapeJsonString(trigger_data.version) << "\"";
    }
    
    oss << "}";
    return oss.str();
}

void TriggerLogger::uploadThread() {
    while (!should_stop_.load()) {
        TriggerData trigger_data;
        bool has_data = false;
        
        // 从队列中取出数据
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (!data_queue_.empty()) {
                trigger_data = data_queue_.front();
                data_queue_.pop();
                has_data = true;
            }
        }
        
        if (has_data && ws_client_) {
            // 转换为JSON并上传
            std::string json = toJson(trigger_data);
            ws_client_->send(json);
        } else {
            // 没有数据时，短暂休眠
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

int64_t TriggerLogger::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

std::string TriggerLogger::formatTimestamp(int64_t timestamp_ms) {
    std::time_t tt = timestamp_ms / 1000;
    std::tm tm_buf;
#if defined(_WIN32)
    localtime_s(&tm_buf, &tt);
#else
    localtime_r(&tt, &tm_buf);
#endif
    std::ostringstream ss;
    ss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

} // namespace utils

