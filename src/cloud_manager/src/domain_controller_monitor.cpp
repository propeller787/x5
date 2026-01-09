#include "cloud_manager/domain_controller_monitor.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>

namespace cloud_manager {

static inline std::string trim(const std::string &s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

static inline std::string to_lower(std::string v) {
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return v;
}

static inline float clamp01(float x) { return std::min(std::max(x, 0.0f), 1.0f); }
static inline float clamp100(float x) { return std::min(std::max(x, 0.0f), 100.0f); }

void DomainControllerMonitor::refresh() {
    std::lock_guard<std::mutex> lk(mu_);
    std::ifstream ifs(status_file_);
    if (!ifs.is_open()) return;
    std::string line;
    while (std::getline(ifs, line)) {
        auto pos = line.find('=');
        if (pos == std::string::npos) continue;
        std::string key = trim(line.substr(0, pos));
        std::string val = trim(line.substr(pos + 1));
        if (key.empty()) continue;
        if (key == "cpu") {
            try {
                std::string v = trim(val);
                bool has_percent = !v.empty() && v.back() == '%';
                if (has_percent) v.pop_back();
                float parsed = std::stof(v);
                // 支持 0..1 或 0..100 两种表示法
                if (!has_percent && parsed > 0.0f && parsed <= 1.0f) {
                    parsed = clamp100(parsed * 100.0f);
                } else {
                    parsed = clamp100(parsed);
                }
                cpu_usage_file_ = parsed;
            } catch(...) { }
        } else if (key == "mem") {
            try {
                std::string v = trim(val);
                bool has_percent = !v.empty() && v.back() == '%';
                if (has_percent) v.pop_back();
                float parsed = std::stof(v);
                if (!has_percent && parsed > 0.0f && parsed <= 1.0f) {
                    parsed = clamp100(parsed * 100.0f);
                } else {
                    parsed = clamp100(parsed);
                }
                mem_usage_file_ = parsed;
            } catch(...) { }
        } else if (key == "network") {
            try {
                std::string v = to_lower(trim(val));
                if (v == "offline" || v == "0") {
                    network_status_file_ = 0;
                } else if (v == "connecting" || v == "1" || v == "reconnecting") {
                    network_status_file_ = 1;
                } else if (v == "online" || v == "connected" || v == "ok" || v == "2") {
                    network_status_file_ = 2;
                } else {
                    // 尝试数值解析
                    network_status_file_ = static_cast<uint32_t>(std::stoul(v));
                    if (network_status_file_ > 2) network_status_file_ = 2;
                }
            } catch(...) { }
        } else if (key == "error") {
            // 支持十六进制（0x...）或逗号分隔的多个错误码
            try {
                std::string v = val;
                // 取第一个非空项（优先级最高）
                size_t comma = v.find(',');
                if (comma != std::string::npos) v = trim(v.substr(0, comma));
                // 支持带 0x 前缀的十六进制，也支持十进制
                int32_t parsed = 0;
                if (v.size() > 2 && (v[0]=='0' && (v[1]=='x' || v[1]=='X'))) {
                    parsed = static_cast<int32_t>(std::stol(v, nullptr, 16));
                } else {
                    parsed = static_cast<int32_t>(std::stol(v, nullptr, 10));
                }
                error_code_ = parsed;
            } catch(...) { }
        } else if (key == "robot_mode") {
            robot_mode_ = val;
        } else if (key == "vehicle_id") {
            vehicle_id_ = val;
        }
    }
}

void DomainControllerMonitor::sample_system() {
    std::lock_guard<std::mutex> lk(mu_);

    // CPU: /proc/stat 第一行 cpu user nice system idle iowait irq softirq steal
    {
        std::ifstream ifs("/proc/stat");
        if (ifs.is_open()) {
            std::string line;
            if (std::getline(ifs, line)) {
                std::istringstream iss(line);
                std::string cpu_label;
                iss >> cpu_label; // "cpu"
                uint64_t user=0, nice=0, system=0, idle=0, iowait=0, irq=0, softirq=0, steal=0;
                iss >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
                uint64_t total = user + nice + system + idle + iowait + irq + softirq + steal;
                uint64_t idle_all = idle + iowait;
                if (total > 0) {
                    if (has_last_cpu_) {
                        uint64_t prev_total = last_user_ + last_nice_ + last_system_ + last_idle_ + last_iowait_ + last_irq_ + last_softirq_ + last_steal_;
                        uint64_t prev_idle = last_idle_ + last_iowait_;
                        uint64_t delta_total = total > prev_total ? (total - prev_total) : 0;
                        uint64_t delta_idle = idle_all > prev_idle ? (idle_all - prev_idle) : 0;
                        if (delta_total > 0) {
                            float usage = (1.0f - static_cast<float>(delta_idle) / static_cast<float>(delta_total)) * 100.0f;
                            cpu_usage_sys_ = clamp100(usage);
                        }
                    }
                    last_user_ = user;
                    last_nice_ = nice;
                    last_system_ = system;
                    last_idle_ = idle;
                    last_iowait_ = iowait;
                    last_irq_ = irq;
                    last_softirq_ = softirq;
                    last_steal_ = steal;
                    has_last_cpu_ = true;
                }
            }
        }
    }

    // Mem: /proc/meminfo 使用 MemTotal 与 MemAvailable
    {
        std::ifstream ifs("/proc/meminfo");
        uint64_t total_kb = 0, avail_kb = 0;
        if (ifs.is_open()) {
            std::string k; uint64_t v; std::string unit;
            while (ifs >> k >> v >> unit) {
                if (k == "MemTotal:") total_kb = v;
                else if (k == "MemAvailable:") { avail_kb = v; break; }
            }
            if (total_kb > 0 && avail_kb <= total_kb) {
                float used = static_cast<float>(total_kb - avail_kb);
                float pct = (used / static_cast<float>(total_kb)) * 100.0f;
                mem_usage_sys_ = clamp100(pct);
            }
        }
    }

    // Network: 简单依据 /proc/net/dev 是否有非 lo 接口及其是否有流量来判断在线
    {
        std::ifstream ifs("/proc/net/dev");
        bool has_iface = false;
        bool has_traffic = false;
        if (ifs.is_open()) {
            std::string line;
            // 跳过前两行表头
            std::getline(ifs, line);
            std::getline(ifs, line);
            while (std::getline(ifs, line)) {
                auto pos = line.find(':');
                if (pos == std::string::npos) continue;
                std::string iface = trim(line.substr(0, pos));
                if (iface == "lo") continue;
                has_iface = true;
                std::istringstream iss(line.substr(pos + 1));
                uint64_t rx_bytes=0, rx_packets=0, rx_errs=0, rx_drop=0, rx_fifo=0, rx_frame=0, rx_compressed=0, rx_multicast=0;
                uint64_t tx_bytes=0, tx_packets=0, tx_errs=0, tx_drop=0, tx_fifo=0, tx_colls=0, tx_carrier=0, tx_compressed=0;
                iss >> rx_bytes >> rx_packets >> rx_errs >> rx_drop >> rx_fifo >> rx_frame >> rx_compressed >> rx_multicast
                    >> tx_bytes >> tx_packets >> tx_errs >> tx_drop >> tx_fifo >> tx_colls >> tx_carrier >> tx_compressed;
                if (rx_bytes > 0 || tx_bytes > 0) has_traffic = true;
            }
        }
        if (has_traffic) network_status_sys_ = 2;
        else if (has_iface) network_status_sys_ = 1;
        else network_status_sys_ = 0;
    }
}
 
} // namespace cloud_manager
