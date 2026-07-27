#include "cloud_manager/domain_controller_monitor.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>

namespace cloud_manager {

static inline std::string trim(const std::string &s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

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
            try { cpu_usage_ = std::stof(val); } catch(...) { }
        } else if (key == "mem") {
            try { mem_usage_ = std::stof(val); } catch(...) { }
        } else if (key == "network") {
            try { network_status_ = static_cast<uint32_t>(std::stoul(val)); } catch(...) { }
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

} // namespace cloud_manager
