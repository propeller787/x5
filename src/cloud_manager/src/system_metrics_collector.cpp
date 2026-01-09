#include "cloud_manager/system_metrics_collector.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <filesystem>

namespace cloud_manager {

static inline uint64_t to_u64(const std::string &s) {
    try { return static_cast<uint64_t>(std::stoull(s)); } catch (...) { return 0; }
}

void SystemMetricsCollector::sample() {
    // CPU: /proc/stat 第一行 cpu user nice system idle iowait irq softirq steal guest guest_nice
    {
        std::ifstream ifs("/proc/stat");
        if (ifs.is_open()) {
            std::string line; std::getline(ifs, line);
            std::istringstream iss(line);
            std::string cpu_label; iss >> cpu_label; // "cpu"
            std::vector<uint64_t> v;
            std::string tok;
            while (iss >> tok) v.push_back(to_u64(tok));
            if (v.size() >= 4) {
                uint64_t user=v[0], nice=v[1], system=v[2], idle=v[3];
                uint64_t iowait = v.size()>4 ? v[4] : 0;
                uint64_t irq = v.size()>5 ? v[5] : 0;
                uint64_t softirq = v.size()>6 ? v[6] : 0;
                uint64_t steal = v.size()>7 ? v[7] : 0;
                uint64_t total = user + nice + system + idle + iowait + irq + softirq + steal;
                if (!cpu_inited_) {
                    prev_idle_ = idle; prev_total_ = total; cpu_inited_ = true;
                } else {
                    uint64_t delta_idle = idle - prev_idle_;
                    uint64_t delta_total = total - prev_total_;
                    prev_idle_ = idle; prev_total_ = total;
                    if (delta_total > 0) {
                        float usage = (1.0f - static_cast<float>(delta_idle) / static_cast<float>(delta_total)) * 100.0f;
                        if (usage < 0.0f) usage = 0.0f; if (usage > 100.0f) usage = 100.0f;
                        cpu_percent_ = usage;
                    }
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
                if (pct < 0.0f) pct = 0.0f; if (pct > 100.0f) pct = 100.0f;
                mem_percent_ = pct;
            }
        }
    }

    // Network: online if default route exists and interface operstate is up
    {
        bool has_default = false; std::string ifname;
        std::ifstream ifs("/proc/net/route");
        std::string iface, destination, gateway, flags;
        if (ifs.is_open()) {
            std::string header; std::getline(ifs, header);
            while (ifs >> iface >> destination >> gateway >> flags) {
                if (destination == "00000000") { has_default = true; ifname = iface; break; }
                std::string dummy; std::getline(ifs, dummy);
            }
        }
        if (!has_default) { network_status_ = 0; }
        else {
            std::string oper = "/sys/class/net/" + ifname + "/operstate";
            std::ifstream opf(oper);
            std::string state; if (opf.is_open()) std::getline(opf, state);
            for (auto &c : state) c = static_cast<char>(std::tolower(c));
            if (state == "up") network_status_ = 2; // online
            else if (state == "dormant" || state == "unknown") network_status_ = 1; // connecting
            else network_status_ = 0; // down
        }
    }
}

} // namespace cloud_manager
