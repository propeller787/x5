#pragma once

#include <cstdint>
#include <string>

namespace cloud_manager {

// 轻量系统指标采集：/proc/stat、/proc/meminfo、/proc/net/route + operstate
class SystemMetricsCollector {
public:
    void sample();

    // 返回百分比 0..100
    float cpu_percent() const { return cpu_percent_; }
    float mem_percent() const { return mem_percent_; }
    // 0=offline 1=connecting 2=online
    uint32_t network_status() const { return network_status_; }

private:
    // CPU: 前一次采样
    uint64_t prev_idle_ = 0;
    uint64_t prev_total_ = 0;
    bool cpu_inited_ = false;
    float cpu_percent_ = 0.0f;

    // Mem
    float mem_percent_ = 0.0f;

    // Network
    uint32_t network_status_ = 0;
};

} // namespace cloud_manager
