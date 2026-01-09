#pragma once

#include <string>
#include <mutex>
#include <cstdint>

namespace cloud_manager {

// 简单解耦：从一个文本键值文件读取域控制器状态（key=value 每行），
// 由外部进程周期写入该文件（例如监控守护进程）。
class DomainControllerMonitor {
public:
    DomainControllerMonitor(const std::string &status_file = "/tmp/domain_controller_status.txt")
        : status_file_(status_file) {}

    // 读取并刷新内部缓存（非阻塞短操作）
    void refresh();
    // 实时采集系统指标（CPU/MEM/NET）
    void sample_system();

    float cpu_usage() const { return cpu_usage_file_; }
    float mem_usage() const { return mem_usage_file_; }
    uint32_t network_status() const { return network_status_file_; }

    float cpu_usage_system() const { return cpu_usage_sys_; }
    float mem_usage_system() const { return mem_usage_sys_; }
    uint32_t network_status_system() const { return network_status_sys_; }
    int32_t error_code() const { return error_code_; }
    std::string robot_mode() const { return robot_mode_; }
    std::string vehicle_id() const { return vehicle_id_; }

    void set_status_file(const std::string &p) { status_file_ = p; }

private:
    std::string status_file_;
    mutable std::mutex mu_;
    float cpu_usage_file_ = 0.0f;
    float mem_usage_file_ = 0.0f;
    uint32_t network_status_file_ = 0;
    float cpu_usage_sys_ = 0.0f;
    float mem_usage_sys_ = 0.0f;
    uint32_t network_status_sys_ = 0;
    int32_t error_code_ = 0;
    std::string robot_mode_;
    std::string vehicle_id_;

    // CPU 采样：保存上次 /proc/stat 的累计 jiffies
    uint64_t last_user_ = 0, last_nice_ = 0, last_system_ = 0, last_idle_ = 0, last_iowait_ = 0,
             last_irq_ = 0, last_softirq_ = 0, last_steal_ = 0;
    bool has_last_cpu_ = false;
};

} // namespace cloud_manager
