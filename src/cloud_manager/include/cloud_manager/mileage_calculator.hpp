#pragma once  // 确保头文件只包含一次
#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <sstream>

// 里程计算器：实时轮数直接算当前里程（当次里程），总里程=历史存储+当前里程，配置文件持久化
// 核心变更：当次里程（current_mileage_）永不重置，始终跟随实时轮数更新
class MileageCalculator {
public:
    // 构造函数：传入车轮直径和配置文件路径
    MileageCalculator(
        float wheel_diameter = 0.45f,  // 默认车轮直径0.45m
        // 1. 核心修改：配置文件路径从 mqtt_demo→cloud_manager（与模块文件夹名一致）
        const std::string& config_path = "src/cloud_manager/config/mileage_config.ini",
        bool enable_log = true
    ) : wheel_diameter_(wheel_diameter),
        wheel_circumference_(M_PI * wheel_diameter),  // 车轮周长=π×直径
        config_path_(config_path),
        log_enabled_(enable_log),
        history_total_mileage_(0.0f),
        current_mileage_(0.0f),
        total_mileage_(0.0f) {
        log_enabled_ = false;
        load_history_total_from_config();
        // 初始化总里程为历史里程，避免在未更新轮数时写回 0 覆盖历史
        total_mileage_ = history_total_mileage_;
        // 2. 日志优化（可选）：将 [MileageCalculator] 改为 [CloudManager-MileageCalculator]，语义更清晰
        if (log_enabled_) {
            std::cout << "[CloudManager-MileageCalculator] 初始化完成：" << std::endl
                      << "  车轮周长：" << std::fixed << std::setprecision(2) << wheel_circumference_ << "m/圈" << std::endl
                      << "  历史总里程（配置文件）：" << history_total_mileage_ << "m" << std::endl;
        }
    }

    // 析构函数：退出时保存最新总里程到配置文件
    ~MileageCalculator() {
        save_total_to_config();
        // 日志优化（可选）：同步模块名
        if (log_enabled_) {
            std::cout << "[CloudManager-MileageCalculator] 程序退出，总里程已保存到配置文件" << std::endl;
        }
    }

    // 核心接口：更新实时轮数，计算当前里程（当次里程）和总里程（功能不变）
    void update_real_time_mileage(int32_t left_count, int32_t right_count) {
        // 1. 过滤无效轮数，计算平均轮数
        float avg_wheel_count = 0.0f;
        int valid_wheels = 0;
        if (left_count >= 0) { avg_wheel_count += left_count; valid_wheels++; }
        if (right_count >= 0) { avg_wheel_count += right_count; valid_wheels++; }
        avg_wheel_count = valid_wheels > 0 ? avg_wheel_count / valid_wheels : 0.0f;

        // 2. 计算当前里程（当次里程）：实时轮数直接计算，永不重置
        current_mileage_ = avg_wheel_count * wheel_circumference_;
        current_mileage_ = current_mileage_ < 0 ? 0.0f : current_mileage_;

        // 3. 计算总里程：历史总里程 + 当前里程（当次里程）
        total_mileage_ = history_total_mileage_ + current_mileage_;

        // 4. 持久化最新总里程
        save_total_to_config();

        // 日志优化（可选）：同步模块名
        if (log_enabled_) {
            std::cout << "[CloudManager-MileageCalculator] 里程更新：" << std::endl
                      << "  实时轮数：左=" << left_count << "，右=" << right_count << " → 平均=" << avg_wheel_count << "圈" << std::endl
                      << "  当前里程（当次，永不重置）：" << current_mileage_ << "m" << std::endl
                      << "  历史总里程：" << history_total_mileage_ << "m" << std::endl
                      << "  最新总里程：" << total_mileage_ << "m" << std::endl;
        }
    }

    // 获取当前里程（当次里程，实时轮数计算，永不重置）（功能不变）
    float get_current_mileage() const { return current_mileage_; }

    // 获取总里程（历史+当前）（功能不变）
    float get_total_mileage() const { return total_mileage_; }

private:
    // 从配置文件读取历史总里程（功能不变，日志优化可选）
    void load_history_total_from_config() {
        if (config_path_.empty()) {
            if (log_enabled_) {
                std::cerr << "[CloudManager-MileageCalculator] 警告：配置文件路径为空，历史总里程初始化为0" << std::endl;
            }
            history_total_mileage_ = 0.0f;
            return;
        }

        if (!std::filesystem::exists(config_path_)) {
            if (log_enabled_) {
                std::cout << "[CloudManager-MileageCalculator] 配置文件不存在，创建默认文件：" << config_path_ << std::endl;
            }
            save_total_to_config(0.0f);
            history_total_mileage_ = 0.0f;
            return;
        }

        std::ifstream config_file(config_path_);
        if (!config_file.is_open()) {
            if (log_enabled_) {
                std::cerr << "[CloudManager-MileageCalculator] 错误：无法打开配置文件 " << config_path_ << "，历史总里程初始化为0" << std::endl;
            }
            history_total_mileage_ = 0.0f;
            return;
        }

        std::string line;
        while (std::getline(config_file, line)) {
            if (line.empty() || line[0] == ';' || line[0] == '#') continue;

            size_t equal_pos = line.find('=');
            if (equal_pos == std::string::npos) continue;

            std::string key = trim(line.substr(0, equal_pos));
            std::string value = trim(line.substr(equal_pos + 1));

            if (key == "history_total_mileage") {
                try {
                    history_total_mileage_ = std::stof(value);
                } catch (...) {
                    if (log_enabled_) {
                        std::cerr << "[CloudManager-MileageCalculator] 错误：配置文件格式错误，历史总里程初始化为0" << std::endl;
                    }
                    history_total_mileage_ = 0.0f;
                }
                break;
            }
        }

        config_file.close();
    }

    // 保存最新总里程到配置文件（功能不变，日志优化可选）
    void save_total_to_config() {
        save_total_to_config(total_mileage_);
    }

    void save_total_to_config(float value) {
        if (config_path_.empty()) {
            if (log_enabled_) {
                std::cerr << "[CloudManager-MileageCalculator] 警告：配置文件路径为空，保存失败" << std::endl;
            }
            return;
        }

        std::ofstream config_file(config_path_);
        if (!config_file.is_open()) {
            if (log_enabled_) {
                std::cerr << "[CloudManager-MileageCalculator] 错误：无法打开配置文件 " << config_path_ << "，保存失败" << std::endl;
            }
            return;
        }

        config_file << "[Mileage]" << std::endl;
        config_file << "history_total_mileage = " << std::fixed << std::setprecision(2) << value << std::endl;
        config_file.close();

        if (log_enabled_) {
            std::cout << "[CloudManager-MileageCalculator] 配置文件已更新：" << config_path_ << " → " << value << "m" << std::endl;
        }
    }

    // 辅助函数：去除字符串前后空格（功能不变）
    std::string trim(const std::string& str) {
        size_t start = str.find_first_not_of(" \t\n\r");
        size_t end = str.find_last_not_of(" \t\n\r");
        return (start == std::string::npos || end == std::string::npos) ? "" : str.substr(start, end - start + 1);
    }

private:
    float wheel_diameter_;          // 车轮直径（m）
    float wheel_circumference_;     // 车轮周长（m/圈）
    std::string config_path_;       // 配置文件路径（已同步修改）
    bool log_enabled_;              // 是否启用日志输出
    float history_total_mileage_;   // 历史总里程（从配置文件读取）
    float current_mileage_;         // 当前里程（当次里程，实时轮数计算，永不重置）
    float total_mileage_;           // 总里程（历史+当前）
};
