// report.cpp
#include "report.hpp"
#include <rclcpp/logging.hpp>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include "std_msgs/msg/int16.hpp"
#include <std_msgs/msg/empty.hpp>
#include "interface/msg/yisbot_status.hpp"

using namespace std::chrono_literals;

std::map<uint32_t, std::string> ReportManager::error_descriptions_;
std::map<ErrorModule, std::string> ReportManager::module_names_;
std::map<ErrorSubModule, std::string> ReportManager::sub_module_names_;

class AutoDrivetorecharge
{
public:
    // 定义回调函数类型
    using DoneCallback = std::function<void(int status, int progress)>;
    using SimpleDoneCallback = std::function<void()>;
    
    // 构造函数1：传入Node指针和简单回调
    AutoDrivetorecharge(rclcpp::Node* node, SimpleDoneCallback done_callback = nullptr)
    : node_(node), simple_done_callback_(done_callback)
    {
        if (!node_)
        {
            throw std::runtime_error("AutoDrivetorecharge: Node指针不能为空");
        }
        
        // 创建发布者
        publisher_ = node_->create_publisher<std_msgs::msg::Int16>("/yisbot/goto", 10);
        
        // 创建订阅者
        subscription_ = node_->create_subscription<interface::msg::YisbotStatus>(
            "/vlago/auto_drive/state", 10,
            std::bind(&AutoDrivetorecharge::on_nav_status, this, std::placeholders::_1));
        
        // 创建定时器，0.2秒后执行一次
        timer_ = node_->create_wall_timer(
            200ms, std::bind(&AutoDrivetorecharge::once_proc, this));
        
        RCLCPP_INFO(node_->get_logger(), "AutoDrivetorecharge 已启动");
    }
    
    // 构造函数2：传入Node指针和带状态的完成回调
    AutoDrivetorecharge(rclcpp::Node* node, DoneCallback done_callback)
    : node_(node), done_callback_with_status_(done_callback)
    {
        if (!node_)
        {
            throw std::runtime_error("AutoDrivetorecharge: Node指针不能为空");
        }
        
        // 创建发布者
        publisher_ = node_->create_publisher<std_msgs::msg::Int16>("/yisbot/goto", 10);
        yisbot_cannel_ = node->create_publisher<std_msgs::msg::Empty>("/yisbot/cancel", 10);
        if(1) {
            std_msgs::msg::Empty pub_msg;
            yisbot_cannel_->publish(pub_msg);
        }
        usleep(100*1000);
        // 创建订阅者
        subscription_ = node_->create_subscription<interface::msg::YisbotStatus>(
            "/vlago/auto_drive/state", 10,
            std::bind(&AutoDrivetorecharge::on_nav_status, this, std::placeholders::_1));
        
        // 创建定时器，0.2秒后执行一次
        timer_ = node_->create_wall_timer(
            200ms, std::bind(&AutoDrivetorecharge::once_proc, this));
        
        RCLCPP_INFO(node_->get_logger(), "AutoDrivetorecharge 已启动");
    }
    
    // 析构函数：释放资源
    ~AutoDrivetorecharge()
    {
        cleanup();
        RCLCPP_INFO(node_->get_logger(), "AutoDrivetorecharge 析构函数调用，资源已释放");
    }
    
    // 手动清理资源的公共接口
    void cleanup()
    {
        // 取消定时器
        if (timer_)
        {
            timer_->cancel();
            RCLCPP_DEBUG(node_->get_logger(), "定时器已取消");
        }
        
        // 重置发布者（释放资源）
        publisher_.reset();
        yisbot_cannel_.reset();
        
        // 重置订阅者（释放资源）
        subscription_.reset();
        
        // 重置定时器（释放资源）
        timer_.reset();
        
        // 注意：node_ 是原始指针，不负责释放，由外部管理
    }
    
    // 设置简单完成回调（无参数）
    void set_done_callback(SimpleDoneCallback callback)
    {
        simple_done_callback_ = callback;
    }
    
    // 设置带参数的完成回调（接收状态和进度）
    void set_done_callback_with_status(DoneCallback callback)
    {
        done_callback_with_status_ = callback;
    }
    
    // 获取节点指针
    rclcpp::Node* get_node()
    {
        return node_;
    }

private:
    void once_proc()
    {
        int id = 1; // 客厅基站
        auto msg = std_msgs::msg::Int16();
        msg.data = id;
        publisher_->publish(msg);
        
        // 取消定时器，只发布一次
        if (timer_)
        {
            timer_->cancel();
        }
        
        RCLCPP_INFO(node_->get_logger(), "已发布导航目标点 ID: %d", id);
    }
    
    void on_done(int status, int progress)
    {
        RCLCPP_INFO(node_->get_logger(), "导航任务完成: status=%d, progress=%d", status, progress);
        
        // 优先使用带参数的回调
        if (done_callback_with_status_)
        {
            done_callback_with_status_(status, progress);
        }
        // 其次使用简单回调
        else if (simple_done_callback_)
        {
            simple_done_callback_();
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "未设置完成回调函数");
        }
    }
    
    void on_nav_status(const interface::msg::YisbotStatus::SharedPtr msg)
    {
        int status = msg->status;  // 0:停止 1:开始行驶 2:遇障碍物绕行 3:遇障碍物停车 4:到达目的地 -1:取消 -2:异常退出 -3:地图不匹配
        int progress = msg->progress;
        
        RCLCPP_INFO(node_->get_logger(), "nav status: %d progress: %d", status, progress);
        
        // 到达目的地或异常退出时结束程序
        if (status == 4 || status == -1 || status == -2 || status == -3)
        {
            on_done(status, progress);
        }
    }
    
    rclcpp::Node* node_;  // 原始指针，不管理生命周期
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr yisbot_cannel_;
    rclcpp::Subscription<interface::msg::YisbotStatus>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    SimpleDoneCallback simple_done_callback_;
    DoneCallback done_callback_with_status_;
};

ReportManager::ReportManager(rclcpp::Node *node,IpadManager* ipad)
    : node_(node),
     _ipad(ipad)
{
    // 创建报告发布者
    report_publisher_ = node_->create_publisher<std_msgs::msg::String>(
        "wheelchair/report", 10);

/* // TODO 增加发布协议
    recharge_state_pub_ = node->create_publisher<std_msgs::msg::Int32>(rechargeStateTopic, 10);
*/

    recharge_service_ = node_->create_service<interface::srv::SceneControls>(
        sceneRechargeService,
        std::bind(&ReportManager::handleRechargeService, this, 
                 std::placeholders::_1, std::placeholders::_2));
    // 初始化错误映射
    initializeErrorMaps();
    RCLCPP_INFO(node_->get_logger(), "报告管理器已初始化");
}

void ReportManager::handleRechargeService(
    const std::shared_ptr<interface::srv::SceneControls::Request> request,
    std::shared_ptr<interface::srv::SceneControls::Response> response) {
    
    std::lock_guard<std::mutex> lock(mode_mutex_);
    
    if (request->start == 1) {
        recharge_mode_enabled_.store(true, std::memory_order_release);
         std::thread([this]() {
            if (0) {
                std::mutex mtx;
                std::condition_variable cv;
                bool done = false;
                // 创建回调，在任务完成时通知
                auto callback = [&mtx, &cv, &done](int status, int progress) {
                    {
                        std::lock_guard<std::mutex> lock(mtx);
                        done = true;
                    }
                    cv.notify_one();
                };
                auto goto_recharge = std::make_shared<AutoDrivetorecharge>(node_, callback);
                // 等待任务完成
                {
                    std::unique_lock<std::mutex> lock(mtx);
                    cv.wait(lock, [&done] { return done; });
                }
                goto_recharge.reset();
            }
            RCLCPP_WARN(node_->get_logger(), "回充功能已开启 (启用执行器，记录最大线速度)");
            _ipad->startRecharge();
            RCLCPP_INFO(node_->get_logger(), "回充功能已开启 (启用执行器，记录最大线速度)END");
         }).detach();

        response->code = 0;
        response->message = "回充功能已开启";
    } else {
        recharge_mode_enabled_.store(false, std::memory_order_release);
        std::thread([this]() {
             RCLCPP_INFO(node_->get_logger(), "回充功能已关闭");
            _ipad->cancelRecharge();
            RCLCPP_INFO(node_->get_logger(), "回充功能已关闭END");
         }).detach();

        response->code = 0;
        response->message = "回充功能已关闭";
    }
}

std::string ReportManager::generateSystemStatusReport(
    WheelchairState wheelchair_state,
    BatteryState battery_state,
    float battery_percentage,
    ChargingState charging_state,
    bool charging_voltage_detected,
    bool recharge_active,
    const std::string &control_mode)
{
    std::stringstream report;

    report << "=== 轮椅系统状态报告 ===\n";
    report << "时间: " << node_->now().seconds() << "s\n\n";

    // 轮椅状态
    report << "1. 轮椅状态:\n";
    report << "   - 当前状态: " << wheelchairStateToString(wheelchair_state) << "\n";
    report << "   - 控制模式: " << control_mode << "\n";
    report << "   - 回充状态: " << (recharge_active ? "激活" : "未激活") << "\n\n";

    // 电池状态
    report << "2. 电池状态:\n";
    report << "   - 电量: " << battery_percentage << "%\n";
    report << "   - 状态: " << batteryStateToString(battery_state) << "\n\n";

    // 充电状态
    report << "3. 充电状态:\n";
    report << "   - 充电状态: " << chargingStateToString(charging_state) << "\n";
    report << "   - 充电电压: " << (charging_voltage_detected ? "检测到" : "未检测") << "\n\n";

    // 错误状态
    report << "4. 错误状态:\n";
    std::vector<ErrorInfo> active_errors = getActiveErrors();
    if (active_errors.empty())
    {
        report << "   - 当前错误: 无\n";
    }
    else
    {
        report << "   - 当前错误: " << active_errors.size() << "个\n";
        for (const auto &error : active_errors)
        {
            report << "     * " << error.module_name << ": " << error.description << "\n";
        }
    }

    // 系统状态
    report << "4. 系统状态:\n";
    report << "   - 运行时间: " << "N/A" << "\n";
    report << "   - 安全状态: 正常\n";
    report << "   - 错误代码: 无\n";

    return report.str();
}

void ReportManager::publishStatusReport(const std::string &report)
{
    auto msg = std_msgs::msg::String();
    msg.data = report;
    report_publisher_->publish(msg);

    // 调用回调函数
    if (status_report_callback_)
    {
        status_report_callback_(report);
    }
}

void ReportManager::reportSystemStatus(
    WheelchairState wheelchair_state,
    BatteryState battery_state,
    float battery_percentage,
    ChargingState charging_state,
    bool charging_voltage_detected,
    bool recharge_active,
    const std::string &control_mode)
{
    std::string report = generateSystemStatusReport(
        wheelchair_state, battery_state, battery_percentage,
        charging_state, charging_voltage_detected,
        recharge_active, control_mode);

    publishStatusReport(report);
    logReport("SYSTEM_STATUS", "系统状态报告已生成");
}

void ReportManager::reportBatteryStatus(BatteryState state, float percentage)
{
    std::stringstream report;
    report << "电池状态报告:\n";
    report << "- 电量: " << percentage << "%\n";
    report << "- 状态: " << batteryStateToString(state) << "\n";
    report << "- 时间: " << node_->now().seconds() << "s";

    publishStatusReport(report.str());
    logReport("BATTERY_STATUS", "电池状态报告已生成");
}

void ReportManager::reportChargingStatus(ChargingState state)
{
    std::stringstream report;
    report << "充电状态报告:\n";
    report << "- 状态: " << chargingStateToString(state) << "\n";
    report << "- 时间: " << node_->now().seconds() << "s";

    publishStatusReport(report.str());
    logReport("CHARGING_STATUS", "充电状态报告已生成");
}

void ReportManager::reportNavigationStatus(const std::string &status)
{
    std::stringstream report;
    report << "导航状态报告:\n";
    report << "- 状态: " << status << "\n";
    report << "- 时间: " << node_->now().seconds() << "s";

    publishStatusReport(report.str());
    logReport("NAVIGATION_STATUS", "导航状态报告已生成");
}

void ReportManager::reportError(const std::string &error_message)
{
    std::stringstream report;
    report << "错误报告:\n";
    report << "- 错误信息: " << error_message << "\n";
    report << "- 时间: " << node_->now().seconds() << "s";

    publishStatusReport(report.str());
    logReport("ERROR", error_message);
}

void ReportManager::reportWarning(const std::string &warning_message)
{
    std::stringstream report;
    report << "警告报告:\n";
    report << "- 警告信息: " << warning_message << "\n";
    report << "- 时间: " << node_->now().seconds() << "s";

    publishStatusReport(report.str());
    logReport("WARNING", warning_message);
}

void ReportManager::reportInfo(const std::string &info_message)
{
    std::stringstream report;
    report << "信息报告:\n";
    report << "- 信息: " << info_message << "\n";
    report << "- 时间: " << node_->now().seconds() << "s";

    publishStatusReport(report.str());
    logReport("INFO", info_message);
}

// ==================== 错误管理功能 ====================

void ReportManager::errorCodeCallback(const std_msgs::msg::UInt32::SharedPtr msg)
{
    uint32_t error_code = msg->data;
    RCLCPP_INFO(node_->get_logger(), "收到错误码: 0x%08X", error_code);
    processErrorCode(error_code);
}

ErrorInfo ReportManager::parseErrorCode(uint32_t error_code)
{
    ErrorInfo info;
    info.error_code = error_code;
    info.timestamp = node_->now().nanoseconds() / 1000000; // 转换为毫秒
    info.is_recovered = false;

    // 解析模块和子模块名称
    ErrorModule module = info.getModule();
    ErrorSubModule sub_module = info.getSubModule();

    if (module_names_.find(module) != module_names_.end())
    {
        info.module_name = module_names_[module];
    }
    else
    {
        info.module_name = "未知模块";
    }

    if (sub_module_names_.find(sub_module) != sub_module_names_.end())
    {
        info.sub_module_name = sub_module_names_[sub_module];
    }
    else
    {
        info.sub_module_name = "未知子模块";
    }

    // 获取错误描述
    if (error_descriptions_.find(error_code) != error_descriptions_.end())
    {
        info.description = error_descriptions_[error_code];
    }
    else
    {
        info.description = getErrorDescription(error_code);
    }

    return info;
}

void ReportManager::processErrorCode(uint32_t error_code)
{
    // 仅记录到活跃错误列表用于报告
    std::lock_guard<std::mutex> lock(error_mutex_);
    ErrorInfo info = parseErrorCode(error_code);
    active_errors_[error_code] = info;
}

void ReportManager::processErrorCode(const ErrorInfo &error_info)
{
    // 仅记录到活跃错误列表用于报告
    std::lock_guard<std::mutex> lock(error_mutex_);
    active_errors_[error_info.error_code] = error_info;
}

void ReportManager::handleNewError(const ErrorInfo &error_info)
{
    // 添加到活跃错误列表
    active_errors_[error_info.error_code] = error_info;

    // 生成报告
    std::stringstream report;
    report << "错误报告 - " << error_info.module_name << " - "
           << error_info.sub_module_name << "\n";
    report << "错误码: 0x" << std::hex << std::setw(8) << std::setfill('0')
           << error_info.error_code << "\n";
    report << "描述: " << error_info.description << "\n";
    report << "时间: " << std::dec << error_info.timestamp << "ms\n";
    report << "严重程度: " << (error_info.isCritical() ? "严重" : "一般");

    if (error_info.isCritical())
    {
        reportError(report.str());
        RCLCPP_ERROR(node_->get_logger(), "检测到严重错误: %s - %s",
                     error_info.module_name.c_str(),
                     error_info.description.c_str());
    }
    else
    {
        reportWarning(report.str());
        RCLCPP_WARN(node_->get_logger(), "检测到新错误: %s - %s",
                    error_info.module_name.c_str(),
                    error_info.description.c_str());
    }

    // 调用回调函数
    if (error_callback_)
    {
        error_callback_(error_info);
    }

    if (error_info.isCritical() && critical_error_callback_)
    {
        critical_error_callback_(error_info);
    }
}

void ReportManager::notifyErrorRecovered(uint32_t error_code)
{
    std::lock_guard<std::mutex> lock(error_mutex_);

    if (active_errors_.find(error_code) != active_errors_.end())
    {
        ErrorInfo &error_info = active_errors_[error_code];
        error_info.is_recovered = true;

        // 生成恢复报告
        std::stringstream report;
        report << "错误恢复 - " << error_info.module_name << "\n";
        report << "错误码: 0x" << std::hex << error_code << "\n";
        report << "描述: " << error_info.description << "\n";
        report << "恢复时间: " << std::dec << node_->now().nanoseconds() / 1000000 << "ms";

        reportInfo(report.str());

        // 调用恢复回调
        if (recovery_callback_)
        {
            recovery_callback_(error_info);
        }

        RCLCPP_INFO(node_->get_logger(), "错误码 0x%08X 已恢复", error_code);

        // 从活跃错误列表中移除
        active_errors_.erase(error_code);
    }
}

// 查询接口
bool ReportManager::hasCriticalErrors() const
{
    std::lock_guard<std::mutex> lock(error_mutex_);

    for (const auto &pair : active_errors_)
    {
        if (pair.second.isCritical() && !pair.second.is_recovered)
        {
            return true;
        }
    }
    return false;
}

bool ReportManager::hasError(uint32_t error_code) const
{
    std::lock_guard<std::mutex> lock(error_mutex_);
    return active_errors_.find(error_code) != active_errors_.end();
}

std::vector<ErrorInfo> ReportManager::getActiveErrors() const
{
    std::lock_guard<std::mutex> lock(error_mutex_);

    std::vector<ErrorInfo> errors;
    for (const auto &pair : active_errors_)
    {
        if (!pair.second.is_recovered)
        {
            errors.push_back(pair.second);
        }
    }
    return errors;
}

std::vector<ErrorInfo> ReportManager::getCriticalErrors() const
{
    std::lock_guard<std::mutex> lock(error_mutex_);

    std::vector<ErrorInfo> critical_errors;
    for (const auto &pair : active_errors_)
    {
        if (pair.second.isCritical() && !pair.second.is_recovered)
        {
            critical_errors.push_back(pair.second);
        }
    }
    return critical_errors;
}

// 回调设置
void ReportManager::setStatusReportCallback(StatusReportCallback callback)
{
    status_report_callback_ = callback;
}

void ReportManager::setErrorCallback(ErrorCallback callback)
{
    error_callback_ = callback;
}

void ReportManager::setCriticalErrorCallback(ErrorCallback callback)
{
    critical_error_callback_ = callback;
}

void ReportManager::setRecoveryCallback(ErrorCallback callback)
{
    recovery_callback_ = callback;
}

// 错误描述获取
std::string ReportManager::getErrorDescription(uint32_t error_code)
{
    ErrorModule module = static_cast<ErrorModule>((error_code >> 24) & 0xFF);
    ErrorSubModule sub_module = static_cast<ErrorSubModule>((error_code >> 16) & 0xFF);
    uint16_t error_num = error_code & 0xFFFF;

    std::stringstream description;
    description << getModuleName(module) << " - "
                << getSubModuleName(sub_module) << " - 错误编号: 0x"
                << std::hex << error_num;

    return description.str();
}

std::string ReportManager::getModuleName(ErrorModule module)
{
    if (module_names_.find(module) != module_names_.end())
    {
        return module_names_[module];
    }
    return "未知模块";
}

std::string ReportManager::getSubModuleName(ErrorSubModule sub_module)
{
    if (sub_module_names_.find(sub_module) != sub_module_names_.end())
    {
        return sub_module_names_[sub_module];
    }
    return "未知子模块";
}

// 初始化静态映射
void ReportManager::initializeErrorMaps()
{
    // 模块名称映射
    module_names_[ErrorModule::SYSTEM] = "系统";
    module_names_[ErrorModule::IMU] = "IMU";
    module_names_[ErrorModule::GPS] = "GPS";
    module_names_[ErrorModule::LIDAR1] = "左雷达";
    module_names_[ErrorModule::LIDAR2] = "右雷达";
    module_names_[ErrorModule::LIDAR3] = "左斜雷达";
    module_names_[ErrorModule::LIDAR4] = "右斜雷达";
    module_names_[ErrorModule::LIDAR5] = "后雷达";
    module_names_[ErrorModule::CAN0] = "CAN0总线";
    module_names_[ErrorModule::CAN1] = "CAN1总线";
    module_names_[ErrorModule::CAN2] = "CAN2总线";
    module_names_[ErrorModule::DRIVER] = "电机驱动器";
    module_names_[ErrorModule::TOP] = "TOP驱动器";
    module_names_[ErrorModule::OBSTACLE] = "避障功能";
    module_names_[ErrorModule::RECHARGE] = "回充功能";

    // 子模块名称映射
    sub_module_names_[ErrorSubModule::HARDWARE] = "硬件";
    sub_module_names_[ErrorSubModule::DATA] = "数据";
    sub_module_names_[ErrorSubModule::TIMING] = "时间同步";
    sub_module_names_[ErrorSubModule::CONFIG] = "配置";
    sub_module_names_[ErrorSubModule::COMMUNICATION] = "通信";
    sub_module_names_[ErrorSubModule::CALIBRATION] = "标定";
    sub_module_names_[ErrorSubModule::PERFORMANCE] = "性能";
    sub_module_names_[ErrorSubModule::POWER] = "电源";
    sub_module_names_[ErrorSubModule::TEMPERATURE] = "温度";
    sub_module_names_[ErrorSubModule::OTHER] = "其他";

    // 根据文档初始化具体的错误描述
    // 系统错误 (0x01)
    error_descriptions_[0x01010101] = "系统硬件故障";
    error_descriptions_[0x01020101] = "系统数据无效";
    error_descriptions_[0x01030101] = "系统时间同步失败";

    // IMU错误 (0x10)
    error_descriptions_[0x10010101] = "IMU硬件故障";
    error_descriptions_[0x10020102] = "IMU数据无效";
    error_descriptions_[0x10030103] = "IMU数据超时";

    // 雷达错误示例
    error_descriptions_[0x12010101] = "左雷达硬件故障";
    error_descriptions_[0x12020101] = "左雷达数据无效";
    error_descriptions_[0x13010101] = "右雷达硬件故障";
    error_descriptions_[0x16010101] = "后雷达硬件故障";

    // CAN总线错误
    error_descriptions_[0x02010101] = "CAN0硬件故障";
    error_descriptions_[0x02020105] = "CAN0通信失败";
    error_descriptions_[0x03010101] = "CAN1硬件故障";

    // 电机驱动器错误
    error_descriptions_[0x20010108] = "电机驱动器电源异常";
    error_descriptions_[0x20010109] = "电机驱动器温度过高";

    // 避障功能错误
    error_descriptions_[0x50010101] = "避障功能硬件故障";
    error_descriptions_[0x50020107] = "避障功能性能低下";

    // 回充功能错误（根据文档）
    error_descriptions_[0x51010101] = "回充检测失败（超时）";
    error_descriptions_[0x51010102] = "回充对接失败";
    error_descriptions_[0x51010103] = "回充电压异常";
    error_descriptions_[0x51010104] = "回充过程中断";
    error_descriptions_[0x51010105] = "回充电流异常";
}

// 辅助函数
std::string ReportManager::wheelchairStateToString(WheelchairState state) const
{
    switch (state)
    {
    case WheelchairState::READY:
        return "就绪中";
    case WheelchairState::WALKING:
        return "行走中";
    case WheelchairState::ROTATING:
        return "旋转中";
    case WheelchairState::SEARCHING:
        return "搜索中";
    case WheelchairState::CHARGING:
        return "充电中";
    default:
        return "未知状态";
    }
}

std::string ReportManager::batteryStateToString(BatteryState state) const
{
    switch (state)
    {
    case BatteryState::NORMAL:
        return "正常";
    case BatteryState::LOW:
        return "低电量";
    case BatteryState::CRITICAL:
        return "严重低电量";
    case BatteryState::CHARGING:
        return "充电中";
    case BatteryState::FULL:
        return "充满";
    default:
        return "未知";
    }
}

std::string ReportManager::chargingStateToString(ChargingState state) const
{
    switch (state)
    {
    case ChargingState::IDLE:
        return "空闲";
    case ChargingState::NAVIGATING:
        return "导航中";
    case ChargingState::CHARGING:
        return "充电中";
    case ChargingState::UNAVAILABLE:
        return "不可用";
    case ChargingState::COMPLETED:
        return "完成";
    default:
        return "未知";
    }
}

void ReportManager::logReport(const std::string &category, const std::string &message)
{
    RCLCPP_INFO(node_->get_logger(), "[%s] %s", category.c_str(), message.c_str());
}