/**
 * @file ota_manager.h
 * @brief OTA 功能独立管理类：订阅 OTA 相关 topic，调用 OTA 服务，供 TFT 查询与展示。
 */

#pragma once

#include "control/define.h"
#include "interface/srv/package_check.hpp"
#include "interface/msg/ota_package_info.hpp"
#include "interface/srv/package_download.hpp"
#include "interface/srv/package_upgrade.hpp"
#include "interface/msg/ota_download_msg.hpp"
#include "interface/msg/ota_download_progress.hpp"
#include "interface/msg/ota_upgrade_msg.hpp"
#include "interface/msg/ota_upgrade_progress.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <string>
#include <functional>

namespace tft_service {

/** OTA 升级状态（供 TFT 显示与业务判断） */
struct OtaStatus {
    std::string current_version;   ///< 当前软件版本号
    std::string new_version;       ///< 最新可用版本号
    std::string history_version;   ///< 历史版本号（与 current_version 一致）
    std::string new_description;   ///< 新版本描述
    int32_t new_size{0};          ///< 新版本包大小（字节）
    std::string message;          ///< 服务响应消息
    bool is_downloading{false};   ///< 是否正在下载（true 有下载任务 false 无下载任务）
    bool is_upgrading{false};     ///< 是否正在升级（true 有升级任务 false 无升级任务）
    double download_progress{0.0}; ///< 下载进度 0.0~100.0
    double upgrade_progress{0.0};  ///< 升级进度 0.0~100.0
    std::string download_message;  ///< 下载过程消息
    std::string upgrade_message;   ///< 升级过程消息
};

/**
 * @brief OTA 功能独立管理类：订阅 OTA 相关 topic，调用 OTA 服务，供 TFT 查询与展示。
 */
class OtaManager {
public:
    static OtaManager& instance();

    /** 在节点内初始化：创建订阅与服务客户端 */
    void init(rclcpp::Node* node);

    /** 是否已初始化 */
    bool isInitialized() const { return node_ != nullptr; }

    /** 获取当前 OTA 状态（线程安全） */
    OtaStatus getStatus() const;

    /** 设置页面切换回调：由 TftService 注册，OtaManager 内部通过此回调切换 TFT 页面（线程安全） */
    void setOnSwitchPageRequested(std::function<void(int page_id)> cb) {
        on_switch_page_ = std::move(cb);
    }
    void setOnShowDialogRequested(std::function<void(const std::string& dialog_id, int page_id)> cb) {
        on_show_dialog_ = std::move(cb);
    }
    /** 请求切换 TFT 页面（调用已注册的回调，无回调则忽略） */
    void requestSwitchPage(int page_id);

    /** 设置 TFT 数值写入回调（uint16_t） */
    void setOnWriteTftValue(std::function<void(uint16_t addr, uint16_t value)> cb) { on_write_tft_value_ = std::move(cb); }
    /** 设置 TFT 字符串写入回调 */
    void setOnWriteTftText(std::function<void(uint16_t addr, const std::string& text)> cb) { on_write_tft_text_ = std::move(cb); }
    /** 设置 TFT 浮点数写入回调 */
    void setOnWriteTftFloat(std::function<void(uint16_t addr, float value)> cb) { on_write_tft_float_ = std::move(cb); }

    /** 请求写入 TFT 数值到指定地址 */
    void requestWriteTftValue(uint16_t addr, uint16_t value);
    /** 请求写入 TFT 字符串到指定地址 */
    void requestWriteTftText(uint16_t addr, const std::string& text);
    /** 请求写入 TFT 浮点数到指定地址 */
    void requestWriteTftFloat(uint16_t addr, float value);

    OtaManager(const OtaManager&) = delete;
    OtaManager& operator=(const OtaManager&) = delete;
    // 处理tft触屏事件：检查更新、重试检查更新、现在更新、立即安装
    void handleTftTouchEvent(int16_t reg);

private:
    OtaManager() = default;

    void onOtaPackageInfo(const interface::msg::OtaPackageInfo::SharedPtr msg);
    void onDownloadMessage(const interface::msg::OtaDownloadMsg::SharedPtr msg);
    void onDownloadProgress(const interface::msg::OtaDownloadProgress::SharedPtr msg);
    void onUpgradeMessage(const interface::msg::OtaUpgradeMsg::SharedPtr msg);
    void onUpgradeProgress(const interface::msg::OtaUpgradeProgress::SharedPtr msg);
    void handlePackageCheckResponse(const interface::srv::PackageCheck::Response::SharedPtr resp);

    // 检查更新
    void checkUpdate();
    // 现在更新
    void nowUpdate();
    // 立即安装
    void installNow();
    // 判断ota界面调用
    void checkOtaInterface();
private:
    OtaStatus ota_upgrade_status_;
    mutable std::mutex mutex_;
    rclcpp::Node* node_{nullptr};
    rclcpp::Subscription<interface::msg::OtaPackageInfo>::SharedPtr ota_package_info_sub_;
    // 下载失败信息
    rclcpp::Subscription<interface::msg::OtaDownloadMsg>::SharedPtr download_message_sub_;
    // 下载进度
    rclcpp::Subscription<interface::msg::OtaDownloadProgress>::SharedPtr download_progress_sub_;
    // 升级失败信息
    rclcpp::Subscription<interface::msg::OtaUpgradeMsg>::SharedPtr upgrade_message_sub_;
    // 升级进度
    rclcpp::Subscription<interface::msg::OtaUpgradeProgress>::SharedPtr upgrade_progress_sub_;

    rclcpp::Client<interface::srv::PackageCheck>::SharedPtr package_check_client_;
    rclcpp::Client<interface::srv::PackageDownload>::SharedPtr package_download_client_;
    rclcpp::Client<interface::srv::PackageUpgrade>::SharedPtr package_upgrade_client_;

    /** 启动 30 秒后执行一次 package_check 请求（一次性定时器） */
    rclcpp::TimerBase::SharedPtr package_check_delay_timer_;

    /** 页面切换回调：TftService 注册 page_dialog_manager_->switchToPage */
    std::function<void(int)> on_switch_page_;
    /** 显示弹窗回调：TftService 注册 page_dialog_manager_->showDialog */
    std::function<void(const std::string& dialog_id, int page_id)> on_show_dialog_;
    /** TFT 数值写入回调 */
    std::function<void(uint16_t, uint16_t)> on_write_tft_value_;
    /** TFT 字符串写入回调 */
    std::function<void(uint16_t, const std::string&)> on_write_tft_text_;
    /** TFT 浮点数写入回调 */
    std::function<void(uint16_t, float)> on_write_tft_float_;

    /** 页面 ID：OTA 界面 */
    int ota_page_id_{LATEST_VERSION_PAGE_ID};
    /** 是否已调用过 ota 界面 */
    bool is_called_ota_interface_{false};
};

}  // namespace tft_service
