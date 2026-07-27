/**
 * @file ota_manager.cpp
 * @brief OTA 功能独立管理类实现。
 */

#include "ota_manager.h"
#include "tft_user_config.h"
#include "control/define.h"
#include <utils/topic.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace tft_service {

OtaManager& OtaManager::instance() {
    static OtaManager inst;
    return inst;
}

void OtaManager::init(rclcpp::Node* node) {
    if (!node) return;
    std::lock_guard<std::mutex> lock(mutex_);
    node_ = node;
    auto qos = rclcpp::QoS(10);
    ota_package_info_sub_ = node_->create_subscription<interface::msg::OtaPackageInfo>(
        otaPackageInfoTopic, qos,
        [this](const interface::msg::OtaPackageInfo::SharedPtr msg) { onOtaPackageInfo(msg); });
    download_message_sub_ = node_->create_subscription<interface::msg::OtaDownloadMsg>(
        otaDownloadMessageTopic, qos,
        [this](const interface::msg::OtaDownloadMsg::SharedPtr msg) { onDownloadMessage(msg); });
    download_progress_sub_ = node_->create_subscription<interface::msg::OtaDownloadProgress>(
        otaDownloadProgressTopic, qos,
        [this](const interface::msg::OtaDownloadProgress::SharedPtr msg) { onDownloadProgress(msg); });  
    upgrade_message_sub_ = node_->create_subscription<interface::msg::OtaUpgradeMsg>(
        otaUpgradeMessageTopic, qos,
        [this](const interface::msg::OtaUpgradeMsg::SharedPtr msg) { onUpgradeMessage(msg); });
    upgrade_progress_sub_ = node_->create_subscription<interface::msg::OtaUpgradeProgress>(
        otaUpgradeProgressTopic, qos,
        [this](const interface::msg::OtaUpgradeProgress::SharedPtr msg) { onUpgradeProgress(msg); });

    package_check_client_ = node_->create_client<interface::srv::PackageCheck>(UpgradePackageCheckService);
    package_download_client_ = node_->create_client<interface::srv::PackageDownload>(UpgradePackageDownloadService);
    package_upgrade_client_ = node_->create_client<interface::srv::PackageUpgrade>(UpgradePackageUpgradeService);

    using namespace std::chrono_literals;
    package_check_delay_timer_ = node_->create_wall_timer(10s, [this]() {
        if (!package_check_client_ || !package_check_client_->service_is_ready()) {
            RCLCPP_WARN(rclcpp::get_logger("OtaManager"), "PackageCheck service not ready, skip delayed check");
            interface::srv::PackageCheck::Response::SharedPtr resp = std::make_shared<interface::srv::PackageCheck::Response>();
            resp->code = 3; resp->message = "service not ready";
            handlePackageCheckResponse(resp);
            return;
        }
        auto req = std::make_shared<interface::srv::PackageCheck::Request>();
        RCLCPP_INFO(rclcpp::get_logger("OtaManager"), "PackageCheck delayed: send request");
        package_check_client_->async_send_request(req, [this](rclcpp::Client<interface::srv::PackageCheck>::SharedFuture future) {
            interface::srv::PackageCheck::Response::SharedPtr resp = std::make_shared<interface::srv::PackageCheck::Response>();
            try {
                resp = future.get();
            } catch (const std::exception& e) {
                resp->code = 3; resp->message = "network check failed";
                RCLCPP_WARN(rclcpp::get_logger("OtaManager"), "network check failed: %s", e.what());
            }
            std::lock_guard<std::mutex> lock(mutex_);
            handlePackageCheckResponse(resp);
            package_check_delay_timer_->cancel();
        });
    });

    RCLCPP_INFO(rclcpp::get_logger("OtaManager"), "OtaManager initialized");
}

OtaStatus OtaManager::getStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ota_upgrade_status_;
}

void OtaManager::requestSwitchPage(int page_id) {
    if (on_switch_page_) {
        RCLCPP_INFO(rclcpp::get_logger("OtaManager"), "requestSwitchPage(%d): callback registered", page_id);
        on_switch_page_(page_id);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("OtaManager"), "requestSwitchPage(%d): callback not registered", page_id);
    }
}

void OtaManager::requestWriteTftValue(uint16_t addr, uint16_t value) {
    if (on_write_tft_value_) {
        on_write_tft_value_(addr, value);
    }
}

void OtaManager::requestWriteTftText(uint16_t addr, const std::string& text) {
    if (on_write_tft_text_) {
        on_write_tft_text_(addr, text);
    }
}

void OtaManager::requestWriteTftFloat(uint16_t addr, float value) {
    if (on_write_tft_float_) {
        on_write_tft_float_(addr, value);
    }
}

void OtaManager::onOtaPackageInfo(const interface::msg::OtaPackageInfo::SharedPtr msg) {
    if (!msg) return;
    std::lock_guard<std::mutex> lock(mutex_);
    ota_upgrade_status_.new_version = msg->new_version;
    ota_upgrade_status_.new_description = msg->new_description;
    // 字节转换为MB / KB / B
    std::string size_str;
    if (msg->new_size / 1024 / 1024 > 0) {
        size_str = std::to_string(msg->new_size / 1024 / 1024) + "MB";
    } else if (msg->new_size / 1024 > 0) {
        size_str = std::to_string(msg->new_size / 1024) + "KB";
    } else {
        size_str = std::to_string(msg->new_size) + "B";
    }
    ota_upgrade_status_.new_size = msg->new_size;
    ota_upgrade_status_.is_downloading = true;
    ota_page_id_ = OTA_DOWNLOAD_PAGE_ID;
    bool is_en = (TftUserConfig::instance().getLanguage() == Language::EN);
    if (is_en) {
        requestWriteTftText(DOWNLOAD_VERSION_ADDRESS, ota_upgrade_status_.new_version + " (To be updated)");
    } else {
        requestWriteTftText(DOWNLOAD_VERSION_ADDRESS, ota_upgrade_status_.new_version + " (待更新)");
    }
    requestWriteTftText(DOWNLOAD_VERSION_SIZE_ADDRESS, size_str);
    // ota有新包更新，页面变更
    TftUserConfig::instance().setOtaNewPackageFlag(true);
    
}

void OtaManager::onDownloadMessage(const interface::msg::OtaDownloadMsg::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    // int32_t code = msg->error_code;
    // 执行弹窗 下载失败
    if (on_show_dialog_) {
        on_show_dialog_("ota_download_failure", 0);
        ota_upgrade_status_.message = msg->msg;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("OtaManager"), "onDownloadMessage: show dialog callback not registered");
    }
    ota_page_id_ = OTA_DOWNLOAD_PAGE_ID;
}

void OtaManager::onDownloadProgress(const interface::msg::OtaDownloadProgress::SharedPtr msg) {
    if (!msg) return;
    std::lock_guard<std::mutex> lock(mutex_);
    ota_upgrade_status_.download_progress = msg->value;
    ota_upgrade_status_.is_downloading = (msg->value > 0 && msg->value < 100);
    requestWriteTftValue(DOWNLOAD_PROGRESS_ADDRESS, (int)(msg->value * 100 / 100));
    if (ota_page_id_ == OTA_DOWNLOAD_PAGE_ID && ota_upgrade_status_.is_downloading) {
        ota_page_id_ = OTA_DOWNLOAD_PROGRESS_PAGE_ID;
        checkOtaInterface();
    }
    if (msg->value >= 100.0) {
        // 下载完成后，只有在下载界面才切换到安装界面
        ota_upgrade_status_.is_downloading = false;
        ota_upgrade_status_.is_upgrading = true;
        if (ota_page_id_ == OTA_DOWNLOAD_PROGRESS_PAGE_ID) {
            ota_page_id_ = OTA_INSTALL_PAGE_ID;
            checkOtaInterface();
        }
        ota_page_id_ = OTA_INSTALL_PAGE_ID;
    }
}

void OtaManager::onUpgradeMessage(const interface::msg::OtaUpgradeMsg::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    int32_t code = msg->error_code;
    std::string error_msg = msg->msg;
    if (ota_page_id_ == OTA_INSTALL_PAGE_ID) {
        if (code == 1 || code == 2) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (on_show_dialog_) {
                on_show_dialog_("ota_upgrade_failure", 1); // value == 1含义为电量低于80%或者车速不为0
            }
        }
    } else if (ota_page_id_ == OTA_INSTALL_PROGRESS_PAGE_ID) {
        // 切换页面
        ota_page_id_ = OTA_INSTALL_FAILURE_PAGE_ID;
        requestWriteTftText(FAILURE_PAGE_TC_CONTENT_ADDRESS, TftUserConfig::instance().getOtaUpgradeFailureDescription());
        checkOtaInterface();
        // 点击我已知晓后，从设置页再进来回到检查更新界面
        ota_page_id_ = CHECK_UPDATE_PAGE_ID;
    }
    ota_upgrade_status_.message = error_msg;
}
/**
 * @brief 升级进度回调
 * @param msg 升级进度消息，0.0~100.0
 */
void OtaManager::onUpgradeProgress(const interface::msg::OtaUpgradeProgress::SharedPtr msg) {
    if (!msg) return;
    std::lock_guard<std::mutex> lock(mutex_);
    double upgrade_progress = msg->value;
    ota_upgrade_status_.upgrade_progress = upgrade_progress;
    ota_upgrade_status_.is_upgrading = (upgrade_progress > 0 && upgrade_progress < 100);
    requestWriteTftValue(UPGRADE_PROGRESS_BAR_ADDRESS, (int)(upgrade_progress * 100 / 100));
    if (ota_page_id_ == OTA_INSTALL_PAGE_ID && ota_upgrade_status_.is_upgrading) {
        ota_page_id_ = OTA_INSTALL_PROGRESS_PAGE_ID;
        checkOtaInterface();
    }
    if (upgrade_progress >= 100.0) {
        std::string version = msg->new_version;
        ota_upgrade_status_.upgrade_progress = upgrade_progress;
        ota_upgrade_status_.is_upgrading = false;
        ota_upgrade_status_.is_downloading = false;
        ota_page_id_ = OTA_INSTALL_SUCCESS_PAGE_ID;
        checkOtaInterface();
        // 更新当前版本号
        TftUserConfig::instance().setCurrentVersion(version);
        requestWriteTftText(CURRENT_VERSION_ADDRESS, version);
        // 当升级完成后，设置界面有更新提示界面变更为无可用更新界面
        TftUserConfig::instance().setOtaNewPackageFlag(false);
        // 当升级完成时，再记录当前版本号
        TftUserConfig::instance().setCurrentVersion(version);
        // 安装完成后，再次进入时恢复为已是最新版本界面
        ota_page_id_ = LATEST_VERSION_PAGE_ID;
    }
}

void OtaManager::handlePackageCheckResponse(const interface::srv::PackageCheck::Response::SharedPtr resp) {
    if (!resp){
        RCLCPP_WARN(rclcpp::get_logger("OtaManager"), "handlePackageCheckResponse: response is null");
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("OtaManager"), "handlePackageCheckResponse: response received");
    RCLCPP_INFO(rclcpp::get_logger("OtaManager"), "handlePackageCheckResponse: code=%d", resp->code);
    RCLCPP_INFO(rclcpp::get_logger("OtaManager"), "handlePackageCheckResponse: new_version=%s", resp->new_version.c_str());
    RCLCPP_INFO(rclcpp::get_logger("OtaManager"), "handlePackageCheckResponse: history_version=%s", resp->history_version.c_str());
    RCLCPP_INFO(rclcpp::get_logger("OtaManager"), "handlePackageCheckResponse: new_description=%s", resp->new_description.c_str());
    RCLCPP_INFO(rclcpp::get_logger("OtaManager"), "handlePackageCheckResponse: new_size=%d", resp->new_size);
    RCLCPP_INFO(rclcpp::get_logger("OtaManager"), "handlePackageCheckResponse: message=%s", resp->message.c_str());
    switch (resp->code) {
        case 0: {
            ota_upgrade_status_.is_downloading = true;
            ota_page_id_ = OTA_DOWNLOAD_PAGE_ID;
            ota_upgrade_status_.new_version = resp->new_version;
            ota_upgrade_status_.history_version = resp->history_version;
            ota_upgrade_status_.new_description = resp->new_description;
            // 字节转换为MB / KB / B
            std::string new_size_str;
            if (resp->new_size > 1024 * 1024) {
                new_size_str = std::to_string(resp->new_size / 1024 / 1024) + "MB";
            } else if (resp->new_size > 1024) {
                new_size_str = std::to_string(resp->new_size / 1024) + "KB";
            } else {
                new_size_str = std::to_string(resp->new_size) + "B";
            }
            ota_upgrade_status_.message = resp->message;
            TftUserConfig::instance().setCurrentVersion(ota_upgrade_status_.history_version);
            bool is_en = (TftUserConfig::instance().getLanguage() == Language::EN);
            if (is_en) {
                requestWriteTftText(DOWNLOAD_VERSION_ADDRESS, ota_upgrade_status_.new_version + " (To be updated)");
            }
            else {
                requestWriteTftText(DOWNLOAD_VERSION_ADDRESS, ota_upgrade_status_.new_version + " (待更新)");
            }
            requestWriteTftText(DOWNLOAD_VERSION_SIZE_ADDRESS, new_size_str);
            requestWriteTftText(CURRENT_VERSION_ADDRESS, ota_upgrade_status_.history_version);
            // 如果有更新的话，设置界面显示有更新
            TftUserConfig::instance().setOtaNewPackageFlag(true);
            RCLCPP_INFO(rclcpp::get_logger("OtaManager"), "PackageCheck delayed: code=%d new=%s", resp->code, resp->new_version.c_str());
            break;
        }
        case 1: {
            ota_upgrade_status_.is_downloading = false;
            ota_upgrade_status_.is_upgrading = false;
            ota_page_id_ = LATEST_VERSION_PAGE_ID;
            TftUserConfig::instance().setOtaNewPackageFlag(false);
            break;
        }
        case 2: {
            ota_upgrade_status_.is_downloading = false;
            ota_upgrade_status_.is_upgrading = true;
            ota_page_id_ = OTA_INSTALL_PAGE_ID;
            // 如果有更新的话，设置界面显示有更新，直到安装完成
            TftUserConfig::instance().setOtaNewPackageFlag(true);
            break;
        }
        case 3: {
            ota_upgrade_status_.message = resp->message;
            ota_page_id_ = UPDATE_FAILURE_PAGE_ID;
            TftUserConfig::instance().setOtaNewPackageFlag(false);
            break;
        }
        default: {
            break;
        }
    }
}

void OtaManager::handleTftTouchEvent(int16_t reg) {
    switch (reg) {
        case CHECK_UPDATE_ADDRESS:
            checkUpdate();
            break;
        case NOW_UPDATE_ADDRESS:
            nowUpdate();
            break;
        case INSTALL_NOW_ADDRESS:
            installNow();
            break;
        case TOUCH_EVENT_SETTING_PAGE_DEVICE_UPGRADE:
            // 判断ota界面调用
            RCLCPP_INFO(rclcpp::get_logger("OtaManager"), "handleTftTouchEvent: ota interface called");
            is_called_ota_interface_ = true;
            checkOtaInterface();
            break;
        case TOUCH_EVENT_LATEST_VERSION_PAGE_RETURN_PREVIOUS:
            is_called_ota_interface_ = false;
            break;
        case TOUCH_EVENT_CHECK_UPDATE_PAGE_RETURN_PREVIOUS:
            is_called_ota_interface_ = false;
            break;
        case TOUCH_EVENT_UPDATE_FAILURE_PAGE_RETURN_PREVIOUS:
            is_called_ota_interface_ = false;
            break;
        case TOUCH_EVENT_DOWNLOAD_PAGE_RETURN_PREVIOUS:
            is_called_ota_interface_ = false;
            break;
        case TOUCH_EVENT_INSTALL_PAGE_RETURN_PREVIOUS:
            is_called_ota_interface_ = false;
            break;
        case TOUCH_EVENT_INSTALL_SUCCESS_PAGE_IMMEDIATE_EXPERIENCE:
            ota_page_id_ = LATEST_VERSION_PAGE_ID;
            requestWriteTftText(CURRENT_VERSION_ADDRESS, ota_upgrade_status_.new_version);
            is_called_ota_interface_ = false;
            break;
        default:
            break;
    }
}

void OtaManager::checkUpdate() {
    if (!package_check_client_ || !package_check_client_->service_is_ready()) {
        RCLCPP_WARN(rclcpp::get_logger("OtaManager"), "PackageCheck service not ready, skip check update");
        interface::srv::PackageCheck::Response::SharedPtr resp = std::make_shared<interface::srv::PackageCheck::Response>();
        resp->code = 3; resp->message = "service not ready";
        handlePackageCheckResponse(resp);
        checkOtaInterface();
        return;
    }
    auto req = std::make_shared<interface::srv::PackageCheck::Request>();
    package_check_client_->async_send_request(req, [this](rclcpp::Client<interface::srv::PackageCheck>::SharedFuture future) {
        interface::srv::PackageCheck::Response::SharedPtr resp = std::make_shared<interface::srv::PackageCheck::Response>();
        try {
            resp = future.get();
        } catch (const std::exception& e) {
            resp->code = 3; resp->message = "network check failed";
            RCLCPP_WARN(rclcpp::get_logger("OtaManager"), "network check failed: %s", e.what());
        }
        std::lock_guard<std::mutex> lock(mutex_);
        handlePackageCheckResponse(resp);
        checkOtaInterface();
    });
}

void OtaManager::nowUpdate() {
    if (!package_download_client_ || !package_download_client_->service_is_ready()) {
        RCLCPP_WARN(rclcpp::get_logger("OtaManager"), "PackageDownload service not ready, skip now update");
        return;
    }
    auto req = std::make_shared<interface::srv::PackageDownload::Request>();
    package_download_client_->async_send_request(req, [this](rclcpp::Client<interface::srv::PackageDownload>::SharedFuture future) {
        try {
            auto resp = future.get();
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("OtaManager"), "PackageDownload failed: %s", e.what());
        }
    });
}

void OtaManager::installNow() {
    if (!package_upgrade_client_ || !package_upgrade_client_->service_is_ready()) {
        RCLCPP_WARN(rclcpp::get_logger("OtaManager"), "PackageUpgrade service not ready, skip install now");
        return;
    }
    auto req = std::make_shared<interface::srv::PackageUpgrade::Request>();
    RCLCPP_INFO(rclcpp::get_logger("OtaManager"), "installNow: send request");
    package_upgrade_client_->async_send_request(req, [this](rclcpp::Client<interface::srv::PackageUpgrade>::SharedFuture future) {
        try {
            auto resp = future.get();
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("OtaManager"), "PackageUpgrade failed: %s", e.what());
        }
    });
}

void OtaManager::checkOtaInterface() {
    if (!is_called_ota_interface_) {
        return;
    }
    requestSwitchPage(ota_page_id_);
}
}  // namespace tft_service
