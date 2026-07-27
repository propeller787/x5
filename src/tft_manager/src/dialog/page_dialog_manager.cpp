/**
 * @file page_dialog_manager.cpp
 * @brief 页面/弹窗/Toast 管理器实现
 */

#include "page_dialog_manager.h"
#include "control/define.h"
#include "tft_user_config.h"
#include "tft/tft_module.h"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>

namespace tft_service {

struct PageDialogManager::Impl {
    rclcpp::TimerBase::SharedPtr toast_timer;
    rclcpp::Node* node{nullptr};
};

PageDialogManager::PageDialogManager(TftModule* tft) : tft_(tft) {
    impl_ = std::make_unique<Impl>();
}

PageDialogManager::~PageDialogManager() = default;

bool PageDialogManager::loadConfig(const std::string& yaml_path) {
    return config_.load(yaml_path);
}

void PageDialogManager::setCurrentPage(int page_id) {
    current_page_id_ = page_id;
}

void PageDialogManager::switchToPage(int page_id) {
    // 弹窗未关闭时，禁止从主界面切到其他页面，避免用户跳页绕过弹窗处理
    if (current_page_id_ == MAIN_PAGE_ID &&
        page_id != MAIN_PAGE_ID &&
        !current_dialog_id_.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("PageDialogManager"),
                    "Reject page switch while dialog is showing: from=%d to=%d dialog=%s",
                    current_page_id_, page_id, current_dialog_id_.c_str());
        return;
    }

    int from = current_page_id_;
    current_page_id_ = page_id;
    if (tft_ && tft_->isConnected()) {
        tft_->sendDcBusWrite(STATION_ADDRESS, PAGE_ID_ADDRESS, static_cast<uint16_t>(page_id));
        sendStringToAddress(page_id);
    }
    if (on_page_switch_) on_page_switch_(from, page_id);
}

int PageDialogManager::getInitialPageOnStartup() {
    int state = TftUserConfig::instance().getAppBindPageState();
    if(state != 0) {
        return MAIN_PAGE_ID;
    }else{
        int download_state = TftUserConfig::instance().getAppDownloadPageState();
        if(download_state == 1) {
            return MAIN_PAGE_ID;
        }else if(download_state == 2) {
            return VEHICLE_BINDING_QR_CODE_PAGE_ID;
        }else if(download_state == 0) {
            return APP_DOWNLOAD_QR_CODE_PAGE_ID;
        }
    }
    // Bug4 fix: download_state 为未知值时兜底返回主页面，避免函数无 return 产生 UB
    return MAIN_PAGE_ID;
}

void PageDialogManager::showDialog(const std::string& dialog_id, int body_index) {
    // 当有弹窗未消失时，再次触发任何弹窗直接忽略
    if (!current_dialog_id_.empty()) return;
    const DialogConfig* d = findDialog(dialog_id);
    if (!d || !isDialogAvailableOnPage(*d)) {
        RCLCPP_ERROR(rclcpp::get_logger("PageDialogManager"), "不在可用界面内");
        return;
    }

    current_body_index_ = body_index;  // 记录，供 refreshCurrentDialogTexts 使用

    if (tft_ && tft_->isConnected()) {
        bool is_en = (TftUserConfig::instance().getLanguage() == Language::EN);
        tft_->sendDcBusWrite(STATION_ADDRESS, PHYSICAL_BUTTON_FAILURE_OPEN_ADDRESS, d->tft_popup_id);
        if (d->title_addr) {
            const std::string& title_text = d->getTitle(is_en);
            if (!title_text.empty()) {
                tft_->sendDcBusTextWrite(STATION_ADDRESS, d->title_addr, title_text);
            }
        }
        if (d->body_addr && !d->body_texts.empty()) {
            size_t idx = (body_index >= 0 && static_cast<size_t>(body_index) < d->body_texts.size())
                        ? static_cast<size_t>(body_index) : 0;
            const std::string& body = d->getBodyText(is_en, idx);
            if (!body.empty()) {
                tft_->sendDcBusTextWrite(STATION_ADDRESS, d->body_addr, body);
            }
        }
    }
    current_dialog_id_ = dialog_id;
}

void PageDialogManager::refreshCurrentDialogTexts() {
    if (current_dialog_id_.empty()) return;
    const DialogConfig* d = findDialog(current_dialog_id_);
    if (!d || !tft_ || !tft_->isConnected()) return;

    bool is_en = (TftUserConfig::instance().getLanguage() == Language::EN);
    if (d->title_addr) {
        const std::string& title_text = d->getTitle(is_en);
        if (!title_text.empty()) {
            tft_->sendDcBusTextWrite(STATION_ADDRESS, d->title_addr, title_text);
        }
    }
    if (d->body_addr && !d->body_texts.empty()) {
        size_t idx = static_cast<size_t>(
            (current_body_index_ >= 0 && static_cast<size_t>(current_body_index_) < d->body_texts.size())
            ? current_body_index_ : 0);
        const std::string& body = d->getBodyText(is_en, idx);
        if (!body.empty()) {
            tft_->sendDcBusTextWrite(STATION_ADDRESS, d->body_addr, body);
        }
    }
}

void PageDialogManager::closeDialog() {
    if (current_dialog_id_.empty()) return;
    std::string dialog_id = current_dialog_id_;
    current_dialog_id_.clear();
    if (tft_ && tft_->isConnected()) {
        // parked_failure 等物理按键失效弹窗：发送 0 到 0x1109 使 TFT 弹窗消失
        RCLCPP_DEBUG(rclcpp::get_logger("PageDialogManager"), "dialog_id: %s", dialog_id.c_str());
        if (dialog_id == "parked_failure"
            || dialog_id == "remote_control_failure"
            || dialog_id == "obstacle_avoidance_block"
            || dialog_id == "cruise_block"
            || dialog_id == "cruise_exit"
            || dialog_id == "recharge_block"
            || dialog_id == "ota_upgrade_failure"
            || dialog_id == "ota_download_failure"
            || dialog_id == "recharge_exit")
        {
            tft_->sendDcBusWrite(STATION_ADDRESS, PHYSICAL_BUTTON_FAILURE_OPEN_ADDRESS, 0);
        }
        // 切回底层页面
        //tft_->sendDcBusWrite(STATION_ADDRESS, PAGE_ID_ADDRESS, static_cast<uint16_t>(current_page_id_));
    }
}

void PageDialogManager::showToast(const std::string& toast_id) {
    const ToastConfig* t = findToast(toast_id);
    if (!t || !isToastAvailableOnPage(*t)) return;
    if (tft_ && tft_->isConnected()) {
        tft_->sendDcBusWrite(STATION_ADDRESS, TOAST_POPUP_ADDRESS, t->tft_popup_id);
    }
    toast_timer_active_ = true;
    if (impl_->node) {
        if (impl_->toast_timer) impl_->toast_timer->cancel();
        impl_->toast_timer = impl_->node->create_wall_timer(
            std::chrono::milliseconds(t->duration_ms),
            [this]() { dismissToast(); });
    }
}

void PageDialogManager::setNode(rclcpp::Node* node) {
    impl_->node = node;
}

void PageDialogManager::dismissToast() {
    toast_timer_active_ = false;
    if (impl_->toast_timer) impl_->toast_timer->cancel();
    if (tft_ && tft_->isConnected()) {
        tft_->sendDcBusWrite(STATION_ADDRESS, TOAST_POPUP_ADDRESS, 0);
    }
}

void PageDialogManager::onTouchEvent(uint16_t register_addr) {
    if (!current_dialog_id_.empty()) {
        const DialogConfig* d = findDialog(current_dialog_id_);
        if (d) {
            for (const auto& a : d->touch_actions) {
                if (a.register_addr == register_addr) {
                    std::string dialog_id = current_dialog_id_;  // 先保存，避免 closeDialog 清空后为空
                    
                    if (register_addr == DIALOG_IGNORE_ADDRESS
                        || register_addr == DIALOG_RETRY_ADDRESS_SAFETY_FUNCTION
                        || register_addr == DIALOG_RETRY_ADDRESS_CRUISE_MODE
                        || register_addr == DIALOG_RETRY_ADDRESS_RECHARGE
                        || register_addr == DIALOG_KNOWN_ADDRESS
                        || register_addr == DIALOG_OTA_KNOWN_RECHARGE
                        || register_addr == DIALOG_OTA_DOWNLOAD_RECHARGE) {
                        closeDialog();
                    }
                    if (register_addr == DIALOG_RETRY_ADDRESS_SAFETY_FUNCTION
                        || register_addr == DIALOG_RETRY_ADDRESS_CRUISE_MODE
                        || register_addr == DIALOG_RETRY_ADDRESS_RECHARGE) {
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        executeTouchAction(a.action, dialog_id);
                    }
                    if(dialog_id == "recharge_block" && register_addr == DIALOG_IGNORE_ADDRESS) {
                        //给 1104 发送 0
                        tft_->sendDcBusWrite(STATION_ADDRESS, RECHARGE_UP_SLIDE_BUTTON_ADDRESS, 0);
                    }
                    return;
                }
            }
        }
        return;
    }
    // 无弹窗时：检查当前页面是否有触屏切换
    const PageConfig* p = findPage(current_page_id_);
    if (p) {
        for (const auto& tp : p->touch_to_page) {
            if (tp.first == register_addr && tp.second >= 0) {
                RCLCPP_DEBUG(rclcpp::get_logger("PageDialogManager"), "register_addr: %u switchToPage: %d", register_addr, tp.second);
                switchToPage(tp.second);
                return;
            }
        }
    }
}

void PageDialogManager::executeTouchAction(TouchActionType action, const std::string& dialog_id) {
    if (on_action_) on_action_(action, dialog_id);
    if (on_recharge_retry_) on_recharge_retry_(action, dialog_id);
}

const DialogConfig* PageDialogManager::findDialog(const std::string& id) const {
    for (const auto& d : config_.dialogs) {
        if (d.id == id) return &d;
    }
    return nullptr;
}

const ToastConfig* PageDialogManager::findToast(const std::string& id) const {
    for (const auto& t : config_.toasts) {
        if (t.id == id) return &t;
    }
    return nullptr;
}

bool PageDialogManager::isDialogAvailableOnPage(const DialogConfig& d) const {
    return std::find(d.pages.begin(), d.pages.end(), current_page_id_) != d.pages.end();
}

bool PageDialogManager::isToastAvailableOnPage(const ToastConfig& t) const {
    return std::find(t.pages.begin(), t.pages.end(), current_page_id_) != t.pages.end();
}

bool PageDialogManager::updateComponentNumber(const std::string& component_id, int value) {
    const ComponentConfig* c = findComponent(component_id);
    if (!c || c->type != ComponentType::Number || !isComponentAvailableOnPage(*c)) return false;
    if (!tft_ || !tft_->isConnected()) return false;
    tft_->sendDcBusWrite(STATION_ADDRESS, c->address, static_cast<uint16_t>(value));
    return true;
}

bool PageDialogManager::updateComponentText(const std::string& component_id, const std::string& text) {
    const ComponentConfig* c = findComponent(component_id);
    if (!c || c->type != ComponentType::String || !isComponentAvailableOnPage(*c)) return false;
    if (!tft_ || !tft_->isConnected()) return false;
    tft_->sendDcBusTextWrite(STATION_ADDRESS, c->address, text);
    return true;
}

const ComponentConfig* PageDialogManager::findComponent(const std::string& id) const {
    for (const auto& c : config_.components) {
        if (c.id == id) return &c;
    }
    return nullptr;
}

bool PageDialogManager::isComponentAvailableOnPage(const ComponentConfig& c) const {
    return std::find(c.pages.begin(), c.pages.end(), current_page_id_) != c.pages.end();
}

const PageConfig* PageDialogManager::findPage(int page_id) const {
    for (const auto& p : config_.pages) {
        if (p.id == page_id) return &p;
    }
    return nullptr;
}

void PageDialogManager::sendStringToAddress(const int page_id) {
    RCLCPP_DEBUG(rclcpp::get_logger("PageDialogManager"), "sendStringToAddress: %d", page_id);
    if (tft_ && tft_->isConnected()) {
        switch (page_id) {
            case APP_DOWNLOAD_QR_CODE_PAGE_ID:
                tft_->sendDcBusTextWrite(STATION_ADDRESS, APP_DOWNLOAD_QR_CODE_ADDRESS, TftUserConfig::instance().getAppDownloadCode());
                break;
            case VEHICLE_BINDING_QR_CODE_PAGE_ID:
                tft_->sendDcBusTextWrite(STATION_ADDRESS, VEHICLE_BINDING_QR_CODE_ADDRESS, TftUserConfig::instance().getSettingPageBindingQrCode());
                break;
            case SETTING_PAGE_BINDING_QR_CODE_PAGE_ID:
                tft_->sendDcBusTextWrite(STATION_ADDRESS, SETTING_PAGE_VEHICLE_BINDING_QR_CODE_ADDRESS, TftUserConfig::instance( ).getSettingPageBindingQrCode());
                break;
            case MAIN_PAGE_ID:
                tft_->sendDcBusWrite(STATION_ADDRESS, RECHARGE_UP_SLIDE_BUTTON_ADDRESS, 0);
                break;
            case SETTING_PAGE_ID:
                tft_->sendDcBusTextWrite(STATION_ADDRESS, USER_ID, TftUserConfig::instance().getUsername());
                if(TftUserConfig::instance().getAppBindPageState() == 2) {
                    tft_->sendDcBusWrite(STATION_ADDRESS, USER_AVATAR_ADDRESS, 1);
                }else{
                    tft_->sendDcBusWrite(STATION_ADDRESS, USER_AVATAR_ADDRESS, 0);
                }

                // 这里判断一下ota是否有升级包 
                if(TftUserConfig::instance().getOtaNewPackageFlag()) {
                    tft_->sendDcBusWrite(STATION_ADDRESS, SETTING_PAGE_SHOW_UPDATE_ADDRESS, 1);
                }else{
                    tft_->sendDcBusWrite(STATION_ADDRESS, SETTING_PAGE_SHOW_UPDATE_ADDRESS, 0);
                }

                break;
            default:
                break;
        }
    }
}

}  // namespace tft_service
