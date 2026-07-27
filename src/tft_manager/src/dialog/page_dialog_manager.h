/**
 * @file page_dialog_manager.h
 * @brief 统一管理页面、弹窗、Toast：配置驱动，数量与绑定可扩展。
 * 弹窗：点击触屏才消失，支持返回/重试/关闭。
 * Toast：定时自动消失。
 */

#pragma once

#include "page_dialog_config.h"
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace rclcpp {
class Node;
}

namespace tft_service {

class TftModule;

/**
 * @brief 页面/弹窗/Toast 管理器：配置驱动，根据当前页面与触屏事件处理弹窗与 Toast。
 */
class PageDialogManager {
public:
    using OnActionCallback = std::function<void(TouchActionType action, const std::string& dialog_id)>;
    using OnPageSwitchCallback = std::function<void(int from_page, int to_page)>;

    explicit PageDialogManager(TftModule* tft);
    ~PageDialogManager();

    /** 加载配置（YAML 路径），失败返回 false */
    bool loadConfig(const std::string& yaml_path);

    /** 设置当前页面 ID（切换页时调用） */
    void setCurrentPage(int page_id);

    /** 切换至指定页面（发送 TFT 命令并更新当前页） */
    void switchToPage(int page_id);

    /** 开机首屏：app_download_page_state==0 返回 APP_DOWNLOAD_QR_CODE_PAGE_ID，否则 MAIN_PAGE_ID */
    static int getInitialPageOnStartup();

    /** 显示弹窗：需点击触屏才消失；body_index 指定使用第几个正文（同一标题可配置多个正文） */
    void showDialog(const std::string& dialog_id, int body_index = 0);
    /** 关闭当前弹窗 */
    void closeDialog();
    /** 语言切换后刷新当前弹窗的标题与正文（无弹窗时无操作） */
    void refreshCurrentDialogTexts();

    /** 显示 Toast：定时自动消失 */
    void showToast(const std::string& toast_id);

    /** 更新组件数值（组件需在当前页面可用） */
    bool updateComponentNumber(const std::string& component_id, int value);

    /** 更新组件字符串（组件需在当前页面可用） */
    bool updateComponentText(const std::string& component_id, const std::string& text);

    /** 处理触屏事件（寄存器地址 0x6100 等）：弹窗时根据配置执行动作并关闭 */
    void onTouchEvent(uint16_t register_addr);

    /** 设置动作回调（返回上一页、重试、关闭时通知业务） */
    void setOnActionCallback(OnActionCallback cb) { on_action_ = std::move(cb); }

    /** 设置回充重试回调 */
    void setOnRechargeRetryCallback(OnActionCallback cb) { on_recharge_retry_ = std::move(cb); }

    /** 设置页面切换回调 */
    void setOnPageSwitchCallback(OnPageSwitchCallback cb) { on_page_switch_ = std::move(cb); }

    /** 设置 ROS 节点（用于 Toast 定时器，可选） */
    void setNode(rclcpp::Node* node);

    /** 当前页面 ID */
    int getCurrentPage() const { return current_page_id_; }

    /** 当前是否正在显示弹窗 */
    bool isDialogShowing() const { return !current_dialog_id_.empty(); }

    /** 当前是否正在显示 Toast */
    bool isToastShowing() const { return toast_timer_active_; }

private:
    void dismissToast();
    void executeTouchAction(TouchActionType action, const std::string& dialog_id);
    const DialogConfig* findDialog(const std::string& id) const;
    const ToastConfig* findToast(const std::string& id) const;
    const ComponentConfig* findComponent(const std::string& id) const;
    bool isDialogAvailableOnPage(const DialogConfig& d) const;
    bool isToastAvailableOnPage(const ToastConfig& t) const;
    bool isComponentAvailableOnPage(const ComponentConfig& c) const;
    const PageConfig* findPage(int page_id) const;
    // 判断页面类型 向对应地址发送字符串
    void sendStringToAddress(const int page_id);
    TftModule* tft_{nullptr};
    PageDialogToastConfig config_;
    int current_page_id_{0};
    std::string current_dialog_id_;
    int current_body_index_{0};   ///< showDialog 时记录的 body_index，供 refresh 使用
    bool toast_timer_active_{false};
    OnActionCallback on_action_;
    OnActionCallback on_recharge_retry_;
    OnPageSwitchCallback on_page_switch_;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace tft_service
