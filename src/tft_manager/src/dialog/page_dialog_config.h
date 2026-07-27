/**
 * @file page_dialog_config.h
 * @brief 页面、弹窗、Toast 配置结构（从 YAML 加载，数量与绑定可扩展）
 */

#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace tft_service {

/** 触屏动作类型 */
enum class TouchActionType { ReturnPrev, Retry, Close };

/** 单条触屏动作配置 */
struct TouchActionConfig {
    uint16_t register_addr{0};
    TouchActionType action{TouchActionType::Close};
};

/** 弹窗配置：需点击触屏才消失
 *  标题/正文各一个 TFT 地址；body_texts 为不同场景的正文，showDialog(id, body_index) 指定使用哪个
 *  title_en / body_texts_en 为可选英文字段；缺失时自动 fallback 到中文 */
struct DialogConfig {
    std::string id;
    uint16_t tft_popup_id{0};
    std::vector<int> pages;  // 在哪些页面可用
    std::vector<TouchActionConfig> touch_actions;
    uint16_t title_addr{0};                       ///< 标题 TFT 地址
    std::string title;                            ///< 中文标题（原字段，兼容保留）
    std::string title_en;                         ///< 英文标题（可选，缺失则 fallback 到 title）
    uint16_t body_addr{0};                        ///< 正文 TFT 地址
    std::vector<std::string> body_texts;          ///< 中文正文列表（原字段）
    std::vector<std::string> body_texts_en;       ///< 英文正文列表（可选，缺失则 fallback 到 body_texts）

    /** 按 is_english 返回标题：EN 有内容则用 EN，否则 fallback 到中文 */
    const std::string& getTitle(bool is_english) const {
        if (is_english && !title_en.empty()) return title_en;
        return title;
    }
    /** 按 is_english 与 idx 返回正文：EN 列表有内容则用 EN，否则 fallback 到中文 */
    const std::string& getBodyText(bool is_english, size_t idx) const {
        const auto& texts = (is_english && !body_texts_en.empty()) ? body_texts_en : body_texts;
        if (texts.empty()) return title;
        return texts[idx < texts.size() ? idx : 0];
    }
};

/** Toast 配置：定时自动消失 */
struct ToastConfig {
    std::string id;
    uint16_t tft_popup_id{0};
    std::vector<int> pages;
    int duration_ms{2000};
};

/** 页面配置 */
struct PageConfig {
    int id{0};
    std::string name;
    /** 页面内触屏事件：切换至固定页码 (register -> next_page_id) */
    std::vector<std::pair<uint16_t, int>> touch_to_page;
};

/** 组件类型：数字 或 字符串 */
enum class ComponentType { Number, String };

/** 组件配置：不同页面中固定组件地址，用于更新数字或字符串 */
struct ComponentConfig {
    std::string id;
    uint16_t address{0};     // 如 0x1000
    ComponentType type{ComponentType::Number};
    std::vector<int> pages;  // 在哪些页面可用
};

/** 完整配置 */
struct PageDialogToastConfig {
    std::vector<PageConfig> pages;
    std::vector<DialogConfig> dialogs;
    std::vector<ToastConfig> toasts;
    std::vector<ComponentConfig> components;

    bool load(const std::string& yaml_path);
};

}  // namespace tft_service
