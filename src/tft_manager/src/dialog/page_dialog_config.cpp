/**
 * @file page_dialog_config.cpp
 * @brief 从 YAML 加载页面、弹窗、Toast 配置
 */

#include "page_dialog_config.h"
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace tft_service {

namespace {

TouchActionType parseAction(const std::string& s) {
    if (s == "return_prev") return TouchActionType::ReturnPrev;
    if (s == "retry") return TouchActionType::Retry;
    return TouchActionType::Close;
}

}  // namespace

bool PageDialogToastConfig::load(const std::string& yaml_path) {
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        pages.clear();
        dialogs.clear();
        toasts.clear();
        components.clear();

        if (root["pages"]) {
            for (const auto& n : root["pages"]) {
                PageConfig p;
                p.id = n["id"].as<int>(0);
                p.name = n["name"].as<std::string>("");
                if (n["touch_events"]) {
                    for (const auto& te : n["touch_events"]) {
                        uint16_t reg = static_cast<uint16_t>(te["register"].as<int>(0));
                        int next = te["next_page"].as<int>(0);
                        p.touch_to_page.emplace_back(reg, next);
                    }
                }
                pages.push_back(p);
            }
        }

        if (root["dialogs"]) {
            for (const auto& n : root["dialogs"]) {
                DialogConfig d;
                d.id = n["id"].as<std::string>("");
                d.tft_popup_id = static_cast<uint16_t>(n["tft_popup_id"].as<int>(0));
                if (n["pages"]) {
                    for (const auto& p : n["pages"]) d.pages.push_back(p.as<int>());
                }
                if (n["touch_actions"]) {
                    for (const auto& a : n["touch_actions"]) {
                        TouchActionConfig tac;
                        tac.register_addr = static_cast<uint16_t>(a["register"].as<int>(0));
                        tac.action = parseAction(a["action"].as<std::string>("close"));
                        d.touch_actions.push_back(tac);
                    }
                }
                if (n["title_addr"]) d.title_addr = static_cast<uint16_t>(n["title_addr"].as<int>(0));
                d.title = n["title"].as<std::string>("");
                d.title_en = n["title_en"].as<std::string>("");
                if (n["body_addr"]) d.body_addr = static_cast<uint16_t>(n["body_addr"].as<int>(0));
                if (n["body_texts"]) {
                    for (const auto& bt : n["body_texts"]) {
                        d.body_texts.push_back(bt.as<std::string>(""));
                    }
                }
                if (n["body_texts_en"]) {
                    for (const auto& bt : n["body_texts_en"]) {
                        d.body_texts_en.push_back(bt.as<std::string>(""));
                    }
                }
                dialogs.push_back(d);
            }
        }

        if (root["toasts"]) {
            for (const auto& n : root["toasts"]) {
                ToastConfig t;
                t.id = n["id"].as<std::string>("");
                t.tft_popup_id = static_cast<uint16_t>(n["tft_popup_id"].as<int>(0));
                if (n["pages"]) {
                    for (const auto& p : n["pages"]) t.pages.push_back(p.as<int>());
                }
                t.duration_ms = n["duration_ms"].as<int>(2000);
                toasts.push_back(t);
            }
        }

        if (root["components"]) {
            for (const auto& n : root["components"]) {
                ComponentConfig c;
                c.id = n["id"].as<std::string>("");
                c.address = static_cast<uint16_t>(n["address"].as<int>(0));
                std::string type_str = n["type"].as<std::string>("number");
                c.type = (type_str == "string") ? ComponentType::String : ComponentType::Number;
                if (n["pages"]) {
                    for (const auto& p : n["pages"]) c.pages.push_back(p.as<int>());
                }
                components.push_back(c);
            }
        }
        return true;
    } catch (const YAML::Exception&) {
        return false;
    }
}

}  // namespace tft_service
