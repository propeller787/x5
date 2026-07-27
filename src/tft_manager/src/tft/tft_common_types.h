/**
 * @file tft_common_types.h
 * @brief TFT 模块通用类型：语言、界面类型、时间结构体等，与 ModeStatus 解耦便于复用。
 */

#pragma once

namespace tft_service {

/** 语言/地区类型 */
enum class LINGUISTIC_TYPE {
    CN,  // 中文（中国）
    EN,  // 英文（美国）
    DE,  // 德文（德国）
    RU,  // 俄文（俄国）
    JP,  // 日语（日本）
    FR   // 法语（法国）
};

/** TFT 界面类型 */
enum class TFT_TYPE {
    STANDARD,              // 标准
    CRUISE,                // 巡航
    AUTOMATIC_RETRACTION,  // 回充
    OBSTACLE_AVOIDANCE,    // 避障
    VIDEO,                 // 视频
    ALARM,                  // 预警
    SETTING,                // 设置
    OTA
};

/** 年/月/日/时/分/秒/星期，用于 TFT 时间下发 */
struct TFT_time {
    int year{0};
    int month{0};
    int day{0};
    int hour{0};
    int minute{0};
    int second{0};
    int week{0};
};

}  // namespace tft_service
