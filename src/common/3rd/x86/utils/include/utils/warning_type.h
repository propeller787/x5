#pragma once

// 告警类型 1 地上前向 2 地上左向 3 地上右向 4 地上后向 5 负平面前向 6 负平面左向 7 负平面右向
enum class WarningType {
    FORWARD = 1, // 地上前向
    LEFT = 2, // 地上左向
    RIGHT = 3, // 地上右向
    BACKWARD = 4, // 地上后向
    NEGATIVE_PLANE_FORWARD = 5, // 负平面前向
    NEGATIVE_PLANE_LEFT = 6, // 负平面左向
    NEGATIVE_PLANE_RIGHT = 7, // 负平面右向
};

// 告警等级 1 减速 2 刹停 3 驶离减速区 4 驶离刹停区
enum class WarningLevel {
    NONE = 0, // 正常
    DECELERATION = 1, // 减速
    BRAKE = 2, // 刹停
    LEAVE_DECELERATION_ZONE = 3, // 驶离减速区
    LEAVE_BRAKE_ZONE = 4, // 驶离刹停区
};

// 对应灯带效果
enum class LightEffect {
    FORWARD_WARNING = 1, // 前向减速
    LATERAL_WARNING = 2, // 侧向减速
    BACKWARD_WARNING = 3, // 后向减速
    FORWARD_BRAKE_WARNING = 4, // 前方刹停
    LATERAL_BRAKE_WARNING = 5, // 侧向刹停
    BACKWARD_BRAKE_WARNING = 6, // 后向刹停
    FORWARD_LEAVE_WARNING = 7, // 驶离减速区
    LATERAL_LEAVE_WARNING = 8, // 驶离刹停区
};

// 对应语音效果
enum class VoiceEffect {
    FORWARD_WARNING = 1, // 前向减速
    LATERAL_WARNING = 2, // 侧向减速
    BACKWARD_WARNING = 3, // 后向减速
    FORWARD_BRAKE_WARNING = 4, // 前方刹停
    LATERAL_BRAKE_WARNING = 5, // 侧向刹停
    BACKWARD_BRAKE_WARNING = 6, // 后向刹停
    FORWARD_LEAVE_WARNING = 7, // 驶离减速区
    LATERAL_LEAVE_WARNING = 8, // 驶离刹停区
};

enum class PopupType : int8_t {
    POPUP_TYPE_JOYSTICK = 1, // 请先切换至摇杆模式
    POPUP_TYPE_CLOSE_HAND_BRAKE = 2,  // 请先闭合手动抱闸开关
    POPUP_TYPE_RELEASE_PARKING = 3,  // 请先解除驻车
    POPUP_TYPE_PLUG_OUT_POWER = 4,  // 请先拔出电源
    POPUP_TYPE_LEAVE_STATION = 5,  // 请先将车辆驶离基站
    POPUP_TYPE_FORWARD_JOYSTICK = 6,//请向前拨动摇杆 仅巡航模式下有效
    //由于长时间无操作，定速巡航已退出
    POPUP_TYPE_CRUISE_EXIT = 7,
};