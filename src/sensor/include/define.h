#pragma once

namespace sensor {

enum class PopupType : int8_t {
    POPUP_TYPE_JOYSTICK = 1, // 请先切换至摇杆模式
    POPUP_TYPE_CLOSE_HAND_BRAKE = 2,  // 请先闭合手动抱闸开关
    POPUP_TYPE_RELEASE_PARKING = 3,  // 请先解除驻车
    POPUP_TYPE_PLUG_OUT_POWER = 4,  // 请先拔出电源
    POPUP_TYPE_LEAVE_STATION = 5,  // 请先将车辆驶离基站
};

enum class SafetyStatus : int8_t {
    SAFETY_STATUS_OFF = 0,  // 安全功能关闭
    SAFETY_STATUS_ON = 1,  // 安全功能开启
};

enum class ModeType : int8_t {
    MODE_TYPE_CRUISE = 1,  // 巡航
    MODE_TYPE_RECHARGE = 2,  // 回充
    MODE_TYPE_HELPCAR = 3,  // 推行
    MODE_TYPE_STANDARD = 4,  // 标准 
    MODE_TYPE_BLUETOOTH = 5  // 蓝牙控车
};

// 抱闸状态
enum class BrakeStatus : int8_t {
    BRAKE_STATUS_CLOSE = 0,  // 抱闸关闭
    BRAKE_STATUS_OPEN = 1,  // 抱闸打开
};

} // namespace sensor::chassis