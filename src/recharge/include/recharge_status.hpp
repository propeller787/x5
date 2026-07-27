#ifndef RECHARGE_STATUS_HPP
#define RECHARGE_STATUS_HPP

// 回充状态上报枚举 (对应图片上半部分)
enum RechargeReportStatus
{
    RECHARGE_EVENT_OBSTACLE_DETECTED = 1,   // 前往充电站中
    RECHARGE_EVENT_STATION_NOT_FOUND = 2,   // 未找到充电站
    RECHARGE_FEEDBACK_SUCCESS_DOCKED = 100      // 成功对接
    // RECHARGE_EVENT_OBSTACLE_DETECTED = 4,   // 检测到障碍物
    // RECHARGE_EVENT_START_CHARGING = 5,      // 开始充电
    // RECHARGE_EVENT_STOP_CHARGING = 6        // 停止充电
};

// 回充结果反馈枚举 (对应图片下半部分)
enum RechargeFeedbackResult
{
    // RECHARGE_FEEDBACK_SUCCESS_DOCKED = 100,     // 成功-正常对接
    RECHARGE_FEEDBACK_FAILED_TIMEOUT = 201,     // 停止-找不到基站超时
    RECHARGE_FEEDBACK_FAILED_OBSTACLE = 202,    // 停止-遇到障碍物
    RECHARGE_FEEDBACK_FAILED_USER_CANCEL = 203, // 停止-人工操作取消
    RECHARGE_FEEDBACK_FAILED_OTHER = 299        // 停止-其他故障
};

#endif // RECHARGE_STATUS_HPP