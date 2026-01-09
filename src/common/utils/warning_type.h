#pragma once
//告警等级  (地面以上) 1 前向减速 2 侧向减速 3 后向减速 4 前方刹停 5 侧向刹停 6 后向刹停 7 驶离减速区 8 驶离刹停区
//          (地面以下) 9 前向减速区 10 侧向减速区 11 前方刹停区 12 侧向刹停区  13 驶离减速区 14 驶离刹停区
enum class WarningType {
    //正常
    FORWARD_WARNING = 1, // 前向减速
    LATERAL_WARNING = 2, // 侧向减速
    BACKWARD_WARNING = 3, // 后向减速
    FORWARD_BRAKE_WARNING = 4, // 前方刹停
    LATERAL_BRAKE_WARNING = 5, // 侧向刹停
    BACKWARD_BRAKE_WARNING = 6, // 后向刹停
    FORWARD_LEAVE_WARNING = 7, // 驶离减速区
    LATERAL_LEAVE_WARNING = 8, // 驶离刹停区

    FORWARD_NEGATIVE_PLANE_WARNING = 9, // 前向负平面减速区
    LATERAL_NEGATIVE_PLANE_WARNING = 10, // 侧向负平面减速区
    FORWARD_NEGATIVE_PLANE_BRAKE_WARNING = 11, // 前向负平面刹停区
    LATERAL_NEGATIVE_PLANE_BRAKE_WARNING = 12, // 侧向负平面刹停区
    FORWARD_NEGATIVE_PLANE_LEAVE_WARNING = 13, // 驶离负平面减速区
    LATERAL_NEGATIVE_PLANE_LEAVE_WARNING = 14, // 驶离负平面刹停区
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