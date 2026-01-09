#pragma once
// 行驶中
#define BATTERY_LEVEL_FRAME_ID (0x5000u) // 电量 5000
// 电量颜色 8000
#define LOW_BATTERY_LEVEL_FRAME_ID (0x8000u)   // 低电量 8000
#define LOW_BATTERY_LEVEL_FRAME_ID_3 (0x8001u) // 低电量 8001
#define SPEED_FRAME_ID (0x5020u)               // 速度 5020
#define GEAR_FRAME_ID (0x5040u)                // 档位 5040
#define OBSTACLE_AVOIDANCE_FRAME_ID (0x5100u)  // 避障状态 5100
#define TIME_FRAME_ID (0x6000u)                // 时间 6000

// 红色 F800
#define RED_COLOR (0xF800u) // 红色 0xF800

// 停车相关参数宏定义
#define PARKING_WAVE_FRAME_ID (0x5060u)         // 停车波形 0x5060
#define PARKING_MILEAGE_FRAME_ID (0x5080u)      // 停车里程 0x5080
#define PARKING_MILEAGE_UNIT_FRAME_ID (0x6080u) // 停车里程单位 0x6080
#define PARKING_DATE_FRAME_ID (0x6100u)         // 停车日期 0x6100
#define PARKING_TIME_FRAME_ID (0x6180u)         // 停车时间 0x6180
#define NETWORK_FRAME_ID (0x50A0u)              // 网络状态 0x50A0
#define TEMPERATURE_FRAME_ID (0x50C0u)          // 温度 0x50C0

// 健康检测相关参数宏定义
#define HEALTH_HEART_RATE_FRAME_ID (0x5120u)              // 心率 5120
#define HEALTH_BLOOD_OXYGEN_FRAME_ID (0x5140u)            // 血氧 5140
#define HEALTH_BLOOD_OXYGEN_SATURATION_FRAME_ID (0x5160u) // 血氧饱和度 5160
#define HEALTH_COUNTDOWN_FRAME_ID (0x5180u)               // 健康倒计时 5180

// 倒车影像
#define BACKUP_CAR_PAGE_ID (0x4000u) // 倒车影像页面ID 4000

// 页面编号  5AA5 07 82 0084 5A01 0001
// 停车
#define PARKING_PAGE_ID 42
// 手动驾驶
#define MANUAL_DRIVE_PAGE_ID 1
// 充电
#define CHARGING_PAGE_ID_02 2
#define CHARGING_PAGE_ID_03 3
#define CHARGING_PAGE_ID_04 4
#define CHARGING_PAGE_ID_05 5
#define CHARGING_PAGE_ID_06 6
// 健康
#define HEALTH_PAGE_ID_07 7
#define HEALTH_PAGE_ID_08 8
#define HEALTH_PAGE_ID_09 9
#define HEALTH_PAGE_ID_10 10
#define HEALTH_PAGE_ID_11 11
#define HEALTH_PAGE_ID_12 12
#define HEALTH_PAGE_ID_13 13
#define HEALTH_PAGE_ID_14 14
#define HEALTH_PAGE_ID_15 15

// 通行障碍 17
#define OBSTACLE_PASSAGE_PAGE_ID 16

// 健康倒计时
#define HEALTH_COUNTDOWN_PAGE_ID 24
// 坐姿
#define SEAT_POSITION_PAGE_ID 18
// 左右脚全
#define LEFT_RIGHT_FOOT_PAGE_ID 19
// 右脚
#define RIGHT_FOOT_PAGE_ID 20
// 左脚
#define LEFT_FOOT_PAGE_ID 21
// 低电量10%
#define LOW_BATTERY_PAGE_ID 22
// 20%
#define LOW_BATTERY_20_PAGE_ID 23

// 巡航前推摇杆提醒
#define CRUISE_JOYSTICK_REMINDER_PAGE_ID 25
// 去充电
#define GO_TO_CHARGER_PAGE_ID 26
// 充电桩障碍
#define CHARGER_OBSTACLE_PAGE_ID 27
// 设备err
#define DEVICE_ERROR_PAGE_ID 28
// 寻找充电桩
#define FIND_CHARGER_PAGE_ID 29
// 未找到充电桩
#define NOT_FOUND_CHARGER_PAGE_ID 30
// 巡航障碍
#define OBSTACLE_AVOIDANCE_PAGE_ID 31
// 巡航
#define CRUISE_PAGE_ID 32
// 驻车
#define PARKING_PAGE_ID_33 33
// 设置
#define SETTING_PAGE_ID 34
// 倒车影像
#define REVERSE_IMAGE_PAGE_ID 35

// audio type
//  1-3  健康
//  4-6  模式切换
//  7-10  电量状态提醒
//  11-17 AI巡航
//  18-19 自动回充
//  20       安全坐姿检测
//  21       前方通行障碍
//  22   预警声音
#define AUDIO_TYPE_HEALTH_1 1
#define AUDIO_TYPE_HEALTH_2 2
#define AUDIO_TYPE_HEALTH_3 3
#define AUDIO_TYPE_MODE_SWITCH_4 4
#define AUDIO_TYPE_MODE_SWITCH_5 5
#define AUDIO_TYPE_MODE_SWITCH_6 6
#define AUDIO_TYPE_POWER_STATE_REMINDER_7 7
#define AUDIO_TYPE_POWER_STATE_REMINDER_8 8
#define AUDIO_TYPE_POWER_STATE_REMINDER_9 9
#define AUDIO_TYPE_POWER_STATE_REMINDER_10 10
#define AUDIO_TYPE_AI_CRUISE_11 11
#define AUDIO_TYPE_AI_CRUISE_12 12
#define AUDIO_TYPE_AI_CRUISE_13 13
#define AUDIO_TYPE_AI_CRUISE_14 14
#define AUDIO_TYPE_AI_CRUISE_15 15
#define AUDIO_TYPE_AI_CRUISE_16 16
#define AUDIO_TYPE_AI_CRUISE_17 17
#define AUDIO_TYPE_AUTO_RECHARGE_18 18
#define AUDIO_TYPE_AUTO_RECHARGE_19 19
#define AUDIO_TYPE_SAFETY_SEAT_DETECTION_20 20
#define AUDIO_TYPE_OBSTACLE_AVOIDANCE_21 21
#define AUDIO_TYPE_ALARM_AUDIO_22 22

// 电量显示地址
// 三位数
#define ELECTRIC_INDEX_3 407
// 两位数
#define ELECTRIC_INDEX_2 401
// 一位数
#define ELECTRIC_INDEX_1 395

// 电量显示Y坐标
#define ELECTRIC_Y_3 7
#define ELECTRIC_Y_2 7
#define ELECTRIC_Y_1 7