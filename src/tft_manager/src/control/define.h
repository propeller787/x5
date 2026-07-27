#pragma once

// =============================================================================
// TFT 屏 DCBUS 串口通信分为 3 类（均使用 STATION_ADDRESS）：
//
// 1. 切换页：sendDcBusWrite(STATION_ADDRESS, PAGE_ID_ADDRESS, 页面宏定义)
// 2. 发送字符串：sendDcBusTextWrite(STATION_ADDRESS, 地址号, 字符串)
// 3. 修改数值：sendDcBusWrite(STATION_ADDRESS, 组件地址, 数值)
// =============================================================================

// 站点
#define STATION_ADDRESS 0
// 页码
#define PAGE_ID_ADDRESS 0x0100
// APP 下载二维码页面
#define APP_DOWNLOAD_QR_CODE_PAGE_ID 2
// 车辆绑定二维码页面
#define VEHICLE_BINDING_QR_CODE_PAGE_ID 3
// 绑定成功页面
#define BINDING_SUCCESS_PAGE_ID 4
// 设置界面页面
#define SETTING_PAGE_ID 7
// 隐私声明页面
#define PRIVACY_DECLARATION_PAGE_ID 5
// 主界面
#define MAIN_PAGE_ID 0
// 倒车界面
#define BACKWARD_PAGE_ID 6
// 线充页面
#define LINE_CHARGE_PAGE_ID 9
// 基站充页面
#define BASE_STATION_CHARGE_PAGE_ID 10
// 设置界面中绑定二维码页面
#define SETTING_PAGE_BINDING_QR_CODE_PAGE_ID 8
// 已是最新版本页面
#define LATEST_VERSION_PAGE_ID 11
// 检查更新页面
#define CHECK_UPDATE_PAGE_ID 12
// 更新失败界面
#define UPDATE_FAILURE_PAGE_ID 13
// 下载界面
#define OTA_DOWNLOAD_PAGE_ID 14
// 下载进度界面
#define OTA_DOWNLOAD_PROGRESS_PAGE_ID 15
// 安装界面
#define OTA_INSTALL_PAGE_ID 16
// 安装完成界面
#define OTA_INSTALL_SUCCESS_PAGE_ID 18
// 安装进度界面
#define OTA_INSTALL_PROGRESS_PAGE_ID 17
// 前往基站界面
#define GO_TO_BASE_STATION_PAGE_ID 19
// 安装失败界面
#define OTA_INSTALL_FAILURE_PAGE_ID 31

// 智能跟随已激活 38
#define SMART_FOLLOW_ACTIVE_PAGE_ID 38
// 智能跟随开始跟随页面
#define SMART_FOLLOW_START_FOLLOW_PAGE_ID 39

// 触屏响应
//  app下载页面中暂不下载按钮
#define TOUCH_EVENT_APP_DOWNLOAD_PAGE_DONT_DOWNLOAD 0x1017
// appp下载页面中已下载下一步按钮
#define TOUCH_EVENT_APP_DOWNLOAD_PAGE_DOWNLOAD_NEXT 0x101B
// 车辆绑定页面中暂不绑定按钮
#define TOUCH_EVENT_VEHICLE_BINDING_PAGE_DONT_BIND 0x1018
// 车辆绑定页面中返回上一页按钮 0x6103
#define TOUCH_EVENT_VEHICLE_BINDING_PAGE_RETURN_PREVIOUS 0x101C
// 模式切换按钮
#define TOUCH_EVENT_MODE_SWITCH 0x1103
// 声音开关按钮
#define TOUCH_EVENT_SOUND_SWITCH 0x1010
// 语言开关按钮
#define TOUCH_EVENT_LANGUAGE_SWITCH 0x0119

// 语言接收地址
#define LANGUAGE_RECEIVE_ADDRESS 0x1031
// 设置页面中返回上一页按钮 0x6103
#define TOUCH_EVENT_SETTING_PAGE_RETURN_PREVIOUS 0x1020
// 回充开始
#define TOUCH_EVENT_RECHARGE_START 0x1101
// 运动模式切换按钮
#define TOUCH_EVENT_SPORT_MODE_SWITCH 0x1103
// 隐私声明按钮
#define TOUCH_EVENT_PRIVACY_DECLARATION 0x1022
// 设置界面二维码绑定按钮
#define TOUCH_EVENT_SETTING_PAGE_QR_CODE_BIND 0x1023
// 设置界面设备升级按钮
#define TOUCH_EVENT_SETTING_PAGE_DEVICE_UPGRADE 0x101D
// 智能跟随开始根据按钮
#define TOUCH_EVENT_SMART_FOLLOW_START_FOLLOW 0x110B
// 智能跟随显示
#define TOUCH_EVENT_SMART_FOLLOW_DISPLAY 0x1032

// ota相关
// 已是最新版本页面返回按钮
#define TOUCH_EVENT_LATEST_VERSION_PAGE_RETURN_PREVIOUS 0x1025
// 检查更新页面返回按钮
#define TOUCH_EVENT_CHECK_UPDATE_PAGE_RETURN_PREVIOUS 0x1026
// 更新失败界面返回按钮
#define TOUCH_EVENT_UPDATE_FAILURE_PAGE_RETURN_PREVIOUS 0x1027
// 下载界面返回按钮
#define TOUCH_EVENT_DOWNLOAD_PAGE_RETURN_PREVIOUS 0x1028
// 下载进度界面返回按钮
#define TOUCH_EVENT_DOWNLOAD_PROGRESS_PAGE_RETURN_PREVIOUS 0x101E
// 安装界面返回按钮
#define TOUCH_EVENT_INSTALL_PAGE_RETURN_PREVIOUS 0x101F
// 安装成功立即体验按钮
#define TOUCH_EVENT_INSTALL_SUCCESS_PAGE_IMMEDIATE_EXPERIENCE 0x1029

// 年份
#define YEAR_ADDRESS 0x0160
// 月份
#define MONTH_ADDRESS 0x0161
// 日期
#define DAY_ADDRESS 0x0162
// 小时
#define HOUR_ADDRESS 0x0164
// 分钟
#define MINUTE_ADDRESS 0x0165
// 秒
#define SECOND_ADDRESS 0x0166
// 设置日期时间
#define SET_DATE_TIME_ADDRESS 0x0167
// 设置日期时间地址2
#define SET_DATE_TIME_ADDRESS_2 0x5501
// 设置使能开关，0：不可触控；1：可以触控
#define SET_ENABLE_ADDRESS 0x101A

// 4G信号强度
#define FOUR_G_SIGNAL_STRENGTH_ADDRESS 0x1002
// 声音开关
#define SOUND_SWITCH_ADDRESS 0x1010
// 电量
#define BATTERY_LEVEL_ADDRESS 0x1001
// 速度
#define SPEED_ADDRESS 0x1100
// 档位 + 功能模式 字符串地址
#define GEAR_AND_FUNCTION_MODE_ADDRESS 0x1500
// 档位状态 0 D 1 R 2 P(驻车)
#define GEAR_STATUS_ADDRESS 0x100D
// 功能状态 0 空 1 巡航 2 蓝牙控车
#define FUNCTION_STATUS_ADDRESS 0x100E
// 蓝牙状态 0 关 1 开
#define BLUETOOTH_STATUS_ADDRESS 0x100F
// toast 弹窗 0 取消 1 已驻车 2 驻车已解除
#define TOAST_POPUP_ADDRESS 0x1004
// 剩余里程
#define REMAINING_MILEAGE_ADDRESS 0x1108
// 安全功能开启状态 0 关闭 1 开启
#define SAFETY_FUNCTION_STATUS_ADDRESS 0x100B
// 车辆模式 0 舒适 1 运动
#define CAR_MODE_ADDRESS 0x100C
// 下载app二维码地址
#define APP_DOWNLOAD_QR_CODE_ADDRESS 0x1F00
// 车辆绑定二维码地址
#define VEHICLE_BINDING_QR_CODE_ADDRESS 0x1E00
// 回充上滑按钮
#define RECHARGE_UP_SLIDE_BUTTON_ADDRESS 0x1104
// 设置界面车辆绑定二维码地址
#define SETTING_PAGE_VEHICLE_BINDING_QR_CODE_ADDRESS 0x1D00

// 前向预警效果地址
#define FORWARD_WARNING_EFFECT_ADDRESS 0x102A
// 后向预警效果地址
#define BACKWARD_WARNING_EFFECT_ADDRESS 0x102B
// 左向预警效果地址
#define LEFT_WARNING_EFFECT_ADDRESS 0x102C
// 右向预警效果地址
#define RIGHT_WARNING_EFFECT_ADDRESS 0x102D
// 设置界面中
// 用户ID
#define USER_ID 0x1400
// 用户头像
#define USER_AVATAR_ADDRESS 0x1000

// ota相关
// 当前版本号
#define CURRENT_VERSION_ADDRESS 0x1300
// 待下载版本号
#define DOWNLOAD_VERSION_ADDRESS 0x1B00
// 待下载版本大小
#define DOWNLOAD_VERSION_SIZE_ADDRESS 0x1C00
// 下载进度
#define DOWNLOAD_PROGRESS_ADDRESS 0x1015
// 待安装版本号
#define INSTALL_VERSION_ADDRESS 0x1380
// 升级进度条
#define UPGRADE_PROGRESS_BAR_ADDRESS 0x1016

// 弹窗地址
#define PHYSICAL_BUTTON_FAILURE_OPEN_ADDRESS 0x1109
// 抱闸已失效
#define BRAKE_FAILURE_ADDRESS 0x1030

// 弹窗已知晓
#define DIALOG_KNOWN_ADDRESS 0x1081
// 弹窗忽略
#define DIALOG_IGNORE_ADDRESS 0x1082
// 弹窗重新激活-安全功能
#define DIALOG_RETRY_ADDRESS_SAFETY_FUNCTION 0x1083
// 弹窗重新激活-定速巡航
#define DIALOG_RETRY_ADDRESS_CRUISE_MODE 0x1084
// 弹窗重新激活-回充
#define DIALOG_RETRY_ADDRESS_RECHARGE 0x1085
// 弹窗已知晓--OTA下载检验界面使用
#define DIALOG_OTA_DOWNLOAD_RECHARGE 0x1086
// 弹窗已知晓--OTA安装检验界面使用
#define DIALOG_OTA_KNOWN_RECHARGE 0x1087
// ota相关
// 检查更新
#define CHECK_UPDATE_ADDRESS 0x1012
// 现在更新
#define NOW_UPDATE_ADDRESS 0x1014
// 立即安装
#define INSTALL_NOW_ADDRESS 0x102E
// 设置界面显示是否有可用更新
#define SETTING_PAGE_SHOW_UPDATE_ADDRESS 0x1011
// 失败界面 tc内容
#define FAILURE_PAGE_TC_CONTENT_ADDRESS 0x1600
// 倒车线
#define BACKWARD_LINE_ADDRESS 0x2000

enum joystick_failure_type // 摇杆失效类型
{
    JOYSTICK_FAILURE_TYPE_BRAKE_CLOSED = 0, // 请先闭合手动抱闸开关
    JOYSTICK_FAILURE_TYPE_DRIVING_MODE = 1, // 请先切换至摇杆模式
    JOYSTICK_FAILURE_TYPE_POWER_OFF = 2, // 请先拔出电源
    JOYSTICK_FAILURE_TYPE_PARKING_RELEASED = 3, // 请先解除驻车
    JOYSTICK_FAILURE_TYPE_BASE_LEAVED = 4, // 请先将车辆驶离基站
};

enum physical_button_failure_type // 物理按键失效类型
{
    PHYSICAL_BUTTON_FAILURE_TYPE_BRAKE_CLOSED = 0, // 请先闭合手动抱闸开关
    PHYSICAL_BUTTON_FAILURE_TYPE_DRIVING_MODE = 1, // 请先切换至摇杆模式
};
enum cruise_mode_failure_type // 定速巡航开启条件不满足类型
{
    CRUISE_MODE_FAILURE_TYPE_POWER_OFF = 0, // 请先拔出电源
    CRUISE_MODE_FAILURE_TYPE_PARKING_RELEASED = 1, // 请先解除驻车
    CRUISE_MODE_FAILURE_TYPE_FORWARD_MOVEMENT = 2, // 请向前行拨动摇杆
    CRUISE_MODE_FAILURE_TYPE_DRIVING_MODE = 3, // 请先切换至驾驶模式
    CRUISE_MODE_FAILURE_TYPE_BRAKE_CLOSED = 4, // 请先闭合手动抱闸开关
};

enum cruise_process_state_type // 定速巡航过程状态类型
{
    CRUISE_PROCESS_STATE_TYPE_EXIT = 0, // 由于长时间无操作，定速巡航已退出
    CRUISE_PROCESS_STATE_TYPE_ACTIVE = 1, // 请于道路通畅时尝试激活该功能
};

// 回充开启条件不满足类型
enum recharge_failure_type // 回充开启条件不满足类型
{
    RECHARGE_FAILURE_TYPE_POWER_OFF = 0, // "请先拔出电源"
    RECHARGE_FAILURE_TYPE_CHARGING = 1, // "正在充电中"
    RECHARGE_FAILURE_TYPE_PARKING_RELEASED = 2, // "请先解除驻车"
    RECHARGE_FAILURE_TYPE_DRIVING_MODE = 3, // "请先切换至驾驶模式"
    RECHARGE_FAILURE_TYPE_BRAKE_CLOSED = 4, // "请先闭合手动抱闸开关"
    RECHARGE_FAILURE_TYPE_BASE_LEAVED = 5, // "请检查基站电源情况，同时保证车辆在基站附近"
    RECHARGE_FAILURE_TYPE_MOVE_TO_BASE = 6, // "请移动车辆至基站附近"
};

enum light_type_type // 灯效类型
{
    LIGHT_TYPE_TYPE_FLASH_RED = 1, // 整体闪烁红色
    LIGHT_TYPE_TYPE_CONSTANT_BLUE = 2, // 整体常亮蓝色
    LIGHT_TYPE_TYPE_CONSTANT_RED = 3, // 整体常亮红色
    LIGHT_TYPE_TYPE_CONSTANT_YELLOW = 4, // 整体常亮黄色
    LIGHT_TYPE_TYPE_CONSTANT_GREEN = 5, // 整体常亮绿色
    LIGHT_TYPE_TYPE_CONSTANT_ORANGE = 6, // 整体常亮橙色
};

enum light_type_scene_type // 灯效场景类型
{
    LIGHT_TYPE_SCENE_TYPE_SMART_AVOIDANCE = 1, // 智能避障
    LIGHT_TYPE_SCENE_TYPE_BACKWARD = 2, // 倒车
    LIGHT_TYPE_SCENE_TYPE_CRUISE = 3, // 定速巡航
    LIGHT_TYPE_SCENE_TYPE_RECHARGE = 4, // 自主回充
    LIGHT_TYPE_SCENE_TYPE_BLUETOOTH = 5, // 蓝牙遥控
    LIGHT_TYPE_SCENE_TYPE_PARKING = 6, // 驻车
    LIGHT_TYPE_SCENE_TYPE_PUSHING = 7, // 推行
    LIGHT_TYPE_SCENE_TYPE_JOYSTICK = 8, // 摇杆模式（橙/红由车速自动判断）
    LIGHT_TYPE_SCENE_TYPE_BRAKE_FAILURE = 9, // 抱闸失效
    LIGHT_TYPE_SCENE_TYPE_CHARGING = 10, // 充电中
};

enum tts_code_type // TTS播报类型 下面编号 + 0x80
{
    TTS_CODE_TYPE_CRUISE = 0x81, // 智能巡航开启
    TTS_CODE_TYPE_RECHARGE = 0x82, // 自主回充开启
    TTS_CODE_TYPE_REMOVE_OBSTACLES = 0x83, // 请移除障碍物
    TTS_CODE_TYPE_CHARGING = 0x84, // 车辆开始充电
    TTS_CODE_TYPE_PARKING = 0x85, // 已驻车
    TTS_CODE_TYPE_PARKING_RELEASED = 0x86, // 驻车解除
    TTS_CODE_TYPE_DRIVING_MODE = 0x87, // 已切换至驾驶模式
    TTS_CODE_TYPE_PUSHING_MODE = 0x88, // 已切换至推行模式
    TTS_CODE_TYPE_GEAR_4 = 0x89, // 当前4挡
    TTS_CODE_TYPE_GEAR_3 = 0x8A, // 当前3挡
    TTS_CODE_TYPE_GEAR_2 = 0x8B, // 当前2挡
    TTS_CODE_TYPE_GEAR_1 = 0x8C, // 当前1挡
    TTS_CODE_TYPE_LOW_BATTERY = 0x8D, // 电量过低，请及时充电
    TTS_CODE_TYPE_BRAKE_OPEN = 0x8E, // 电子抱闸已开启
    TTS_CODE_TYPE_BRAKE_CLOSE = 0x8F, // 电子抱闸已关闭
    TTS_CODE_TYPE_BLUETOOTH = 0x90, // 蓝牙遥控已开启
    TTS_CODE_TYPE_SMART_AVOIDANCE = 0x91, // 智能躲避避障已开启
    TTS_CODE_TYPE_FOLLOWING = 0x92, // 跟随中
};

enum prompt_tone_type // 提示音类型
{
    PROMPT_TONE_TYPE_OBSTACLE = 1, // 障碍物
    PROMPT_TONE_TYPE_ACTIVATION = 2, // 功能激活
    PROMPT_TONE_TYPE_FAULT = 3, // 故障
    PROMPT_TONE_TYPE_EXIT = 4, // 功能退出
};

enum ota_upgrade_failure_type // ota升级失败类型
{
    UPGRADE_FAILURE_TYPE_FILE = 0, // 文件校验失败
    UPGRADE_FAILURE_TYPE_NOT_STILL = 1, // 电量低于80%或者处于非静止状态
};