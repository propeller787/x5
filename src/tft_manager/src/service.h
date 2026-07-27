#pragma once
#include "tft_audio.h"
#include "muduo/net/EventLoop.h"
#include "muduo/net/EventLoopThread.h"
#include "tft/tft_module.h"
#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <mutex>
#include <unordered_map>
#include <interface/msg/backup_car.hpp>
#include <interface/msg/recharge_status.hpp>
#include <interface/msg/control.hpp>
#include <interface/srv/scene_controls.hpp>
#include <muduo/net/TimerId.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <utils/topic.h>
#include <std_msgs/msg/float32_multi_array.hpp>
namespace tft_service {

enum tft_toast_type {
    TFT_TOAST_TYPE_NONE = 0,
    TFT_TOAST_TYPE_PARKING = 1, // 已驻车
    TFT_TOAST_TYPE_UNPARKING = 2, // 驻车已解除
    TFT_TOAST_TYPE_PUSHING = 3,// 推行状态下无法驻车
    TFT_TOAST_TYPE_PUSHING_MODE = 4,//当前为推行模式
    TFT_TOAST_TYPE_JOYSTICK_FAILED = 5,//摇杆失败，当前摇杆不可用
    TFT_TOAST_TYPE_JOYSTICK_MODE = 6,//当前为摇杆模式
    TFT_TOAST_TYPE_SMART_AVOIDANCE_ENABLED = 7,// 智能躲避已开启
    TFT_TOAST_TYPE_SMART_AVOIDANCE_DISABLED = 8,// 智能躲避已退出
    TFT_TOAST_TYPE_3_GEAR = 9,// 当前为3档位
    TFT_TOAST_TYPE_2_GEAR = 10,// 当前为2档位
    TFT_TOAST_TYPE_1_GEAR = 11,// 当前为1档位
    TFT_TOAST_TYPE_CRUISE_CONTROL_ENABLED = 12,// 定速巡航已开启
    TFT_TOAST_TYPE_CRUISE_CONTROL_DISABLED = 13,// 定速巡航已退出
    TFT_TOAST_TYPE_BLUETOOTH_REMOTE_ENABLED = 14,// 蓝牙遥控已开启
    TFT_TOOTH_TYPE_BLUETOOTH_REMOTE_DISABLED = 15,// 蓝牙遥控已退出
    TFT_TOAST_TYPE_LOW_POWER = 16,// 当前电量低于20%请及时充电
    TFT_TOAST_TYPE_ELECTRONIC_BRAKE_FAILED = 17,// 电子抱闸已失效，请立即复原开关
    TFT_TOAST_TYPE_ELECTRONIC_BRAKE_ENABLED = 18,// 抱闸已生效
    TFT_TOAST_TYPE_AUTO_RETURN_CHARGE_DISABLED = 19,// 自主回充已退出
    TFT_TOAST_TYPE_VERY_LOW_POWER = 20,// 当前电量低于10%请及时充电
    TFT_TOAST_TYPE_FOLLOW_ME_ACTIVATED = 21,// 智能跟随已激活
    TFT_TOAST_TYPE_FOLLOW_ME_ENABLED = 22,// 智能跟随已开启
    TFT_TOAST_TYPE_FOLLOW_ME_PAUSED = 23,// 智能跟随已暂停
    TFT_TOAST_TYPE_FOLLOW_ME_RESUMED = 24,// 智能跟随已恢复
    TFT_TOAST_TYPE_FOLLOW_ME_STOPPED = 25,// 智能跟随已退出
    TFT_TOAST_TYPE_4_GEAR = 26,// 当前为4档位

};

enum tft_language_type {
    TFT_LANGUAGE_TYPE_CN = 0,// 中文
    TFT_LANGUAGE_TYPE_EN = 1,// 英文
};

class PageDialogManager;
class PhysicalButtonHandler;

class TftService : public rclcpp::Node {
public:
    TftService(const rclcpp::NodeOptions& options);
    ~TftService();
private:
    void setupTouchEventWiring();
    void sendTftTime();
    void onSettingPageTouch(uint16_t reg, uint16_t value);
    void onBackupCar(const interface::msg::BackupCar::SharedPtr msg);
    // void onAuxiliaryStatus(const std_msgs::msg::Bool::SharedPtr msg);
    void setupPublishers();
    /** 异步调用 idControlService + bluetoothMacService，结果写入 TftUserConfig（不阻塞 executor） */
    void fetchSnAndBluetoothMacAsync();
    /** 开机为主界面时，同步剩余里程、电量、速度、车辆模式、安全功能、档位、档位+功能模式到 TFT */
    void syncMainPageInitialData();
    /** 将当前语言下的档位+功能模式字符串写入 TFT（GEAR_AND_FUNCTION_MODE_ADDRESS） */
    void sendGearAndFunctionModeTextToTft();
    // 根据触屏事件，更新绑定下载情况
    void updateBindingDownloadStatus(uint16_t reg);
    // 回充上滑按钮：收到寄存器 0x1104 值为 1 时启动，3 秒后写 0（一次性，不阻塞）
    void onRechargeUpSlideButton(uint16_t reg, uint16_t value);
    // 更新TFT显示的toast类型
    void updateTftToastType(tft_toast_type type);

    // 判断驻车toast显示
    void checkParkingToast(int16_t value);
    // 判断推行/摇杆toast显示
    void checkPushingOrJoystickToast(std::string value);
    // 判断智能躲避toast显示
    void checkSmartAvoidanceToast(bool status);
    // 判断定速巡航toast显示
    void checkCruiseControlToast(bool status);
    // 判断蓝牙遥控toast显示
    void checkBluetoothRemoteToast(bool status);
    // 判断电量toast显示
    void checkLowPowerToast(int16_t value);
    // 判断前向制动/减速动画显示 1 减速 2 刹停
    void checkForwardBrakeDecelerationAnimation(int16_t value);
    // 判断后向制动/减速动画显示 1 减速 2 刹停
    void checkBackwardBrakeDecelerationAnimation(int16_t value);
    // 判断左向制动/减速动画显示 1 减速 2 刹停
    void checkLeftBrakeDecelerationAnimation(int16_t value);
    // 判断右向制动/减速动画显示 1 减速 2 刹停
    void checkRightBrakeDecelerationAnimation(int16_t value);
    // 判断抱闸状态toast显示
    void checkBrakeStatusToast(int16_t value);

    // 判断摇杆失效弹窗显示
    void checkJoystickFailureToast(joystick_failure_type type);

    // 物理按键失效
    void onPhysicalButtonFailure(const std_msgs::msg::Int32::SharedPtr msg);
    /** 将四向碰撞预警等级汇总给灯效（任一向 >0 视为有预警） */
    void pushCollisionWarningLevelsToLightController(int16_t value);
    /** 监听车道警告点消息 */
    void onLaneWarningPoint(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
private:
    rclcpp::Logger logger_;
    rclcpp::Subscription<interface::msg::BackupCar>::SharedPtr backup_car_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auxiliary_status_sub_;
    // 物理按键失效
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr physical_button_failure_sub_;
    // 车道警告点订阅
    std::mutex lane_warning_point_mutex_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lane_warning_point_sub_;
    bool initial_page_was_main_{false};  ///< 开机是否为主界面，用于延迟同步主页面数据
    int page_before_backup_{-1};  ///< 进入倒车页前的页面，退出倒车时返回
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tft_car_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tft_sound_switch_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tft_language_pub_;
    /** TFT 读寄存器有数据时发布（std_msgs/Bool），供其它节点感知触屏/读回事件 */
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tft_touch_event_pub_;
    std::unique_ptr<TftAudioPublisher> tft_audio_;
    /** 用于「开始充电」TTS 边沿检测，-1 表示尚未收到过有效状态 */
    int last_charging_status_for_audio_{-1};
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cruise_end_reason_audio_sub_;
    rclcpp::TimerBase::SharedPtr tft_time_timer_;
    /** 延后执行：在 executor spin 后通过 idControlService/bluetoothMacService 获取 SN 与蓝牙 MAC（避免构造函数中阻塞） */
    rclcpp::TimerBase::SharedPtr fetch_sn_mac_timer_;
    /** 回充上滑按钮：收到寄存器 0x1104 值为 1 时启动，3 秒后写 0（一次性，不阻塞） */
    rclcpp::TimerBase::SharedPtr recharge_up_slide_timer_;
    std::shared_ptr<TftModule> tft_module_;
    std::unique_ptr<PageDialogManager> page_dialog_manager_;
    std::unique_ptr<PhysicalButtonHandler> physical_button_handler_;
    /** toast 2 秒后消失：一次性定时器，到期发送 TFT_TOAST_TYPE_NONE 清除 */
    rclcpp::TimerBase::SharedPtr tft_toast_type_timer_;
    // 电量 toast：程序运行期间仅提示一次（20% 与 10% 各一次）
    bool low_power_toast_shown_{false};
    bool very_low_power_toast_shown_{false};
    /** 上次档位部分（如 "一档"），用于 checkPushingOrJoystickToast 档位修改时不更新 toast */
    std::string last_gear_str_{};
    std::string first_scene_mode_str_{}; // 修复开机后会报"切换至驾驶模式"的问题，仅作为标志位
    /** 前/后/左/右制动减速动画：2 秒未收到新消息则发送 0 关闭 */
    rclcpp::TimerBase::SharedPtr forward_brake_anim_timer_;
    rclcpp::TimerBase::SharedPtr backward_brake_anim_timer_;
    rclcpp::TimerBase::SharedPtr left_brake_anim_timer_;
    rclcpp::TimerBase::SharedPtr right_brake_anim_timer_;
};

}