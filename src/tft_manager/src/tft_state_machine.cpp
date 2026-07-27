/**
 * @file tft_state_machine.cpp
 * @brief 状态机实现：订阅整车数据、场景模式、设备异常，缓存供查询。
 */

#include "tft_state_machine.h"
#include "tft_user_config.h"
#include "control/light_scene_controller.h"
#include "dialog/page_dialog_manager.h"
#include <interface/srv/scene_controls.hpp>
#include <utils/topic.h>
#include <utils/warning_type.h>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>

namespace tft_service {

namespace {
// 错误码格式 0xSSMMEE1EE2：EE2 0x01=发生 0x02=解除
constexpr uint8_t kErrorLevelOccur = 0x01;
constexpr uint8_t kErrorLevelRecover = 0x02;

constexpr int16_t kReverseLeftBaseX = 180;
constexpr int16_t kReverseRightBaseX = 540;
constexpr int16_t kReverseBaseY = 780;

/** TFT 显示速度上限 (km/h)：舒适 1/2/3→1.5/3/4.5，运动→2/4/6；四档 8；其余不截断 */
float tftDisplaySpeedCapKmh(int gear, int vehicle_mode) {
    if (TftStateMachine::instance().isPushingMode() 
        || TftStateMachine::instance().getBrakeStatus() == 1) {
        return 1e9f;
    }
    const bool sport = (vehicle_mode == 2);
    switch (gear) {
        case 1: return sport ? 2.0f : 1.5f;
        case 2: return sport ? 4.0f : 3.0f;
        case 3: return sport ? 6.0f : 4.5f;
        case 4: return sport ? 8.0f : 6.0f;
        default: return 1e9f;
    }
}
}  // namespace

const char* errorEventSystemName(uint8_t system) {
    switch (system) {
        case 0x01: return "系统基础";
        case 0x02: return "can0";
        case 0x03: return "can1";
        case 0x04: return "can2";
        case 0x10: return "IMU";
        case 0x11: return "GPS";
        case 0x12: return "单线雷达#1左";
        case 0x13: return "单线雷达#2右";
        case 0x14: return "单线雷达#3左斜";
        case 0x15: return "单线雷达#4右斜";
        case 0x16: return "单线雷达#5后";
        case 0x17: return "相机#1";
        case 0x18: return "相机#2";
        case 0x19: return "屏幕TFT";
        case 0x20: return "电机驱动器";
        case 0x21: return "上控";
        default: return "未知";
    }
}

TftStateMachine& TftStateMachine::instance() {
    static TftStateMachine inst;
    return inst;
}

void TftStateMachine::init(rclcpp::Node* node) {
    if (!node) return;
    std::lock_guard<std::mutex> lock(mutex_);
    node_ = node;
    scene_mode_.clear();
    car_info_.reset();
    active_error_codes_.clear();
    joystick_event_ = "center";
    joystick_forward_consecutive_ = 0;
    joystick_backward_consecutive_ = 0;
    joystick_center_consecutive_ = 0;
    auto qos = rclcpp::QoS(10);
    car_info_sub_ = node_->create_subscription<interface::msg::CarInfo>(
        carInfoTopic, qos,
        [this](const interface::msg::CarInfo::SharedPtr msg) { onCarInfo(msg); });
    scene_mode_sub_ = node_->create_subscription<std_msgs::msg::String>(
        sceneServiceModeTopic, qos,
        [this](const std_msgs::msg::String::SharedPtr msg) { onSceneMode(msg); });
    error_event_sub_ = node_->create_subscription<interface::msg::HexData>(
        errorEventTopic, qos,
        [this](const interface::msg::HexData::SharedPtr msg) { onErrorEvent(msg); });
    joystick_sub_ = node_->create_subscription<interface::msg::Joystick>(
        joystickTopic, qos,
        [this](const interface::msg::Joystick::SharedPtr msg) { onJoystick(msg); });
    joystick_event_pub_ = node_->create_publisher<std_msgs::msg::String>(tftJoystickEventTopic, 10);
    auxiliary_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        auxiliaryStatusTopic, qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { onAuxiliaryStatus(msg); });
    collision_warning_sub_ = node_->create_subscription<interface::msg::CollisionWarning>(
        collisionWarningTopic, qos,
        [this](const interface::msg::CollisionWarning::SharedPtr msg) { onCollisionWarning(msg); });
    device_bind_status_sub_ = node_->create_subscription<interface::msg::VehicleBindStatus>(
        cloudManagerDeviceBindPubTopic, qos,
        [this](const interface::msg::VehicleBindStatus::SharedPtr msg) { onDeviceBindStatus(msg); });
    wheel_twist_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        carWheelSpeedStaTopic, 10,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { onWheelTwistStamped(msg); });
    manage_recharge_client_ = node_->create_client<interface::srv::SceneControls>(manageRechargeService);
    manage_followme_client_ = node_->create_client<interface::srv::SceneControls>(manageFollowmeService);
}

void TftStateMachine::onCarInfo(const interface::msg::CarInfo::SharedPtr msg) {
    if (!msg) return;

    // Bug1 fix: 将所有共享状态的读写统一置于同一把锁的保护范围内，
    // 并在锁内收集需触发的回调及参数，锁外再统一调用，避免持锁调用外部代码（Bug6同类问题）。
    std::function<void()> cb_gear_icon;
    std::function<void()> cb_scene_mode_str;
    std::function<void()> cb_remaining_mileage;
    //std::function<void()> cb_speed;
    std::function<void()> cb_remaining_electricity;
    std::function<void()> cb_bluetooth_ctrl_mode;
    std::function<void()> cb_charging_status;
    std::function<void()> cb_brake_status;
    bool should_apply_forward_light = false;
    bool should_switch_main_from_base = false;
    bool should_apply_bluetooth_light = false;
    bool should_apply_charging_light  = false;
    bool should_apply_brake_light     = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        car_info_ = *msg;
        
        if (car_info_) {
            //保留小数点两位
            car_info_->speedkmh = std::round(car_info_->speedkmh * 100) / 100;
            car_info_->speed = static_cast<int16_t>(std::lround(static_cast<double>(speed_)));
        }
        // 档位图标
        if (gearNumberToIcon(car_info_->lockswitch) != gear_icon_) {
            gear_icon_ = gearNumberToIcon(car_info_->lockswitch);
            // 挡位变化时，设置灯效
            should_apply_forward_light = true;
            if (on_gear_icon_changed_) {
                int icon = gear_icon_;
                cb_gear_icon = [this, icon]() { on_gear_icon_changed_(icon); };
            }
        }

        // 档位字符串
        if (gearNumberToStr(car_info_->gear) != scene_gear_str_) {
            scene_gear_str_ = gearNumberToStr(car_info_->gear);
            scene_mode_gear_str_ = scene_gear_str_ + " | " + scene_mode_str_;
            if (on_scene_mode_str_changed_) {
                std::string s = scene_mode_gear_str_;
                cb_scene_mode_str = [this, s]() { on_scene_mode_str_changed_(s); };
            }
        }

        // 剩余里程
        if (car_info_->remainingmileage != remaining_mileage_) {
            remaining_mileage_ = car_info_->remainingmileage;
            if (on_remaining_mileage_changed_) {
                int m = remaining_mileage_;
                cb_remaining_mileage = [this, m]() { on_remaining_mileage_changed_(m); };
            }
        }

        // 速度
        // if ((std::round(car_info_->speedkmh * 10) / 10) != speed_) {
        //     speed_ = std::round(car_info_->speedkmh * 10) / 10;
        //     if (on_speed_changed_) {
        //         float sp = speed_;
        //         cb_speed = [this, sp]() { on_speed_changed_(sp); };
        //     }
        // }

        // 电量
        if (car_info_->electricquantity != remaining_electricity_)
        {
            // 充电中动态灯效变化
            if (car_info_->getcharging == 1 || car_info_->getcharging == 2)
            {
                if (remaining_electricity_ >= 0 && remaining_electricity_ <= 10 && car_info_->electricquantity > 10)
                {
                    should_apply_charging_light = true;

                }
                else if (remaining_electricity_ > 10 && remaining_electricity_ <= 80 && car_info_->electricquantity > 80)
                {
                    should_apply_charging_light = true;

                }
            }
            remaining_electricity_ = car_info_->electricquantity;
            if (on_remaining_electricity_changed_)
            {
                int e = remaining_electricity_;
                cb_remaining_electricity = [this, e]()
                { on_remaining_electricity_changed_(e); };
            }
        }

        // 蓝牙控车状态
        {
            bool new_bluetooth_ctrl_mode = (car_info_->bluetooth_ctrl == 2);
            if (new_bluetooth_ctrl_mode != bluetooth_ctrl_mode_) {
                bluetooth_ctrl_mode_ = new_bluetooth_ctrl_mode;
                if (bluetooth_ctrl_mode_) should_apply_bluetooth_light = true;
                if (on_bluetooth_ctrl_mode_changed_) {
                    bool bm = bluetooth_ctrl_mode_;
                    cb_bluetooth_ctrl_mode = [this, bm]() { on_bluetooth_ctrl_mode_changed_(bm); };
                }
            }
        }

        // 充电状态
        if (car_info_->getcharging != -1 && car_info_->getcharging != charging_status_) {
            charging_status_ = car_info_->getcharging;
            should_apply_charging_light = true;
            if (on_charging_status_changed_) {
                int cs = charging_status_;
                cb_charging_status = [this, cs]() { on_charging_status_changed_(cs); };
            }
        }
        if (charging_status_ == 0 && page_dialog_mgr_ &&
            page_dialog_mgr_->getCurrentPage() == BASE_STATION_CHARGE_PAGE_ID) {
            should_switch_main_from_base = true;
        }
        // 抱闸状态
        if (car_info_->left_brakests != brake_status_) {
            brake_status_ = car_info_->left_brakests;
            // 挡位变化时，修改灯效
            should_apply_brake_light = true;
            if (on_brake_status_changed_) {
                int bs = brake_status_;
                cb_brake_status = [this, bs]() { on_brake_status_changed_(bs); };
            }
        }
    }  // mutex_ 已释放，锁外执行所有副作用

    if (should_apply_forward_light) {
        adaptGearIconLight();
    }
    if (should_apply_bluetooth_light)
        LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_BLUETOOTH);
    if (should_apply_charging_light)
        adaptChargingLight(car_info_->getcharging);
    if (should_apply_brake_light) {
        adaptBrakeLight();
    }
    
    if (should_switch_main_from_base && page_dialog_mgr_) {
        page_dialog_mgr_->switchToPage(MAIN_PAGE_ID);
    }

    // 速度变化时刷新摇杆灯效
    //if (cb_speed) LightSceneController::instance().refreshJoystickLightIfCurrentScene();

    // 回调通知（锁外安全调用）
    if (cb_gear_icon)             cb_gear_icon();
    if (cb_scene_mode_str)        cb_scene_mode_str();
    if (cb_remaining_mileage)     cb_remaining_mileage();
    //if (cb_speed)                 cb_speed();
    if (cb_remaining_electricity) cb_remaining_electricity();
    if (cb_bluetooth_ctrl_mode)   cb_bluetooth_ctrl_mode();
    if (cb_charging_status)       cb_charging_status();
    if (cb_brake_status)          cb_brake_status();
}

void TftStateMachine::onWheelTwistStamped(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (!msg) return;
    std::function<void()> cb_speed;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        float kmh = std::abs(static_cast<float>(msg->twist.linear.x) * 3.6f);
        // TFT 显示速度上限：随 TftUserConfig 车辆模式(1舒适/2运动)与档位（非底盘限速）**推行模式下不适用
        if (car_info_) {
            const int g = static_cast<int>(car_info_->gear);
            const float cap = tftDisplaySpeedCapKmh(g, TftUserConfig::instance().getVehicleMode());
            if (kmh > cap) {
                kmh = cap;
            }
        }
        wheel_angular_z_ = static_cast<float>(msg->twist.angular.z);
        const float rounded = std::round(kmh * 100.f) / 100.f;
        if (rounded != speed_) {
            speed_ = rounded;
            if (car_info_) {
                car_info_->speedkmh = rounded;
                car_info_->speed = static_cast<int16_t>(std::lround(static_cast<double>(rounded)));
            }
            if (on_speed_changed_) {
                float sp = speed_;
                cb_speed = [this, sp]() { on_speed_changed_(sp); };
            }
        }
    }
    // 推行模式不调整灯效
    if (cb_speed && !isPushingMode() && (brake_status_ == 0)) LightSceneController::instance().refreshJoystickLightIfCurrentScene();
    if (cb_speed) cb_speed();
}

void TftStateMachine::onSceneMode(const std_msgs::msg::String::SharedPtr msg) {
    if (!msg) return;

    // Bug6 fix: 在锁内仅修改共享状态并收集需调用的回调，锁外再执行回调，
    // 避免持锁期间调用外部代码（可能反向获取 mutex_ 形成死锁）。
    std::function<void()> cb_cruise_mode;
    std::function<void()> cb_scene_mode_str;
    std::function<void()> cb_recharge_scene;
    bool should_apply_cruise_light   = false;
    bool should_apply_recharge_light = false;
    bool should_apply_pushing_light  = false;
    bool should_apply_standard_light = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        scene_mode_ = msg->data;

        // 巡航模式
        {
            bool new_cruise_mode = (scene_mode_ == "cruise");
            if (new_cruise_mode != cruise_mode_) {
                cruise_mode_ = new_cruise_mode;
                if (cruise_mode_) {
                    RCLCPP_INFO(rclcpp::get_logger("TftStateMachine"), "cruise_mode_ = true");
                    should_apply_cruise_light = true;
                }
                if (on_cruise_mode_changed_) {
                    bool cm = cruise_mode_;
                    cb_cruise_mode = [this, cm]() { on_cruise_mode_changed_(cm); };
                }
            }
        }

        // 回充模式
        {
            bool new_recharge_status = (scene_mode_ == "recharge");
            if (new_recharge_status != recharge_status_) {
                recharge_status_ = new_recharge_status;
                if (recharge_status_) should_apply_recharge_light = true;
                if (on_recharge_scene_changed_) {
                    bool r = recharge_status_;
                    cb_recharge_scene = [this, r]() { on_recharge_scene_changed_(r); };
                }
            }
        }

        // 推行模式
        {
            bool new_pushing_status = (scene_mode_ == "helpcar");
            if (new_pushing_status != pushing_status_) {
                pushing_status_ = new_pushing_status;
                if (pushing_status_) should_apply_pushing_light = true;
            }
        }

        // 标准模式
        {
            bool new_standard_status = (scene_mode_ == "standard");
            if (new_standard_status != standard_status_) {
                standard_status_ = new_standard_status;
                if (standard_status_) {
                    RCLCPP_INFO(rclcpp::get_logger("TftStateMachine"), "standard_status_ = true");
                    should_apply_standard_light = true;
                }
            }
        }

        // 场景模式字符串
        std::string new_str = scene_mode_str_;
        if (car_info_ && car_info_->lockswitch == 0) {
            bool is_en = (TftUserConfig::instance().getLanguage() == Language::EN);
            new_str = (scene_mode_ == "helpcar")
                ? (is_en ? "Pushing Mode"  : "推行模式")
                : (is_en ? "Driving Mode" : "驾驶模式");
        }
        if (new_str != scene_mode_str_) {
            scene_mode_str_ = new_str;
            scene_mode_gear_str_ = scene_gear_str_ + " | " + scene_mode_str_;
            if (on_scene_mode_str_changed_) {
                std::string s = scene_mode_gear_str_;
                cb_scene_mode_str = [this, s]() { on_scene_mode_str_changed_(s); };
            }
        }
    }  // mutex_ 已释放

    rclcpp::sleep_for(std::chrono::milliseconds(5));
    if (brake_status_) {
        RCLCPP_ERROR(rclcpp::get_logger("TftStateMachine"), "物理抱闸关闭，不调整模式灯效");
    } else {
        // 锁外执行灯效和回调
    if (should_apply_cruise_light)
        LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_CRUISE);
    if (should_apply_recharge_light)
        LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_RECHARGE);
    if (should_apply_pushing_light)
        LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_PUSHING);
    if (should_apply_standard_light)
        LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_JOYSTICK);
    }
    

    if (cb_cruise_mode)       cb_cruise_mode();
    if (cb_scene_mode_str)   cb_scene_mode_str();
    if (cb_recharge_scene)    cb_recharge_scene();
}

void TftStateMachine::onErrorEvent(const interface::msg::HexData::SharedPtr msg) {
    if (!msg) return;
    uint32_t v = msg->hex_value;
    // uint8_t ss = static_cast<uint8_t>((v >> 24) & 0xFF);
    // uint8_t mm = static_cast<uint8_t>((v >> 16) & 0xFF);
    // uint8_t ee1 = static_cast<uint8_t>((v >> 8) & 0xFF);
    uint8_t ee2 = static_cast<uint8_t>(v & 0xFF);
    uint32_t key = (v >> 8);  // SSMMEE1 24bit
    std::lock_guard<std::mutex> lock(mutex_);
    if (ee2 == kErrorLevelOccur) {
        active_error_codes_.insert(key);
    } else if (ee2 == kErrorLevelRecover) {
        active_error_codes_.erase(key);
    }
}

void TftStateMachine::onJoystick(const interface::msg::Joystick::SharedPtr msg) {
    if (!msg) return;
    int16_t x = static_cast<int16_t>(msg->rocker_x);
    int16_t y = static_cast<int16_t>(msg->rocker_y);
    std::string new_event;
    bool joystick_event_changed = false;
    std::function<void()> cb_reverse_guide_curves;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (x < 0) {
            updateReverseGuideCurves(x, y);
            if (on_reverse_guide_curves_changed_) {
                ReverseGuideCurves curves = reverse_guide_curves_;
                cb_reverse_guide_curves = [this, curves]() { on_reverse_guide_curves_changed_(curves); };
            }
        } else {
            reverse_guide_curves_.valid = false;
        }
        if (x > kJoystickForwardThreshold) {
            joystick_forward_consecutive_ = (joystick_forward_consecutive_ < 5) ? joystick_forward_consecutive_ + 1 : 5;
            joystick_backward_consecutive_ = 0;
            joystick_center_consecutive_ = 0;
            new_event = (joystick_forward_consecutive_ >= 5) ? "forward" : joystick_event_;
        } else if (x < kJoystickBackwardThreshold) {
            joystick_backward_consecutive_ = (joystick_backward_consecutive_ < 5) ? joystick_backward_consecutive_ + 1 : 5;
            joystick_forward_consecutive_ = 0;
            joystick_center_consecutive_ = 0;
            new_event = (joystick_backward_consecutive_ >= 5) ? "backward" : joystick_event_;
        } else if (y < kJoystickBackwardThreshold || y > kJoystickForwardThreshold) {
            joystick_horizontal_consecutive_ = (joystick_horizontal_consecutive_ < 5) ? joystick_horizontal_consecutive_ + 1 : 5;
            joystick_center_consecutive_ = 0;
            new_event = (joystick_horizontal_consecutive_ >= 5) ? "horizontal" : joystick_event_;
        } else {
            joystick_center_consecutive_ = (joystick_center_consecutive_ < 5) ? joystick_center_consecutive_ + 1 : 5;
            joystick_forward_consecutive_ = 0;
            joystick_backward_consecutive_ = 0;
            joystick_horizontal_consecutive_ = 0;
            new_event = (joystick_center_consecutive_ >= 5) ? "center" : joystick_event_;
        }
        if (new_event != joystick_event_) {
            joystick_event_ = new_event;
            joystick_event_changed = true;
            std_msgs::msg::String pub_msg;
            pub_msg.data = joystick_event_;
            joystick_event_pub_->publish(pub_msg);
            if(joystick_event_ != "center") {
                checkJoystickFailureType();
            }
        }
    }
    // 摇杆刚进入 backward 且当前为回充/跟随时，关闭对应模式（边沿触发，避免重复 async 请求）
    if (joystick_event_changed && joystick_event_ == "backward") {
        const std::string mode = getSceneMode();
        bool closed_special_mode = false;
        if (mode == "recharge" && manage_recharge_client_) {
            auto request = std::make_shared<interface::srv::SceneControls::Request>();
            request->start = 0;
            manage_recharge_client_->async_send_request(request);
            closed_special_mode = true;
        } else if (mode == "followme" && manage_followme_client_) {
            auto request = std::make_shared<interface::srv::SceneControls::Request>();
            request->start = 0;
            manage_followme_client_->async_send_request(request);
            closed_special_mode = true;
        }
        if (closed_special_mode && page_dialog_mgr_) {
            page_dialog_mgr_->switchToPage(MAIN_PAGE_ID);
        }
    }

    // if (joystick_event_changed) {
    //     LightSceneController::instance().refreshJoystickLightIfCurrentScene();
    // }
    if (cb_reverse_guide_curves) {
        cb_reverse_guide_curves();
    }
}

TftStateMachine::ReverseGuideCurves TftStateMachine::getReverseGuideCurves() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return reverse_guide_curves_;
}

void TftStateMachine::updateReverseGuideCurves(int16_t rocker_x, int16_t rocker_y) {
    const float norm = std::clamp(std::abs(static_cast<float>(rocker_x)) / 100.0f, 0.0f, 1.0f);
    // 坐标定义：x 前后（前方为正），y 左右（右拨为正）。
    // 倒车时用 y 引入左右偏移，右拨(y>0)整条轨迹向右弯，左拨(y<0)向左弯。
    const float steer_norm = std::clamp(static_cast<float>(rocker_y) / 100.0f, -1.0f, 1.0f);
    const int16_t steer_shift = static_cast<int16_t>(std::lround(120.0f * steer_norm * norm));
    const GuidePoint left_p0{kReverseLeftBaseX, kReverseBaseY};
    const GuidePoint left_p1{
        static_cast<int16_t>(std::lround(180.0f - 40.0f * norm + steer_shift * 0.35f)),
        static_cast<int16_t>(700)};
    const GuidePoint left_p2{
        static_cast<int16_t>(std::lround(120.0f - 120.0f * norm + steer_shift * 0.7f)),
        static_cast<int16_t>(540)};
    const GuidePoint left_p3{
        static_cast<int16_t>(std::lround(80.0f - 160.0f * norm + steer_shift)),
        static_cast<int16_t>(380)};

    const GuidePoint right_p0{kReverseRightBaseX, kReverseBaseY};
    const GuidePoint right_p1{
        static_cast<int16_t>(std::lround(540.0f + 40.0f * norm + steer_shift * 0.35f)),
        static_cast<int16_t>(700)};
    const GuidePoint right_p2{
        static_cast<int16_t>(std::lround(600.0f + 120.0f * norm + steer_shift * 0.7f)),
        static_cast<int16_t>(540)};
    const GuidePoint right_p3{
        static_cast<int16_t>(std::lround(640.0f + 160.0f * norm + steer_shift)),
        static_cast<int16_t>(380)};

    for (std::size_t i = 0; i < kReverseCurvePointCount; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(kReverseCurvePointCount - 1);
        reverse_guide_curves_.left[i] = computeCubicBezierPoint(left_p0, left_p1, left_p2, left_p3, t);
        reverse_guide_curves_.right[i] = computeCubicBezierPoint(right_p0, right_p1, right_p2, right_p3, t);
    }
    reverse_guide_curves_.valid = true;
}

TftStateMachine::GuidePoint TftStateMachine::computeCubicBezierPoint(
    const GuidePoint& p0, const GuidePoint& p1, const GuidePoint& p2, const GuidePoint& p3, float t) {
    const float one_minus_t = 1.0f - t;
    const float b0 = one_minus_t * one_minus_t * one_minus_t;
    const float b1 = 3.0f * one_minus_t * one_minus_t * t;
    const float b2 = 3.0f * one_minus_t * t * t;
    const float b3 = t * t * t;
    const float x = b0 * p0.first + b1 * p1.first + b2 * p2.first + b3 * p3.first;
    const float y = b0 * p0.second + b1 * p1.second + b2 * p2.second + b3 * p3.second;
    return {static_cast<int16_t>(std::lround(x)), static_cast<int16_t>(std::lround(y))};
}

void TftStateMachine::checkJoystickFailureType() {
    // 摇杆失效类型
    // Bug2 fix: 调用前统一检查回调是否已注册，避免 std::bad_function_call 崩溃
    if (!on_joystick_failure_type_changed_) return;
    if(joystick_event_ == "forward") {
        if(brake_status_ == 1) {
            on_joystick_failure_type_changed_(joystick_failure_type::JOYSTICK_FAILURE_TYPE_BRAKE_CLOSED);
            return;
        }
        if(scene_mode_ == "helpcar") {
            on_joystick_failure_type_changed_(joystick_failure_type::JOYSTICK_FAILURE_TYPE_DRIVING_MODE);
            return;
        }
        if(charging_status_ == 1) {
            on_joystick_failure_type_changed_(joystick_failure_type::JOYSTICK_FAILURE_TYPE_POWER_OFF);
            return;
        }
        if(gear_icon_ == 2) {
            on_joystick_failure_type_changed_(joystick_failure_type::JOYSTICK_FAILURE_TYPE_PARKING_RELEASED);
            return;
        }
    } else if(joystick_event_ == "backward") {
        if(brake_status_ == 1) {
            on_joystick_failure_type_changed_(joystick_failure_type::JOYSTICK_FAILURE_TYPE_BRAKE_CLOSED);
            return;
        }
        if(scene_mode_ == "helpcar") {
            on_joystick_failure_type_changed_(joystick_failure_type::JOYSTICK_FAILURE_TYPE_DRIVING_MODE);
            return;
        }
        if(charging_status_ == 1) {
            on_joystick_failure_type_changed_(joystick_failure_type::JOYSTICK_FAILURE_TYPE_POWER_OFF);
            return;
        }
        if(charging_status_ == 2) {
            on_joystick_failure_type_changed_(joystick_failure_type::JOYSTICK_FAILURE_TYPE_BASE_LEAVED);
            return;
        }
        if(gear_icon_ == 2) {
            on_joystick_failure_type_changed_(joystick_failure_type::JOYSTICK_FAILURE_TYPE_PARKING_RELEASED);
            return;
        }
    } else if(joystick_event_ == "horizontal") {
        if(brake_status_ == 1) {
            on_joystick_failure_type_changed_(joystick_failure_type::JOYSTICK_FAILURE_TYPE_BRAKE_CLOSED);
            return;
        }
        if(scene_mode_ == "helpcar") {
            on_joystick_failure_type_changed_(joystick_failure_type::JOYSTICK_FAILURE_TYPE_DRIVING_MODE);
            return;
        }
        if(charging_status_ == 1) {
            on_joystick_failure_type_changed_(joystick_failure_type::JOYSTICK_FAILURE_TYPE_POWER_OFF);
            return;
        }
        if(charging_status_ == 2) {
            on_joystick_failure_type_changed_(joystick_failure_type::JOYSTICK_FAILURE_TYPE_BASE_LEAVED);
            return;
        }
        if(gear_icon_ == 2) {
            on_joystick_failure_type_changed_(joystick_failure_type::JOYSTICK_FAILURE_TYPE_PARKING_RELEASED);
            return;
        }
    }
}

void TftStateMachine::onAuxiliaryStatus(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg) return;
    // Bug6 fix: 锁内收集状态，锁外执行回调，避免持锁调用外部代码
    std::function<void()> cb;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        bool new_status = msg->data;
        auxiliary_status_ = (TftUserConfig::instance().getSafetyWarningEnabled() == 1) ? true : false;
        if (auxiliary_status_ != new_status) {
            auxiliary_status_ = new_status;
            if (on_auxiliary_status_changed_) {
                bool s = auxiliary_status_;
                cb = [this, s]() { on_auxiliary_status_changed_(s); };
            }
        }
    }
    if (cb) cb();
}

void TftStateMachine::onCollisionWarning(const interface::msg::CollisionWarning::SharedPtr msg) {
    if (!msg) return;
    RCLCPP_INFO(rclcpp::get_logger("TftStateMachine"), "onCollisionWarning: type=%d level=%d", msg->warning_type, msg->warning_level);

    // Bug6 fix: 锁内仅更新共享状态并收集回调，锁外执行，避免持锁调用外部代码引发死锁
    std::function<void()> cb;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const int16_t wt = msg->warning_type;
        const int16_t wl = msg->warning_level;
        if (wt == static_cast<int16_t>(WarningType::FORWARD)) {
            collision_warning_ = *msg;
            if (on_forward_brake_deceleration_changed_) {
                cb = [this, wl]() { on_forward_brake_deceleration_changed_(wl); };
            }
        } else if (wt == static_cast<int16_t>(WarningType::BACKWARD)) {
            collision_warning_ = *msg;
            if (on_backward_brake_deceleration_changed_) {
                cb = [this, wl]() { on_backward_brake_deceleration_changed_(wl); };
            }
        } else if (wt == static_cast<int16_t>(WarningType::LEFT)) {
            collision_warning_ = *msg;
            if (on_left_brake_deceleration_changed_) {
                cb = [this, wl]() { on_left_brake_deceleration_changed_(wl); };
            }
        } else if (wt == static_cast<int16_t>(WarningType::RIGHT)) {
            collision_warning_ = *msg;
            if (on_right_brake_deceleration_changed_) {
                cb = [this, wl]() { on_right_brake_deceleration_changed_(wl); };
            }
        }
    }
    if (cb) cb();
    RCLCPP_INFO(rclcpp::get_logger("TftStateMachine"), "onCollisionWarning end");
}

void TftStateMachine::onDeviceBindStatus(const interface::msg::VehicleBindStatus::SharedPtr msg) {
    if (!msg) return;
    RCLCPP_INFO(rclcpp::get_logger("TftStateMachine"), "onDeviceBindStatus: status=%d", msg->status);
    std::function<void()> cb;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        int new_status = msg->status; // 0 未绑定 1 已绑定 2 绑定失败
        std::string new_user_id = msg->user_id;
        RCLCPP_INFO(rclcpp::get_logger("TftStateMachine"), "onDeviceBindStatus: status=%d user_id=%s ,device_bind_status_=%d", new_status, new_user_id.c_str(), device_bind_status_ );
        if (device_bind_status_ != new_status) {
            device_bind_status_ = new_status;
            if (on_device_bind_status_changed_) {
                int s = device_bind_status_;
                std::string privacy_policy = msg->privacy_policy;
                cb = [this, s, new_user_id, privacy_policy]() { on_device_bind_status_changed_(s, new_user_id, privacy_policy); };
            }
        }
    }
    if (cb) cb();
}

std::optional<interface::msg::CarInfo> TftStateMachine::getCarInfo() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return car_info_;
}

std::string TftStateMachine::getSceneMode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return scene_mode_;
}

std::set<uint32_t> TftStateMachine::getActiveErrorCodes() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_error_codes_;
}

std::map<uint8_t, std::vector<ErrorEventEntry>> TftStateMachine::getActiveErrorsBySystem() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::map<uint8_t, std::vector<ErrorEventEntry>> out;
    for (uint32_t key : active_error_codes_) {
        ErrorEventEntry e;
        e.system = static_cast<uint8_t>((key >> 16) & 0xFF);
        e.module = static_cast<uint8_t>((key >> 8) & 0xFF);
        e.code = static_cast<uint8_t>(key & 0xFF);
        e.occurred = true;
        out[e.system].push_back(e);
    }
    return out;
}

bool TftStateMachine::hasCanError() const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (uint32_t key : active_error_codes_) {
        uint8_t ss = static_cast<uint8_t>((key >> 16) & 0xFF);
        if (isCanSystem(ss)) return true;
    }
    return false;
}

bool TftStateMachine::hasLidarError() const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (uint32_t key : active_error_codes_) {
        uint8_t ss = static_cast<uint8_t>((key >> 16) & 0xFF);
        if (isLidarSystem(ss)) return true;
    }
    return false;
}

bool TftStateMachine::hasErrorOfSystem(uint8_t system) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (uint32_t key : active_error_codes_) {
        uint8_t ss = static_cast<uint8_t>((key >> 16) & 0xFF);
        if (ss == system) return true;
    }
    return false;
}

bool TftStateMachine::hasJoystickBackwardEvent() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return joystick_event_ == "backward";
}

std::string TftStateMachine::getJoystickEvent() const {
    return joystick_event_;
}

std::string TftStateMachine::gearNumberToStr(int gear) const {
    bool is_en = (TftUserConfig::instance().getLanguage() == Language::EN);
    switch (gear) {
        case 1: return is_en ? "Gear 1" : "一档";
        case 2: return is_en ? "Gear 2" : "二档";
        case 3: return is_en ? "Gear 3" : "三档";
        case 4: return is_en ? "Gear 4" : "四档";
        default: return is_en ? "Unknown" : "未知";
    }
}

int TftStateMachine::gearNumberToIcon(int lockswitch) const {
    if(lockswitch == 1) {
        return 2; // 驻车
    } else {
        return 0;
    }
}

std::string TftStateMachine::refreshLanguageStrings() {
    std::lock_guard<std::mutex> lock(mutex_);
    bool is_en = (TftUserConfig::instance().getLanguage() == Language::EN);

    // 重新计算档位字符串
    if (car_info_) {
        scene_gear_str_ = gearNumberToStr(car_info_->gear);
    } else {
        scene_gear_str_ = gearNumberToStr(TftUserConfig::instance().getGearPosition());
    }

    // 重新计算场景模式字符串
    if (car_info_ && car_info_->lockswitch == 0) {
        scene_mode_str_ = (scene_mode_ == "helpcar")
            ? (is_en ? "Pushing Mode"  : "推行模式")
            : (is_en ? "Driving Mode" : "驾驶模式");
    } else {
        scene_mode_str_ = is_en ? "Driving Mode" : "驾驶模式";
    }

    scene_mode_gear_str_ = scene_gear_str_ + " | " + scene_mode_str_;
    return scene_mode_gear_str_;
}

    // 动态调整D、P档灯光
void TftStateMachine::adaptGearIconLight()
{
    // 倒车灯效在tft_service中
    if (gear_icon_ == 2)
    {
        LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_PARKING);
    }
    else if (gear_icon_ == 0)
    {
        LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_JOYSTICK);
    }
}
    // 动态调整抱闸灯光
void TftStateMachine::adaptBrakeLight()
{
    // 调整抱闸场景灯光
    // 物理抱闸关闭之后，如果电子抱闸开启，灯光切换至推行模式。否则就切换至摇杆模式
    if (brake_status_ == 1)
    {
        LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_BRAKE_FAILURE);
    }
    else
    {
        if (isPushingMode())
        {
            LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_PUSHING);
        }
        else
        {
            LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_JOYSTICK);
        }
    }
}
// 根据充电结束的状态调整灯效
void TftStateMachine::adaptChargingLight(const int &charing_status) {
    if(charing_status == 1 ||charing_status == 2) {
        LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_CHARGING);
    } else if (charing_status == 0) {
        if (isPushingMode()) {
            LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_PUSHING);
        } else if (isParkingMode()) {
            LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_PARKING);
        } else {
            LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_JOYSTICK);
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("TftStateMachine"), "充电状态错误，不处理灯效：%d", charing_status);
    }
}
}  // namespace tft_service
