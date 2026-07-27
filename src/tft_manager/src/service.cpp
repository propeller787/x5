/**
 * @file service.cpp
 * @brief TftService 实现：统一订阅 ROS 话题，分发至各 Control，处理物理按键与异常弹窗。
 * 触屏事件由 TftModule::handleDcBusFrame 接收，通过 setTouchEventCallback 联动 PageDialogManager。
 */

#include "service.h"
#include "dialog/page_dialog_manager.h"
#include "ota_manager.h"
#include "recharge_manager.h"
#include "physical_button_handler.h"
#include "tft_user_config.h"
#include "tft_state_machine.h"
#include "followme_manager.h"
#include "laneWarning/lane_warning_manager.hpp"
#include "control/light_scene_controller.h"
#include "control/define.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <utils/topic.h>
#include <algorithm>
#include <chrono>
#include <ctime>
using namespace std_msgs::msg;
namespace tft_service {

using namespace interface::msg;
using namespace interface::srv;
using std::placeholders::_1;

TftService::TftService(const rclcpp::NodeOptions& options)
    : rclcpp::Node("TftService", options)
    , logger_(rclcpp::get_logger("TftService"))
{
    if (!TftUserConfig::instance().load()) {
        RCLCPP_WARN(logger_, "TftUserConfig load failed, using defaults");
    } else {
        RCLCPP_INFO(logger_, "TftUserConfig loaded from %s",
            TftUserConfig::instance().getConfigPath().c_str());
    }
    physical_button_failure_sub_ = create_subscription<std_msgs::msg::Int32>(physicalButtonFailureTopic, 10, std::bind(&TftService::onPhysicalButtonFailure, this, _1));
    // 监听车道警告点消息
    lane_warning_point_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        laneWarningPointTopic, 10, std::bind(&TftService::onLaneWarningPoint, this, _1));
    // 延后获取 SN/蓝牙 MAC：在定时器回调里用异步服务调用（不能 wait_for，否则会阻塞 executor 导致收不到响应）。
    using namespace std::chrono_literals;
    fetch_sn_mac_timer_ = create_wall_timer(5000ms, [this]() {
        fetch_sn_mac_timer_->cancel();
        if (TftUserConfig::instance().getSnCode().empty() || TftUserConfig::instance().getBluetoothMac().empty()) {
            fetchSnAndBluetoothMacAsync();
        }
    });
    TftStateMachine::instance().init(this);
    LightSceneController::instance().init(this);
    OtaManager::instance().init(this);
    LaneWarningManager::instance().init(this);
    TftStateMachine::instance().setDeviceBindStatus(TftUserConfig::instance().getAppBindPageState(), TftUserConfig::instance().getUsername());

    // 创建 TftModule 与 PageDialogManager，并联动触屏事件
    setupTouchEventWiring();
}

void TftService::pushCollisionWarningLevelsToLightController(int16_t value){
    LightSceneController::instance().updateCollisionWarningStatus(value > 0);
}

void TftService::fetchSnAndBluetoothMacAsync() {
    using namespace std::chrono_literals;
    auto id_client = create_client<interface::srv::IdControls>(idControlService);
    auto mac_client = create_client<interface::srv::BluetoothMac>(bluetoothMacService);
    if (!id_client->wait_for_service(5s)) {
        RCLCPP_WARN(logger_, "idControlService (%s) not available, retry in 3s. Ensure sensor node is running.", idControlService);
        fetch_sn_mac_timer_ = create_wall_timer(3000ms, [this]() {
            fetch_sn_mac_timer_->cancel();
            fetchSnAndBluetoothMacAsync();
        });
        return;
    }
    auto id_req = std::make_shared<interface::srv::IdControls::Request>();
    id_req->id = 0;
    // 必须捕获 id_client、mac_client，否则函数返回后 client 被销毁，响应到达时回调无法执行
    id_client->async_send_request(id_req, [this, id_client, mac_client](rclcpp::Client<interface::srv::IdControls>::SharedFuture future) {
        std::string sn;
        try {
            auto resp = future.get();
            if (resp->code == 0) {
                sn = resp->sn;
            } else {
                RCLCPP_WARN(logger_, "IdControls returned code=%d (expected 0)", resp->code);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "IdControls exception: %s", e.what());
        }
        if (!mac_client->service_is_ready()) {
            // Bug3 fix: 不能在 async 回调（executor线程）中调用阻塞式 wait_for_service()，
            // 否则会卡死 SingleThreadedExecutor 长达 2s，期间所有话题/定时器无响应。
            // 改用非阻塞的 service_is_ready()，不可用时重新发起 3s 后的定时重试。
            RCLCPP_WARN(logger_, "bluetoothMacService (%s) not available. SN=%s. Ensure sensor/chassis is running.", bluetoothMacService, sn.c_str());
            fetch_sn_mac_timer_ = create_wall_timer(std::chrono::milliseconds(3000), [this]() {
                fetch_sn_mac_timer_->cancel();
                fetchSnAndBluetoothMacAsync();
            });
            return;
        }
        TftUserConfig::instance().setSnCode(sn);
        auto mac_req = std::make_shared<interface::srv::BluetoothMac::Request>();
        // 捕获 mac_client 保持其生命周期，否则响应到达前 client 被销毁
        mac_client->async_send_request(mac_req, [this, sn, mac_client](rclcpp::Client<interface::srv::BluetoothMac>::SharedFuture future) {
            std::string bluetooth_mac;
            try {
                auto resp = future.get();
                if (resp->code == 0) {
                    bluetooth_mac = resp->bluetooth_mac;
                } else {
                    RCLCPP_WARN(logger_, "BluetoothMac returned code=%d (expected 0)", resp->code);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(logger_, "BluetoothMac exception: %s", e.what());
            }
            if(bluetooth_mac != "") {
                TftUserConfig::instance().setBluetoothMac(bluetooth_mac);
            }
        });
    });
}

void TftService::setupTouchEventWiring() {
    std::string device = declare_parameter<std::string>("tft_device", "/dev/ttyS4");
    tft_module_ = std::make_shared<TftModule>(device, true, 0xAA55, STATION_ADDRESS);

    // tft_module_->isConnected();

    page_dialog_manager_ = std::make_unique<PageDialogManager>(tft_module_.get());
    page_dialog_manager_->setNode(this);

    std::string config_path;
    try {
        config_path = ament_index_cpp::get_package_share_directory("tft_manager") + "/config/pages_dialogs_toasts.yaml";
    } catch (const std::exception&) {
        RCLCPP_WARN(logger_, "Cannot get tft_manager share dir, pages_dialogs config may be missing");
    }
    if (!config_path.empty() && page_dialog_manager_->loadConfig(config_path)) {
        RCLCPP_INFO(logger_, "PageDialogManager loaded config: %s", config_path.c_str());
        // 初始化页面
        int initial = PageDialogManager::getInitialPageOnStartup();
        page_dialog_manager_->setCurrentPage(initial);
        page_dialog_manager_->switchToPage(initial);
        RCLCPP_DEBUG(logger_, "initial: %d", initial);
        initial_page_was_main_ = (initial == MAIN_PAGE_ID);
    } else {
        RCLCPP_WARN(logger_, "PageDialogManager config load failed or path empty");
    }

    // 从非主界面切回主界面时，刷新档位+功能模式字符串（与设置页语言切换、syncMainPageInitialData 一致）
    if (page_dialog_manager_) {
        page_dialog_manager_->setOnPageSwitchCallback([this](int from_page, int to_page) {
            if (to_page != MAIN_PAGE_ID) return;
            if (from_page == MAIN_PAGE_ID) return;
            sendGearAndFunctionModeTextToTft();
        });
    }

    // 设置页触屏：模式切换、声音开关 -> 发布对应 topic
    tft_car_mode_pub_ = create_publisher<std_msgs::msg::Int32>(tftCarModeTopic, 10);
    tft_sound_switch_pub_ = create_publisher<std_msgs::msg::Int32>(tftSoundSwitchTopic, 10);
    tft_language_pub_ = create_publisher<std_msgs::msg::Int32>(tftLanguageTopic, 10);
    tft_touch_event_pub_ = create_publisher<std_msgs::msg::Bool>(tftTouchEventTopic, 10);
    tft_audio_ = std::make_unique<TftAudioPublisher>();
    tft_audio_->init(this);
    RechargeManager::instance().init(this);
    FollowmeManager::instance().init(this);
    // 回充场景 TTS 由 RechargeManager::attachStateMachineCallbacks() 注册，勿在此处重复 setOnRechargeSceneChanged（后者会覆盖前者）

    // 触屏事件联动：handleDcBusFrame 收到 F2 读请求时调用 onTouchEvent
    tft_module_->setTouchEventCallback([this](uint16_t reg) {
        RCLCPP_DEBUG(rclcpp::get_logger("TftService"), "onTouchEvent: %u", reg);
        if (page_dialog_manager_) {
            updateBindingDownloadStatus(reg);
            page_dialog_manager_->onTouchEvent(reg);
            OtaManager::instance().handleTftTouchEvent(reg);
        }
    });
    tft_module_->setReadResponseHandler([this](uint16_t reg, uint16_t value) {
        // DCBUS F2 读寄存器响应（含数据）时发布 Bool：非零为 true，零为 false
        if (tft_touch_event_pub_) {
            Bool touch_evt;
            touch_evt.data = (value != 0);
            tft_touch_event_pub_->publish(touch_evt);
        }
        onRechargeUpSlideButton(reg, value);
        RechargeManager::instance().onReadResponse(reg, value);
        FollowmeManager::instance().onReadResponse(reg, value);
        onSettingPageTouch(reg, value);
    });

    // 物理按键与弹窗联动：订阅 /vlago/physical/button/failure、/vlago/safety/function、/vlago/cruise/mode
    physical_button_handler_ = std::make_unique<PhysicalButtonHandler>(this, page_dialog_manager_.get(), tft_audio_.get());

    // OtaManager 页面切换与 TFT 写入回调：OTA 流程中需要切换页面或更新显示时调用
    OtaManager::instance().setOnSwitchPageRequested([this](int page_id) {
        if (page_dialog_manager_){
            page_dialog_manager_->switchToPage(page_id);
        }
    });
    OtaManager::instance().setOnShowDialogRequested([this](const std::string& dialog_id, int page_id) {
        if (page_dialog_manager_){
            std::cout << "showDialog: " << dialog_id << " page_id: " << page_id << std::endl;
            page_dialog_manager_->showDialog(dialog_id, page_id);
        }
    });
    OtaManager::instance().setOnWriteTftValue([this](uint16_t addr, uint16_t value) {
        if (tft_module_ && tft_module_->isConnected()) tft_module_->sendDcBusWrite(STATION_ADDRESS, addr, value);
    });
    OtaManager::instance().setOnWriteTftText([this](uint16_t addr, const std::string& text) {
        if (tft_module_ && tft_module_->isConnected()) tft_module_->sendDcBusTextWrite(STATION_ADDRESS, addr, text);
    });
    OtaManager::instance().setOnWriteTftFloat([this](uint16_t addr, float value) {
        if (tft_module_ && tft_module_->isConnected()) tft_module_->sendDcBusFloatWrite(STATION_ADDRESS, addr, value);
    });
    TftStateMachine::instance().setPageDialogManager(page_dialog_manager_.get());
    RechargeManager::instance().setTftAudio(tft_audio_.get());
    RechargeManager::instance().setPageDialogManager(page_dialog_manager_.get());
    RechargeManager::instance().setOnSwitchPage([this](int page_id) {
        if (page_dialog_manager_) page_dialog_manager_->switchToPage(page_id);
    });
    RechargeManager::instance().setTftConnectedPredicate([this]() {
        return tft_module_ && tft_module_->isConnected();
    });
    RechargeManager::instance().setChargingPageUpdateGate([this]() {
        if (!page_dialog_manager_) return true;
        return page_dialog_manager_->getCurrentPage() == MAIN_PAGE_ID;
    });
    RechargeManager::instance().attachStateMachineCallbacks();
    //follow_me
    FollowmeManager::instance().setTftAudio(tft_audio_.get());
    FollowmeManager::instance().setPageDialogManager(page_dialog_manager_.get());
    FollowmeManager::instance().setOnSwitchPage([this](int page_id) {
        if (page_dialog_manager_) page_dialog_manager_->switchToPage(page_id);
    });
    FollowmeManager::instance().setTftConnectedPredicate([this]() {
        return tft_module_ && tft_module_->isConnected();
    });
    FollowmeManager::instance().setChargingPageUpdateGate([this]() {
        if (!page_dialog_manager_) return true;
        return page_dialog_manager_->getCurrentPage() == MAIN_PAGE_ID;
    });

    // 倒车状态：true 打开倒车页（排除 app下载/绑定/绑定成功），false 退出倒车页返回上一页
    auto qos = rclcpp::QoS(10);
    backup_car_sub_ = create_subscription<interface::msg::BackupCar>(
        backupCarTopic, qos, [this](const interface::msg::BackupCar::SharedPtr msg) { onBackupCar(msg); });

    // 安全功能状态：true 开启 false 关闭 -> 写入 TFT 0x100B
    // auxiliary_status_sub_ = create_subscription<std_msgs::msg::Bool>(
    //     auxiliaryStatusTopic, qos, [this](const std_msgs::msg::Bool::SharedPtr msg) { onAuxiliaryStatus(msg); });

    // 功能模式显示：scene_mode_str 变化时回调更新 TFT（档位 + 功能模式）
    TftStateMachine::instance().setOnSceneModeStrChanged([this](const std::string& s) {
        if (!tft_module_ || !tft_module_->isConnected()) return;
        if (!page_dialog_manager_) return;
        tft_module_->sendDcBusTextWrite(STATION_ADDRESS, GEAR_AND_FUNCTION_MODE_ADDRESS, s);
        checkPushingOrJoystickToast(s);
    });

    // 剩余里程显示：remaining_mileage_ 变化时回调更新 TFT
    TftStateMachine::instance().setOnRemainingMileageChanged([this](int mileage) {
        if (!tft_module_ || !tft_module_->isConnected()) return;
        if (!page_dialog_manager_) return;
        tft_module_->sendDcBusWrite(STATION_ADDRESS, REMAINING_MILEAGE_ADDRESS, mileage);
    });

    // 剩余电量显示：remaining_electricity_ 变化时回调更新 TFT，同时判断一次电量 toast
    TftStateMachine::instance().setOnRemainingElectricityChanged([this](int electricity) {
        if (!tft_module_ || !tft_module_->isConnected()) return;
        if (!page_dialog_manager_) return;
        tft_module_->sendDcBusWrite(STATION_ADDRESS, BATTERY_LEVEL_ADDRESS, electricity);
        checkLowPowerToast(static_cast<int16_t>(electricity));
    });

    // 速度显示：speed_ 变化时回调更新 TFT
    TftStateMachine::instance().setOnSpeedChanged([this](float speed) {
        if (!tft_module_ || !tft_module_->isConnected()) return;
        if (!page_dialog_manager_) return;
        tft_module_->sendDcBusFloatWrite(STATION_ADDRESS, SPEED_ADDRESS, speed);
    });

    // 档位图标显示：gear_icon_ 变化时回调更新 TFT
    TftStateMachine::instance().setOnGearIconChanged([this](int icon) {
        if (!tft_module_ || !tft_module_->isConnected()) return;
        if (!page_dialog_manager_) return;
        checkParkingToast(icon);
        tft_module_->sendDcBusWrite(STATION_ADDRESS, GEAR_STATUS_ADDRESS, icon);
        // 记录档位（仅保存 1-4 档）
        auto car_info = TftStateMachine::instance().getCarInfo();
        int gear_num = car_info ? static_cast<int>(car_info->gear) : 1;
        TftUserConfig::instance().setGearPosition(gear_num);
    });

    // 安全功能状态显示：auxiliary_status_ 变化时回调更新 TFT
    TftStateMachine::instance().setOnAuxiliaryStatusChanged([this](bool status) {
        if (!tft_module_ || !tft_module_->isConnected()) return;
        if (!page_dialog_manager_) return;
        tft_module_->sendDcBusWrite(STATION_ADDRESS, SAFETY_FUNCTION_STATUS_ADDRESS, status ? 1 : 0);
        TftUserConfig::instance().setSafetyWarningEnabled(status ? 1 : 0);
        checkSmartAvoidanceToast(status);
    });

    // 巡航模式显示：cruise_mode_ 变化时回调更新 TFT
    TftStateMachine::instance().setOnCruiseModeChanged([this](bool mode) {
        if (!tft_module_ || !tft_module_->isConnected()) return;
        if (!page_dialog_manager_) return;
        tft_module_->sendDcBusWrite(STATION_ADDRESS, FUNCTION_STATUS_ADDRESS, mode ? 1 : 0);
        checkCruiseControlToast(mode);
    });

    // 蓝牙控车状态显示：bluetooth_ctrl_mode_ 变化时回调更新 TFT
    TftStateMachine::instance().setOnBluetoothCtrlModeChanged([this](bool mode) {
        int value = mode ? 2 : 0;
        if (!tft_module_ || !tft_module_->isConnected()) return;
        if (!page_dialog_manager_) return;
        tft_module_->sendDcBusWrite(STATION_ADDRESS, FUNCTION_STATUS_ADDRESS, value);
        checkBluetoothRemoteToast(mode);
    });
    // 充电状态显示：charging_status_ 变化时回调更新 TFT
    // TftStateMachine::instance().setOnChargingStatusChanged([this](int status) {
    //     if (tft_audio_ && last_charging_status_for_audio_ >= 0 &&
    //         last_charging_status_for_audio_ == static_cast<int>(ChargingConnectStatus::NotCharging) &&
    //         (status == static_cast<int>(ChargingConnectStatus::BaseStationCharging))) {
    //         tft_audio_->playTtsCode(TTS_CODE_TYPE_CHARGING);
    //     }
    //     last_charging_status_for_audio_ = status;
    //     if (!tft_module_ || !tft_module_->isConnected()) return;
    //     if (page_dialog_manager_ && page_dialog_manager_->getCurrentPage() != MAIN_PAGE_ID) return;
    //     switch (status) {
    //         case 0:
    //             page_dialog_manager_->switchToPage(MAIN_PAGE_ID);
    //             break;
    //         case 1:
    //             page_dialog_manager_->switchToPage(LINE_CHARGE_PAGE_ID);
    //             break;
    //         case 2:
    //             page_dialog_manager_->switchToPage(BASE_STATION_CHARGE_PAGE_ID);
    //             break;
    //     }
    // });

    // 前向制动/减速状态显示：collision_warning_ 变化时回调更新 TFT
    TftStateMachine::instance().setOnForwardBrakeDecelerationChanged([this](int value) {
        if (tft_audio_) tft_audio_->onCollisionWarningLevel(CollisionAudioTrack::Forward, value);
        pushCollisionWarningLevelsToLightController(value);
        if (!tft_module_ || !tft_module_->isConnected()) return;
        if (page_dialog_manager_ && page_dialog_manager_->getCurrentPage() != MAIN_PAGE_ID) return;
        checkForwardBrakeDecelerationAnimation(value);
    });

    // 后向制动/减速状态显示：collision_warning_ 变化时回调更新 TFT
    TftStateMachine::instance().setOnBackwardBrakeDecelerationChanged([this](int value) {
        if (tft_audio_) tft_audio_->onCollisionWarningLevel(CollisionAudioTrack::Backward, value);
        pushCollisionWarningLevelsToLightController(value);
        if (!tft_module_ || !tft_module_->isConnected()) return;
        if (page_dialog_manager_ && page_dialog_manager_->getCurrentPage() != BACKWARD_PAGE_ID) return;
        checkBackwardBrakeDecelerationAnimation(value);
    });
    
    // 左向制动/减速状态显示：collision_warning_ 变化时回调更新 TFT
    TftStateMachine::instance().setOnLeftBrakeDecelerationChanged([this](int value) {
        if (tft_audio_) tft_audio_->onCollisionWarningLevel(CollisionAudioTrack::Left, value);
        pushCollisionWarningLevelsToLightController(value);
        if (!tft_module_ || !tft_module_->isConnected()) return;
        if (page_dialog_manager_ && page_dialog_manager_->getCurrentPage() != MAIN_PAGE_ID) return;
        checkLeftBrakeDecelerationAnimation(value);
    });
    
    // 右向制动/减速状态显示：collision_warning_ 变化时回调更新 TFT
    TftStateMachine::instance().setOnRightBrakeDecelerationChanged([this](int value) {
        if (tft_audio_) tft_audio_->onCollisionWarningLevel(CollisionAudioTrack::Right, value);
        pushCollisionWarningLevelsToLightController(value);
        if (!tft_module_ || !tft_module_->isConnected()) return;
        if (page_dialog_manager_ && page_dialog_manager_->getCurrentPage() != MAIN_PAGE_ID) return;
        checkRightBrakeDecelerationAnimation(value);
    });

    // 抱闸状态显示：brake_status_ 变化时回调更新 TFT
    TftStateMachine::instance().setOnBrakeStatusChanged([this](int value) {
        if (!tft_module_ || !tft_module_->isConnected()) return;
        if (!page_dialog_manager_) return;
        tft_module_->sendDcBusWrite(STATION_ADDRESS, BRAKE_FAILURE_ADDRESS, value);
        // 抱闸Toast
        checkBrakeStatusToast(value);
    });
    // 摇杆失效状态显示：joystick_failure_type_ 变化时回调更新 TFT
    TftStateMachine::instance().setOnJoystickFailureTypeChanged([this](joystick_failure_type type) {
        if (!tft_module_ || !tft_module_->isConnected()) return;
        if (page_dialog_manager_ && page_dialog_manager_->getCurrentPage() != MAIN_PAGE_ID) return;
        checkJoystickFailureToast(type);
    });

    // 设备绑定状态显示：device_bind_status_ 变化时回调更新 TFT
    TftStateMachine::instance().setOnDeviceBindStatusChanged([this](int value, const std::string& user_id, const std::string& privacy_policy) {
        if (!tft_module_ || !tft_module_->isConnected())
        {
            return;
        }
        if (!page_dialog_manager_)
        {
            return;
        }
        int app_bind_state = TftUserConfig::instance().getAppBindPageState();
        // 保存绑定状态和隐私协议
        TftUserConfig::instance().setAppBindPageState(value == 1 ? 2 : 0);
        TftUserConfig::instance().setPrivacyPolicy(privacy_policy);
        if(value == 0) {
            TftUserConfig::instance().setUsername("未绑定");
            if (app_bind_state != 2) {
                page_dialog_manager_->switchToPage(APP_DOWNLOAD_QR_CODE_PAGE_ID);
            } else {
                page_dialog_manager_->switchToPage(VEHICLE_BINDING_QR_CODE_PAGE_ID);
            } 
        }else if(value == 1) {
            TftUserConfig::instance().setUsername(user_id);
            // 只有当未绑定时，下发绑定成功才会展示绑定成功界面。如果此时为已绑定状态，则直接跳转到主界面
            if (app_bind_state != 2) {
                page_dialog_manager_->switchToPage(BINDING_SUCCESS_PAGE_ID);
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            }
            page_dialog_manager_->switchToPage(MAIN_PAGE_ID);
        }
    });

    // 倒车引导线：向 BACKWARD_LINE_ADDRESS(0x2000) 发送
    // 帧格式：0006 + F800 + 6个点(x,y)，每个坐标 16bit 大端
    TftStateMachine::instance().setOnReverseGuideCurvesChanged([this](const TftStateMachine::ReverseGuideCurves& curves) {
        return ;
        if (!curves.valid) return;
        if (!tft_module_ || !tft_module_->isConnected()) return;
        if (!page_dialog_manager_ || page_dialog_manager_->getCurrentPage() != BACKWARD_PAGE_ID) return;

        auto build_payload = [](const TftStateMachine::GuideCurve& curve) {
            std::vector<uint8_t> payload;
            payload.reserve(2 + 2 + TftStateMachine::kReverseCurvePointCount * 4);
            auto push_u16 = [&payload](uint16_t v) {
                payload.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
                payload.push_back(static_cast<uint8_t>(v & 0xFF));
            };
            push_u16(0x0020);  // 曲线拟合
            push_u16(0x0006);  // 点数
            push_u16(0xF800);  // RGB565 红色
            for (const auto& point : curve) {
                const int x = std::clamp(static_cast<int>(point.first), 0, 65535);
                const int y = std::clamp(static_cast<int>(point.second), 0, 65535);
                push_u16(static_cast<uint16_t>(x));
                push_u16(static_cast<uint16_t>(y));
            }
            return payload;
        };

        tft_module_->sendDcBusWritePayload(STATION_ADDRESS, BACKWARD_LINE_ADDRESS, build_payload(curves.left));
        tft_module_->sendDcBusWritePayload(STATION_ADDRESS, BACKWARD_LINE_ADDRESS, build_payload(curves.right));
    });

    cruise_end_reason_audio_sub_ = create_subscription<std_msgs::msg::Int32>(
        cursorAlarmTopic, qos, [this](const std_msgs::msg::Int32::SharedPtr msg) {
            if (!msg || !tft_audio_) return;
            tft_audio_->onCruiseEndReason(static_cast<CruiseEndAudioReason>(msg->data));
        });

    setupPublishers();

    // 车辆启动后同步系统时间到 TFT 屏（延迟 2 秒等待串口就绪，仅执行一次）
    tft_time_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {
        sendTftTime();
        if (initial_page_was_main_) {
            syncMainPageInitialData();
        }
        tft_time_timer_->cancel();
    });
}

void TftService::onSettingPageTouch(uint16_t reg, uint16_t value) {
    // 5.19新增，绑定界面下允许切换中英文
    if (!page_dialog_manager_ || (page_dialog_manager_->getCurrentPage() != SETTING_PAGE_ID
        && page_dialog_manager_->getCurrentPage() != APP_DOWNLOAD_QR_CODE_PAGE_ID 
        && page_dialog_manager_->getCurrentPage() != VEHICLE_BINDING_QR_CODE_PAGE_ID)) {
        return;
    }
    if (reg == TOUCH_EVENT_MODE_SWITCH) {
        // 驾驶模式和巡航模式开启的时候不响应切换
        std::string scene_mode = TftStateMachine::instance().getSceneMode();
        float speed = TftStateMachine::instance().getSpeedKmh();
        // tips: 这是切换失败是不是要加一个toast或者语音提醒之类的？  
        if(scene_mode == "cruise" || scene_mode == "recharge" || std::abs(speed) > 0.1){
            RCLCPP_INFO(logger_, "当前驾驶模式为：%s，速度为：%f。不支持切换舒适或者运动模式", scene_mode.c_str(), speed);
            return;
        }
        uint16_t mode = TftUserConfig::instance().getVehicleMode();
        uint16_t next = value == 1 ? 2 : 1;
        TftUserConfig::instance().setVehicleMode(static_cast<int>(next));
        Int32 msg;
        msg.data = next;
        tft_car_mode_pub_->publish(msg);
        RCLCPP_INFO(logger_, "TFT 模式切换: %d -> %d", mode, next);
    } else if (reg == TOUCH_EVENT_SOUND_SWITCH) {
        uint16_t sound = TftUserConfig::instance().getSoundOn();
        uint16_t next = value == 1 ? 1 : 0;  // 1 打开 <-> 0 关闭
        TftUserConfig::instance().setSoundOn(next);
        Int32 msg;
        msg.data = next;
        tft_sound_switch_pub_->publish(msg);
        RCLCPP_INFO(logger_, "TFT 声音开关: %d -> %d", sound, next);
    } else if (reg == LANGUAGE_RECEIVE_ADDRESS) {
        Language lang_before = TftUserConfig::instance().getLanguage();
        Language lang_next = (value == 1) ? Language::EN : Language::CN;
        TftUserConfig::instance().setLanguage(lang_next);
        Int32 msg;
        msg.data = static_cast<int32_t>(lang_next);
        tft_language_pub_->publish(msg);
        RCLCPP_INFO(logger_, "TFT 语言开关: %d -> %d",
                    static_cast<int>(lang_before), static_cast<int>(lang_next));
        // 刷新档位+功能模式字符串
        sendGearAndFunctionModeTextToTft();
        // 若当前有弹窗显示，立即以新语言刷新标题与正文
        if (page_dialog_manager_ && page_dialog_manager_->isDialogShowing()) {
            page_dialog_manager_->refreshCurrentDialogTexts();
        }
    }
}

void TftService::onBackupCar(const interface::msg::BackupCar::SharedPtr msg) {
    if (!msg || !page_dialog_manager_) return;
    int cur = page_dialog_manager_->getCurrentPage();
    if (msg->is_backup_car) {
        // 倒车：若当前为 app下载(1)、app绑定(2)、绑定成功(3) 则忽略
        if (cur == APP_DOWNLOAD_QR_CODE_PAGE_ID || cur == VEHICLE_BINDING_QR_CODE_PAGE_ID ||
            cur == BINDING_SUCCESS_PAGE_ID) {
            return;
        }
        if (tft_audio_) {
            tft_audio_->playPromptTone(PROMPT_TONE_TYPE_ACTIVATION);
        }
        page_before_backup_ = cur;
        page_dialog_manager_->switchToPage(BACKWARD_PAGE_ID);
        // 只有在标准模式下，才会调整倒车灯效
        if (TftStateMachine::instance().isStandardMode()) {
            // 设置灯效场景
            LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_BACKWARD);
        }
        RCLCPP_INFO(logger_, "倒车进入: 页面 %d -> 倒车页", cur);
    } else {
        // 退出倒车：若当前在倒车页则返回上一页，并恢复摇杆灯效场景（否则仍为 BACKWARD，车速/预警灯不更新）
        if (cur == BACKWARD_PAGE_ID) {
            int target = (page_before_backup_ >= 0) ? page_before_backup_ : MAIN_PAGE_ID;
            page_dialog_manager_->switchToPage(target);
            if (TftStateMachine::instance().isPushingMode()) {
                LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_PUSHING);
            } else {
                LightSceneController::instance().applyScene(light_type_scene_type::LIGHT_TYPE_SCENE_TYPE_JOYSTICK);
            }
            RCLCPP_INFO(logger_, "倒车退出: 倒车页 -> 页面 %d", target);
            page_before_backup_ = -1;
        }
    }
}

// void TftService::onAuxiliaryStatus(const std_msgs::msg::Bool::SharedPtr msg) {
//     if (!msg || !tft_module_ || !tft_module_->isConnected()) return;
//     uint16_t value = msg->data ? 1 : 0;  // true -> 1 开启, false -> 0 关闭
//     tft_module_->sendDcBusWrite(STATION_ADDRESS, SAFETY_FUNCTION_STATUS_ADDRESS, value);
// }

void TftService::sendTftTime() {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    std::time_t now = std::time(nullptr);
    std::tm* lt = std::localtime(&now);
    if (!lt) return;
    int year = lt->tm_year + 1900;
    int month = lt->tm_mon + 1;
    int day = lt->tm_mday;
    int hour = lt->tm_hour;
    int minute = lt->tm_min;
    int second = lt->tm_sec;

    tft_module_->sendDcBusWrite(STATION_ADDRESS, YEAR_ADDRESS, static_cast<uint16_t>(year));
    tft_module_->sendDcBusWrite(STATION_ADDRESS, MONTH_ADDRESS, static_cast<uint16_t>(month));
    tft_module_->sendDcBusWrite(STATION_ADDRESS, DAY_ADDRESS, static_cast<uint16_t>(day));
    tft_module_->sendDcBusWrite(STATION_ADDRESS, HOUR_ADDRESS, static_cast<uint16_t>(hour));
    tft_module_->sendDcBusWrite(STATION_ADDRESS, MINUTE_ADDRESS, static_cast<uint16_t>(minute));
    tft_module_->sendDcBusWrite(STATION_ADDRESS, SECOND_ADDRESS, static_cast<uint16_t>(second));
    tft_module_->sendDcBusWrite(STATION_ADDRESS, SET_DATE_TIME_ADDRESS, SET_DATE_TIME_ADDRESS_2);

    RCLCPP_INFO(logger_, "TFT 时间已同步: %04d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, minute, second);
}

void TftService::syncMainPageInitialData() {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    RCLCPP_INFO(logger_, "主页面初始数据同步开始 0");
    auto info = TftStateMachine::instance().getCarInfo();
    RCLCPP_INFO(logger_, "主页面初始数据同步开始 1");
    // 语言开关
    tft_module_->sendDcBusWrite(STATION_ADDRESS, TOUCH_EVENT_LANGUAGE_SWITCH, static_cast<int32_t>(TftUserConfig::instance().getLanguage()));
    // 剩余里程
    uint16_t mileage = info ? static_cast<uint16_t>(info->remainingmileage > 0 ? info->remainingmileage : 0) : 0;
    tft_module_->sendDcBusWrite(STATION_ADDRESS, REMAINING_MILEAGE_ADDRESS, mileage);
    // 剩余电量
    int eq = info ? static_cast<int>(info->electricquantity) : 0;
    uint16_t battery = static_cast<uint16_t>(eq < 0 ? 0 : (eq > 100 ? 100 : eq));
    tft_module_->sendDcBusWrite(STATION_ADDRESS, BATTERY_LEVEL_ADDRESS, battery);
    // 速度：与运行时一致，来自 /wheel/twist（见 TftStateMachine::onWheelTwistStamped）
    float speed = TftStateMachine::instance().getSpeedKmh();
    tft_module_->sendDcBusFloatWrite(STATION_ADDRESS, SPEED_ADDRESS, speed);
    // 车辆模式：TftUserConfig 1 舒适 2 运动 -> TFT 0 舒适 1 运动
    int mode = TftUserConfig::instance().getVehicleMode();
    uint16_t tft_mode = (mode == 2) ? 1 : 0;
    tft_module_->sendDcBusWrite(STATION_ADDRESS, CAR_MODE_ADDRESS, tft_mode);
    // 安全功能：开机按已保存状态恢复
    tft_module_->sendDcBusWrite(STATION_ADDRESS, SAFETY_FUNCTION_STATUS_ADDRESS,
        static_cast<uint16_t>(TftUserConfig::instance().getSafetyWarningEnabled() ? 1 : 0));
    // 档位图标寄存器显示的是 D/R/P（由锁车状态驱动），无可靠锁车历史时仍默认 D。
    // 具体 1-4 档位会通过「档位+功能模式」文本按已保存配置恢复（见 refreshLanguageStrings）。
    tft_module_->sendDcBusWrite(STATION_ADDRESS, GEAR_STATUS_ADDRESS, 0);
    // 档位+功能模式：开机根据已保存语言决定初始文本
    sendGearAndFunctionModeTextToTft();
    // 蓝牙状态：开机默认打开
    tft_module_->sendDcBusWrite(STATION_ADDRESS, BLUETOOTH_STATUS_ADDRESS, 1);
    // 4G信号强度 
    tft_module_->sendDcBusWrite(STATION_ADDRESS, FOUR_G_SIGNAL_STRENGTH_ADDRESS, 3);
    // 声音开关：开机默认打开
    tft_module_->sendDcBusWrite(STATION_ADDRESS, SOUND_SWITCH_ADDRESS, TftUserConfig::instance().getSoundOn());
    // 回充上滑按钮：开机默认打开
    tft_module_->sendDcBusWrite(STATION_ADDRESS, RECHARGE_UP_SLIDE_BUTTON_ADDRESS, 0);
    // 车辆功能状态：开机默认 0 空
    tft_module_->sendDcBusWrite(STATION_ADDRESS, FUNCTION_STATUS_ADDRESS, 0);
    // 弹窗状态：开机默认关闭
    tft_module_->sendDcBusWrite(STATION_ADDRESS, PHYSICAL_BUTTON_FAILURE_OPEN_ADDRESS, 0);
     //TOUCH_EVENT_MODE_SWITCH
    tft_module_->sendDcBusWrite(STATION_ADDRESS, TOUCH_EVENT_MODE_SWITCH, TftUserConfig::instance().getVehicleMode() - 1);
    // 智能跟随开关
    tft_module_->sendDcBusWrite(STATION_ADDRESS, TOUCH_EVENT_SMART_FOLLOW_DISPLAY, TftUserConfig::instance().getFollowmeEnabled());
    // 广臣哥需求，设置按钮使能，为了测试当前默认写可用，待后续开发补充
    tft_module_->sendDcBusWrite(STATION_ADDRESS, SET_ENABLE_ADDRESS, 1);
    // 当前版本号
    tft_module_->sendDcBusTextWrite(STATION_ADDRESS, CURRENT_VERSION_ADDRESS, TftUserConfig::instance().getCurrentVersion());
    RCLCPP_INFO(logger_, "主页面初始数据已同步");
}

void TftService::sendGearAndFunctionModeTextToTft() {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    std::string s = TftStateMachine::instance().refreshLanguageStrings();
    tft_module_->sendDcBusTextWrite(STATION_ADDRESS, GEAR_AND_FUNCTION_MODE_ADDRESS, s);
    int gear_icon = TftStateMachine::instance().getGearIcon();
    tft_module_->sendDcBusWrite(STATION_ADDRESS, GEAR_STATUS_ADDRESS, gear_icon);
}

void TftService::setupPublishers() {
    Int32 msg;
    msg.data = 0xFF;
    tft_car_mode_pub_->publish(msg);
}

void TftService::updateBindingDownloadStatus(uint16_t reg) {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    switch (reg) {
        case TOUCH_EVENT_APP_DOWNLOAD_PAGE_DONT_DOWNLOAD:
            TftUserConfig::instance().setAppDownloadPageState(1);
            break;
        case TOUCH_EVENT_APP_DOWNLOAD_PAGE_DOWNLOAD_NEXT:
            TftUserConfig::instance().setAppDownloadPageState(2);
            break;
        case TOUCH_EVENT_VEHICLE_BINDING_PAGE_DONT_BIND:
            TftUserConfig::instance().setAppBindPageState(1);
            break;
        default:
            break;
    }
}
void TftService::onRechargeUpSlideButton(uint16_t reg, uint16_t value) {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    if (reg == RECHARGE_UP_SLIDE_BUTTON_ADDRESS && value == 1) {
        using namespace std::chrono_literals;
        if (recharge_up_slide_timer_) {
            recharge_up_slide_timer_->cancel();
        }
        recharge_up_slide_timer_ = create_wall_timer(3000ms, [this]() {
            if (tft_module_ && tft_module_->isConnected()) {
                tft_module_->sendDcBusWrite(STATION_ADDRESS, RECHARGE_UP_SLIDE_BUTTON_ADDRESS, 2);
                // 等待100ms
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                tft_module_->sendDcBusWrite(STATION_ADDRESS, RECHARGE_UP_SLIDE_BUTTON_ADDRESS, 0);
            }
            recharge_up_slide_timer_->cancel();
            recharge_up_slide_timer_.reset();
        });
    }
}
// 更新的toast 2秒后消失，如果有新的toast，覆盖上一个toast
void TftService::updateTftToastType(tft_toast_type type) {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    tft_module_->sendDcBusWrite(STATION_ADDRESS, TOAST_POPUP_ADDRESS, static_cast<uint16_t>(type));
    using namespace std::chrono_literals;
    if (tft_toast_type_timer_) {
        tft_toast_type_timer_->cancel();
    }
    tft_toast_type_timer_ = create_wall_timer(2000ms, [this]() {
        if (tft_module_ && tft_module_->isConnected()) {
            tft_module_->sendDcBusWrite(STATION_ADDRESS, TOAST_POPUP_ADDRESS,
                static_cast<uint16_t>(tft_toast_type::TFT_TOAST_TYPE_NONE));
        }
        tft_toast_type_timer_->cancel();
        tft_toast_type_timer_.reset();
    });
}

void TftService::checkParkingToast(int16_t value) {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    if (value == 2) {
        // 驻车
        updateTftToastType(tft_toast_type::TFT_TOAST_TYPE_PARKING);
    } else {
        // 驻车已解除
        updateTftToastType(tft_toast_type::TFT_TOAST_TYPE_UNPARKING);
    }
}
// 判断推行/摇杆toast显示(档位修改时不更新) 如果档位修改，发送档位toast(TFT_TOAST_TYPE_1/2/3_GEAR)
// 注意：不依赖字符串内容判断档位/模式，改用状态机数据，兼容中英文。
void TftService::checkPushingOrJoystickToast(std::string value) {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    // value 格式为 "档位 | 模式"，用 " | " 分隔
    const std::string sep = " | ";
    const size_t pos = value.find(sep);
    if (pos == std::string::npos) return;
    std::string gear_str = value.substr(0, pos);
    // 档位部分变化说明是档位修改，不更新推行/摇杆 toast
    if (!last_gear_str_.empty() && gear_str != last_gear_str_) {
        // 通过状态机获取实际档位号，避免比较中/英文字符串
        auto car_info = TftStateMachine::instance().getCarInfo();
        int gear_num = car_info ? static_cast<int>(car_info->gear) : 1;
        if (gear_num == 2) {
            updateTftToastType(tft_toast_type::TFT_TOAST_TYPE_2_GEAR);
        } else if (gear_num == 3) {
            updateTftToastType(tft_toast_type::TFT_TOAST_TYPE_3_GEAR);
        } else if (gear_num == 4) {
            updateTftToastType(tft_toast_type::TFT_TOAST_TYPE_4_GEAR);
        } else {
            updateTftToastType(tft_toast_type::TFT_TOAST_TYPE_1_GEAR);
        }
        last_gear_str_ = gear_str;
        return;
    }
    last_gear_str_ = gear_str;
    if (first_scene_mode_str_.empty()) {
        first_scene_mode_str_ = value;
        return;
    }
    // 通过状态机判断是否为推行模式，兼容中英文
    if (TftStateMachine::instance().isPushingMode()) {
        updateTftToastType(tft_toast_type::TFT_TOAST_TYPE_PUSHING_MODE);
        tft_audio_->playTtsCode(TTS_CODE_TYPE_PUSHING_MODE);
    } else {
        updateTftToastType(tft_toast_type::TFT_TOAST_TYPE_JOYSTICK_MODE);
        tft_audio_->playTtsCode(TTS_CODE_TYPE_DRIVING_MODE);
    }
}

// 判断智能躲避toast和语音显示
void TftService::checkSmartAvoidanceToast(bool status) {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    if (tft_audio_) {
        if (status) {
            tft_audio_->playTtsCode(TTS_CODE_TYPE_SMART_AVOIDANCE);
        } else {
            tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
        }
    }
    // 不在主界面时，智能避障功能只报语音不报toast
    if(page_dialog_manager_->getCurrentPage() != MAIN_PAGE_ID) {
        return;
    }
    if (status) {
        updateTftToastType(tft_toast_type::TFT_TOAST_TYPE_SMART_AVOIDANCE_ENABLED);
    } else {
        updateTftToastType(tft_toast_type::TFT_TOAST_TYPE_SMART_AVOIDANCE_DISABLED);
    }
}

void TftService::checkCruiseControlToast(bool status) {
    if (!tft_module_ || !tft_module_->isConnected()) return;

    int gear_icon = TftStateMachine::instance().getGearIcon();
    int brake_status = TftStateMachine::instance().getBrakeStatus();

    if (tft_audio_) {
        if (status) {
            //关闭此处语音播报
            //tft_audio_->playTtsCode(TTS_CODE_TYPE_CRUISE);
        } else {
            // 为了解决toast播报混乱的问题，因为驻车和物理抱闸开启而退出巡航时，不展示巡航退出提示音和toast
            if (gear_icon == 2 || brake_status == 1) {
                // 驻车和物理抱闸开启时，不做提示音提醒
            } else {
                tft_audio_->playPromptTone(PROMPT_TONE_TYPE_EXIT);
            }
            
        }
    }
    if (status) {
        RCLCPP_DEBUG(logger_, "定速巡航已开启");
        updateTftToastType(tft_toast_type::TFT_TOAST_TYPE_CRUISE_CONTROL_ENABLED);
    } else {
        if (gear_icon == 2 || brake_status == 1) {
            // 驻车和物理抱闸开启时，不做toast提醒
        } else {
            RCLCPP_DEBUG(logger_, "定速巡航已退出");
            updateTftToastType(tft_toast_type::TFT_TOAST_TYPE_CRUISE_CONTROL_DISABLED);
        }
        
    }
}

void TftService::checkBluetoothRemoteToast(bool status) {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    if (status) {
        updateTftToastType(tft_toast_type::TFT_TOAST_TYPE_BLUETOOTH_REMOTE_ENABLED);
    } else {
        updateTftToastType(tft_toast_type::TFT_TOOTH_TYPE_BLUETOOTH_REMOTE_DISABLED);
    }
}

// 判断电量toast显示 当前电量低于20%请及时充电，当前电量低于10%请及时充电 (TFT_TOAST_TYPE_LOW_POWER/TFT_TOAST_TYPE_VERY_LOW_POWER)  程序运行时只执行一次
void TftService::checkLowPowerToast(int16_t value) {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    // 电量回升后，重新播报toast，有防抖策略
    static int very_low_power_count = 0;
    static int low_power_count = 0;

    if (very_low_power_toast_shown_) {
        if (value > 10) {
            very_low_power_count++;
        }
    }

    if (low_power_toast_shown_) {
        if (value > 20) {
            low_power_count++;
        }
    }

    if(very_low_power_count > 5) {
        very_low_power_toast_shown_ = false;
        very_low_power_count = 0;
    }

    if(low_power_count > 5) {
        low_power_toast_shown_ = false;
        low_power_count = 0;
    }
    // 已经弹过 10% 的 toast，则认为提醒过，不再重复提示
    if (very_low_power_toast_shown_) return;
    // 先判断更严重的 10% 档
    if (value <= 10 && !very_low_power_toast_shown_) {
        updateTftToastType(tft_toast_type::TFT_TOAST_TYPE_VERY_LOW_POWER);
        very_low_power_toast_shown_ = true;
        return;
    }
    // 20% 档仅提示一次，且如果后续再降到 10% 时会走上面的分支
    if (value <= 20 && !low_power_toast_shown_) {
        updateTftToastType(tft_toast_type::TFT_TOAST_TYPE_LOW_POWER);
        low_power_toast_shown_ = true;
    }
}


void TftService::checkForwardBrakeDecelerationAnimation(int16_t value) {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    tft_module_->sendDcBusWrite(STATION_ADDRESS, FORWARD_WARNING_EFFECT_ADDRESS, 1);
    using namespace std::chrono_literals;
    if (forward_brake_anim_timer_) { forward_brake_anim_timer_->cancel(); forward_brake_anim_timer_.reset(); }
    if (value == 0) return;  // 已关闭，无需定时
    forward_brake_anim_timer_ = create_wall_timer(1000ms, [this]() {
        if (tft_module_ && tft_module_->isConnected()) {
            tft_module_->sendDcBusWrite(STATION_ADDRESS, FORWARD_WARNING_EFFECT_ADDRESS, 0);
        }
        forward_brake_anim_timer_->cancel();
        forward_brake_anim_timer_.reset();
    });
}

void TftService::checkBackwardBrakeDecelerationAnimation(int16_t value) {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    tft_module_->sendDcBusWrite(STATION_ADDRESS, BACKWARD_WARNING_EFFECT_ADDRESS, 1);
    using namespace std::chrono_literals;
    if (backward_brake_anim_timer_) { backward_brake_anim_timer_->cancel(); backward_brake_anim_timer_.reset(); }
    if (value == 0) return;
    backward_brake_anim_timer_ = create_wall_timer(1000ms, [this]() {
        if (tft_module_ && tft_module_->isConnected()) {
            tft_module_->sendDcBusWrite(STATION_ADDRESS, BACKWARD_WARNING_EFFECT_ADDRESS, 0);
        }
        backward_brake_anim_timer_->cancel();
        backward_brake_anim_timer_.reset();
    });
}

void TftService::checkLeftBrakeDecelerationAnimation(int16_t value) {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    tft_module_->sendDcBusWrite(STATION_ADDRESS, LEFT_WARNING_EFFECT_ADDRESS, 1);
    using namespace std::chrono_literals;
    if (left_brake_anim_timer_) { left_brake_anim_timer_->cancel(); left_brake_anim_timer_.reset(); }
    if (value == 0) return;
    left_brake_anim_timer_ = create_wall_timer(1000ms, [this]() {
        if (tft_module_ && tft_module_->isConnected()) {
            tft_module_->sendDcBusWrite(STATION_ADDRESS, LEFT_WARNING_EFFECT_ADDRESS, 0);
        }
        left_brake_anim_timer_->cancel();
        left_brake_anim_timer_.reset();
    });
}

void TftService::checkRightBrakeDecelerationAnimation(int16_t value) {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    tft_module_->sendDcBusWrite(STATION_ADDRESS, RIGHT_WARNING_EFFECT_ADDRESS, 1);
    using namespace std::chrono_literals;
    if (right_brake_anim_timer_) { right_brake_anim_timer_->cancel(); right_brake_anim_timer_.reset(); }
    if (value == 0) return;
    right_brake_anim_timer_ = create_wall_timer(1000ms, [this]() {
        if (tft_module_ && tft_module_->isConnected()) {
            tft_module_->sendDcBusWrite(STATION_ADDRESS, RIGHT_WARNING_EFFECT_ADDRESS, 0);
        }
        right_brake_anim_timer_->cancel();
        right_brake_anim_timer_.reset();
    });
}
// 1 驻车失败 2 解除驻车失败 3 助行失败（暂未实现）
void TftService::onPhysicalButtonFailure(const std_msgs::msg::Int32::SharedPtr msg) {
    // Bug5 fix: 增加 page_dialog_manager_ 空指针检查，防止初始化中途失败时崩溃
    if (!msg || !tft_module_ || !tft_module_->isConnected() || !page_dialog_manager_) return;
    int value = msg->data;
    switch (value) {
        case 1:
            //tft_module_->sendDcBusWrite(STATION_ADDRESS, PHYSICAL_BUTTON_FAILURE_OPEN_ADDRESS, 2);
            // 推行模式
            if(TftStateMachine::instance().isPushingMode()) {
                // 发送弹窗：助行失败-条件不满足
                page_dialog_manager_->showDialog("parked_failure",1);
                break;
            }
            if(TftStateMachine::instance().getBrakeStatus() == 1) {
                // 发送弹窗：驻车失败-条件不满足
                page_dialog_manager_->showDialog("parked_failure",0);
            }
            break;
        case 2:
            //tft_module_->sendDcBusWrite(STATION_ADDRESS, PHYSICAL_BUTTON_FAILURE_OPEN_ADDRESS, 2);
            if(TftStateMachine::instance().isLineChargingMode()) {
                // 发送弹窗：充电失败
                page_dialog_manager_->showDialog("parked_failure",2);
            }
            break;
        case 3:
            //tft_module_->sendDcBusWrite(STATION_ADDRESS, PHYSICAL_BUTTON_FAILURE_OPEN_ADDRESS, 2);
            if(TftStateMachine::instance().isLineChargingMode()) {
                // 发送弹窗：充电失败
                page_dialog_manager_->showDialog("parked_failure",2);
                break;
            }
            if(TftStateMachine::instance().getBrakeStatus() == 1) {
                // 发送弹窗：驻车失败-条件不满足
                page_dialog_manager_->showDialog("parked_failure",0);
            }
            break;
        default:
            break;
    }
}

// 车道警告点消息回调
void TftService::onLaneWarningPoint(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    {
        // 状态校验 写数据加锁 防止校验时数据修改 
        std::lock_guard<std::mutex> lock(lane_warning_point_mutex_);
        if (!msg || !tft_module_ || !tft_module_->isConnected()) return;
        if (!page_dialog_manager_ || page_dialog_manager_->getCurrentPage() != BACKWARD_PAGE_ID) return;
        // 至少12个点， 左侧6个点：[0-11]，右侧6个点：[12-23] x,y坐标对
        if (msg->data.size() < 24 || msg->data.size() % 2 != 0) {
            RCLCPP_WARN(logger_, "LaneWarningPoint 数据长度错误: %zu", msg->data.size());
            return;
        }
    }
    // 无锁处理数据
    std::vector<std::pair<uint16_t, uint16_t>> all_points;
    for (size_t i = 0; i < msg->data.size(); i += 2) {
        all_points.emplace_back(
            static_cast<uint16_t>(msg->data[i]),
            static_cast<uint16_t>(msg->data[i + 1])
        );
    }
    tft_module_->sendDcBusCurveFitWrite(STATION_ADDRESS, BACKWARD_LINE_ADDRESS, 0xF800, all_points);
}

// 摇杆弹窗显示
void TftService::checkJoystickFailureToast(joystick_failure_type type) {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    //tft_module_->sendDcBusWrite(STATION_ADDRESS, PHYSICAL_BUTTON_FAILURE_OPEN_ADDRESS, 2);
    page_dialog_manager_->showDialog("remote_control_failure", (int)type);
}
// 抱闸状态toast显示
void TftService::checkBrakeStatusToast(int16_t value) {
    if (!tft_module_ || !tft_module_->isConnected()) return;
    if(value == 1) {
        updateTftToastType(TFT_TOAST_TYPE_ELECTRONIC_BRAKE_FAILED);
    } else {
        updateTftToastType(TFT_TOAST_TYPE_ELECTRONIC_BRAKE_ENABLED);
    }
}

TftService::~TftService() = default;

}
