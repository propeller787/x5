// workflow.cpp
#include "workflow.hpp"
#include <rclcpp/logging.hpp>
#include <chrono>
#include <stdexcept>
#include <sstream>
#include "../../common/utils/topic.h"

using namespace std::chrono_literals;

WorkflowManager::WorkflowManager(rclcpp::Node *node)
    : node_(node), current_state_(WheelchairState::READY), station_detected_while_rotating_(false)
{
    // 初始化状态转移表
    initializeTransitionTable();

    // 初始化旋转管理器
    initializeRotationManager();

    initializeSubscriptions(); // 新增

    // 创建状态发布者
    state_publisher_ = node_->create_publisher<std_msgs::msg::String>(
        "/wheelchair/state", 10);

    // 创建回充超时报告发布者
    recharge_timeout_publisher_ = node_->create_publisher<std_msgs::msg::String>(
        "/wheelchair/recharge_timeout", 10);
    pub1_ = node_->create_publisher<std_msgs::msg::Int32>(rechargeStateTopic, 10);

    RCLCPP_INFO(node_->get_logger(), "工作流管理器已初始化");
}
void WorkflowManager::initializeSubscriptions()
{
    // 创建基站检测订阅者
    station_detected_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/station/detected", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            this->stationDetectedCallback(msg);
        });
    rotation_timeout_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/wheelchair/rotation_timeout", 10,
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            if (msg->data == "rotation_timeout")
            {
                RCLCPP_INFO(node_->get_logger(), "收到旋转超时信号，返回就绪状态");
                
                // 如果当前是旋转状态，处理超时
                if (current_state_ == WheelchairState::ROTATING)
                {
                    // 发布回充失败通知
                    publishRechargeStatus("RECHARGE_FAILED", "旋转搜索超时，未检测到基站");
                    
                    // 返回就绪状态
                    transitionToReady();
                }
            }
        });

    RCLCPP_INFO(node_->get_logger(), "基站检测订阅者已初始化");
    node_->declare_parameter<bool>("in_rotation_delay", false);
}

void WorkflowManager::stationDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    try
    {
        bool station_detected = msg->data;
        std::string current_state_str = getCurrentState();

        RCLCPP_DEBUG(node_->get_logger(),
                     "收到基站检测信号: %s, 当前状态: %s",
                     station_detected ? "true" : "false",
                     current_state_str.c_str());

        // 处理旋转中检测到基站的情况
        if (station_detected && current_state_ == WheelchairState::ROTATING)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "旋转中检测到基站，记录检测时间并设置延迟切换");

            // 记录检测时间和状态
            station_detected_while_rotating_ = true;
            station_detected_time_ = node_->now();
            // 通知激光控制器进入旋转延迟模式
            node_->set_parameter(rclcpp::Parameter("in_rotation_delay", true));

            // 启动延迟定时器（2秒后切换状态）
            startRotationDelayTimer();

            // 发布状态信息
            publishState("STATION_DETECTED_WHILE_ROTATING",
                         "旋转中检测到基站，继续旋转2秒后对接");

            return; // 直接返回，不立即切换状态
        }

        // 搜索中检测到基站，立即切换状态（保持原有逻辑）
        if (station_detected && current_state_ == WheelchairState::SEARCHING)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "搜索中检测到基站，切换到行走状态进行对接");

            // 从搜索状态切换到行走状态
            transitionToWalking();

            // 发布状态更新
            publishState("STATION_DETECTED", "检测到基站，开始移动对接");

            // 发布回充状态
            publishRechargeStatus("STATION_DETECTED",
                                  "检测到充电基站，开始导航对接");
        }
        else if (!station_detected)
        {
            // 基站丢失处理
            if (current_state_ == WheelchairState::WALKING)
            {
                RCLCPP_DEBUG(node_->get_logger(), "行走对接过程中基站信号丢失");
            }

            // 如果之前旋转中检测到基站但现在丢失了，重置状态
            if (station_detected_while_rotating_)
            {
                station_detected_while_rotating_ = false;
                stopRotationDelayTimer();
                RCLCPP_WARN(node_->get_logger(),
                            "旋转中检测到的基站信号丢失，重置检测状态");
            }
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理基站检测回调", e);
    }
}

void WorkflowManager::startRotationDelayTimer()
{
    try
    {
        // 如果已有定时器，先停止
        stopRotationDelayTimer();

        // 创建0.5秒延迟定时器
        rotation_delay_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(1500),
            [this]()
            {
                this->rotationDelayCallback();
            });

        RCLCPP_INFO(node_->get_logger(),
                    "旋转延迟定时器已启动 (0秒后切换状态)");
    }
    catch (const std::exception &e)
    {
        handleException("启动旋转延迟定时器", e);
    }
}

void WorkflowManager::stopRotationDelayTimer()
{
    try
    {
        if (rotation_delay_timer_)
        {
            rotation_delay_timer_->cancel();
            rotation_delay_timer_.reset();
            RCLCPP_DEBUG(node_->get_logger(), "旋转延迟定时器已停止");
        }
    }
    catch (const std::exception &e)
    {
        handleException("停止旋转延迟定时器", e);
    }
}

void WorkflowManager::rotationDelayCallback()
{
    try
    {
        if (station_detected_while_rotating_ &&
            current_state_ == WheelchairState::ROTATING)
        {
            node_->set_parameter(rclcpp::Parameter("in_rotation_delay", false));
            // 计算实际旋转时间
            double rotation_time = (node_->now() - station_detected_time_).seconds();
            // 停止旋转并切换到行走状态
            if (rotation_manager_ && rotation_manager_->isRotating())
            {
                rotation_manager_->stopRotation();
            }

            transitionToWalking();

            // 重置状态
            station_detected_while_rotating_ = false;
            stopRotationDelayTimer();

            // 发布状态更新
            publishState("ROTATION_DELAY_COMPLETED",
                         "旋转延迟完成，开始对接");

            publishRechargeStatus("STATION_DETECTED_AFTER_ROTATION",
                                  "旋转检测到基站，延迟2.5秒后开始对接");
        }
        else
        {
            // 状态已改变或检测已失效，清理定时器
            station_detected_while_rotating_ = false;
            stopRotationDelayTimer();
            RCLCPP_DEBUG(node_->get_logger(),
                         "旋转延迟定时器触发，但状态已变化或检测失效");
        }
    }
    catch (const std::exception &e)
    {
        handleException("旋转延迟定时器回调", e);
    }
}

void WorkflowManager::initializeRotationManager()
{
    rotation_manager_ = std::make_unique<RotationManager>(node_);
    RCLCPP_INFO(node_->get_logger(), "旋转管理器已初始化");
}

void WorkflowManager::initializeTransitionTable()
{
    transition_table_.clear();

    // 1. ipad&phone界面启动: 就绪中✅, 其他❌
    std::map<WheelchairState, bool> ipad_start_perms = {
        {WheelchairState::READY, true},
        {WheelchairState::WALKING, false},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, false},
        {WheelchairState::CHARGING, false}};
    transition_table_["ipad_phone_interface_start"] = ipad_start_perms;

    // 2. iPad&phone界面取消: 所有状态✅
    std::map<WheelchairState, bool> ipad_cancel_perms = {
        {WheelchairState::READY, true},
        {WheelchairState::WALKING, true},
        {WheelchairState::ROTATING, true}, // 旋转中❌
        {WheelchairState::SEARCHING, true},
        {WheelchairState::CHARGING, true}};
    transition_table_["ipad_phone_interface_cancel"] = ipad_cancel_perms;

    // 3. 蓝牙断开: 就绪中❌, 其他✅
    std::map<WheelchairState, bool> bluetooth_disconnect_perms = {
        {WheelchairState::READY, false},
        {WheelchairState::WALKING, true},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, true},
        {WheelchairState::CHARGING, true}};
    transition_table_["bluetooth_disconnected"] = bluetooth_disconnect_perms;

    // 4. 蓝牙已连接: 就绪中✅, 其他❌
    std::map<WheelchairState, bool> bluetooth_connect_perms = {
        {WheelchairState::READY, true},
        {WheelchairState::WALKING, false},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, false},
        {WheelchairState::CHARGING, false}};
    transition_table_["bluetooth_connected"] = bluetooth_connect_perms;

    // 5. 基站断电: 就绪中❌, 其他✅
    std::map<WheelchairState, bool> base_power_off_perms = {
        {WheelchairState::READY, false},
        {WheelchairState::WALKING, true},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, true},
        {WheelchairState::CHARGING, true}};
    transition_table_["base_station_power_off"] = base_power_off_perms;

    // 6. 低电量警告: 就绪中✅, 其他❌
    std::map<WheelchairState, bool> low_battery_perms = {
        {WheelchairState::READY, true},
        {WheelchairState::WALKING, false},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, false},
        {WheelchairState::CHARGING, false}};
    transition_table_["low_battery_warning"] = low_battery_perms;

    // 7. 锁车: 就绪中✅, 其他❌
    std::map<WheelchairState, bool> lock_vehicle_perms = {
        {WheelchairState::READY, true},
        {WheelchairState::WALKING, false},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, false},
        {WheelchairState::CHARGING, false}};
    transition_table_["lock_vehicle"] = lock_vehicle_perms;

    // 8. 解锁: 行走中✅, 其他❌
    std::map<WheelchairState, bool> unlock_vehicle_perms = {
        {WheelchairState::READY, false},
        {WheelchairState::WALKING, true},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, false},
        {WheelchairState::CHARGING, false}};
    transition_table_["unlock_vehicle"] = unlock_vehicle_perms;

    // 9. 摇杆后拉: 搜索中✅, 其他❌
    std::map<WheelchairState, bool> joystick_pull_perms = {
        {WheelchairState::READY, false},
        {WheelchairState::WALKING, false},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, true},
        {WheelchairState::CHARGING, false}};
    transition_table_["joystick_pull_back"] = joystick_pull_perms;

    // 10. 摇杆停止: 行走中✅, 搜索中✅, 其他❌
    std::map<WheelchairState, bool> joystick_stop_perms = {
        {WheelchairState::READY, false},
        {WheelchairState::WALKING, true},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, true},
        {WheelchairState::CHARGING, false}};
    transition_table_["joystick_stop"] = joystick_stop_perms;

    // 11. 推行启动: 充电中✅, 其他❌
    std::map<WheelchairState, bool> push_start_perms = {
        {WheelchairState::READY, false},
        {WheelchairState::WALKING, false},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, false},
        {WheelchairState::CHARGING, true}};
    transition_table_["push_start"] = push_start_perms;

    // 12. 推行关闭: 行走中✅, 搜索中✅, 其他❌
    std::map<WheelchairState, bool> push_stop_perms = {
        {WheelchairState::READY, false},
        {WheelchairState::WALKING, true},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, true},
        {WheelchairState::CHARGING, false}};
    transition_table_["push_stop"] = push_stop_perms;

    // 13. 电池-开始充电: 搜索中✅, 充电中✅, 其他❌
    std::map<WheelchairState, bool> battery_start_charging_perms = {
        {WheelchairState::READY, false},
        {WheelchairState::WALKING, false},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, true},
        {WheelchairState::CHARGING, true}};
    transition_table_["battery_start_charging"] = battery_start_charging_perms;

    // 14. 电池-断开充电: 充电中✅, 其他❌
    std::map<WheelchairState, bool> battery_stop_charging_perms = {
        {WheelchairState::READY, false},
        {WheelchairState::WALKING, true},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, false},
        {WheelchairState::CHARGING, true}};
    transition_table_["battery_stop_charging"] = battery_stop_charging_perms;

    // 15. 电池-电量满: 充电中✅, 其他❌
    std::map<WheelchairState, bool> battery_full_perms = {
        {WheelchairState::READY, false},
        {WheelchairState::WALKING, false},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, false},
        {WheelchairState::CHARGING, true}};
    transition_table_["battery_full"] = battery_full_perms;

    // 16. 错误码处理: 所有状态✅
    std::map<WheelchairState, bool> error_code_perms = {
        {WheelchairState::READY, true},
        {WheelchairState::WALKING, true},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, true},
        {WheelchairState::CHARGING, true}};
    transition_table_["error_code_handling"] = error_code_perms;

    // 17. 基站检测丢失: 行走中✅, 其他❌
    std::map<WheelchairState, bool> base_station_lost_perms = {
        {WheelchairState::READY, false},
        {WheelchairState::WALKING, true},
        {WheelchairState::ROTATING, false}, // 旋转中❌
        {WheelchairState::SEARCHING, false},
        {WheelchairState::CHARGING, false}};
    transition_table_["base_station_lost"] = base_station_lost_perms;

    // 18. 搜索30s超时: 搜索中✅, 其他❌
    std::map<WheelchairState, bool> search_timeout_perms = {
        {WheelchairState::READY, false},
        {WheelchairState::WALKING, false},
        {WheelchairState::ROTATING, true},
        {WheelchairState::SEARCHING, true},
        {WheelchairState::CHARGING, false}};
    transition_table_["search_30s_timeout"] = search_timeout_perms;

    std::map<WheelchairState, bool> station_detected_perms = {
        {WheelchairState::READY, false},
        {WheelchairState::WALKING, false},
        {WheelchairState::ROTATING, true},  // 旋转中✅
        {WheelchairState::SEARCHING, true}, // 搜索中✅
        {WheelchairState::CHARGING, false}};
    transition_table_["station_detected"] = station_detected_perms;

    RCLCPP_INFO(node_->get_logger(), "状态转移表已初始化完成");
}

bool WorkflowManager::handleEvent(const std::string &event)
{
    try
    {
        if (!checkEventPermission(event))
        {
            RCLCPP_WARN(node_->get_logger(),
                        "事件 '%s' 在当前状态 '%s' 下不允许执行",
                        event.c_str(),
                        getCurrentState().c_str());
            return false;
        }

        executeEvent(event);
        return true;
    }
    catch (const std::exception &e)
    {
        handleException("处理事件 " + event, e);
        return false;
    }
}

bool WorkflowManager::checkEventPermission(const std::string &event)
{
    if (transition_table_.find(event) == transition_table_.end())
    {
        RCLCPP_WARN(node_->get_logger(), "未知事件: %s", event.c_str());
        return false;
    }

    std::map<WheelchairState, bool> state_permissions = transition_table_[event];

    if (state_permissions.find(current_state_) != state_permissions.end())
    {
        return state_permissions[current_state_];
    }

    return false;
}

void WorkflowManager::executeEvent(const std::string &event)
{
    RCLCPP_INFO(node_->get_logger(), "执行事件: %s", event.c_str());

    // 根据事件类型执行相应处理
    if (event == "ipad_phone_interface_start")
    {
        processIpadStartEvent();
    }
    else if (event == "ipad_phone_interface_cancel")
    {
        processIpadCancelEvent();
    }
    else if (event == "bluetooth_disconnected")
    {
        processBluetoothDisconnected();
    }
    else if (event == "bluetooth_connected")
    {
        processBluetoothConnected();
    }
    else if (event == "base_station_power_off")
    {
        processBaseStationPowerOff();
    }
    else if (event == "low_battery_warning")
    {
        handleLowBatteryEvent(BatteryState::LOW, 0.0f);
    }
    else if (event == "lock_vehicle")
    {
        processLockVehicle();
    }
    else if (event == "unlock_vehicle")
    {
        processUnlockVehicle();
    }
    else if (event == "joystick_pull_back")
    {
        processJoystickPullBack();
    }
    else if (event == "joystick_stop")
    {
        processJoystickStop();
    }
    else if (event == "push_start")
    {
        processPushStart();
    }
    else if (event == "push_stop")
    {
        processPushStop();
    }
    else if (event == "battery_start_charging")
    {
        handleChargingStartEvent();
    }
    else if (event == "battery_stop_charging")
    {
        handleChargingStopEvent();
    }
    else if (event == "battery_full")
    {
        handleChargingFullEvent();
    }
    else if (event == "base_station_lost")
    {
        handleBaseStationLostEvent();
    }
    else if (event == "search_30s_timeout")
    {
        handleSearchTimeoutEvent();
    }
    else if (event == "station_detected")
    {
        RCLCPP_INFO(node_->get_logger(), "处理基站检测成功事件");
        // 实际上这个事件是通过回调处理的，这里只做日志记录
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "未实现的事件处理: %s", event.c_str());
    }

    // 根据事件触发状态转移
    processStateTransition(event);

    // 发布状态更新
    publishState(event, "事件已处理");
}

void WorkflowManager::processStateTransition(const std::string &event)
{
    try
    {
        // 根据事件检查是否需要状态转移
        switch (current_state_)
        {
        case WheelchairState::READY:
            if (event == "low_battery_warning" || event == "ipad_phone_interface_start")
            {
                transitionToSearching();
            }
            else if (event == "unlock_vehicle")
            {
                transitionToWalking();
            }
            break;

        case WheelchairState::WALKING:
            if (event == "joystick_stop" || event == "push_stop" || event == "lock_vehicle")
            {
                transitionToReady();
            }
            else if (event == "base_station_lost")
            {
                transitionToSearching();
            }
            else if (event == "battery_start_charging")
            {
                transitionToCharging();
            }
            break;
        case WheelchairState::ROTATING:
            // 旋转中状态下，所有事件默认不处理，可以添加特定逻辑
            if (event == "station_detected")
            {
                RCLCPP_INFO(node_->get_logger(), "旋转中检测到基站，转为行走对接");
                transitionToWalking();
            }
            break;
        case WheelchairState::SEARCHING:
            if (event == "base_station_lost")
            {
                transitionToReady();
            }
            else if (event == "battery_start_charging")
            {
                transitionToCharging();
            }
            else if (event == "joystick_pull_back")
            {
                transitionToWalking();
            }
            else if (event == "station_detected")
            {
                RCLCPP_INFO(node_->get_logger(), "搜索中检测到基站，转为行走对接");
                transitionToWalking();
            }
            // 搜索超时事件
            else if (event == "search_30s_timeout")
            {
                RCLCPP_WARN(node_->get_logger(), "搜索30秒超时，回充失败");
                // 触发回充失败处理
                handleRechargeFailure("搜索30秒超时");
            }
            break;

        case WheelchairState::CHARGING:
            if (event == "battery_full" ||
                event == "battery_stop_charging" ||
                event == "base_station_power_off")
            {
                transitionToReady();
            }
            else if (event == "push_start")
            {
                transitionToWalking();
            }
            break;
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理状态转移", e);
    }
}

// ==================== 旋转管理接口 ====================

void WorkflowManager::startRotationTimer()
{
    try
    {
        // 创建10秒后检查旋转的定时器
        rotation_check_timer_ = node_->create_wall_timer(
            std::chrono::seconds(10),
            [this]()
            {
                this->rotationCheckCallback();
            });

        RCLCPP_INFO(node_->get_logger(), "旋转检查定时器已启动 (10秒后检查)");
    }
    catch (const std::exception &e)
    {
        handleException("启动旋转检查定时器", e);
    }
}

void WorkflowManager::checkRotationTransition()
{
    try
    {
        if (current_state_ == WheelchairState::SEARCHING)
        {
            double search_duration = (node_->now() - search_start_time_).seconds();

            // 如果搜索时间超过10秒，进入旋转状态
            if (search_duration >= 10.0)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "搜索已持续 %.1f 秒，进入旋转状态", search_duration);
                transitionToRotating();
            }
            else
            {
                RCLCPP_DEBUG(node_->get_logger(),
                             "搜索中，已持续 %.1f 秒，还需 %.1f 秒进入旋转",
                             search_duration, 10.0 - search_duration);
            }
        }
    }
    catch (const std::exception &e)
    {
        handleException("检查旋转转移", e);
    }
}

void WorkflowManager::stopRotationTimer()
{
    try
    {
        if (rotation_check_timer_)
        {
            rotation_check_timer_->cancel();
            rotation_check_timer_.reset();
            RCLCPP_INFO(node_->get_logger(), "旋转检查定时器已停止");
        }
    }
    catch (const std::exception &e)
    {
        handleException("停止旋转检查定时器", e);
    }
}

void WorkflowManager::transitionToRotating()
{
    try
    {
        if (current_state_ == WheelchairState::SEARCHING)
        {
            setState(WheelchairState::ROTATING);

            // 启动旋转
            if (rotation_manager_)
            {
                rotation_manager_->startRotation();
                RCLCPP_INFO(node_->get_logger(), "轮椅开始旋转搜索");
                publishState("WHEELCHAIR_ROTATING", "旋转搜索充电站");
                // 发布回充状态
                publishRechargeStatus("ROTATING_SEARCH",
                                      "搜索10秒未找到基站，开始旋转搜索");
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "旋转管理器未初始化");
            }
            // 停止旋转检查定时器
            stopRotationTimer();
            // 关键修复：进入旋转状态时，停止搜索超时定时器
            stopSearchTimeoutTimer();
            RCLCPP_INFO(node_->get_logger(), "已停止搜索超时定时器，进入旋转搜索模式");
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "当前状态 %s 无法转换为旋转状态",
                        getCurrentState().c_str());
        }
    }
    catch (const std::exception &e)
    {
        handleException("转换到旋转状态", e);
    }
}

void WorkflowManager::transitionToWalking()
{
    try
    {
        // 如果正在旋转，先停止旋转
        if (rotation_manager_ && rotation_manager_->isRotating())
        {
            rotation_manager_->stopRotation();
        }
        //  新增：停止延迟定时器
        stopRotationDelayTimer();

        // 新增：重置检测标志
        station_detected_while_rotating_ = false;
        if (
            current_state_ == WheelchairState::SEARCHING ||
            current_state_ == WheelchairState::ROTATING)
        {
            setState(WheelchairState::WALKING);
            // 停止所有定时器
            stopSearchTimeoutTimer();
            stopRotationTimer();
            RCLCPP_INFO(node_->get_logger(), "轮椅开始行走");
            publishState("WHEELCHAIR_WALKING");
            auto msg = std_msgs::msg::Int32();
            msg.data = RECHARGE_EVENT_OBSTACLE_DETECTED;
            pub1_->publish(msg);
            RCLCPP_WARN(node_->get_logger(), "WorkflowManager 上报状态 2");
        }
    }
    catch (const std::exception &e)
    {
        handleException("转换到行走状态", e);
    }
}

void WorkflowManager::transitionToSearching()
{
    try
    {
        if (current_state_ == WheelchairState::READY ||
            current_state_ == WheelchairState::WALKING)
        {
            setState(WheelchairState::SEARCHING);
            search_start_time_ = node_->now();
            RCLCPP_INFO(node_->get_logger(), "开始搜索充电站");
            publishState("SEARCH_CHARGING_STATION_START");
            startSearchTimeoutTimer();
            startRotationTimer();
            // 发布回充开始通知
            publishRechargeStatus("RECHARGE_STARTED", "回充功能已启动，正在搜索充电站");
        }
    }
    catch (const std::exception &e)
    {
        handleException("转换到搜索状态", e);
    }
}

void WorkflowManager::transitionToCharging()
{
    try
    {
        if (current_state_ == WheelchairState::SEARCHING ||
            current_state_ == WheelchairState::WALKING ||
            current_state_ == WheelchairState::ROTATING) // 增加旋转状态
        {
            setState(WheelchairState::CHARGING);
            RCLCPP_INFO(node_->get_logger(), "开始充电");

            // 停止所有定时器
            stopSearchTimeoutTimer();
            stopRotationTimer();

            // 如果正在旋转，停止旋转
            if (rotation_manager_ && rotation_manager_->isRotating())
            {
                rotation_manager_->stopRotation();
                RCLCPP_INFO(node_->get_logger(), "停止旋转，开始充电");
            }

            publishState("CHARGING_START");

            // 发布回充成功通知
            publishRechargeStatus("RECHARGE_SUCCESS", "成功对接充电站，开始充电");
        }
    }
    catch (const std::exception &e)
    {
        handleException("转换到充电状态", e);
    }
}

void WorkflowManager::transitionToReady()
{
    try
    {
        std::string previous_state = getCurrentState();
        setState(WheelchairState::READY);

        // 停止所有定时器
        stopSearchTimeoutTimer();
        stopRotationTimer();
        stopRotationDelayTimer();

        // 重置检测状态
        station_detected_while_rotating_ = false;

        // 如果正在旋转，停止旋转
        if (rotation_manager_ && rotation_manager_->isRotating())
        {
            rotation_manager_->stopRotation();
            RCLCPP_INFO(node_->get_logger(), "停止旋转，返回到就绪状态");
        }

        RCLCPP_INFO(node_->get_logger(), "返回到就绪状态 (前状态: %s)",
                    previous_state.c_str());
        publishState("WHEELCHAIR_READY");
    }
    catch (const std::exception &e)
    {
        emergencyTransitionToReady("正常转换失败: " + std::string(e.what()));
    }
}

// ==================== 事件处理函数实现 ====================

void WorkflowManager::processIpadStartEvent()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理iPad启动事件");

        // 发布状态更新
        publishState("IPAD_RECHARGE_STARTED", "iPad启动自主回充");

        // 如果当前是就绪状态，自动转移到搜索状态
        if (current_state_ == WheelchairState::READY)
        {
            transitionToSearching();
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "当前状态 %s 无法启动回充", getCurrentState().c_str());
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理iPad启动事件", e);
    }
}

void WorkflowManager::processIpadCancelEvent()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理iPad取消事件");

        // 发布状态更新
        publishState("IPAD_RECHARGE_CANCELLED", "iPad取消自主回充");

        // 清理旋转延迟相关状态
        station_detected_while_rotating_ = false;
        stopRotationDelayTimer();

        // 根据当前状态决定是否取消回充
        if (current_state_ == WheelchairState::SEARCHING ||
            current_state_ == WheelchairState::WALKING || current_state_ == WheelchairState::ROTATING)
        {
            transitionToReady();
            RCLCPP_INFO(node_->get_logger(), "iPad自主回充已取消，返回到就绪状态");

            // 发布回充取消通知
            publishRechargeStatus("RECHARGE_CANCELLED", "用户取消回充");
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理iPad取消事件", e);
    }
}

void WorkflowManager::processBluetoothDisconnected()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理蓝牙断开事件");

        // 发布状态更新
        publishState("BLUETOOTH_DISCONNECTED", "蓝牙连接已断开");

        // 如果正在回充或充电，退出流程
        if (current_state_ == WheelchairState::SEARCHING ||
            current_state_ == WheelchairState::CHARGING)
        {
            RCLCPP_WARN(node_->get_logger(), "蓝牙断开，退出回充流程");
            handleRechargeFailure("蓝牙断开导致回充失败");
            transitionToReady();
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理蓝牙断开事件", e);
    }
}

void WorkflowManager::processBluetoothConnected()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理蓝牙连接成功事件");

        // 发布状态更新
        publishState("BLUETOOTH_CONNECTED", "蓝牙连接成功");
    }
    catch (const std::exception &e)
    {
        handleException("处理蓝牙连接事件", e);
    }
}

void WorkflowManager::processBaseStationPowerOff()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理基站断电事件");

        // 发布状态更新
        publishState("BASE_STATION_POWER_OFF", "充电基站断电");

        // 如果正在回充或充电，退出流程
        if (current_state_ == WheelchairState::SEARCHING ||
            current_state_ == WheelchairState::CHARGING)
        {
            RCLCPP_WARN(node_->get_logger(), "基站断电，退出回充流程");
            handleRechargeFailure("基站断电导致回充失败");
            transitionToReady();
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理基站断电事件", e);
    }
}

void WorkflowManager::handleLowBatteryEvent(BatteryState state, float percentage)
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理低电量事件");

        // 根据电池状态采取行动
        if (state == BatteryState::CRITICAL)
        {
            RCLCPP_ERROR(node_->get_logger(), "严重低电量! 自动启动回充搜索");
            publishState("CRITICAL_BATTERY_EMERGENCY", "自动启动回充");

            // 紧急情况下，无论当前状态如何都尝试启动回充
            if (current_state_ == WheelchairState::READY)
            {
                transitionToSearching();
            }
            else if (current_state_ == WheelchairState::WALKING)
            {
                // 如果是行走状态，先停止行走再搜索
                RCLCPP_WARN(node_->get_logger(), "严重低电量，停止行走并启动回充搜索");
                transitionToReady();
                // 短暂延迟后启动搜索
                transitionToSearching();
            }
        }
        else if (state == BatteryState::LOW)
        {
            RCLCPP_WARN(node_->get_logger(), "低电量警告，建议启动回充");
            publishState("LOW_BATTERY_WARNING", "建议启动回充");

            // 如果当前是就绪状态，自动启动回充
            if (current_state_ == WheelchairState::READY)
            {
                RCLCPP_INFO(node_->get_logger(), "自动启动回充搜索");
                transitionToSearching();
            }
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理低电量事件", e);
    }
}

void WorkflowManager::processLockVehicle()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理锁车操作");

        // 发布状态更新
        publishState("VEHICLE_LOCKED", "轮椅已锁定");

        // 如果正在回充或充电，退出流程
        if (current_state_ == WheelchairState::SEARCHING ||
            current_state_ == WheelchairState::CHARGING)
        {
            RCLCPP_WARN(node_->get_logger(), "锁车操作，退出回充流程");
            transitionToReady();

            // 发布回充取消通知
            publishRechargeStatus("RECHARGE_CANCELLED", "锁车操作取消回充");
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理锁车事件", e);
    }
}

void WorkflowManager::processUnlockVehicle()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理解锁操作");

        // 发布状态更新
        publishState("VEHICLE_UNLOCKED", "轮椅已解锁");

        // 根据状态处理解锁
        if (current_state_ == WheelchairState::READY)
        {
            RCLCPP_INFO(node_->get_logger(), "解锁后准备行走");
            transitionToWalking();
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理解锁事件", e);
    }
}

void WorkflowManager::processJoystickPullBack()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理摇杆后拉操作");

        // 发布状态更新
        publishState("JOYSTICK_PULL_BACK", "摇杆后拉");

        // 如果正在回充或充电，退出流程
        if (current_state_ == WheelchairState::SEARCHING ||
            current_state_ == WheelchairState::CHARGING)
        {
            RCLCPP_INFO(node_->get_logger(), "用户手动干预，退出回充流程");
            transitionToReady();

            // 发布回充取消通知
            publishRechargeStatus("RECHARGE_CANCELLED", "用户手动中断回充");
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理摇杆后拉事件", e);
    }
}

void WorkflowManager::processJoystickStop()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理摇杆停止操作");

        // 发布状态更新
        publishState("JOYSTICK_STOP", "摇杆停止");

        // 如果正在回充或充电，退出流程
        if (current_state_ == WheelchairState::SEARCHING ||
            current_state_ == WheelchairState::CHARGING)
        {
            RCLCPP_INFO(node_->get_logger(), "用户停止操作，退出回充流程");
            transitionToReady();

            // 发布回充取消通知
            publishRechargeStatus("RECHARGE_CANCELLED", "用户停止操作取消回充");
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理摇杆停止事件", e);
    }
}

void WorkflowManager::processPushStart()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理推行启动");

        // 发布状态更新
        publishState("PUSH_START", "推行模式启动");

        // 如果正在充电，退出流程
        if (current_state_ == WheelchairState::CHARGING)
        {
            RCLCPP_INFO(node_->get_logger(), "推行操作，退出充电流程");
            transitionToReady();

            // 发布充电中断通知
            publishRechargeStatus("CHARGING_INTERRUPTED", "推行操作中断充电");
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理推行启动事件", e);
    }
}

void WorkflowManager::processPushStop()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理推行停止");

        // 发布状态更新
        publishState("PUSH_STOP", "推行模式停止");

        // 根据状态处理推行停止
        if (current_state_ == WheelchairState::WALKING)
        {
            RCLCPP_INFO(node_->get_logger(), "推行中停止，返回就绪状态");
            transitionToReady();
        }
        else if (current_state_ == WheelchairState::SEARCHING)
        {
            RCLCPP_INFO(node_->get_logger(), "搜索中停止推行，返回就绪状态");
            transitionToReady();
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理推行停止事件", e);
    }
}

void WorkflowManager::handleChargingStartEvent()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理充电开始事件");

        // 发布状态更新
        publishState("BATTERY_START_CHARGING", "开始充电");

        // 如果当前在搜索状态，转移到充电状态
        if (current_state_ == WheelchairState::SEARCHING)
        {
            transitionToCharging();
        }
        else if (current_state_ == WheelchairState::WALKING)
        {
            RCLCPP_WARN(node_->get_logger(), "行走中开始充电，切换到充电状态");
            transitionToCharging();
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理充电开始事件", e);
    }
}

void WorkflowManager::handleChargingStopEvent()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理充电停止事件");

        // 发布状态更新
        publishState("BATTERY_STOP_CHARGING", "停止充电");

        // 如果正在充电，退出流程
        if (current_state_ == WheelchairState::CHARGING)
        {
            RCLCPP_WARN(node_->get_logger(), "充电停止，退出回充流程");
            transitionToReady();

            // 发布充电停止通知
            publishRechargeStatus("CHARGING_STOPPED", "充电已停止");
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理充电停止事件", e);
    }
}

void WorkflowManager::handleChargingFullEvent()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理电池充满事件");

        // 发布状态更新
        publishState("BATTERY_FULL", "电池已充满");

        // 如果正在充电，退出流程
        if (current_state_ == WheelchairState::CHARGING)
        {
            RCLCPP_INFO(node_->get_logger(), "电池充满，退出回充流程");
            transitionToReady();

            // 发布充电完成通知
            publishRechargeStatus("CHARGING_COMPLETED", "电池已充满，充电完成");
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理电池充满事件", e);
    }
}

void WorkflowManager::handleBaseStationLostEvent()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理基站丢失事件");

        // 发布状态更新
        publishState("CHARGING_STATION_LOST", "充电基站丢失");

        // 如果正在回充或充电，退出流程
        if (current_state_ == WheelchairState::SEARCHING ||
            current_state_ == WheelchairState::CHARGING)
        {
            RCLCPP_WARN(node_->get_logger(), "基站丢失，退出回充流程");
            transitionToReady();
            // 发布回充失败通知
            publishRechargeStatus("RECHARGE_FAILED", "基站丢失导致回充失败");
        }
    }
    catch (const std::exception &e)
    {
        handleException("处理基站丢失事件", e);
    }
}

void WorkflowManager::handleSearchTimeoutEvent()
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "处理搜索超时事件");

        // 记录当前状态，便于调试
        std::string current_state_str = getCurrentState();
        RCLCPP_INFO(node_->get_logger(), "搜索超时发生时的状态: %s", current_state_str.c_str());

        // 搜索超时30秒视为回充失败
        RCLCPP_WARN(node_->get_logger(), "搜索30秒超时，回充失败");

        // 发布回充失败通知
        publishRechargeStatus("RECHARGE_FAILED", "搜索充电站秒超时");

        // 退出回充流程
        transitionToReady();
    }
    catch (const std::exception &e)
    {
        handleException("处理搜索超时事件", e);
    }
}

void WorkflowManager::handleRechargeFailure(const std::string &reason)
{
    try
    {
        RCLCPP_WARN(node_->get_logger(), "回充失败: %s", reason.c_str());

        // 发布回充失败通知
        publishRechargeStatus("RECHARGE_FAILED", "回充失败: " + reason);

        // 记录错误日志
        std::stringstream error_log;
        error_log << "回充流程异常终止\n";
        error_log << "原因: " << reason << "\n";
        error_log << "时间: " << node_->now().seconds() << "s";

        RCLCPP_ERROR(node_->get_logger(), "%s", error_log.str().c_str());
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(node_->get_logger(),
                     "处理回充失败时发生异常: %s (原失败原因: %s)",
                     e.what(), reason.c_str());
    }
}

// ==================== 异常处理函数 ====================

void WorkflowManager::handleException(const std::string &context, const std::exception &e)
{
    std::string error_msg = context + " 异常: " + e.what();
    RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());

    // 记录异常发生时的状态
    std::string previous_state = getCurrentState();

    try
    {
        // 任何异常都视为系统故障，尝试紧急恢复
        emergencyTransitionToReady(error_msg);

        // 记录异常详细信息
        std::stringstream log;
        log << "异常处理记录:\n";
        log << "异常位置: " << context << "\n";
        log << "异常信息: " << e.what() << "\n";
        log << "前状态: " << previous_state << "\n";
        log << "处理时间: " << node_->now().seconds() << "s";

        RCLCPP_ERROR(node_->get_logger(), "%s", log.str().c_str());
    }
    catch (const std::exception &inner_e)
    {
        // 如果异常处理也失败，记录致命错误
        RCLCPP_FATAL(node_->get_logger(),
                     "异常处理过程中再次发生异常: %s (原异常: %s)",
                     inner_e.what(), error_msg.c_str());

        // 尝试最后的恢复
        try
        {
            current_state_ = WheelchairState::READY;
            RCLCPP_INFO(node_->get_logger(), "强制设置状态为就绪");
        }
        catch (...)
        {
            RCLCPP_FATAL(node_->get_logger(), "无法恢复系统状态");
        }
    }
}

void WorkflowManager::emergencyTransitionToReady(const std::string &reason)
{
    try
    {
        std::string previous_state = getCurrentState();

        // 停止所有定时器
        if (search_timeout_timer_)
        {
            search_timeout_timer_->cancel();
            search_timeout_timer_.reset();
        }
        if (rotation_check_timer_)
        {
            rotation_check_timer_->cancel();
            rotation_check_timer_.reset();
        }
        if (rotation_delay_timer_) // 新增：停止延迟定时器
        {
            rotation_delay_timer_->cancel();
            rotation_delay_timer_.reset();
        }

        // 重置检测状态
        station_detected_while_rotating_ = false;

        // 停止旋转
        if (rotation_manager_ && rotation_manager_->isRotating())
        {
            rotation_manager_->stopRotation();
        }

        // 强制设置状态
        current_state_ = WheelchairState::READY;

        // 发布紧急状态
        auto msg = std_msgs::msg::String();
        msg.data = "EMERGENCY_READY:系统异常后强制就绪 - " + reason;
        state_publisher_->publish(msg);

        // 如果之前是回充相关状态，发布回充失败
        if (previous_state == "搜索中" || previous_state == "充电中" || previous_state == "旋转中")
        {
            publishRechargeStatus("RECHARGE_EMERGENCY_FAILURE",
                                  "系统异常导致回充紧急终止: " + reason);
        }

        RCLCPP_WARN(node_->get_logger(), "紧急状态转换到就绪 (原因: %s)", reason.c_str());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "紧急状态转换失败: %s", e.what());
        // 仍然尝试设置状态
        current_state_ = WheelchairState::READY;
    }
}

// ==================== 回充状态管理 ====================

void WorkflowManager::publishRechargeStatus(const std::string &status, const std::string &message)
{
    try
    {
        // 发布到专用话题
        auto msg = std_msgs::msg::String();
        msg.data = status + ": " + message;
        recharge_timeout_publisher_->publish(msg);

        // 同时记录日志
        RCLCPP_WARN(node_->get_logger(), "[回充状态] %s: %s", status.c_str(), message.c_str());

        // 调用外部回调（如果有）
        if (recharge_status_callback_)
        {
            recharge_status_callback_(status, message);
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "发布回充状态时发生异常: %s (状态: %s, 消息: %s)",
                     e.what(), status.c_str(), message.c_str());
    }
}

// ==================== 定时器管理 ====================

void WorkflowManager::startSearchTimeoutTimer()
{
    try
    {
        // 如果已经有定时器，先取消
        if (search_timeout_timer_)
        {
            search_timeout_timer_->cancel();
        }

        // 创建30秒定时器
        search_timeout_timer_ = node_->create_wall_timer(
            std::chrono::seconds(30),
            [this]()
            {
                this->searchTimeoutCallback();
            });

        RCLCPP_INFO(node_->get_logger(), "搜索超时定时器已启动 (30秒)");
    }
    catch (const std::exception &e)
    {
        handleException("启动搜索超时定时器", e);
    }
}

void WorkflowManager::stopSearchTimeoutTimer()
{
    try
    {
        if (search_timeout_timer_)
        {
            search_timeout_timer_->cancel();
            search_timeout_timer_.reset();
            RCLCPP_INFO(node_->get_logger(), "搜索超时定时器已停止");
        }
    }
    catch (const std::exception &e)
    {
        handleException("停止搜索超时定时器", e);
    }
}

// ==================== 定时器回调 ====================

void WorkflowManager::rotationCheckCallback()
{
    try
    {
        if (current_state_ == WheelchairState::SEARCHING)
        {
            checkRotationTransition();
        }
        else
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "旋转检查回调触发，但当前不在搜索状态");
            stopRotationTimer();
        }
    }
    catch (const std::exception &e)
    {
        handleException("旋转检查定时器回调", e);
    }
}

void WorkflowManager::searchTimeoutCallback()
{
    try
    {
        // 只在搜索状态下处理搜索超时，旋转状态由RotationManager处理
        if (current_state_ == WheelchairState::SEARCHING)
        {
            double total_search_time = (node_->now() - search_start_time_).seconds();

            RCLCPP_WARN(node_->get_logger(),
                        "搜索 %.1f 秒超时，回充失败",
                        total_search_time);
            handleSearchTimeoutEvent();
        }
        else if (current_state_ == WheelchairState::ROTATING)
        {
            // 旋转状态不应该触发搜索超时，这应该是个bug
            RCLCPP_ERROR(node_->get_logger(),
                         "错误：旋转状态下触发了搜索超时回调！当前状态: %s",
                         getCurrentState().c_str());

            // 停止搜索超时定时器
            stopSearchTimeoutTimer();
            RCLCPP_INFO(node_->get_logger(), "已纠正：停止旋转状态下的搜索超时定时器");
        }
        else
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "搜索超时定时器触发，但当前不在搜索状态: %s",
                         getCurrentState().c_str());
            stopSearchTimeoutTimer();
        }
    }
    catch (const std::exception &e)
    {
        handleException("搜索超时定时器回调", e);
    }
}

// ==================== 辅助函数 ====================

std::string WorkflowManager::getCurrentState() const
{
    return stateToString(current_state_);
}

void WorkflowManager::setState(WheelchairState new_state)
{
    try
    {
        std::string old_state = getCurrentState();
        current_state_ = new_state;

        RCLCPP_WARN(node_->get_logger(),
                    "状态转移: %s -> %s",
                    old_state.c_str(),
                    getCurrentState().c_str());
        // 增强状态发布：包含时间戳和前后状态
        std::stringstream enhanced_state;
        enhanced_state << "[" << node_->now().seconds() << "] "
                       << old_state << " -> " << getCurrentState();

        // 发布状态到ROS话题
        auto msg = std_msgs::msg::String();
        msg.data = enhanced_state.str();
        state_publisher_->publish(msg);

        // 调用外部回调
        if (state_update_callback_)
        {
            state_update_callback_(getCurrentState(), "状态已更新");
        }
    }
    catch (const std::exception &e)
    {
        handleException("设置状态", e);
    }
}

std::string WorkflowManager::stateToString(WheelchairState state) const
{
    switch (state)
    {
    case WheelchairState::READY:
        return "就绪中";
    case WheelchairState::WALKING:
        return "行走中";
    case WheelchairState::ROTATING: // 新增
        return "旋转中";
    case WheelchairState::SEARCHING:
        return "搜索中";
    case WheelchairState::CHARGING:
        return "充电中";
    default:
        return "未知状态";
    }
}

void WorkflowManager::publishState(const std::string &state_str, const std::string &message)
{
    try
    {
        auto msg = std_msgs::msg::String();
        if (message.empty())
        {
            msg.data = state_str;
        }
        else
        {
            msg.data = state_str + ":" + message;
        }
        state_publisher_->publish(msg);

        // 调用外部回调
        if (state_update_callback_)
        {
            state_update_callback_(state_str, message);
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "发布状态时发生异常: %s (状态: %s, 消息: %s)",
                     e.what(), state_str.c_str(), message.c_str());
    }
}

void WorkflowManager::setStateUpdateCallback(StateUpdateCallback callback)
{
    state_update_callback_ = callback;
    RCLCPP_INFO(node_->get_logger(), "已设置状态更新回调");
}

void WorkflowManager::setRechargeStatusCallback(StateUpdateCallback callback)
{
    recharge_status_callback_ = callback;
    RCLCPP_INFO(node_->get_logger(), "已设置回充状态回调");
}

void WorkflowManager::setRechargeErrorCallback(std::function<void(const ErrorInfo &)> callback)
{
    recharge_error_callback_ = callback;
    RCLCPP_INFO(node_->get_logger(), "已设置回充错误回调");
}

void WorkflowManager::triggerEvent(const std::string &event_name)
{
    try
    {
        // 检查事件权限
        if (!checkEventPermission(event_name))
        {
            RCLCPP_WARN(node_->get_logger(),
                        "内部触发事件 '%s' 在当前状态 '%s' 下不允许执行",
                        event_name.c_str(),
                        getCurrentState().c_str());
            return;
        }

        // 执行事件
        executeEvent(event_name);
    }
    catch (const std::exception &e)
    {
        handleException("内部触发事件 " + event_name, e);
    }
}