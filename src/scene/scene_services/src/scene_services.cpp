#include "scene_services.h"

#include <set>

/*
plug Clang-Format
IndentWidth: '4'
ColumnLimit: 160
*/
using namespace std::placeholders;

namespace service::scene {

SecneMode find_mode(const std::string &str_mode) {
    for (auto &ch : mode_str_map) {
        if (ch.second == str_mode) {
            return ch.first;
        }
    }
    return Mode_zero;
}

SceneServices::SceneServices(std::string name) : Node(name) {
    node_ = rclcpp::Node::make_shared("scene_services_handle");
    auto node = node_;
    curr_mode_ = Mode_zero;

    yisbot_goto_ = node->create_publisher<std_msgs::msg::Int16>("/yisbot/goto", 10);
    yisbot_cannel_ = node->create_publisher<std_msgs::msg::Empty>("/yisbot/cancel", 10);
    recharge_ctrl_pub_ = node->create_publisher<std_msgs::msg::String>("/recharge/ctrl", 10);
    sence_pub_ = node->create_publisher<interface::msg::VlagoScene>(operationSceTopic, 10);
    mode_status_pub_ = node->create_publisher<interface::msg::ReportMode>(modeStatusTopic, 10);
    recharge_status_pub_ = node->create_publisher<interface::msg::RechargeStatus>(rechargeStatusTopic, 10);
    mode_pub_ = node->create_publisher<std_msgs::msg::String>(sceneServiceModeTopic, 10);

    for (auto &enable : mode_enable_) {
        enable = true;
    }

    mode_pub_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
        std_msgs::msg::String report_msg;
        report_msg.data = mode_str_map[curr_mode_];
        mode_pub_->publish(report_msg);
    });

    mode_client_list_[Mode_Auxiliary] = auxiliary_client_ = node->create_client<interface::srv::SceneControls>(sceneAuxiliaryService);
    // mode_client_list_[Mode_Narrow] = narrow_client_ =
    // node->create_client<interface::srv::SceneControls>(sceneNarrowService);
    mode_client_list_[Mode_Narrow] = nullptr;
    mode_client_list_[Mode_Standard] = standard_client_ = node->create_client<interface::srv::SceneControls>(sceneStandardService);
    mode_client_list_[Mode_Cruise] = cruise_client_ = node->create_client<interface::srv::SceneControls>(sceneCruiseService);
    // mode_client_list_[Mode_Dance] = dance_client_ =
    // node->create_client<interface::srv::SceneControls>(sceneDanceService);
    mode_client_list_[Mode_Dance] = nullptr;
    // mode_client_list_[Mode_Recharge] = recharge_client_ =
    // node->create_client<interface::srv::SceneControls>(sceneRechargeService);
    mode_client_list_[Mode_Recharge] = nullptr;
    // mode_client_list_[Mode_Followme] = followme_client_ =
    // node->create_client<interface::srv::SceneControls>(sceneFollowmeService);
    mode_client_list_[Mode_Followme] = nullptr;
    mode_client_list_[Mode_Autoindoor] = nullptr;

    RCLCPP_INFO(getnode()->get_logger(), "SceneServices init wait_for_service 1");

    while (!auxiliary_client_->wait_for_service(std::chrono::seconds(3))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(getnode()->get_logger(), "auxiliary_client_  while waiting for the service. Exiting.");
            exit(0);
        }
        RCLCPP_INFO(getnode()->get_logger(), "auxiliary_client_service not available, waiting again...");
    }

    RCLCPP_INFO(getnode()->get_logger(), "SceneServices init create_service");
    // auto rechargeService = "/vlago/auto_charge";
    recharge_ctrl_ = this->create_service<interface::srv::RechargeControls>(rechargeService, std::bind(&SceneServices::rechargeCtrl, this, _1, _2));
    followme_ctrl_ = this->create_service<interface::srv::SceneControls>(manageFollowmeService, std::bind(&SceneServices::followmeCtrl, this, _1, _2));
    mode_enable_ctrl_ = this->create_service<interface::srv::ModeEnable>(vlagoSettingService, std::bind(&SceneServices::modeEnable, this, _1, _2));
    mode_enable_single_ctrl_ =
        this->create_service<interface::srv::KeyValueInt>(vlagoModeEnableService, std::bind(&SceneServices::modeEnableSingle, this, _1, _2));
    autoindoor_ctrl_ = this->create_service<interface::srv::SceneControls>(autoindoorService, std::bind(&SceneServices::autoindoorCtrl, this, _1, _2));

    // start && goto
    autoindoor_nav_goto_ =
        this->create_service<interface::srv::AutoindoorNav>("/vlago/startAutoDrive", std::bind(&SceneServices::autoindoorNavGoto, this, _1, _2));

    // cannel && stop
    autoindoor_nav_cannel_ =
        this->create_service<interface::srv::AutoindoorNav>("/vlago/stopAutoDrive", std::bind(&SceneServices::autoindoorNavCannel, this, _1, _2));

    feed_narrow_ctrl_ = this->create_service<interface::srv::SceneControls>(feedNarrowService, std::bind(&SceneServices::feedNarrow, this, _1, _2));
    feed_recharge_ctrl_ = this->create_service<interface::srv::SceneControls>(feedRechargeService, std::bind(&SceneServices::feedRecharge, this, _1, _2));
    feed_followme_ctrl_ = this->create_service<interface::srv::SceneControls>(feedFollwmeService, std::bind(&SceneServices::feedFollowme, this, _1, _2));
    feed_dance_ctrl_ = this->create_service<interface::srv::SceneControls>(feedDanceService, std::bind(&SceneServices::feedDance, this, _1, _2));
    feed_cruise_ctrl_ = this->create_service<interface::srv::SceneControls>(manageCruiseService, std::bind(&SceneServices::feedCruise, this, _1, _2));

    RCLCPP_INFO(getnode()->get_logger(), "SceneServices init OnTrigger Mode_Auxiliary 3");
    OnTrigger(Mode_Auxiliary, true);
    RCLCPP_INFO(getnode()->get_logger(), "SceneServices init wait_for_service end");

    recharge_feed_sub_ = this->create_subscription<std_msgs::msg::Int32>("/recharge/feed", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
        // printf("got autobot_indoor_checkin_ :%d\n", msg->data);
        int start = msg->data;
        std::thread([this, start]() {
            // 1 搜索充电站 2 未找到充电站 3找到充电站并前往 4检测到障碍物
            // 5开始充电
            if (start == 2) {
                // 未找到充电站
                OnlineRechargeStatus(2);
                OnTrigger(Mode_Recharge, false);
            } else if (start == 3) {
                // 3找到充电站并前往
                OnlineRechargeStatus(3);
            } else if (start == 4) {
                // 4检测到障碍物
                OnlineRechargeStatus(4);
                OnTrigger(Mode_Recharge, false);
            } else if (start == 5) {
                // 5开始充电
                OnlineRechargeStatus(5);
                OnTrigger(Mode_Recharge, false);
            } else if (start == 6) {
                // 6已停止充电
                OnlineRechargeStatus(6);
                OnTrigger(Mode_Recharge, false);
            }
        }).detach(); // 分离线程，让它独立运行
    });


    autobot_checkin_sub_ = this->create_subscription<std_msgs::msg::Bool>("/vlago/auto_drive/checkin", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        // printf("got autobot_indoor_checkin_ :%d\n", msg->data);
        autobot_indoor_checkin_ = msg->data;
    });
}

SceneServices::~SceneServices() {}

rclcpp::Node::SharedPtr SceneServices::getnode() { return node_; }

bool SceneServices::mockRechargeCtrl(int on_off) {
    mock_recharge_count_++;
    if (mock_recharge_count_ % 4 > 2) {
        OnlineRechargeStatus(1);
        mock_recharge_status_ = 1;
        mock_recharge_timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), [this]() {
            if (mock_recharge_status_ == 1) {
                mock_recharge_status_ = 3;
                OnlineRechargeStatus(3);
            } else if (mock_recharge_status_ == 2) {
                mock_recharge_status_ = 5;
                OnlineRechargeStatus(5);
                mock_recharge_timer_->cancel();
            } else if (mock_recharge_status_ == 3) {
                mock_recharge_status_ = 5;
                OnlineRechargeStatus(5);
            } else {
                mock_recharge_timer_->cancel();
            }
        });
        return true;
    } else {
        OnlineRechargeStatus(2);
        return false;
    }
}

void SceneServices::rechargeCtrl(const std::shared_ptr<interface::srv::RechargeControls::Request> request,
                                 std::shared_ptr<interface::srv::RechargeControls::Response> response) {
    RCLCPP_INFO(this->get_logger(), "get rechargeCtrl 0: on_off: %d", request->on_off);
    if (request->on_off) {
        OnTrigger(Mode_Recharge, true);
        response->code = 0;
        response->type = request->on_off;
        response->message = "success";
        RCLCPP_INFO(this->get_logger(), "get rechargeCtrl end: on_off: %d", request->on_off);
        return;
    }

    if (!request->on_off) {
        OnTrigger(Mode_Recharge, false);
        response->code = 0;
        response->type = request->on_off;
        response->message = "success";
        RCLCPP_INFO(this->get_logger(), "get rechargeCtrl end: on_off: %d", request->on_off);
        return;
    }
}

void SceneServices::followmeCtrl(const std::shared_ptr<interface::srv::SceneControls::Request> request,
                                 std::shared_ptr<interface::srv::SceneControls::Response> response) {
    /*
    1. 关闭其他模式
    2. 其他模式关闭上报。
    3. 启动followme相关程序
    4. 启动followme的控车
    5. 开启
    */
    RCLCPP_INFO(this->get_logger(), "get followmeCtrl 0: status: %d", request->start);
    std::thread([this, request]() {
        if (request->start) {
            OnTrigger(Mode_Followme, true);
            RCLCPP_INFO(this->get_logger(), "get followmeCtrl end: status: %d", request->start);
            return;
        }

        /*
        1. 关闭followme的控车
        2. 关闭followme相关程序
        3. 上报followme关闭状态。
        4. 启动辅助驾驶
        5. 上报辅助驾驶开启状态。
        */
        if (!request->start) {
            OnTrigger(Mode_Followme, false);
            RCLCPP_INFO(this->get_logger(), "get followmeCtrl end: status: %d", request->start);
            return;
        }
    }).detach(); // 分离线程，让它独立运行
    response->code = 0;
    response->message = "success";
}

void SceneServices::autoindoorCtrl(const std::shared_ptr<interface::srv::SceneControls::Request> request,
                                   std::shared_ptr<interface::srv::SceneControls::Response> response) {
    SecneMode tigger_mode = Mode_Autoindoor;
    RCLCPP_INFO(this->get_logger(), "get autoindoorCtrl 0: status: %d", request->start);
    std::thread([this, tigger_mode, request]() {
        if (request->start) {
            OnTrigger(tigger_mode, true);
            RCLCPP_INFO(this->get_logger(), "get autoindoorCtrl end: status: %d", request->start);
            return;
        } else {
            OnTrigger(tigger_mode, false);
            RCLCPP_INFO(this->get_logger(), "get autoindoorCtrl end: status: %d", request->start);
            return;
        }
    }).detach(); // 分离线程，让它独立运行
    response->code = 0;
    response->message = "success";
}

void SceneServices::autoindoorNavGoto(const std::shared_ptr<interface::srv::AutoindoorNav::Request> request,
                                      std::shared_ptr<interface::srv::AutoindoorNav::Response> response) {
    RCLCPP_INFO(this->get_logger(), "autoindoorNavGoto: id: %d", request->id);
    if (false == autobot_indoor_checkin_) {
        response->code = 1; // 0可用 ，1不可用
        response->message = "not checkin";
        return;
    }
    if (curr_mode_ != Mode_Autoindoor) {
        std::set ignore_mode_list = {Mode_Narrow, Mode_Cruise, Mode_Dance, Mode_Followme};
        if (ignore_mode_list.find(curr_mode_) != ignore_mode_list.end()) {
            // 不支持
            response->code = -1;
            response->message = "other mode locked";
            RCLCPP_WARN(this->get_logger(), "已锁定其他模式， 无法进入自动驾驶");
            return;
        }
        std::thread([this, request]() {
            OnTrigger(Mode_Autoindoor, true);
            std_msgs::msg::Int16 pub_msg;
            pub_msg.data = request->id;
            RCLCPP_INFO(this->get_logger(), "已切换到自动驾驶");
            yisbot_goto_->publish(pub_msg);
            printf("1 发布目标 id:%d\n", pub_msg.data);
            // sleep(2);
            // yisbot_goto_->publish(pub_msg);
            // printf("2 发布目标 id:%d\n", pub_msg.data);
            // sleep(1);
            // yisbot_goto_->publish(pub_msg);
            // printf("3 发布目标 id:%d\n", pub_msg.data);
        }).detach(); // 分离线程，让它独立运行
    } else {
        std_msgs::msg::Int16 pub_msg;
        pub_msg.data = request->id;
        yisbot_goto_->publish(pub_msg);
    }
    response->code = 0;
    response->message = "success";
}

void SceneServices::autoindoorNavCannel(const std::shared_ptr<interface::srv::AutoindoorNav::Request> request,
                                        std::shared_ptr<interface::srv::AutoindoorNav::Response> response) {
    RCLCPP_INFO(this->get_logger(), "autoindoorNavCannel");
    std_msgs::msg::Empty pub_msg;
    yisbot_cannel_->publish(pub_msg);
    response->code = 0;
    response->message = "success";
}

void SceneServices::feedRecharge(const std::shared_ptr<interface::srv::SceneControls::Request> request,
                                 std::shared_ptr<interface::srv::SceneControls::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received feedRecharge: status: %d", request->start);
    std::thread([this, request]() {
        // 1 搜索充电站 2 未找到充电站 3找到充电站并前往 4检测到障碍物
        // 5开始充电
        if (request->start == 2) {
            // 未找到充电站
            OnlineRechargeStatus(2);
            OnTrigger(Mode_Recharge, false);
        } else if (request->start == 3) {
            // 3找到充电站并前往
            OnlineRechargeStatus(3);
        } else if (request->start == 4) {
            // 4检测到障碍物
            OnlineRechargeStatus(4);
            OnTrigger(Mode_Recharge, false);
        } else if (request->start == 5) {
            // 5开始充电
            OnlineRechargeStatus(5);
            OnTrigger(Mode_Recharge, false);
        } else if (request->start == 6) {
            // 6已停止充电
            OnlineRechargeStatus(6);
            OnTrigger(Mode_Recharge, false);
        }
    }).detach(); // 分离线程，让它独立运行
    response->code = 1;
    response->message = "success";
}

void SceneServices::feedNarrow(const std::shared_ptr<interface::srv::SceneControls::Request> request,
                               std::shared_ptr<interface::srv::SceneControls::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received feedNarrow: status: %d", request->start);
    std::thread([this, request]() {
        int ret = 0;
        if (request->start == 0) {
            // 停止
            OnTrigger(Mode_Narrow, false);
        } else if (request->start == 1) {
            // 启动
            if (mode_enable_[Mode_Narrow]) {
                ret = 0;
                OnTrigger(Mode_Narrow, true);
            } else {
                RCLCPP_INFO(this->get_logger(), "feedNarrow  disabled status: %d", request->start);
                ret = 1;
            }
        } else if (request->start == 2) {
            // 暂停
            ret = 1;
        }
    }).detach(); // 分离线程，让它独立运行
    response->code = 0;
    response->message = "success";
}

void SceneServices::feedFollowme(const std::shared_ptr<interface::srv::SceneControls::Request> request,
                                 std::shared_ptr<interface::srv::SceneControls::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received feedFollowme: status: %d", request->start);
    std::thread([this, request]() {
        if (request->start == 0) {
            // 停止
            // OnTrigger(Mode_Followme, false);
        } else if (request->start == 1) {
            // 启动
        }
    }).detach(); // 分离线程，让它独立运行
    response->code = 1;
    response->message = "success";
}

void SceneServices::feedDance(const std::shared_ptr<interface::srv::SceneControls::Request> request,
                              std::shared_ptr<interface::srv::SceneControls::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received feedNarrow: status: %d", request->start);
    std::thread([this, request]() {
        if (request->start == 0) {
            // 停止
            OnTrigger(Mode_Dance, false);
        } else if (request->start == 1) {
            // 启动
            OnTrigger(Mode_Dance, true);
        } else if (request->start == 2) {
            // 暂停
        }
    }).detach(); // 分离线程，让它独立运行
    response->code = 0;
    response->message = "success";
}

void SceneServices::feedCruise(const std::shared_ptr<interface::srv::SceneControls::Request> request,
                               std::shared_ptr<interface::srv::SceneControls::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received feedCruise: status: %d", request->start);
    std::thread([this, request]() {
        if (request->start == 0) {
            // 停止
            OnTrigger(Mode_Cruise, false);
        } else if (request->start == 1) {
            // 启动
            OnTrigger(Mode_Cruise, true);
        } else if (request->start == 2) {
            // 暂停
        }
    }).detach(); // 分离线程，让它独立运行
    response->code = 0;
    response->message = "success";
}

void SceneServices::modeEnable(const std::shared_ptr<interface::srv::ModeEnable::Request> request,
                               std::shared_ptr<interface::srv::ModeEnable::Response> response) {
    std::thread([this, request]() {
        enum SecneMode mode;
        mode = Mode_Narrow;
        if (!request->narrow) {
            if (curr_mode_ == mode) {
                doStop(mode);
            }
        }
        RCLCPP_INFO(getnode()->get_logger(), "modeEnable start narrow: %d auxiliary: %d  followme: %d", request->narrow, request->auxiliary, request->followme);
        mode_enable_[mode] = request->narrow;

        mode = Mode_Auxiliary;
        if (!request->auxiliary) {
            if (curr_mode_ == Mode_Auxiliary) {
                doScencSwitch(Mode_Standard, curr_mode_);
            }
        } else {
            if (curr_mode_ == Mode_Standard) {
                doScencSwitch(Mode_Auxiliary, curr_mode_);
            }
        }
        mode_enable_[mode] = request->auxiliary;

        mode = Mode_Followme;
        if (!request->followme) {
            if (curr_mode_ == mode) {
                doStop(mode);
            }
        }
        mode_enable_[mode] = request->followme;
    }).detach(); // 分离线程，让它独立运行
    response->code = 0;
    char msg[512];
    sprintf(msg, "{narrow: %s, auxiliary: %s, followme: %s}", request->narrow ? "true" : "false", request->auxiliary ? "true" : "false",
            request->followme ? "true" : "false");
    response->message = "success";
}

void SceneServices::modeEnableSingle(const std::shared_ptr<interface::srv::KeyValueInt::Request> request,
                                     std::shared_ptr<interface::srv::KeyValueInt::Response> response) {
    std::thread([this, request]() {
        const std::string mode_name = request->name;
        enum SecneMode mode = find_mode(request->name);
        bool enable = request->value;
        RCLCPP_INFO(getnode()->get_logger(), "modeEnableSingle mode_name:%s enable: %d", mode_name.c_str(), enable);
        mode_enable_[mode] = enable;
        /*
        1、enable安全
            1.1 当前标准： 停止标准+启动安全
        2、disable安全
            2.1 当前安全： 停止安全+启动标准
        */
        if (mode == Mode_Auxiliary) {
            if (!enable) {
                //
                if (curr_mode_ == Mode_Auxiliary) {
                    doScencSwitch(Mode_Standard, curr_mode_);
                }
            } else {
                if (curr_mode_ == Mode_Standard) {
                    doScencSwitch(Mode_Auxiliary, curr_mode_);
                }
            }
        }
        SecneMode default_mode = getDefaultMode();
        /*
        1、enable狭窄
            1.1、不做处理
        2、disable狭窄
            2.1、当前狭窄：停止狭窄空间 + 启动默认(安全/标准)。
        */
        if (mode == Mode_Narrow) {
            if (!enable) { // disable narrow
                if (curr_mode_ == mode) {
                    doScencSwitch(default_mode, curr_mode_);
                }
            }
        }

        /*
        1、enable自主跟随
            1.1、不做处理
        2、disable自主跟随
            2.1、当前自主跟随：停止自主跟随 + 启动默认(安全/标准)。
        */
        if (mode == Mode_Followme) {
            if (!enable) { // disable followme
                if (curr_mode_ == mode) {
                    doScencSwitch(default_mode, curr_mode_);
                }
            }
        }
    }).detach(); // 分离线程，让它独立运行
    response->code = 0;
    response->message = "success";
}

bool SceneServices::waitControlService(enum SecneMode mode) {
    auto client = mode_client_list_[mode];
    if (client == nullptr) {
        return true;
    }
    // 等待Service服务器可用
    int try_time = 10;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(getnode()->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(getnode()->get_logger(), "%s service not available, waiting again...", mode_str_map[mode]);
        if (try_time-- == 0) {
            RCLCPP_ERROR(getnode()->get_logger(), "%s service not available...", mode_str_map[mode]);
            return false;
        }
    }
    return true;
}

void SceneServices::OnConfigEnable(enum SecneMode mode, bool enable) { mode_enable_[mode] = enable; }

SecneMode SceneServices::getDefaultMode() {
    SecneMode default_mode = Mode_Auxiliary;
    if (!mode_enable_[Mode_Auxiliary]) {
        default_mode = Mode_Standard;
    }
    return default_mode;
}

void SceneServices::OnTrigger(enum SecneMode tigger_mode, bool start) {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    SecneMode default_mode = getDefaultMode();

    // 屏蔽不变更情况
    if (start) {
        if (tigger_mode == curr_mode_) {
            return;
        }
    } else {
        if (tigger_mode != curr_mode_) {
            return;
        }
    }

    // 禁止外部切出
    if ((curr_mode_ == Mode_Dance) && (tigger_mode != Mode_Dance)) {
        RCLCPP_INFO(this->get_logger(), "OnTrigger %s faild, now is Mode_Dance", mode_str_map[curr_mode_]);
        return;
    }
    if ((curr_mode_ == Mode_Followme) && (tigger_mode != Mode_Followme)) {
        RCLCPP_INFO(this->get_logger(), "OnTrigger %s faild, now is Mode_Followme", mode_str_map[curr_mode_]);
        return;
    }
    if ((curr_mode_ == Mode_Recharge) && (tigger_mode != Mode_Recharge)) {
        RCLCPP_INFO(this->get_logger(), "OnTrigger %s faild, now is Mode_Recharge", mode_str_map[curr_mode_]);
        return;
    }
    if ((curr_mode_ == Mode_Autoindoor) && (tigger_mode != Mode_Autoindoor)) {
        RCLCPP_INFO(this->get_logger(), "OnTrigger %s faild, now is Mode_Recharge", mode_str_map[curr_mode_]);
        return;
    }

    // 通用切换
    if (start) {
        doScencSwitch(tigger_mode, curr_mode_);
    } else {
        // 切换到默认模式
        doScencSwitch(default_mode, curr_mode_);
    }
}

void SceneServices::OnlineReportMode(enum SecneMode opened_mode, enum SecneMode closed_mode) {
    if (closed_mode == Mode_zero) {
        closed_mode = last_mode_;
    }
    interface::msg::ReportMode report_msg;
    interface::msg::NamedStatus status1;
    status1.name = mode_str_map[opened_mode];
    status1.status = 1;
    report_msg.mode_list.push_back(status1);
    if (closed_mode != Mode_zero) {
        interface::msg::NamedStatus status2;
        status2.name = mode_str_map[closed_mode];
        status2.status = 0;
        report_msg.mode_list.push_back(status2);
    }
    mode_status_pub_->publish(report_msg);
}

void SceneServices::OnlineReportStatus(enum SecneMode mode, int status) {}

void SceneServices::OnlineRechargeStatus(int status) {
    interface::msg::RechargeStatus report_msg;
    report_msg.status = status;
    recharge_status_pub_->publish(report_msg);
}

bool SceneServices::do_recharge_ctrl(bool start) {
    std_msgs::msg::String report_msg;
    report_msg.data = start ? "start" : "stop";
    recharge_ctrl_pub_->publish(report_msg);
}

bool SceneServices::doScencSwitch(enum SecneMode new_mode, enum SecneMode old_mode) {
    RCLCPP_INFO(getnode()->get_logger(), "doScencSwitch 1 new : %s old_mode: %s", mode_str_map[new_mode], mode_str_map[old_mode]);

    bool ret = true;
    RCLCPP_INFO(getnode()->get_logger(), "doScencSwitch 2 new : %s old_mode: %s", mode_str_map[new_mode], mode_str_map[old_mode]);

    if (!doStop(old_mode)) {
        ret = false;
    }

    RCLCPP_INFO(getnode()->get_logger(), "doScencSwitch 3 new : %s old_mode: %s", mode_str_map[new_mode], mode_str_map[old_mode]);
    if (!doStart(new_mode)) {
        ret = false;
    }
    RCLCPP_INFO(getnode()->get_logger(), "doScencSwitch 4 new : %s old_mode: %s", mode_str_map[new_mode], mode_str_map[old_mode]);
    ChangeMode(new_mode);
    if (old_mode == Mode_Recharge) {
        do_recharge_ctrl(false);
    }
    if (new_mode == Mode_Recharge) {
        do_recharge_ctrl(true);
    }
    return ret;
}

bool SceneServices::check_mode_in(SecneMode comming_mode) {
    if (curr_mode_ == comming_mode) {
        return true;
    }
    std::set<SecneMode> stop_self_list = {Mode_Narrow, Mode_Cruise, Mode_Dance, Mode_Recharge, Mode_Followme};
    if (stop_self_list.find(comming_mode) != stop_self_list.end()) {
        return false;
    }
    return true;
}

bool SceneServices::ChangeMode(enum SecneMode new_mode) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (new_mode == curr_mode_) {
            return false;
        }
        RCLCPP_WARN(getnode()->get_logger(), "ChangeMode old:%s --> : %s", mode_str_map[curr_mode_], mode_str_map[new_mode]);
        last_mode_ = curr_mode_;
        curr_mode_ = new_mode;

        std_msgs::msg::String report_msg;
        report_msg.data = mode_str_map[curr_mode_];
        mode_pub_->publish(report_msg);
    }
    OnlineReportMode(new_mode, last_mode_);
}

bool SceneServices::doStart(enum SecneMode mode) {
    RCLCPP_INFO(getnode()->get_logger(), "doStart 1 %s ", mode_str_map[mode]);
    std::set ignore_mode_list = {Mode_Narrow, Mode_Dance, Mode_Followme, Mode_Autoindoor};
    if (ignore_mode_list.find(mode) != ignore_mode_list.end()) {
        return true;
    }

    RCLCPP_INFO(getnode()->get_logger(), "doStart 2 %s ", mode_str_map[mode]);
    if (!waitControlService(mode)) {
        RCLCPP_ERROR(getnode()->get_logger(), "waitControlService faild. Exiting. %d", mode);
        return false;
    }

    bool ret = true;
    RCLCPP_INFO(getnode()->get_logger(), "doStart 3 %s ", mode_str_map[mode]);
    if (mode == Mode_Recharge) {
        // do_recharge_ctrl(true);
    } else {
        auto ctrl_client = mode_client_list_[mode];
        if (ctrl_client) {
            auto request = std::make_shared<interface::srv::SceneControls::Request>();
            request->start = 1;
            RCLCPP_INFO(getnode()->get_logger(), "doStart 4 %s ", mode_str_map[mode]);
            auto result = ctrl_client->async_send_request(request);
            auto timeout = std::chrono::seconds(5);
            if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(getnode()->get_logger(), "doStart Failed to call service faild %d", mode);
                return false;
            }
            if (result.get()->code == 0) {
            } else {
                ret = false;
            }
        }
    }
    RCLCPP_INFO(getnode()->get_logger(), "doStart done %s ", mode_str_map[mode]);
    return ret;
}

bool SceneServices::doStop(enum SecneMode mode) {
    std::set ignore_mode_list = {Mode_Narrow, Mode_Dance, Mode_Followme, Mode_Autoindoor};
    if (ignore_mode_list.find(mode) != ignore_mode_list.end()) {
        return true;
    }
    if (mode == SecneMode::Mode_zero) {
        return true;
    }
    if (!waitControlService(mode)) {
        RCLCPP_ERROR(getnode()->get_logger(), "waitControlService faild. Exiting. %d", mode);
        return false;
    }

    bool ret = true;
    if (mode == Mode_Recharge) {
        // do_recharge_ctrl(false);
    } else {
        auto ctrl_client = mode_client_list_[mode];
        if (ctrl_client) {
            auto request = std::make_shared<interface::srv::SceneControls::Request>();
            request->start = 0;
            auto result = ctrl_client->async_send_request(request);
            auto timeout = std::chrono::seconds(5);
            if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(getnode()->get_logger(), "Failed to call service faild %d", mode);
                return false;
            }
            if (result.get()->code == 0) {
            } else {
                ret = false;
            }
        }
    }

    return ret;
}

} // namespace service::scene
