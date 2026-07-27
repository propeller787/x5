#include "scene_services.h"

#include <set>

/*
plug Clang-Format
IndentWidth: '4'
ColumnLimit: 160
*/
using namespace std::placeholders;

namespace service::scene
{

    SecneMode find_mode(const std::string &str_mode)
    {
        for (auto &ch : mode_str_map)
        {
            if (ch.second == str_mode)
            {
                return ch.first;
            }
        }
        return Mode_zero;
    }

    SceneServices::SceneServices(std::string name) : Node(name)
    {
        node_ = rclcpp::Node::make_shared("scene_services_handle");
        auto node = node_;
        curr_mode_ = Mode_zero;

        mode_status_pub_ = node->create_publisher<interface::msg::ReportMode>(modeStatusTopic, 10);
        mode_str_pub_ = node->create_publisher<std_msgs::msg::String>(sceneServiceModeTopic, 10);

        for (auto &enable : mode_enable_)
        {
            enable = true;
        }

        // 定时发送当前模式
        mode_pub_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
            std_msgs::msg::String report_msg;
            report_msg.data = mode_str_map[curr_mode_];
            mode_str_pub_->publish(report_msg); 
        });

        mode_client_list_[Mode_Standard] = standard_client_ = node->create_client<interface::srv::SceneControls>(sceneStandardService);
        mode_client_list_[Mode_Cruise] = cruise_client_ = node->create_client<interface::srv::SceneControls>(sceneCruiseService);
        mode_client_list_[Mode_Helpcar] = nullptr;
        mode_client_list_[Mode_Recharge] = recharge_client_ = node->create_client<interface::srv::SceneControls>(sceneRechargeService);

        RCLCPP_INFO(getnode()->get_logger(), "SceneServices init wait_for_service 1");

        while (!standard_client_->wait_for_service(std::chrono::seconds(3)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(getnode()->get_logger(), "standard_client_  while waiting for the service. Exiting.");
                exit(0);
            }
            RCLCPP_INFO(getnode()->get_logger(), "standard_client_service not available, waiting again...");
        }


        RCLCPP_INFO(getnode()->get_logger(), "SceneServices init create_service");
        feed_recharge_ctrl_ = this->create_service<interface::srv::SceneControls>(manageRechargeService, std::bind(&SceneServices::feedRecharge, this, _1, _2));
        feed_cruise_ctrl_ = this->create_service<interface::srv::SceneControls>(manageCruiseService, std::bind(&SceneServices::feedCruise, this, _1, _2));
        feed_help_ctrl_ = this->create_service<interface::srv::SceneControls>(manageHelpcarService, std::bind(&SceneServices::feedHelpcar, this, _1, _2));
        feed_standard_ctrl_ = this->create_service<interface::srv::SceneControls>(manageStandardService, std::bind(&SceneServices::feedStandard, this, _1, _2));
        feed_bluetooth_ctrl_ = this->create_service<interface::srv::SceneControls>(manageBluetoothService, std::bind(&SceneServices::feedBluetooth, this, _1, _2));

        // mode_enable_ctrl_ = this->create_service<interface::srv::ModeEnable>(vlagoSettingService, std::bind(&SceneServices::modeEnable, this, _1, _2));
        mode_enable_single_ctrl_ =
            this->create_service<interface::srv::KeyValueInt>(vlagoModeEnableService, std::bind(&SceneServices::modeEnableSingle, this, _1, _2));

        RCLCPP_INFO(getnode()->get_logger(), "SceneServices init OnTrigger Mode_Standard 3");
        OnTrigger(Mode_Standard, true);
        RCLCPP_INFO(getnode()->get_logger(), "SceneServices init wait_for_service end");
    }

    SceneServices::~SceneServices() {}

    rclcpp::Node::SharedPtr SceneServices::getnode() { return node_; }

    void SceneServices::feedRecharge(const std::shared_ptr<interface::srv::SceneControls::Request> request,
                                     std::shared_ptr<interface::srv::SceneControls::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedRecharge: status: %d", request->start);
        std::thread([this, request]()
                    {
        // 0 结束回充 1 开始回充
        if (request->start == 0) {
            OnTrigger(Mode_Recharge, false);
        } else if (request->start == 1) {
            OnTrigger(Mode_Recharge, true);
        } })
            .detach(); // 分离线程，让它独立运行
        response->code = 1;
        response->message = "success";
    }

    void SceneServices::feedCruise(const std::shared_ptr<interface::srv::SceneControls::Request> request,
                                   std::shared_ptr<interface::srv::SceneControls::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedCruise: status: %d", request->start);
        std::thread([this, request]()
                    {
        if (request->start == 0) {
            // 停止
            OnTrigger(Mode_Cruise, false);
        } else if (request->start == 1) {
            // 启动
            OnTrigger(Mode_Cruise, true);
        } else if (request->start == 2) {
            // 暂停
        } })
            .detach(); // 分离线程，让它独立运行
        response->code = 0;
        response->message = "success";
    }

    void SceneServices::feedStandard(const std::shared_ptr<interface::srv::SceneControls::Request> request,
                                   std::shared_ptr<interface::srv::SceneControls::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedStandard: status: %d", request->start);
        std::thread([this, request]()
        {
        if (request->start == 0) {
            OnTrigger(Mode_Standard, false);
        } else if (request->start == 1) {
            // 启动
            OnTrigger(Mode_Standard, true);
        } })
            .detach(); // 分离线程，让它独立运行
        response->code = 0;
        response->message = "success";
    }

    void SceneServices::feedHelpcar(const std::shared_ptr<interface::srv::SceneControls::Request> request,
                                   std::shared_ptr<interface::srv::SceneControls::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedHelpcar: status: %d", request->start);
        std::thread([this, request]()
        {
        if (request->start == 0) {
            // 停止
            OnTrigger(Mode_Helpcar, false);
        } else if (request->start == 1) {
            // 启动
            OnTrigger(Mode_Helpcar, true);
        } else if (request->start == 2) {
            // 暂停
        } })
            .detach(); // 分离线程，让它独立运行
        response->code = 0;
        response->message = "success";
    }
    void SceneServices::feedBluetooth(const std::shared_ptr<interface::srv::SceneControls::Request> request,
                                   std::shared_ptr<interface::srv::SceneControls::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedBluetooth: status: %d", request->start);
        std::thread([this, request]()
        {
        if (request->start == 0) {
            OnTrigger(Mode_Bluetooth, false);
        } else if (request->start == 1) {
            OnTrigger(Mode_Bluetooth, true);
        } })
            .detach(); // 分离线程，让它独立运行
        response->code = 0;
        response->message = "success";
    }

    void SceneServices::modeEnableSingle(const std::shared_ptr<interface::srv::KeyValueInt::Request> request,
                                         std::shared_ptr<interface::srv::KeyValueInt::Response> response)
    {
        std::thread([this, request]()
        {
            const std::string mode_name = request->name;
            enum SecneMode mode = find_mode(request->name);
            bool enable = request->value;
            RCLCPP_INFO(getnode()->get_logger(), "modeEnableSingle mode_name:%s enable: %d", mode_name.c_str(), enable);
            mode_enable_[mode] = enable;
            SecneMode default_mode = getDefaultMode();
        }).detach(); // 分离线程，让它独立运行
        response->code = 0;
        response->message = "success";
    }

    bool SceneServices::waitControlService(enum SecneMode mode)
    {
        auto client = mode_client_list_[mode];
        if (client == nullptr)
        {
            return true;
        }
        // 等待Service服务器可用
        int try_time = 10;
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(getnode()->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(getnode()->get_logger(), "%s service not available, waiting again...", mode_str_map[mode]);
            if (try_time-- == 0)
            {
                RCLCPP_ERROR(getnode()->get_logger(), "%s service not available...", mode_str_map[mode]);
                return false;
            }
        }
        return true;
    }

    SecneMode SceneServices::getDefaultMode()
    {
        return Mode_Standard;
    }

    void SceneServices::OnTrigger(enum SecneMode tigger_mode, bool start)
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        SecneMode default_mode = getDefaultMode();

        // 屏蔽不变更情况
        if (start)
        {
            if (tigger_mode == curr_mode_)
            {
                return;
            }
        }
        else
        {
            if (tigger_mode != curr_mode_)
            {
                return;
            }
        }

        // 禁止外部切出
        if ((curr_mode_ == Mode_Recharge) && (tigger_mode != Mode_Recharge))
        {
            RCLCPP_INFO(this->get_logger(), "OnTrigger %s faild, now is Mode_Recharge", mode_str_map[curr_mode_]);
            return;
        }
        RCLCPP_INFO(getnode()->get_logger(), "OnTrigger tigger_mode:%s start:%d default_mode:%s curr_mode_ : %s",  mode_str_map[tigger_mode], start, mode_str_map[default_mode], mode_str_map[curr_mode_]);
        // 通用切换
        if (start)
        {
            doScencSwitch(tigger_mode, curr_mode_);
        }
        else
        {
            // 切换到默认模式
            doScencSwitch(default_mode, curr_mode_);
        }
    }

    void SceneServices::OnlineReportMode(enum SecneMode opened_mode, enum SecneMode closed_mode)
    {
        if (closed_mode == Mode_zero)
        {
            closed_mode = last_mode_;
        }
        interface::msg::ReportMode report_msg;
        interface::msg::NamedStatus status1;
        status1.name = mode_str_map[opened_mode];
        status1.status = 1;
        report_msg.mode_list.push_back(status1);
        if (closed_mode != Mode_zero)
        {
            interface::msg::NamedStatus status2;
            status2.name = mode_str_map[closed_mode];
            status2.status = 0;
            report_msg.mode_list.push_back(status2);
        }
        mode_status_pub_->publish(report_msg);
    }

    bool SceneServices::doScencSwitch(enum SecneMode new_mode, enum SecneMode old_mode)
    {
        RCLCPP_INFO(getnode()->get_logger(), "doScencSwitch 1 new : %s old_mode: %s", mode_str_map[new_mode], mode_str_map[old_mode]);
        // 计算时间差
        auto start_time = std::chrono::steady_clock::now();
        bool ret = true;
        RCLCPP_INFO(getnode()->get_logger(), "doScencSwitch 2 new : %s old_mode: %s", mode_str_map[new_mode], mode_str_map[old_mode]);

        if (!doStop(old_mode))
        {
            ret = false;
        }
        RCLCPP_INFO(getnode()->get_logger(), "doScencSwitch 3 new : %s old_mode: %s", mode_str_map[new_mode], mode_str_map[old_mode]);
        if (!doStart(new_mode))
        {
            ret = false;
        }
        RCLCPP_INFO(getnode()->get_logger(), "doScencSwitch 4 new : %s old_mode: %s", mode_str_map[new_mode], mode_str_map[old_mode]);
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        RCLCPP_INFO(getnode()->get_logger(), "doScencSwitch 5 duration: %d ms", duration.count());
        ChangeMode(new_mode);
        return ret;
    }

    bool SceneServices::ChangeMode(enum SecneMode new_mode)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (new_mode == curr_mode_)
            {
                return false;
            }
            RCLCPP_WARN(getnode()->get_logger(), "ChangeMode old:%s --> : %s", mode_str_map[curr_mode_], mode_str_map[new_mode]);
            last_mode_ = curr_mode_;
            curr_mode_ = new_mode;

            std_msgs::msg::String report_msg;
            report_msg.data = mode_str_map[curr_mode_];
            mode_str_pub_->publish(report_msg);
        }
        OnlineReportMode(new_mode, last_mode_);
        return true;
    }

    bool SceneServices::doStart(enum SecneMode mode)
    {
        RCLCPP_INFO(getnode()->get_logger(), "doStart start %s ", mode_str_map[mode]);

        if (mode == Mode_Bluetooth)
        {
            mode = Mode_Standard;
        }
        std::set ignore_mode_list = {Mode_Followme};
        if (ignore_mode_list.find(mode) != ignore_mode_list.end())
        {
            return true;
        }

        if (!waitControlService(mode))
        {
            RCLCPP_ERROR(getnode()->get_logger(), "waitControlService faild. Exiting. %d", mode);
            return false;
        }

        bool ret = true;

        auto ctrl_client = mode_client_list_[mode];
        if (ctrl_client)
        {
            auto request = std::make_shared<interface::srv::SceneControls::Request>();
            request->start = 1;
            RCLCPP_INFO(getnode()->get_logger(), "doStart 4 %s ", mode_str_map[mode]);
            auto result = ctrl_client->async_send_request(request);
            auto timeout = std::chrono::seconds(5);
            if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result, timeout) != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(getnode()->get_logger(), "doStart Failed to call service faild %d", mode);
                return false;
            }
            if (result.get()->code == 0)
            {
            }
            else
            {
                ret = false;
            }
        }
        RCLCPP_INFO(getnode()->get_logger(), "doStart done %s ", mode_str_map[mode]);
        return ret;
    }

    bool SceneServices::doStop(enum SecneMode mode)
    {
        std::set ignore_mode_list = {Mode_Followme};

        if (mode == Mode_Bluetooth)
        {
            mode = Mode_Standard;
        }
        if (ignore_mode_list.find(mode) != ignore_mode_list.end())
        {
            return true;
        }
        if (mode == SecneMode::Mode_zero)
        {
            return true;
        }
        if (!waitControlService(mode))
        {
            RCLCPP_ERROR(getnode()->get_logger(), "waitControlService faild. Exiting. %d", mode);
            return false;
        }

        bool ret = true;

        auto ctrl_client = mode_client_list_[mode];
        if (ctrl_client)
        {
            auto request = std::make_shared<interface::srv::SceneControls::Request>();
            request->start = 0;
            auto result = ctrl_client->async_send_request(request);
            auto timeout = std::chrono::seconds(5);
            if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result, timeout) != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(getnode()->get_logger(), "Failed to call service faild %d", mode);
                return false;
            }
            if (result.get()->code == 0)
            {
            }
            else
            {
                ret = false;
            }
        }

        return ret;
    }

} // namespace service::scene
