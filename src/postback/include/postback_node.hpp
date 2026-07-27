#pragma once
#include <rclcpp/rclcpp.hpp>
#include <postback/oss_manager.hpp>
#include <vector>
#include <memory>

namespace postback { 
    class PostbackNode : public rclcpp::Node {
    public:
        PostbackNode();
        ~PostbackNode();

        
    private:
        void Initialize();
        void load_ros_parameters();
        bool request_sn_via_service(uint32_t id, std::string& sn, std::string& err);


        
        void handle_timer_policy();

        void upload_logs();
        void upload_bags();

        bool upload_zipped_directories(const std::vector<std::string>& directories, const std::string& zip_file, const std::string& oss_path);

        OSSManager::oss_auto_config oss_config_;
        std::shared_ptr<OSSManager> oss_manager_;

        std::string log_path_ = "/opt/freego/log/archive";
        std::string bag_path_ = "/opt/freego/bags";
        std::string oss_path_ = "logs";

        /* 车辆信息 */
        std::string vin_ = "ZQKJ001";
        std::string model_ = "x5"; // 目前车辆只有X5模型
        std::string oss_bucket_name_;
        
        /* 定时器 */
        rclcpp::TimerBase::SharedPtr timer_;
    };
}