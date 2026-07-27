#include "postback_node.hpp"
#include "postback/utils.hpp"
#include "utils/topic.h"
#include "interface/srv/id_controls.hpp"
#include <filesystem>
#include <chrono>
#include <thread>

namespace postback {
PostbackNode::PostbackNode() : rclcpp::Node("postback_node") {
    Initialize();
}

PostbackNode::~PostbackNode() {
    
}

void PostbackNode::Initialize() {
    // 加载 ROS 参数
    load_ros_parameters();
    oss_config_.bucketName = oss_bucket_name_;
    oss_manager_ = std::make_shared<OSSManager>(oss_config_);
    oss_manager_->set_receive_handler([this](size_t /*increment*/, int64_t transfered, int64_t total, void* /*userData*/) {
        static double last_logged_progress = 0.0;
        if (total > 0) {
            double progress = (static_cast<double>(transfered) / static_cast<double>(total)) * 100.0;
            if (progress - last_logged_progress >= 10.0 || progress >= 100.0) {
                RCLCPP_INFO(this->get_logger(), "OSS 上传进度：%.1f%% (%.2f kb / %.2f kb)", progress, static_cast<double>(transfered) / 1024.0, static_cast<double>(total) / 1024.0);
                last_logged_progress = progress;
            }
        }
    });
    
    auto timer_callback = std::bind(&PostbackNode::handle_timer_policy, this);
    timer_ = this->create_wall_timer(std::chrono::minutes(5), timer_callback);
    
    RCLCPP_INFO(this->get_logger(), "PostbackNode initialized");
    RCLCPP_INFO(this->get_logger(), "VIN: %s", vin_.c_str());
    RCLCPP_INFO(this->get_logger(), "Model: %s", model_.c_str());
    RCLCPP_INFO(this->get_logger(), "oss_bucket_name_: %s", oss_bucket_name_.c_str());

    RCLCPP_INFO(this->get_logger(), "Log path: %s", log_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Bag path: %s", bag_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "OSS path: %s", oss_path_.c_str());
}

void PostbackNode::load_ros_parameters() {

    this->declare_parameter<std::string>("vin", "");
    this->declare_parameter<std::string>("model", "");
    this->declare_parameter<std::string>("oss_bucket_name", "");

    this->get_parameter("vin", vin_);
    this->get_parameter("model", model_);
    this->get_parameter("oss_bucket_name", oss_bucket_name_);
    RCLCPP_INFO(this->get_logger(), "oss_bucket_name_: %s", oss_bucket_name_.c_str());
    {
        std::string sn, msg;
        if(request_sn_via_service(0, sn, msg)) {
            vin_ = sn;
        }
    }
    RCLCPP_INFO(this->get_logger(), "vin: %s  model: %s", vin_.c_str(), model_.c_str());
}

bool PostbackNode::request_sn_via_service(uint32_t id, std::string& sn, std::string& err) {
    RCLCPP_INFO(this->get_logger(), "🔍 开始通过 Service 请求 SN id=%d", id);
    try {
        // 等待服务可用（带超时）
        auto client = this->create_client<interface::srv::IdControls>(idControlService);
        if (!client) {
            err = "创建 IdControls Service 客户端失败";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
        if (!client->wait_for_service(std::chrono::seconds(1))) {
            err = "IdControls Service 不可用：/vlago/car_id";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
        auto request = std::make_shared<interface::srv::IdControls::Request>();
        request->id = id;
        RCLCPP_INFO(this->get_logger(), "📤 发送 IdControls 请求");
        // 同步调用（带超时）
        auto future = client->async_send_request(request);
        auto result = spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(3));
        if (result == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            sn = response->sn;
            uint32_t code = response->code;
            RCLCPP_INFO(this->get_logger(), "✅ IdControls 请求成功： sn='%s', code=%d", sn.c_str(), code);            
        } else {
            err = "IdControls Service 调用失败（超时或其他异常）";
            RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
            return false;
        }
    } catch (const std::exception& e) {
        err = "IdControls Service 调用异常：" + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "❌ %s", err.c_str());
        return false;
    }
    return true;
}

void PostbackNode::handle_timer_policy() {
    RCLCPP_INFO(this->get_logger(), "Executing timer policy");
    upload_logs();
    upload_bags();
}

bool PostbackNode::upload_zipped_directories(const std::vector<std::string>& directories, const std::string& zip_file, const std::string& oss_path) {
    if (directories.empty()) return false;
    std::string err;
    if (!utils::add_directorys_files_to_zip(zip_file, directories, err)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to add directories to zip: %s", err.c_str());
        return false;
    }
    
    if (oss_manager_->upload_file(zip_file, oss_path, err)) {
        RCLCPP_INFO(this->get_logger(), "File uploaded successfully: %s", zip_file.c_str());
        std::filesystem::remove(zip_file);
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to upload directories: %s, error: %s", zip_file.c_str(), err.c_str());
        return false;
    }
}

void PostbackNode::upload_logs() {
    std::string err;
    auto directories = utils::collect_recent_directories(log_path_, err);
    if (directories.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to collect log directories: %s", err.c_str());
        return;
    }    
    
    auto date_str = utils::get_date_str();
    auto uuid = utils::get_UUID_str();
    auto zip_file = "/tmp/" + vin_ + "_" + uuid + "_info.zip";
    auto oss_path = oss_path_ + "/" + model_ + "/" + date_str + "/log/" + vin_ + "_" + uuid + "_info.zip";
    
    RCLCPP_INFO(this->get_logger(), "Uploading zipped log directories: %s to %s", zip_file.c_str(), oss_path.c_str());
    upload_zipped_directories(directories, zip_file, oss_path);
}

void PostbackNode::upload_bags() {
    std::string err;
    auto directories = utils::collect_recent_directories(bag_path_, err);
    if (directories.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to collect bag directories: %s", err.c_str());
        return;
    }    
    
    auto date_str = utils::get_date_str();
    auto uuid = utils::get_UUID_str();
    auto zip_file = "/tmp/" + vin_ + "_" + uuid + "_bag.zip";
    auto oss_path = oss_path_ + "/" + model_ + "/" + date_str + "/bag/" + vin_ + "_" + uuid + "_bag.zip";
    
    RCLCPP_INFO(this->get_logger(), "Uploading zipped bag directories: %s to %s", zip_file.c_str(), oss_path.c_str());
    upload_zipped_directories(directories, zip_file, oss_path);
}

} // namespace postback
