#include <rclcpp/rclcpp.hpp>
#include "cloud_manager/oss_uploader.hpp"
#include <ctime>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <fstream>
#include <sstream>
#include <unistd.h>  // 用于Linux/macOS环境变量检查（可选）
#include <filesystem>
#include <chrono>
#include <random>
#include <thread>

// 检查zip工具是否安装
bool check_zip_installed(const rclcpp::Logger& logger) {
    std::string cmd = "which zip > /dev/null 2>&1";
    int ret = system(cmd.c_str());
    if (ret != 0) {
        RCLCPP_ERROR(logger, "未检测到zip工具，请先安装：sudo apt install zip -y");
        return false;
    }
    return true;
}

// 检查/tmp目录权限
bool check_tmp_permission(const rclcpp::Logger& logger) {
    std::string cmd = "ls -ld /tmp | grep 'drwxrwxrwt' > /dev/null 2>&1";
    int ret = system(cmd.c_str());
    if (ret != 0) {
        RCLCPP_WARN(logger, "/tmp目录权限异常，尝试修复...");
        int fix_ret = system("sudo chmod 1777 /tmp");
        (void)fix_ret; // 显式标记已处理返回值以消除编译警告
        // 再次检查
        ret = system(cmd.c_str());
        if (ret != 0) {
            RCLCPP_ERROR(logger, "/tmp目录权限修复失败，请手动执行：sudo chmod 1777 /tmp");
            return false;
        }
    }
    return true;
}

// 创建zip包的工具函数（增加环境检查）
std::string create_zip_file(const std::string& txt_path, const std::string& zip_path, const rclcpp::Logger& logger) {
    // 先检查环境依赖
    if (!check_zip_installed(logger) || !check_tmp_permission(logger)) {
        return "";
    }

    // 构建带详细输出的命令（去掉-q参数便于调试）
    std::string cmd = "zip -j -b /tmp \"" + zip_path + "\" \"" + txt_path + "\" 2>/tmp/zip_error.log";
    RCLCPP_INFO(logger, "执行zip命令：%s", cmd.c_str());
    int ret = system(cmd.c_str());
    
    if (ret != 0) {
        // 读取错误日志
        std::ifstream err_file("/tmp/zip_error.log");
        std::string err_msg((std::istreambuf_iterator<char>(err_file)), std::istreambuf_iterator<char>());
        // 解析错误码（system返回值为信号值*256 + 退出码）
        int exit_code = WEXITSTATUS(ret);
        RCLCPP_ERROR(logger, "创建zip包失败！退出码：%d，错误信息：%s", exit_code, err_msg.c_str());
        return "";
    }
    
    // 验证文件生成
    if (!std::ifstream(zip_path)) {
        RCLCPP_ERROR(logger, "zip命令执行成功，但文件未生成：%s", zip_path.c_str());
        return "";
    }
    
    RCLCPP_INFO(logger, "成功创建zip包：%s", zip_path.c_str());
    return zip_path;
}

// 验证zip包是否有效
bool is_zip_valid(const std::string& zip_path, const rclcpp::Logger& logger) {
    if (!std::ifstream(zip_path)) {
        RCLCPP_ERROR(logger, "zip包不存在：%s", zip_path.c_str());
        return false;
    }
    
    std::string cmd = "unzip -t \"" + zip_path + "\" > /tmp/zip_verify.log 2>&1";
    int ret = system(cmd.c_str());
    
    if (ret != 0) {
        std::ifstream verify_log("/tmp/zip_verify.log");
        std::string verify_msg((std::istreambuf_iterator<char>(verify_log)), std::istreambuf_iterator<char>());
        RCLCPP_ERROR(logger, "zip包验证失败！信息：%s", verify_msg.c_str());
        return false;
    }
    
    return true;
}

// 检查文件是否存在且非空
bool is_file_valid(const std::string& file_path, const rclcpp::Logger& logger) {
    std::ifstream file(file_path);
    if (!file.good()) {
        RCLCPP_ERROR(logger, "文件不存在或无法访问：%s", file_path.c_str());
        return false;
    }
    
    file.seekg(0, std::ios::end);
    if (file.tellg() <= 0) {
        RCLCPP_ERROR(logger, "文件为空：%s", file_path.c_str());
        return false;
    }
    
    return true;
}

// trim helper
static inline std::string trim(const std::string &s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

// 读取简单的 key=value 配置文件（每行一个），忽略注释和空行
static bool read_simple_config(const std::string &path, std::map<std::string,std::string> &out, const rclcpp::Logger &logger) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        RCLCPP_ERROR(logger, "无法打开配置文件：%s", path.c_str());
        return false;
    }
    std::string line;
    while (std::getline(ifs, line)) {
        std::string s = trim(line);
        if (s.empty() || s[0] == '#') continue;
        auto pos = s.find('=');
        if (pos == std::string::npos) continue;
        std::string key = trim(s.substr(0,pos));
        std::string val = trim(s.substr(pos+1));
        if (!key.empty()) out[key] = val;
    }
    return true;
}

// 从环境变量读取阿里云密钥（核心改造函数）
bool get_aliyun_credentials(std::string& access_key, std::string& secret_key, const rclcpp::Logger& logger) {
    // 读取环境变量
    const char* ak_env = std::getenv("ALIBABA_CLOUD_ACCESS_KEY_ID");
    const char* sk_env = std::getenv("ALIBABA_CLOUD_ACCESS_KEY_SECRET");

    // 检查环境变量是否存在
    if (ak_env == nullptr || sk_env == nullptr) {
        RCLCPP_ERROR(logger, "❌ 未检测到阿里云密钥环境变量！");
        RCLCPP_ERROR(logger, "请先配置环境变量：");
        RCLCPP_ERROR(logger, "Linux/macOS:");
        RCLCPP_ERROR(logger, "  export ALIBABA_CLOUD_ACCESS_KEY_ID=你的AccessKey ID");
        RCLCPP_ERROR(logger, "  export ALIBABA_CLOUD_ACCESS_KEY_SECRET=你的AccessKey Secret");
        RCLCPP_ERROR(logger, "Windows:");
        RCLCPP_ERROR(logger, "  1. 右键「此电脑」→「属性」→「高级系统设置」→「环境变量」");
        RCLCPP_ERROR(logger, "  2. 新建系统变量：ALIBABA_CLOUD_ACCESS_KEY_ID（值为你的AccessKey ID）");
        RCLCPP_ERROR(logger, "  3. 新建系统变量：ALIBABA_CLOUD_ACCESS_KEY_SECRET（值为你的AccessKey Secret）");
        RCLCPP_ERROR(logger, "  4. 重启终端/系统后重试");
        return false;
    }

    // 检查环境变量是否为空
    access_key = std::string(ak_env);
    secret_key = std::string(sk_env);
    if (access_key.empty() || secret_key.empty()) {
        RCLCPP_ERROR(logger, "❌ 阿里云密钥环境变量值为空！请检查配置");
        return false;
    }

    RCLCPP_INFO(logger, "✅ 成功读取阿里云密钥环境变量");
    return true;
}

class OSSLogUploaderNode : public rclcpp::Node {
public:
    OSSLogUploaderNode() : Node("oss_log_upload_node") {
        auto logger = this->get_logger();

        // 1. 配置阿里云OSS基础信息（可根据需要改为从参数/配置文件读取）
        const std::string OSS_ENDPOINT = "oss-cn-beijing.aliyuncs.com"; 
        const std::string OSS_BUCKET = "test-oss-log-2025";              

        // 2. 从环境变量读取阿里云密钥（核心改造：替代硬编码）
        std::string OSS_ACCESS_KEY;
        std::string OSS_SECRET_KEY;
        if (!get_aliyun_credentials(OSS_ACCESS_KEY, OSS_SECRET_KEY, logger)) {
            throw std::runtime_error("Failed to get Aliyun credentials");
        }

        // 3. 创建OSS上传器
        oss_uploader_ = std::make_shared<OSSUploader>(OSS_ENDPOINT, OSS_BUCKET, OSS_ACCESS_KEY, OSS_SECRET_KEY);

        // 4. 配置日志参数：优先读取 config_file（key=value），否则从 ROS 参数读取
        this->declare_parameter<std::string>("config_file", "");
        this->declare_parameter<std::string>("device_model", "x5");
        this->declare_parameter<std::string>("device_id", "X001");
        this->declare_parameter<std::string>("business_type", "can");
        this->declare_parameter<std::string>("business_types", "can,zip,log,hotlog");
        this->declare_parameter<std::string>("custom_type", "info");
        this->declare_parameter<std::string>("custom_types", "info,info,info,info");
        this->declare_parameter<std::string>("file_suffix", "zip");
        this->declare_parameter<bool>("use_uuid", false);
        this->declare_parameter<std::string>("buffer_dir", "/tmp/oss_upload_buffer");
        this->declare_parameter<int>("upload_interval_seconds", 1800);  // 默认30分钟

        // read config file if provided
        std::string cfg_file = this->get_parameter("config_file").as_string();
        std::map<std::string,std::string> cfg_map;
        if (!cfg_file.empty()) {
            if (!read_simple_config(cfg_file, cfg_map, logger)) {
                RCLCPP_WARN(logger, "继续使用ROS参数或默认值，因为读取配置文件失败");
            }
        }

        auto param_or_cfg = [&](const std::string &key, const std::string &param_default)->std::string{
            if (cfg_map.find(key) != cfg_map.end()) return cfg_map[key];
            return this->get_parameter(key).as_string();
        };

        log_config_.device_model = param_or_cfg("device_model", "x5");
        log_config_.device_id = param_or_cfg("device_id", "X001");
        // single-default kept for backward compatibility
        log_config_.business_type = param_or_cfg("business_type", "can");
        log_config_.custom_type = param_or_cfg("custom_type", "info");
        log_config_.file_suffix = param_or_cfg("file_suffix", "zip");
        log_config_.upload_time = std::time(nullptr);
        log_config_.use_uuid = this->get_parameter("use_uuid").as_bool();
        buffer_dir_ = this->get_parameter("buffer_dir").as_string();
        std::string business_types_csv = this->get_parameter("business_types").as_string();
        std::string custom_types_csv = this->get_parameter("custom_types").as_string();

        // helper: split CSV into vector
        auto split_csv = [](const std::string &s)->std::vector<std::string>{
            std::vector<std::string> out;
            std::string cur;
            for (char c : s) {
                if (c == ',') {
                    if (!cur.empty()) out.push_back(cur);
                    cur.clear();
                } else if (!isspace((unsigned char)c)) {
                    cur.push_back(c);
                }
            }
            if (!cur.empty()) out.push_back(cur);
            return out;
        };

        business_types_ = split_csv(business_types_csv);
        if (business_types_.empty()) business_types_ = { log_config_.business_type };
        custom_types_ = split_csv(custom_types_csv);
        if (custom_types_.empty()) custom_types_ = { log_config_.custom_type };

        int upload_interval_seconds = this->get_parameter("upload_interval_seconds").as_int();
        upload_timer_ = this->create_wall_timer(
            std::chrono::seconds(upload_interval_seconds),
            std::bind(&OSSLogUploaderNode::perform_upload, this)
        );

        RCLCPP_INFO(logger, "OSS日志上传节点初始化完成，每 %d 秒上传一次", upload_interval_seconds);

        // 启动时立即执行一次上传
        perform_upload();
    }

private:
    void perform_upload() {
        auto logger = this->get_logger();
        RCLCPP_INFO(logger, "开始执行定时OSS日志上传");

        // 5. 在暂存缓冲区创建临时目录并写入待上传文件，上传成功后删除缓冲区内容
        try {
            namespace fs = std::filesystem;
            // For each business_type, create a per-type buffer and upload
            for (size_t i = 0; i < business_types_.size(); ++i) {
                std::string btype = business_types_[i];
                std::string ctype = (i < custom_types_.size()) ? custom_types_[i] : custom_types_.back();

                // unique suffix: deviceid + millis + random + business_type
                auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
                std::mt19937_64 rng((uint64_t)now_ms + (uint64_t)i);
                std::uniform_int_distribution<uint64_t> dist(0, UINT64_MAX);
                uint64_t rnd = dist(rng);
                std::string uniq = log_config_.device_id + "_" + std::to_string(now_ms) + "_" + std::to_string(rnd) + "_" + btype;

                fs::path buf_root(buffer_dir_);
                fs::path buf = buf_root / uniq;
                fs::create_directories(buf);

                // 创建TXT文件到缓冲区
                fs::path txt_path = buf / (log_config_.device_id + std::string("_log.txt"));
            {
                std::ofstream txt_file(txt_path, std::ios::binary);
                if (!txt_file.is_open()) {
                    RCLCPP_ERROR(logger, "无法创建TXT文件：%s", txt_path.c_str());
                    continue;
                }
                txt_file << "ROS2 OSS Upload Test: " << log_config_.device_model << "\n";
                txt_file << "测试时间: " << std::ctime(&log_config_.upload_time);
                txt_file.close();
            }

            if (!is_file_valid(txt_path.string(), logger)) {
                RCLCPP_ERROR(logger, "缓冲区TXT文件无效：%s", txt_path.c_str());
                fs::remove_all(buf);
                continue;
            }

                // zip 文件名放在缓冲区
                fs::path zip_path = buf / (log_config_.device_id + std::string("_") + std::to_string(now_ms) + std::string(".") + log_config_.file_suffix);
            // 删除旧文件（如果存在）
            if (fs::exists(zip_path)) fs::remove(zip_path);

                if (create_zip_file(txt_path.string(), zip_path.string(), logger).empty()) {
                    RCLCPP_ERROR(logger, "创建ZIP包失败，清理缓冲区");
                    fs::remove_all(buf);
                    continue;
                }

            if (!is_zip_valid(zip_path.string(), logger)) {
                RCLCPP_ERROR(logger, "ZIP包无效，清理缓冲区");
                fs::remove_all(buf);
                continue;
            }

                // set per-upload business_type and custom_type
                OSSLogConfig per_cfg = log_config_;
                per_cfg.business_type = btype;
                per_cfg.custom_type = ctype;

                // 上传缓冲区中的 zip 文件，增加重试策略（最多3次，指数退避）
                bool upload_result = false;
                const int max_attempts = 3;
                for (int attempt = 1; attempt <= max_attempts; ++attempt) {
                    upload_result = oss_uploader_->upload_log_file(zip_path.string(), per_cfg);
                    if (upload_result) {
                        RCLCPP_INFO(logger, "上传尝试 %d 成功", attempt);
                        break;
                    }
                    RCLCPP_WARN(logger, "上传尝试 %d 失败，将在下次重试前等待", attempt);
                    // 指数退避：1s, 2s, 4s
                    int sleep_ms = (1 << (attempt - 1)) * 1000;
                    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
                }
                if (upload_result) {
                    RCLCPP_INFO(logger, "✅ 日志上传成功：%s -> %s/%s", zip_path.c_str(), per_cfg.device_model.c_str(), per_cfg.business_type.c_str());
                    // 上传成功后删除缓冲区
                    try {
                        fs::remove_all(buf);
                        RCLCPP_INFO(logger, "已清理缓冲区：%s", buf.c_str());
                    } catch (const std::exception &e) {
                        RCLCPP_WARN(logger, "清理缓冲区时发生异常：%s", e.what());
                    }
                } else {
                    RCLCPP_ERROR(logger, "❌ 日志上传失败：%s，保留缓冲区以便重试：%s", zip_path.c_str(), buf.c_str());
                }
            }

        } catch (const std::exception &e) {
            RCLCPP_ERROR(logger, "运行时异常：%s", e.what());
        }
    }

    std::shared_ptr<OSSUploader> oss_uploader_;
    OSSLogConfig log_config_;
    std::string buffer_dir_;
    std::vector<std::string> business_types_;
    std::vector<std::string> custom_types_;
    rclcpp::TimerBase::SharedPtr upload_timer_;
};

int main(int argc, char**argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<OSSLogUploaderNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("oss_log_upload_node"), "节点初始化失败：%s", e.what());
        return -1;
    }
    rclcpp::shutdown();
    return 0;
}
