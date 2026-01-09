#ifndef POSTBACK_OSS_UPLOADER_HPP_
#define POSTBACK_OSS_UPLOADER_HPP_

#include <string>
#include <ctime>

namespace postback {  // 加入postback命名空间

// 日志上传配置结构体（对外暴露的核心接口，支持动态配置）
struct OSSLogConfig {
    std::string device_model;   // 设备型号（如x5）
    std::string device_id;      // 设备ID（如X001）
    std::string business_type;  // 文件业务类型（can/zip/log/hotlog）
    std::string custom_type;    // 自定义类型（如info，无则传空字符串）
    std::string file_suffix;    // 文件后缀（zip/log/txt等）
    std::time_t upload_time;    // 上传时间（默认0为当前时间，用于生成日期）
    bool use_uuid;              // 是否使用UUID替代时间戳（预留扩展，默认false）
    std::string uuid;           // 自定义UUID（use_uuid=true时生效）
};

class OSSUploader {
public:
    // 构造函数：传入阿里云OSS基础配置
    OSSUploader(const std::string& endpoint, 
                const std::string& bucket, 
                const std::string& access_key, 
                const std::string& secret_key);
    
    // 析构函数
    ~OSSUploader();
    
    // ========== 通用日志上传接口（对外核心接口） ==========
    bool upload_log_file(const std::string& local_file_path,  // 本地日志文件/压缩包路径
                         const OSSLogConfig& log_config);     // 日志配置（动态传入）

    // ========== 基础上传接口（保留，兼容原有逻辑） ==========
    bool upload_file(const std::string& local_file, const std::string& oss_path);

private:
    // HMAC-SHA1签名（兼容OpenSSL 1.1.x/3.0+）
    std::string hmac_sha1(const std::string& key, const std::string& data);
    
    // 生成RFC1123格式时间（OSS签名用）
    std::string get_rfc1123_time();

    // 生成毫米级时间戳（13位，如1731795893000）
    std::string get_millisecond_timestamp(std::time_t base_time = 0);

    // 生成符合规范的OSS日志路径+文件名
    std::string generate_oss_log_path(const OSSLogConfig& log_config);

    // 阿里云OSS基础配置
    std::string oss_endpoint_;
    std::string oss_bucket_;
    std::string oss_access_key_;
    std::string oss_secret_key_;
};

}  // namespace postback

#endif // POSTBACK_OSS_UPLOADER_HPP_