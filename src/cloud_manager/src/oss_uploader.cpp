#include "cloud_manager/oss_uploader.hpp"
#include <rclcpp/rclcpp.hpp>
#include <curl/curl.h>
#include <openssl/hmac.h>
#include <openssl/sha.h>
#include <openssl/evp.h>
#include <openssl/err.h>
#include <string>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <stdio.h>
#include <stdexcept>
#include <cstddef>
#include <sys/stat.h>
#include <uuid/uuid.h>  // 预留UUID支持（需安装libuuid-dev）
#include <algorithm>
#include <cctype>

// 构造函数：初始化OSS配置和CURL
OSSUploader::OSSUploader(const std::string& endpoint, 
                         const std::string& bucket, 
                         const std::string& access_key, 
                         const std::string& secret_key)
    : oss_endpoint_(endpoint),
      oss_bucket_(bucket),
      oss_access_key_(access_key),
      oss_secret_key_(secret_key) {
    curl_global_init(CURL_GLOBAL_ALL);
}

// 析构函数：清理CURL资源
OSSUploader::~OSSUploader() {
    curl_global_cleanup();
}

// 生成毫米级时间戳（13位，秒+毫秒）
std::string OSSUploader::get_millisecond_timestamp(std::time_t base_time) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    
    // 使用提供的 base_time（秒级）作为时间基准，如果未提供则使用当前秒数
    // 毫秒部分取自当前时间的纳秒精度，保证13位毫秒级时间戳
    std::time_t use_time = (base_time == 0) ? ts.tv_sec : base_time;
    long ms = ts.tv_nsec / 1000000; // 纳秒转毫秒
    
    // 拼接为13位时间戳（补零确保毫秒是3位）
    std::stringstream ss;
    ss << use_time << std::setw(3) << std::setfill('0') << ms;
    return ss.str();
}

// 生成符合规范的OSS日志路径（核心：匹配logs/<设备型号>/<日期>/<业务类型>/<文件名>）
std::string OSSUploader::generate_oss_log_path(const OSSLogConfig& log_config) {
    // 1. 生成日期字符串（YYYYMMDD）
    std::time_t use_time = (log_config.upload_time == 0) ? std::time(nullptr) : log_config.upload_time;
    struct tm tm = *gmtime(&use_time);
    char date_buf[32] = {0};
    strftime(date_buf, sizeof(date_buf), "%Y%m%d", &tm);
    std::string date_str = date_buf;

    // helper: 小写化
    auto to_lower = [](std::string s){
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
        return s;
    };

    // helper: 去除路径危险字符
    auto sanitize = [](const std::string &s){
        std::string out;
        out.reserve(s.size());
        for (char c : s) {
            if (c == '/' || c == '\\') continue;
            out.push_back(c);
        }
        return out;
    };

    // 2. 生成文件名核心部分（设备ID + 时间戳/UUID）
    std::string filename_core = sanitize(log_config.device_id) + "_";
    if (log_config.use_uuid) {
        // 优先使用UUID（预留扩展）
        if (log_config.uuid.empty()) {
            uuid_t uuid_obj;
            uuid_generate(uuid_obj);
            char uuid_str[37] = {0};
            uuid_unparse(uuid_obj, uuid_str);
            filename_core += uuid_str;
        } else {
            filename_core += sanitize(log_config.uuid);
        }
    } else {
        // 默认使用毫米级时间戳
        filename_core += get_millisecond_timestamp(log_config.upload_time);
    }

    // 3. 规范自定义类型和文件后缀
    std::string custom_type = sanitize(log_config.custom_type);
    std::string file_suffix = log_config.file_suffix;
    // 去除可能的前导点
    if (!file_suffix.empty() && file_suffix[0] == '.') file_suffix = file_suffix.substr(1);

    std::string filename;
    if (!custom_type.empty()) {
        filename = filename_core + "_" + custom_type + "." + file_suffix; // deviceid_ts_type.suffix
    } else {
        filename = filename_core + "." + file_suffix; // deviceid_ts.suffix
    }

    // 4. 拼接完整OSS路径
    // 4. 业务类型小写化并做白名单处理（针对X5示例的常用类型）
    std::string device_model = to_lower(sanitize(log_config.device_model));
    std::string business_type = to_lower(sanitize(log_config.business_type));
    if (business_type.empty()) business_type = "other";

    // 对 X5 平台常见业务类型做规范化映射
    if (device_model == "x5") {
        if (business_type == "can" || business_type == "zip" || business_type == "log" || business_type == "hotlog") {
            // 保持原值
        } else {
            // 其他类型归类为 "other"
            business_type = "other";
        }
    }

    std::string oss_path = "logs/" + device_model + "/" + date_str + "/" + business_type + "/" + filename;
    return oss_path;
}

// 兼容所有OpenSSL版本的HMAC-SHA1签名
std::string OSSUploader::hmac_sha1(const std::string& key, const std::string& data) {
    unsigned char digest[EVP_MAX_MD_SIZE] = {0};
    size_t digest_len = 0;
    
    #if OPENSSL_VERSION_NUMBER >= 0x30000000L
    // OpenSSL 3.0+ 接口
    EVP_MAC* mac = EVP_MAC_fetch(nullptr, "HMAC", nullptr);
    if (!mac) {
        RCLCPP_ERROR(rclcpp::get_logger("OSS_UPLOADER"), "OpenSSL 3.0+ 获取HMAC失败");
        return "";
    }
    EVP_MAC_CTX* ctx = EVP_MAC_CTX_new(mac);
    OSSL_PARAM params[2] = {0};
    params[0] = OSSL_PARAM_construct_utf8_string("digest", (char*)"SHA1", 0);
    
    size_t key_len = key.length();
    if (EVP_MAC_init(ctx, (unsigned char*)key.c_str(), key_len, params) != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("OSS_UPLOADER"), "OpenSSL 3.0+ HMAC初始化失败");
        EVP_MAC_CTX_free(ctx);
        EVP_MAC_free(mac);
        return "";
    }
    
    size_t data_len = data.length();
    EVP_MAC_update(ctx, (unsigned char*)data.c_str(), data_len);
    EVP_MAC_final(ctx, digest, &digest_len, sizeof(digest));
    
    EVP_MAC_CTX_free(ctx);
    EVP_MAC_free(mac);
    #else
    // OpenSSL 1.1.x 接口
    HMAC_CTX* ctx = HMAC_CTX_new();
    int old_key_len = static_cast<int>(key.length());
    HMAC_Init_ex(ctx, key.c_str(), old_key_len, EVP_sha1(), nullptr);
    HMAC_Update(ctx, (unsigned char*)data.c_str(), data.length());
    
    unsigned int old_digest_len = 0;
    HMAC_Final(ctx, digest, &old_digest_len);
    digest_len = static_cast<size_t>(old_digest_len);
    
    HMAC_CTX_free(ctx);
    #endif

    // Base64编码
    static const char base64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string result;
    int i = 0, j = 0;
    unsigned char tmp[3] = {0};
    int digest_int_len = static_cast<int>(digest_len);

    while (i < digest_int_len) {
        tmp[j++] = digest[i++];
        if (j == 3) {
            result += base64_table[(tmp[0] & 0xfc) >> 2];
            result += base64_table[((tmp[0] & 0x03) << 4) | ((tmp[1] & 0xf0) >> 4)];
            result += base64_table[((tmp[1] & 0x0f) << 2) | ((tmp[2] & 0xc0) >> 6)];
            result += base64_table[tmp[2] & 0x3f];
            j = 0;
        }
    }

    if (j > 0) {
        for (int k = j; k < 3; k++) tmp[k] = 0;
        result += base64_table[(tmp[0] & 0xfc) >> 2];
        result += base64_table[((tmp[0] & 0x03) << 4) | ((tmp[1] & 0xf0) >> 4)];
        if (j == 1) {
            result += '=';
        } else {
            result += base64_table[((tmp[1] & 0x0f) << 2) | ((tmp[2] & 0xc0) >> 6)];
        }
        result += '=';
    }

    return result;
}

// 生成RFC1123格式时间（OSS签名必备）
std::string OSSUploader::get_rfc1123_time() {
    time_t now = time(nullptr);
    struct tm tm = *gmtime(&now);
    char buf[128] = {0};
    strftime(buf, sizeof(buf), "%a, %d %b %Y %H:%M:%S GMT", &tm);
    return std::string(buf);
}

// 基础文件上传接口（底层实现，兼容原有逻辑）
bool OSSUploader::upload_file(const std::string& local_file, const std::string& oss_path) {
    // 1. 校验本地文件是否存在且可读
    struct stat file_stat;
    if (stat(local_file.c_str(), &file_stat) != 0 || !(file_stat.st_mode & S_IRUSR)) {
        RCLCPP_ERROR(rclcpp::get_logger("OSS_UPLOADER"), "本地文件不存在或无读权限：%s", local_file.c_str());
        return false;
    }

    FILE* fp = fopen(local_file.c_str(), "rb");
    if (!fp) {
        RCLCPP_ERROR(rclcpp::get_logger("OSS_UPLOADER"), "本地文件打开失败：%s", local_file.c_str());
        return false;
    }

    // 2. 初始化CURL
    CURL* curl = curl_easy_init();
    if (!curl) {
        RCLCPP_ERROR(rclcpp::get_logger("OSS_UPLOADER"), "CURL初始化失败");
        fclose(fp);
        return false;
    }

    // 3. 构造OSS签名
    std::string date = get_rfc1123_time();
    
    // 原错误代码
// std::string string_to_sign = "PUT\n\n\n" + date + "\n/" + oss_bucket_ + "/" + oss_path;

    // 修复后：补充Content-Type为application/octet-stream（和请求头一致）
    std::string content_type = "application/octet-stream";
    std::string string_to_sign = "PUT\n\n" + content_type + "\n" + date + "\n/" + oss_bucket_ + "/" + oss_path;

    std::string signature = hmac_sha1(oss_secret_key_, string_to_sign);
    
    if (signature.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("OSS_UPLOADER"), "HMAC-SHA1签名失败");
        fclose(fp);
        curl_easy_cleanup(curl);
        return false;
    }

    // 4. 构造请求头
    std::string auth_header = "Authorization: OSS " + oss_access_key_ + ":" + signature;
    std::string date_header = "Date: " + date;
    std::string host_header = "Host: " + oss_bucket_ + "." + oss_endpoint_;
    
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, auth_header.c_str());
    headers = curl_slist_append(headers, date_header.c_str());
    headers = curl_slist_append(headers, "Content-Type: application/octet-stream");
    headers = curl_slist_append(headers, host_header.c_str());

    // 5. 构造OSS完整URL
    std::string oss_url = "https://" + oss_bucket_ + "." + oss_endpoint_ + "/" + oss_path;

    // 6. 设置CURL上传参数
    curl_easy_setopt(curl, CURLOPT_URL, oss_url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_PUT, 1L);
    curl_easy_setopt(curl, CURLOPT_READDATA, fp);
    curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE, (curl_off_t)file_stat.st_size); // 传入文件大小
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L); // 测试阶段关闭SSL校验，生产环境开启
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);

    // 7. 执行上传
    CURLcode res = curl_easy_perform(curl);

        // 新增：获取HTTP状态码
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    // bool success = (res == CURLE_OK);

    // if (success) {
    //     RCLCPP_INFO(rclcpp::get_logger("OSS_UPLOADER"), "文件上传成功：%s -> %s", local_file.c_str(), oss_path.c_str());
    // } else {
    //     RCLCPP_ERROR(rclcpp::get_logger("OSS_UPLOADER"), "文件上传失败：%s，错误：%s", local_file.c_str(), curl_easy_strerror(res));
    // }

    
    // 修正成功判定：res==CURLE_OK 且 http_code==200
    bool success = (res == CURLE_OK) && (http_code == 200);

    if (success) {
        RCLCPP_INFO(rclcpp::get_logger("OSS_UPLOADER"), "文件上传成功：%s -> %s", local_file.c_str(), oss_path.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("OSS_UPLOADER"), "文件上传失败：%s，错误：%s，HTTP状态码：%ld", local_file.c_str(), curl_easy_strerror(res), http_code);
    }


    // 8. 清理资源
    fclose(fp);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    return success;
}

// 通用日志上传接口（对外核心接口，封装规范路径生成）
bool OSSUploader::upload_log_file(const std::string& local_file_path, const OSSLogConfig& log_config) {
    // 1. 参数合法性校验（对外接口的基础防护）
    if (local_file_path.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("OSS_UPLOADER"), "本地文件路径为空");
        return false;
    }
    if (log_config.device_model.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("OSS_UPLOADER"), "设备型号未配置");
        return false;
    }
    if (log_config.device_id.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("OSS_UPLOADER"), "设备ID未配置");
        return false;
    }
    if (log_config.business_type.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("OSS_UPLOADER"), "业务类型未配置");
        return false;
    }
    if (log_config.file_suffix.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("OSS_UPLOADER"), "文件后缀未配置");
        return false;
    }

    // 2. 生成符合规范的OSS路径
    std::string oss_log_path = generate_oss_log_path(log_config);
    RCLCPP_INFO(rclcpp::get_logger("OSS_UPLOADER"), "生成规范OSS路径：%s", oss_log_path.c_str());

    // 3. 调用基础上传接口完成上传
    return upload_file(local_file_path, oss_log_path);
}

