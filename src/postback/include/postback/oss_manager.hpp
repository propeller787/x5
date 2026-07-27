#pragma once
#include <alibabacloud/oss/OssClient.h>
#include <fstream>
#include <memory>

class OSSManager
{  
    public:
        struct oss_auto_config {
            /** 测试用例配置 */
            const std::string endpoint = "https://cn-beijing.oss.aliyuncs.com";
            const std::string region = "cn-beijing";
            std::string bucketName = "zqkj-iot-test";
        };
        OSSManager(const oss_auto_config& config);
        ~OSSManager();

        bool upload_file(const std::string& local_file, const std::string& oss_path, std::string& err);

        void set_receive_handler(std::function<void(size_t increment, int64_t transfered, int64_t total, void* userData)> cb){
            process_callback_ = std::move(cb);
        }

    private:
        oss_auto_config config_;
        std::unique_ptr<AlibabaCloud::OSS::OssClient> oss_client_;
        std::function<void(size_t increment, int64_t transfered, int64_t total, void* userData)> process_callback_;
};