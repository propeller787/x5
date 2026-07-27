#include "ota_agent/oss_manager.hpp"
#include <sys/stat.h>
#include <libgen.h>

using namespace AlibabaCloud::OSS;
OSSManager::OSSManager(const oss_auto_config& config): config_(config)
{
    InitializeSdk();
    ClientConfiguration conf;
    conf.signatureVersion = SignatureVersionType::V4;
    /* 从环境变量中获取访问凭证。运行本代码示例之前，请确保已设置环境变量OSS_ACCESS_KEY_ID和OSS_ACCESS_KEY_SECRET。*/
    // auto credentialsProvider = std::make_shared<SimpleCredentialsProvider>(config.accessKeyId, config.accessKeySecret);
    auto credentialsProvider = std::make_shared<EnvironmentVariableCredentialsProvider>();
    oss_client_ = std::unique_ptr<OssClient>(new OssClient(config.endpoint, credentialsProvider, conf));
    oss_client_->SetRegion(config.region);
}   

OSSManager::~OSSManager()
{
    ShutdownSdk();
}

bool OSSManager::upload_file(const std::string& local_file, const std::string& oss_path)
{
    if (local_file.empty() || oss_path.empty()) {
        return false;
    }

    struct stat st;
    if (stat(local_file.c_str(), &st) != 0) {
        return false;
    }

    if (!S_ISREG(st.st_mode)) {
        return false;
    }
    std::string bucket = config_.bucketName;
    UploadObjectRequest request(bucket, oss_path, local_file);
    if (process_callback_){
        TransferProgress progressCallback = { process_callback_ , nullptr };
        request.setTransferProgress(progressCallback);
    }
    /* 断点上传文件 */
    auto outcome = oss_client_->ResumableUploadObject(request);
    if (outcome.isSuccess()) {
        return true;
    }
    
    return false;
}


bool OSSManager::download_file(const std::string& oss_path, const std::string& local_file, std::string& err){
    
    std::string checkpointFilePath = local_file.substr(0, local_file.find_last_of('/'));
    struct stat st;
    if (stat(checkpointFilePath.c_str(), &st) != 0) {
        if (mkdir(checkpointFilePath.c_str(), 0755) != 0) {
            err = "Failed to create directory: " + checkpointFilePath;
            return false;
        }
    } else if (!S_ISDIR(st.st_mode)) {
        err = "Path exists but is not a directory: " + checkpointFilePath;
        return false;
    }
    DownloadObjectRequest request(config_.bucketName, oss_path, local_file, checkpointFilePath);
    if (process_callback_){
        TransferProgress progressCallback = { process_callback_ , nullptr };
        request.setTransferProgress(progressCallback);
    }
    auto outcome = oss_client_->ResumableDownloadObject(request);
    if (outcome.isSuccess()) {    
        return true;
    }
    std::ostringstream oss;
    oss << "GetObjectToFile fail"
        << ", code: " << outcome.error().Code()
        << ", message: " << outcome.error().Message()
        << ", requestId: " << outcome.error().RequestId();
    err = oss.str();
    return false;
}