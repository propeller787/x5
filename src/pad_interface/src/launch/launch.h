#pragma once
#include "monitor.h"
#include "program_controller.h"
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <mutex>
#include <unordered_map>
#include <utils/singleton.h>
#include <vector>
namespace service {
class Launch {
    typedef struct Program {
        std::string name; // 程序名称
        std::string path; // 可执行文件路径
        std::vector<std::string> args; // 启动参数
        int restart_limit; // 最大重启次数
    } Program;

    typedef struct Scene {
        std::string name; // 场景名称
        std::vector<Program> programs; // 程序列表
    } Scene;

private:
    Launch(void);
    ~Launch();
    void loadConfig(const std::string& config_path); // 加载场景配置
    bool getScene(const std::string& scene_name, Scene& scene); // 获取场景信息

public:
    static Launch* getInstance()
    {
        // 双检锁
        if (instance_ == nullptr) {
            std::lock_guard<std::mutex> lock(mtx_); // 加锁保证线程安全
            if (instance_ == nullptr) {
                instance_ = new Launch();
            }
        }
        return instance_;
    }

public:
    void startScene(std::string scene);
    void stopScene(std::string scene);

private:
    static Launch* instance_;
    static std::mutex mtx_;
    std::unordered_map<std::string, Scene> scenes_;
    Monitor monitor_;
    std::string scens_name_;
    std::vector<ProgramController> controllers_;
};
}