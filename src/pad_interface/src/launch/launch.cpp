#include "launch.h"
#include <cstdlib>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <utils/topic.h>
using std::placeholders::_1;
namespace service {
Launch* Launch::instance_ = nullptr;
std::mutex Launch::mtx_;
using json = nlohmann::json;
Launch::Launch(void)
{
    // node_ = std::make_shared<rclcpp::Node>("launch");
    loadConfig("etc/launch.json");
}
Launch::~Launch()
{
    monitor_.stopMonitoring();
    for (auto& controller : controllers_) {
        controller.stop();
    }
}
void Launch::loadConfig(const std::string& config_path)
{
    // 打开配置文件
    std::ifstream config_file(config_path);
    if (!config_file.is_open()) {
        std::cerr << "无法打开配置文件: " << config_path << std::endl;
        return;
    }

    // 解析 JSON 配置
    json j;
    config_file >> j;

    // 读取场景信息
    for (const auto& scene_json : j["scenes"]) {
        Scene scene;
        scene.name = scene_json["name"];

        for (const auto& program_json : scene_json["programs"]) {
            Program program;
            program.name = program_json["name"].get<std::string>();
            program.path = program_json["path"].get<std::string>();
            for (auto& arg : program_json["args"]) {
                program.args.push_back(arg.get<std::string>());
            }
            program.restart_limit = program_json["restart_limit"];
            scene.programs.push_back(program);
        }

        scenes_[scene.name] = scene;
    }
}

bool Launch::getScene(const std::string& scene_name, Scene& scene)
{
    auto it = scenes_.find(scene_name);
    if (it != scenes_.end()) {
        scene = it->second;
        return true;
    } else {
        std::cerr << "未找到场景: " << scene_name << std::endl;
        // 可以选择抛出一个异常或者返回一个默认构造的 Scene 对象
        return false;
    }
}

void Launch::startScene(std::string scene)
{
    if (scens_name_ != scene) {
        if (controllers_.size() > 0) {
            for (auto& controller : controllers_) {
                controller.stop();
            }
        }
    }
    Scene cene;
    if (!(getScene(scene, cene))) {
        return;
    }
    for (const auto& program : cene.programs) {
        controllers_.emplace_back(program.path, program.args, program.restart_limit);
        controllers_.back().start();
        monitor_.addProgram(&controllers_.back());
    }
    monitor_.startMonitoring();
    scens_name_ = scene;
}

void Launch::stopScene(std::string scene)
{
    if (scens_name_ != scene) {
        return;
    }
    for (auto& controller : controllers_) {
        controller.stop();
    }
}

}