#pragma once
#include <string>
#include <vector>
#include <filesystem>
#include <chrono>
#include <random>
#include <sstream>

namespace postback {
namespace utils {

/**
 * @brief 生成 UUID 字符串
 * @return UUID 字符串
 */
std::string get_UUID_str();

/**
 * @brief 获取当前日期字符串，格式为 YYYYMMDD
 * @return 日期字符串
 */
std::string get_date_str();

/**
 * @brief 收集指定路径下最近修改的目录
 * @param path 目录路径
 * @return 最近修改的目录列表
 */
std::vector<std::string> collect_recent_directories(const std::string& path, std::string &err);

/**
 * @brief 将多个文件夹和文件压缩到 zip 文件
 * @param zip_path 输出的 zip 文件路径
 * @param directory_file_paths 要压缩的文件/文件夹路径列表
 * @param err 错误描述
 * @return 成功返回 true，失败返回 false
 */
bool add_directorys_files_to_zip(const std::string& zip_path, const std::vector<std::string>& directory_file_paths, std::string &err);

} // namespace utils
} // namespace postback
