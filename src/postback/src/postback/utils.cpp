#include "postback/utils.hpp"
#include <iostream>
#include <zip.h>
#include <fstream>

namespace postback {
namespace utils {

std::string get_UUID_str() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, 15);
    std::stringstream ss;
    ss << std::hex;
    for (int i = 0; i < 8; i++) ss << dis(gen);
    ss << "-";
    for (int i = 0; i < 4; i++) ss << dis(gen);
    ss << "-";
    for (int i = 0; i < 4; i++) ss << dis(gen);
    ss << "-";
    for (int i = 0; i < 4; i++) ss << dis(gen);
    ss << "-";
    for (int i = 0; i < 12; i++) ss << dis(gen);
    return ss.str();
}

std::string get_date_str() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now = *std::localtime(&time_t_now);
    char date_buffer[9];
    std::strftime(date_buffer, sizeof(date_buffer), "%Y%m%d", &tm_now);
    return std::string(date_buffer);
}

std::vector<std::string> collect_recent_directories(const std::string& path, std::string &err) {  
    namespace fs = std::filesystem;
    std::vector<std::string> directories;
    if (!fs::exists(path)) return directories;  
    
    auto now = std::chrono::system_clock::now();
    
    for (const auto& entry : fs::directory_iterator(path)) {
        if (!entry.is_directory()) continue;
        
        auto file_time = fs::last_write_time(entry.path());
        // 尝试将文件时间转换为系统时钟时间
        try {
            auto file_time_system = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
                file_time - fs::file_time_type::clock::now() + std::chrono::system_clock::now()
            );
            auto diff = std::chrono::duration_cast<std::chrono::seconds>(now - file_time_system).count();
            if (std::abs(diff) <= 60) {
                directories.emplace_back(entry.path().string());
            }
        } catch (const std::exception& e) {
            err = "时间转换错误: " + std::string(e.what());
        }
    }
    return directories;
}
bool add_directorys_files_to_zip(const std::string& zip_path, const std::vector<std::string>& directory_file_paths, std::string &err) {
    namespace fs = std::filesystem;
    
    int zip_error = 0;
    zip_t* zip = zip_open(zip_path.c_str(), ZIP_CREATE | ZIP_TRUNCATE, &zip_error);
    if (!zip) {
        err = "Failed to create zip file: " + zip_path;
        return false;
    }

    zip_int64_t index = 0;
    zip_source_t* source = nullptr;

    auto add_file_to_zip = [&](const fs::path& file_path, const fs::path& relative_path) {
        source = zip_source_file(zip, file_path.c_str(), 0, 0);
        if (!source) {
            err = "Failed to add file: " + file_path.string();
            return false;
        }
        
        std::string entry_name = relative_path.string();
        index = zip_file_add(zip, entry_name.c_str(), source, ZIP_FL_ENC_GUESS);
        if (index < 0) {
            err = "Failed to add file to zip: " + entry_name;
            zip_source_free(source);
            return false;
        }
        return true;
    };

    auto add_directory_to_zip = [&](const fs::path& dir_path) {
        if (!fs::exists(dir_path) || !fs::is_directory(dir_path)) {
            err = "Directory does not exist: " + dir_path.string();
            return true;
        }

        for (const auto& entry : fs::recursive_directory_iterator(dir_path)) {
            if (entry.is_regular_file()) {
                fs::path relative_path = fs::relative(entry.path(), dir_path.parent_path());
                if (!add_file_to_zip(entry.path(), relative_path)) {
                    return false;
                }
            }
        }
        return true;
    };

    for (const auto& path_str : directory_file_paths) {
        fs::path path(path_str);
        
        if (!fs::exists(path)) {
            err = "Path does not exist: " + path_str;
            zip_close(zip);
            return false;
        }

        if (fs::is_directory(path)) {
            if (!add_directory_to_zip(path)) {
                zip_close(zip);
                return false;
            }
        } else if (fs::is_regular_file(path)) {
            if (!add_file_to_zip(path, path.filename())) {
                zip_close(zip);
                return false;
            }
        } else {
            err = "Unsupported path type: " + path_str;
            zip_close(zip);
            return false;
        }
    }

    if (zip_close(zip) != 0) {
        err = "Failed to close zip file";
        return false;
    }

    return true;
}

} // namespace utils
} // namespace postback
