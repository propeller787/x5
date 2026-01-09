#include "vl_common.h"
#include "defer.h"
// #include <base/FileUtil.h>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace utils {

std::string getProcessPath(void)
{
    char processDir[1000] = { 0 };

    if (readlink("/proc/self/exe", processDir, sizeof(processDir)) > 0) {
        auto end = strrchr(processDir, '/');
        if (end != nullptr) {
            *(end + 1) = '\0';
        }
    }
    return processDir;
}

void hexPrint(const char* data, size_t size)
{
    printf("{");
    for (size_t i = 0; i < size; i++) {
        printf("0x%02X,", (uint8_t)data[i]);
    }
    printf("\b}\n");
}

void hexPrint(const std::string& data)
{
    hexPrint(data.data(), data.size());
}

std::string getProcessName(void)
{
    char processDir[1000] = { 0 };
    char* end { nullptr };

    if (readlink("/proc/self/exe", processDir, sizeof(processDir)) > 0) {
        end = strrchr(processDir, '/');
        if (end != nullptr) {
            return end + 1;
        }
    }
    return "";
}

std::string generateRandomString(size_t len)
{
    std::default_random_engine engine(time(nullptr));
    std::uniform_int_distribution<int> u(32, 126);
    std::string randomStr;
    randomStr.resize(len);
    for (size_t i = 0; i < len; i++) {
        randomStr[i] = u(engine);
    }
    return randomStr;
}

bool isFileExist(const std::string& filePath)
{
    if (access(filePath.c_str(), F_OK)) {
        return false;
    }
    return true;
}

bool readFileAllDataToString(const std::string& path, std::string& out)
{
    std::fstream fs(path, std::ios::in);
    if (!fs) {
        return false;
    }
    std::stringstream buffer;
    buffer << fs.rdbuf();
    out = buffer.str();
    return true;
}

bool writeStringAllDataToFile(const std::string& in, const std::string& path, bool isApp)
{
    std::_Ios_Openmode module = std::ios::out;
    if (isApp) {
        module |= std::ios::app;
    } else {
        module |= std::ios::trunc;
    }
    std::fstream fs(path, module);
    if (!fs) {
        return false;
    }
    fs << in;
    fs.close();
    return true;
}

std::string getUUID(void)
{
    std::fstream file("/proc/sys/kernel/random/uuid", std::ios::in);
    std::string retv = "generate uuid failure!";
    std::getline(file, retv);
    return retv;
}

time_t strTimeToUnixTimestamp(const std::string& timeStamp)
{
    struct tm tm;
    memset(&tm, 0, sizeof(tm));

    sscanf(timeStamp.c_str(), "%d-%d-%d %d:%d:%d",
        &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
        &tm.tm_hour, &tm.tm_min, &tm.tm_sec);

    tm.tm_year -= 1900;
    tm.tm_mon--;

    return mktime(&tm);
}

}
