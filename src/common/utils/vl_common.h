#pragma once

#include <string>

#define UNUSED(name) ((void)name)
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0])

namespace utils {

/**
 * @brief
 * @param pidDirPath [in]存放pid的路径,如果运行中出错会强制退出程序
 */
extern void instanceRun(const std::string& pidDirPath);
/**
 * @brief get the process path
 * @return success:name fault:empty
 */
extern std::string getProcessPath(void);
/**
 * @brief get the process name
 * @return success:name fault:empty
 */
extern std::string getProcessName(void);
/**
 * @brief Check if the folder exists
 * @param filePath[in] file path
 * @return true
 * @return false
 */
extern bool isFileExist(const std::string& filePath);
/**
 * @brief Read all data from the file into a string
 * @param path[in] file path
 * @param out[out] result
 * @return true
 * @return false
 */
extern bool readFileAllDataToString(const std::string& path, std::string& out);
/**
 * @brief write all data to path
 * @param in[in] content
 * @param path[in] file path
 * @param isApp[in] is append
 * @return true
 * @return false
 */
extern bool writeStringAllDataToFile(const std::string& in, const std::string& path, bool isApp = false);
/**
 * @brief 返回uuid
 * @return std::string
 */
extern std::string getUUID(void);
/**
 * @brief 随机生成字符串
 * @param len 随机生成字符串的长度
 * @return std::string
 */
extern std::string generateRandomString(size_t len);
/**
 * @brief hex打印
 * @param data[in] 需要打印的数据
 * @param size[in] 需要打印数据的字节个数
 */
extern void hexPrint(const char* data, size_t size);
/**
 * @brief hex打印
 * @param data[in] 需要打印的数据
 */
extern void hexPrint(const std::string& data);
/**
 * @brief 时间字符串转unix时间戳
 * @param timeStamp[in]时间字符串,格式为%d-%d-%d %d:%d:%d
 * @return time_t
 */
extern time_t strTimeToUnixTimestamp(const std::string& timeStamp);

}
