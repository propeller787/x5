#include "zmqmmw.h"
#include "../3rd/x86/muduo/include/muduo/base/FileUtil.h"
#include "../utils/defer.h"
#include <assert.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <zmq.hpp>

using namespace muduo::FileUtil;
namespace mmw {
/**
 * @brief ZmqPublisher类的构造函数
 * @param localAddr[in] 发布者本地绑定的地址
 */
ZmqPublisher::ZmqPublisher(std::string localAddr)
    : localAddr_(std::move(localAddr))
{
    return;
}
/**
 * @brief ZmqPublisher对象的初始化动作,完成一些初始化功能
 * @return true 成功
 * @return false 失败
 */
bool ZmqPublisher::init(void)
{
    if (localAddr_.empty()) {
        return false;
    }
    // 如果通信方式是ipc的话且文件加没有被创建好,那先创建文件夹

    auto prefixPos = localAddr_.find("ipc://");
    if (prefixPos == 0) {
        auto pos = localAddr_.rfind("/");
        assert(!(pos == localAddr_.npos || pos <= prefixPos));
        auto prrfixLen = strlen("ipc://");
        auto dirPath = std::string(localAddr_.data() + prrfixLen, pos - prrfixLen);
        //assert(!(!isFileExist(dirPath) && !afl::file_util::createRecursionDir(dirPath.c_str())));
    }

    publisher_.bind(localAddr_);
    return true;
}
/**
 * @brief 根据topic发布一条消息
 * @param topic[in] 要发布消息的topic缓冲区地址
 * @param topicSize[in] 要发布消息的topic字节个数
 * @param content[in] 要发送消息的content缓冲区地址
 * @param contentSize[in] 要发送消息的content字节个数
 * @return true
 * @return false
 */
bool ZmqPublisher::publish(const char* topic, size_t topicSize, const char* content, size_t contentSize)
{
    if (topic == nullptr && topicSize != 0) {
        return false;
    }
    if (content == nullptr && contentSize != 0) {
        return false;
    }
    auto result = publisher_.send(zmq::const_buffer(topic, topicSize), zmq::send_flags::sndmore);
    if (!result.has_value() || result.value() != topicSize) {
        return false;
    }
    result = publisher_.send(zmq::const_buffer(content, contentSize), zmq::send_flags::none);
    if (!result.has_value() || result.value() != contentSize) {
        return false;
    }
    return true;
}
/**
 * @brief ZmqSubscriber类的构造函数
 * @param connectAddrList[in] 连接地址表
 */
ZmqSubscriber::ZmqSubscriber(std::set<std::string> connectAddrSet)
    : connectAddrSet_(std::move(connectAddrSet))
{
    return;
}
/**
 * @brief ZmqSubscriber对象的初始化动作,完成一些初始化功能
 * @return true 成功
 * @return false 失败
 */
bool ZmqSubscriber::init(void)
{
    for (const auto& addr : connectAddrSet_) {
        if (addr.empty()) {
            continue;
        }
        subscriber_.connect(addr);
    }
    return true;
}
/**
 * @brief 根据topic订阅一条消息
 * @param topic[in] 订阅消息的topic
 * @return true 成功
 * @return false 失败
 */
bool ZmqSubscriber::subscribe(const std::string& topic)
{
    if (topic.find(" ") != topic.npos) {
        return false;
    }
    // 开启 tcp 保活机制
    int tcpKeepAlive = 1; // context
    subscriber_.setsockopt(ZMQ_TCP_KEEPALIVE, &tcpKeepAlive, sizeof(tcpKeepAlive));

    // 网络连接空闲60s 即发送保活包
    int tcpKeepIdle = 60; // context
    subscriber_.setsockopt(ZMQ_TCP_KEEPALIVE_IDLE, &tcpKeepIdle, sizeof(tcpKeepIdle));
    subscriber_.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());
    return true;
}
/**
 * @brief 根据topic取消订阅一条消息
 * @param topic[in] 取消订阅消息的topic
 * @return true 成功
 * @return false 失败
 */
bool ZmqSubscriber::unSubscribe(const std::string& topic)
{
    if (topic.find(" ") != topic.npos) {
        return false;
    }
    subscriber_.setsockopt(ZMQ_UNSUBSCRIBE, topic.c_str(), topic.size());
    return true;
}
/**
 * @brief 此函数需要需要被重写,以多态的方式调用此函数,此函数需要被重写为非阻塞模式
 * 否则可能会在程序关闭时进行阻塞,且此函数重写时要调用回调函数
 * @param func[in] 回调函数,此函数格式为:void(const void*, size_t, const void*, size_t),参数依次为
 * 存储topic的首地址,topic的字节长度,存储content的首地址,存储content的字节长度
 */
void ZmqSubscriber::readyMessage(ReadCallBack& func)
{
    if (connectAddrSet_.empty()) {
        return;
    }
    uint64_t time = 0;
    zmq::message_t topic;
    zmq::message_t content;
    zmq::message_t* data = &topic;
    do {
        if (time == 1) {
            data = &content;
        }
        auto result = subscriber_.recv(*data);
        if (!result.has_value() || result.value() != data->size()) {
            continue;
        }
        ++time;
    } while (data->more());
    if (time == 2 && func) {
        func((char*)topic.data(), topic.size(), (char*)content.data(), content.size());
    }
}
}