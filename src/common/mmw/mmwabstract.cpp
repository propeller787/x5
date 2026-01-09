#include "mmwabstract.h"
#include "muduo/net/EventLoopThread.h"
using namespace muduo::net;
namespace mmw {
/**
 * @brief 根据topic发布一条消息
 * @param topic[in] 要发布消息的topic
 * @param content[in] 消息的具体内容
 * @return true 成功
 * @return false 失败
 */
bool PublisherAbstract::publish(const std::string& topic, const std::string& content)
{
    return publish(topic.data(), topic.size(), content.data(), content.size());
}
/**
 * @brief SubscriberAbstract类的析构函数
 */
SubscriberAbstract::~SubscriberAbstract(void)
{
    workFlags_ = false;
}
/**
 * @brief 此函数内部开启一个线程来运行readyMessage函数,来接收订阅到的数据所以,
 * 此类的派生类需要重写readyMessage函数
 * @param func[in] 读取回调函数
 * @return true 启动成功
 * @return false 启动失败
 */
bool SubscriberAbstract::readStart(ReadCallBack func)
{
    std::call_once(onceFlag_, [=] {
        eventLoopthreadPtr_ = std::make_shared<muduo::net::EventLoopThread>();
        workFlags_ = true;
        eventLoopthreadPtr_->startLoop()->runInLoop([=] {
            auto func_ = std::move(func);
            while (workFlags_) {
                if (func_) {
                    readyMessage(func_);
                }
            }
        });
    });

    return true;
}

}