#pragma once
#include "muduo/base/noncopyable.h"
#include "muduo/net/EventLoop.h"
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

namespace muduo::net {
class EventLoopThread;
};

namespace mmw {

class MMWAbstract : public muduo::noncopyable {
public:
    MMWAbstract(void) = default;
    virtual ~MMWAbstract(void) = default;
    virtual bool init(void) = 0;
};

class PublisherAbstract : public MMWAbstract {
public:
    virtual bool init(void) = 0;
    virtual bool publish(const char* topic, size_t topicSize, const char* content, size_t contentSize) = 0;
    bool publish(const std::string& topic, const std::string& content);
};

class SubscriberAbstract : public MMWAbstract {
public:
    using ReadCallBack = std::function<void(const char*, size_t, const char*, size_t)>;
    virtual ~SubscriberAbstract(void);
    bool readStart(ReadCallBack func);
    virtual bool init(void) = 0;
    virtual bool subscribe(const std::string& topic) = 0;
    virtual bool unSubscribe(const std::string& topic) = 0;

protected:
    virtual void readyMessage(ReadCallBack& func) = 0;

private:
    std::once_flag onceFlag_;
    std::atomic<bool> workFlags_;
    std::shared_ptr<muduo::net::EventLoopThread> eventLoopthreadPtr_ { nullptr };
};

}