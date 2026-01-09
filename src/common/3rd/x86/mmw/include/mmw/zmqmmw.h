#pragma once

#include "../utils/vl_common.h"
#include "mmwabstract.h"
#include <atomic>
#include <set>
#include <zmq.hpp>
using namespace utils;
namespace mmw {

class ZmqPublisher : public PublisherAbstract {
public:
    ZmqPublisher(std::string localIp = "");
    virtual bool init(void) override final;
    virtual bool publish(const char* topic, size_t topicSize, const char* content, size_t contentSize) override final;

private:
    std::string localAddr_;
    zmq::context_t context_ { 1 };
    zmq::socket_t publisher_ { context_, ZMQ_PUB };
};

class ZmqSubscriber : public SubscriberAbstract {
public:
    ZmqSubscriber(std::set<std::string> connectAddrList);
    virtual bool init(void) override final;
    virtual bool subscribe(const std::string& topic) override final;
    virtual bool unSubscribe(const std::string& topic) override final;

protected:
    virtual void readyMessage(ReadCallBack& func) override final;

private:
    std::set<std::string> connectAddrSet_;
    zmq::context_t context_ { 1 };
    zmq::socket_t subscriber_ { context_, ZMQ_SUB };
};

}
