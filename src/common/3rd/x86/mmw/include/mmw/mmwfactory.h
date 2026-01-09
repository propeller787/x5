#pragma once

#include "zmqmmw.h"
namespace mmw {

enum class MMWSelector : uint32_t {
    ZMQ = 0
};

class MMWFactory {
public:
    template <typename... Args>
    std::shared_ptr<PublisherAbstract> getPublisher(MMWSelector mmwSelector, Args&&... args)
    {
        switch (mmwSelector) {
        case MMWSelector::ZMQ:
            return std::make_shared<ZmqPublisher>(std::forward<Args>(args)...);
        default:
            return nullptr;
        }
    }

    template <typename... Args>
    std::shared_ptr<SubscriberAbstract> getSubscriber(MMWSelector mmwSelector, Args&&... args)
    {
        switch (mmwSelector) {
        case MMWSelector::ZMQ:
            return std::make_shared<ZmqSubscriber>(std::forward<Args>(args)...);
        default:
            return nullptr;
        }
    }
};

}