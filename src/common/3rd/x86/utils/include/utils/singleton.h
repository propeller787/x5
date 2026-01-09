#pragma once

#include "noncopyable.h"
#include <memory>

namespace utils {

template <typename T>
class Singleton : public Noncopyable {
public:
    template <typename... Args>
    static T& getInstance(Args&&... args)
    {
        static T t(std::forward<Args>(args)...);
        return t;
    }
};

}