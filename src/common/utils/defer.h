#pragma once

#include <functional>

class defer_t final {
public:
    template <typename F>
    explicit defer_t(F&& f) noexcept
        : f_ {
            std::forward<F>(f)
        }
    {
    }

    ~defer_t()
    {

        if (f_) {

            f_();
        }
    }

    defer_t(const defer_t&) = delete;
    defer_t& operator=(const defer_t&) = delete;

    defer_t(defer_t&& rhs) noexcept
        : f_ {
            std::move(rhs.f_)
        }
    {
    }

    defer_t& operator=(defer_t&& rhs) noexcept
    {

        f_ = std::move(rhs.f_);
        return *this;
    }

private:
    std::function<void()> f_;
};

class defer_maker final {

public:
    template <typename F>
    defer_t operator+(F&& f)
    {

        return defer_t {
            std::forward<F>(f)
        };
    }
};

#define DEFER_CONCAT_NAME(l, r) l##r
#define DEFER_CREATE_NAME(l, r) DEFER_CONCAT_NAME(l, r)
#define defer auto DEFER_CREATE_NAME(defer_, __COUNTER__) = defer_maker {} +
#define defer_scope defer[&]