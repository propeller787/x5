#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>

#include <stdarg.h>

namespace sensor::utils {

class CommandExecutor {
public:
    explicit CommandExecutor(rclcpp::Logger logger)
        : logger_(logger)
    {
    }

    bool run(const std::string& cmd)
    {
        RCLCPP_INFO(logger_, "execute system command %s", cmd.c_str());

        int retval = system(cmd.c_str());
        if (-1 == retval) {
            RCLCPP_ERROR(logger_, "    system function error!");
            return false;
        }

        if (WIFEXITED(retval)) {
            if (0 == (retval = WEXITSTATUS(retval))) {
                RCLCPP_INFO(logger_, "    command exit successfully!");
                return true;
            } else {
                RCLCPP_ERROR(logger_, "    command exit fail, error code:%d", retval);
                return false;
            }
        }

        RCLCPP_ERROR(logger_, "    command runs error!");
        return false;
    }

private:
    rclcpp::Logger logger_;
};

static inline size_t stringFormatAppendImpl(std::string& dst, const char* format, va_list ap)
{
    size_t bufsz = 1024;
    int result = 0;
    va_list backup_ap; // see "man va_start"

    while (true) {
        std::vector<char> buffer(bufsz);
        char* bufaddr = &buffer[0];

        va_copy(backup_ap, ap);
        result = vsnprintf(bufaddr, bufsz, format, backup_ap);
        va_end(backup_ap);

        if (result < static_cast<int>(bufsz)) {
            result = (result <= 0) ? 0 : (dst.append(bufaddr, result), result);
            break;
        }

        bufsz = result + 1;
    }

    return result;
}

static inline std::string stringFormat(const char* format, ...)
{
    std::string result;
    va_list ap;
    va_start(ap, format);
    stringFormatAppendImpl(result, format, ap);
    va_end(ap);
    return result;
}

// Hexadecimal character and number correspondence table
const static char g_hexNum[256] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};
const static char g_hexStr[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', '\0' };

// The hexadecimal string is decoded into a normal display string
static inline std::string decodeHexString(const std::string& hexstr)
{
    std::string buf;
    size_t index = 0;
    while (hexstr[index] != '\0') {
        buf += static_cast<char>((g_hexNum[(int)hexstr[index]] << 4) | g_hexNum[(int)hexstr[index + 1]]);
        index += 2;
    }
    return buf;
}

// The string is encoded into a hexadecimal display string
static inline std::string encodeToHexString(const std::string& str)
{
    std::string hexbuf;
    int index = 0;
    for (auto c : str) {
        hexbuf += g_hexStr[static_cast<unsigned char>(c) >> 4];
        hexbuf += g_hexStr[static_cast<unsigned char>(c) & 0x0F];
        ++index;
    }
    return hexbuf;
}

namespace detail {

class Noncopyable {
public:
    Noncopyable(const Noncopyable&) = delete;
    void operator=(const Noncopyable&) = delete;

protected:
    Noncopyable() = default;
    ~Noncopyable() = default;
};

template <typename Callback>
struct SlotImpl;

template <typename Callback>
struct SignalImpl : public Noncopyable {
    using SlotList = std::vector<std::weak_ptr<SlotImpl<Callback>>>;

    SignalImpl()
        : slots_(std::make_shared<SlotList>())
    {
    }

    void copyOnWrite()
    {
        // mutex_.assertLocked();
        if (!slots_.unique()) {
            slots_ = std::make_shared<SlotList>(*slots_);
        }
        // assert(slots_.unique());
    }

    void clean()
    {
        std::lock_guard lock(mutex_);
        copyOnWrite();
        SlotList& list(*slots_);
        typename SlotList::iterator it(list.begin());
        while (it != list.end()) {
            if (it->expired()) {
                it = list.erase(it);
            } else {
                ++it;
            }
        }
    }

    std::mutex mutex_;
    std::shared_ptr<SlotList> slots_;
};

template <typename Callback>
struct SlotImpl : public Noncopyable {
    using Owner = SignalImpl<Callback>;

    SlotImpl(const std::shared_ptr<Owner>& owner, Callback&& cb)
        : owner_(owner)
        , cb_(cb)
        , tie_()
        , tied_(false)
    {
    }

    SlotImpl(const std::shared_ptr<Owner>& owner, Callback&& cb, const std::shared_ptr<void>& tie)
        : owner_(owner)
        , cb_(cb)
        , tie_(tie)
        , tied_(true)
    {
    }

    ~SlotImpl()
    {
        std::shared_ptr<Owner> owner(owner_.lock());
        if (owner) {
            owner->clean();
        }
    }

    std::weak_ptr<Owner> owner_; /* the signal entity */
    Callback cb_;
    std::weak_ptr<void> tie_; /* the member function owner entity */
    bool tied_;
};

}

/// This is the handle for a slot
///
/// The slot will remain connected to the signal for the life time of the
/// returned Slot object (and its copies).
using Slot = std::shared_ptr<void>;

template <typename Signature>
class Signal;

template <typename RET, typename... ARGS>
class Signal<RET(ARGS...)> : public detail::Noncopyable {
    using Callback = std::function<void(ARGS...)>;
    using SignalImpl = detail::SignalImpl<Callback>;
    using SlotImpl = detail::SlotImpl<Callback>;

public:
    Signal()
        : impl_(std::make_shared<SignalImpl>())
    {
    }
    ~Signal() = default;

    Slot connect(Callback&& func)
    {
        std::shared_ptr<SlotImpl> slotImpl = std::make_shared<SlotImpl>(impl_, std::forward<Callback>(func));
        add(slotImpl);
        return slotImpl;
    }

    Slot connect(Callback&& func, const std::shared_ptr<void>& tie)
    {
        std::shared_ptr<SlotImpl> slotImpl = std::make_shared<SlotImpl>(impl_, std::forward<Callback>(func), tie);
        add(slotImpl);
        return slotImpl;
    }

    size_t count()
    {
        size_t retval = 0;
        SignalImpl& impl(*impl_);
        {
            std::lock_guard lock(impl.mutex_);
            retval = impl.slots_->size();
        }
        return retval;
    }

    void call(ARGS&&... args)
    {
        SignalImpl& impl(*impl_);
        std::shared_ptr<typename SignalImpl::SlotList> slots;
        {
            std::lock_guard lock(impl.mutex_);
            slots = impl.slots_;
        }
        typename SignalImpl::SlotList& s(*slots);
        for (typename SignalImpl::SlotList::const_iterator it = s.begin(); it != s.end(); ++it) {
            std::shared_ptr<SlotImpl> slotImpl = it->lock();
            if (slotImpl) {
                std::shared_ptr<void> guard;
                if (slotImpl->tied_) {
                    guard = slotImpl->tie_.lock();
                    if (guard) {
                        slotImpl->cb_(args...);
                    }
                } else {
                    slotImpl->cb_(args...);
                }
            }
        }
    }

private:
    void add(const std::shared_ptr<SlotImpl>& slot)
    {
        SignalImpl& impl(*impl_);
        {
            std::lock_guard lock(impl.mutex_);
            impl.copyOnWrite();
            impl.slots_->push_back(slot);
        }
    }

    const std::shared_ptr<SignalImpl> impl_;
};

}
