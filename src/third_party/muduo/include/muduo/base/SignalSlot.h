#ifndef MUDUO_BASE_SIGNALSLOT_H
#define MUDUO_BASE_SIGNALSLOT_H

#include "noncopyable.h"
#include "Mutex.h"

#include <memory>
#include <vector>
#include <functional>

namespace muduo {

namespace detail {

template<typename Callback>
struct SlotImpl;

template<typename Callback>
struct SignalImpl: public noncopyable {
    using SlotList = std::vector<std::weak_ptr<SlotImpl<Callback>>>;

    SignalImpl() :
            slots_(std::make_shared<SlotList>()) {
    }

    void copyOnWrite() {
        mutex_.assertLocked();
        if (!slots_.unique()) {
            slots_ = std::make_shared<SlotList>(*slots_);
        }
        assert(slots_.unique());
    }

    void clean() {
        MutexLockGuard lock(mutex_);
        copyOnWrite();
        SlotList &list(*slots_);
        typename SlotList::iterator it(list.begin());
        while (it != list.end()) {
            if (it->expired()) {
                it = list.erase(it);
            } else {
                ++it;
            }
        }
    }

    MutexLock mutex_;
    std::shared_ptr<SlotList> slots_;
};

template<typename Callback>
struct SlotImpl: public noncopyable {
    using Owner = SignalImpl<Callback>;

    SlotImpl(const std::shared_ptr<Owner> &owner, Callback &&cb) :
            owner_(owner), cb_(cb), tie_(), tied_(false) {
    }

    SlotImpl(const std::shared_ptr<Owner> &owner, Callback &&cb, const std::shared_ptr<void> &tie) :
            owner_(owner), cb_(cb), tie_(tie), tied_(true) {
    }

    ~SlotImpl() {
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

template<typename Signature>
class Signal;

template<typename RET, typename ... ARGS>
class Signal<RET(ARGS...)> : public noncopyable {
    using Callback = std::function<void(ARGS...)>;
    using SignalImpl = detail::SignalImpl<Callback>;
    using SlotImpl = detail::SlotImpl<Callback>;

public:
    Signal() :
            impl_(std::make_shared<SignalImpl>()) {
    }
    ~Signal() = default;

    Slot connect(Callback &&func) {
        std::shared_ptr<SlotImpl> slotImpl = std::make_shared<SlotImpl>(impl_, std::forward<Callback>(func));
        add(slotImpl);
        return slotImpl;
    }

    Slot connect(Callback &&func, const std::shared_ptr<void> &tie) {
        std::shared_ptr<SlotImpl> slotImpl = std::make_shared<SlotImpl>(impl_, std::forward<Callback>(func), tie);
        add(slotImpl);
        return slotImpl;
    }

    size_t count() {
        size_t retval = 0;
        SignalImpl &impl(*impl_);
        {
            MutexLockGuard lock(impl.mutex_);
            retval = impl.slots_->size();
        }
        return retval;
    }

    void call(ARGS &&... args) {
        SignalImpl &impl(*impl_);
        std::shared_ptr<typename SignalImpl::SlotList> slots;
        {
            MutexLockGuard lock(impl.mutex_);
            slots = impl.slots_;
        }
        typename SignalImpl::SlotList &s(*slots);
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
    void add(const std::shared_ptr<SlotImpl> &slot) {
        SignalImpl &impl(*impl_);
        {
            MutexLockGuard lock(impl.mutex_);
            impl.copyOnWrite();
            impl.slots_->push_back(slot);
        }
    }

    const std::shared_ptr<SignalImpl> impl_;
};

} // namespace muduo

#endif // MUDUO_BASE_SIGNALSLOT_H

