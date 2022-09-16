#include <foo/blocking_timer.h>

#include <mutex>

#include <ros/callback_queue.h>
#include <ros/timer_manager.h>

#include <foo/timeit.h>

namespace foo::rob {

// robbing ros::Timer::Impl::timer_handle_: private variable from private class
std::optional<int32_t> Handle(ros::Timer &t);
template <typename Ret, Ret ros::Timer::*ImplDp>
struct HandleAccess {
  friend std::optional<int32_t> Handle(ros::Timer &t) {
    return std::invoke(ImplDp, t) ? std::make_optional(std::invoke(ImplDp, t)->timer_handle_) : std::nullopt;
  }
};
template struct HandleAccess<decltype(ros::Timer::impl_), &ros::Timer::impl_>;

// robbing timer's removal_id: calling private method which returns a private type, protected by private mutex
using TimerManager = ros::TimerManager<ros::Time, ros::Duration, ros::TimerEvent>;
uint64_t RemovalId(TimerManager &m, int32_t thandle);
template <typename Info, Info (TimerManager::*FindTimerFp)(int32_t), boost::mutex TimerManager::*MtxDp>
struct RemovalIdAccess {
  friend uint64_t RemovalId(TimerManager &m, int32_t thandle) {
    std::lock_guard lock(std::invoke(MtxDp, m));
    return (uint64_t)(std::invoke(FindTimerFp, m, thandle).get());
  }
};
template struct RemovalIdAccess<TimerManager::TimerInfoPtr, &TimerManager::findTimer, &TimerManager::timers_mutex_>;

// accessing protected CallbackQueue::getIDInfo: inheriting from class and exposing via using and friend
struct CallbackQueue : public ros::CallbackQueue {
  using ros::CallbackQueue::IDInfoPtr;
  friend IDInfoPtr IdInfo(ros::CallbackQueue &q, uint64_t removal_id) {
    return std::invoke(&CallbackQueue::getIDInfo, q, removal_id);
  }
  friend uint64_t CallingInThisThread(ros::CallbackQueue &q) {
    std::invoke(&CallbackQueue::setupTLS, q);
    return std::invoke(&CallbackQueue::tls_, q)->calling_in_this_thread;
  }
};
CallbackQueue::IDInfoPtr IdInfo(ros::CallbackQueue &q, uint64_t removal_id);
uint64_t CallingInThisThread(ros::CallbackQueue &q);

}  // namespace foo::rob

namespace foo {

struct Timer::Impl {
  Impl(const ros::Timer &timer) : timer(timer) {}
  ~Impl() { stop(); }

  void stop() {
    if (!timer.hasStarted()) {
      return;
    }

    rob::CallbackQueue::IDInfoPtr id_info;
    if (auto thandle = rob::Handle(timer)) {
      if (auto removal_id = rob::RemovalId(rob::TimerManager::global(), *thandle)) {
        id_info = rob::IdInfo(*ros::getGlobalCallbackQueue(), removal_id);
      }
    }

    TIMEIT(timer.stop());

    if (id_info) {
      if (id_info->id != rob::CallingInThisThread(*ros::getGlobalCallbackQueue())) {
        TIMEIT(std::lock_guard(id_info->calling_rw_mutex));
      }
    }
  }

  ros::Timer timer;
};

Timer::Timer(const ros::Timer &rhs) : impl_(std::make_shared<Impl>(rhs)) {}

Timer &Timer::operator=(const ros::Timer &rhs) {
  return *this = Timer(rhs);
}

void Timer::start() {
  if (impl_) impl_->timer.start();
}

void Timer::stop() {
  if (impl_) impl_->stop();
}

bool Timer::hasPending() {
  return impl_ ? impl_->timer.hasPending() : false;
}

void Timer::setPeriod(const ros::Duration &period, bool reset) {
  if (impl_) impl_->timer.setPeriod(period, reset);
}

bool Timer::hasStarted() const {
  return impl_ ? impl_->timer.hasStarted() : false;
}

bool Timer::isValid() const {
  return impl_ ? impl_->timer.isValid() : false;
}

Timer::operator bool() const {
  return isValid();
}

bool Timer::operator<(const Timer &rhs) {
  return impl_ < rhs.impl_;
}

bool Timer::operator==(const Timer &rhs) {
  return impl_ == rhs.impl_;
}

bool Timer::operator!=(const Timer &rhs) {
  return impl_ != rhs.impl_;
}

}  // namespace foo
