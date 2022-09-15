#include <optional>
#include <thread>
#include <ros/ros.h>
#include <ros/timer.h>
#include <ros/timer_manager.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>
#include <sstream>
#include <mutex>
#include <foo/timeit.h>

namespace foo {

class Timer {
 public:
  Timer() = default;
  Timer(const ros::Timer& rhs);
  Timer& operator=(const ros::Timer& other);

  // see ros::Timer for api documentation
  void start();
  void stop();
  bool hasPending();
  void setPeriod(const ros::Duration &period, bool reset = true);
  bool hasStarted() const;
  bool isValid() const;
  explicit operator bool() const;
  bool operator<(const Timer& rhs);
  bool operator==(const Timer& rhs);
  bool operator!=(const Timer& rhs);

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

namespace rob {

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

}  // namespace rob

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

    (foo::Timeit("timer.stop()"), timer.stop());

    if (id_info) {
      if (id_info->id != rob::CallingInThisThread(*ros::getGlobalCallbackQueue())) {
        (foo::Timeit("calling_rw_mutex"), std::lock_guard(id_info->calling_rw_mutex));
      }
    }
  }

  ros::Timer timer;
};

Timer::Timer(const ros::Timer& rhs) : impl_(std::make_shared<Impl>(rhs)) {}
Timer& Timer::operator=(const ros::Timer& rhs) { return *this = Timer(rhs); }

void Timer::start() { if (impl_) impl_->timer.start(); }
void Timer::stop() { if (impl_) impl_->stop(); }
bool Timer::hasPending() { return impl_ ? impl_->timer.hasPending() : false; }
void Timer::setPeriod(const ros::Duration &period, bool reset) { if (impl_) impl_->timer.setPeriod(period, reset); }
bool Timer::hasStarted() const { return impl_ ? impl_->timer.hasStarted() : false; }
bool Timer::isValid() const { return impl_ ? impl_->timer.isValid() : false; }
Timer::operator bool() const { return isValid(); }
bool Timer::operator<(const Timer& rhs) { return impl_ < rhs.impl_; }
bool Timer::operator==(const Timer& rhs) { return impl_ == rhs.impl_; }
bool Timer::operator!=(const Timer& rhs) { return impl_ != rhs.impl_; }

}  // namespace foo

struct Server {
  Server() {
    ros::NodeHandle nh;
    quick_timer1 = nh.createTimer(ros::Duration(0.0), &Server::OnQuickTick, this, false, true);
    quick_timer2 = nh.createTimer(ros::Duration(0.0), &Server::OnQuickTick, this, false, true);
    quick_timer3 = nh.createTimer(ros::Duration(0.0), &Server::OnQuickTick, this, false, true);
    timer = nh.createTimer(ros::Duration(0.0), &Server::OnTick, this, false, true);
    sub = nh.subscribe("/top", 1, &Server::OnTopic, this);
    srv = nh.advertiseService("/srv", &Server::OnSrv, this);
    stop = nh.advertiseService("/stop", &Server::OnStop, this);
  }

  void Stop() {
    ROS_WARN_STREAM("xxx: stopping...");
    timer.stop();
    timer = foo::Timer();
    ROS_WARN_STREAM("xxx: ...stopped");

    foo::Timer x = timer;
    foo::Timer y(timer);
    y = x;

    timer < timer;
    timer == timer;
    timer != timer;
    if (timer) {}
  }

  void OnTopic(std_msgs::Int32 x) {
    ROS_WARN_STREAM("xxx: OnTopic");
  }

  bool OnSrv(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp) {
    resp.message = "srv";
    resp.success = true;
    return true;
  }

  bool OnStop(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp) {
    Stop();
    resp.message = "stop";
    resp.success = true;
    return true;
  }

  void OnQuickTick(const ros::TimerEvent &event) {}

  void OnTick(const ros::TimerEvent &event) {
    ROS_WARN_STREAM("xxx: OnTick tic...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ROS_WARN_STREAM("xxx: OnTick ...toc");
  }

  std::mutex mtx;
  ros::Timer quick_timer1;
  ros::Timer quick_timer2;
  ros::Timer quick_timer3;
  foo::Timer timer;
  ros::Subscriber sub;
  ros::ServiceServer srv;
  ros::ServiceServer stop;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "server");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  Server srv;

#if 1
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  srv.Stop();
#else
  ros::waitForShutdown();
#endif

  return 0;
}
