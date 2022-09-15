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
  void blocking_stop();
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

struct Timer::Impl {
  Impl(const ros::Timer &timer) : timer(timer) {}
  ~Impl() { blocking_stop(); }

  void stop() {
    // access to: ros::Timer::impl_
    if (timer.impl_) {
      int32_t thandle = timer.impl_->timer_handle_;
      ROS_WARN_STREAM("aaa: " << thandle);

      // rob to access: ros::TimerManager::timers_mutex_
      // rob to access: ros::TimerManager::TimerInfoPtr
      // rob to access: ros::TimerManager::findTimer
      auto &mgr = ros::TimerManager<ros::Time, ros::Duration, ros::TimerEvent>::global();
      if (auto info = (std::lock_guard(mgr.timers_mutex_), mgr.findTimer(thandle))) {
        uint64_t removal_id = (uint64_t)info.get();
        ROS_WARN_STREAM("aaa: " << removal_id);

        // derive to access: ros::CallbackQueue::IDInfoPtr
        // derive to access: ros::CallbackQueue::getIDInfo
        ros::CallbackQueue& queue = *ros::getGlobalCallbackQueue();
        id_info = queue.getIDInfo(removal_id);
      }
    }

    (foo::Timeit("timer.stop()"), timer.stop());
  }

  void blocking_stop() {
    ROS_WARN_STREAM("xxx: stopping...");
    stop();
    if (id_info) {
      (foo::Timeit("calling_rw_mutex"), std::lock_guard(id_info->calling_rw_mutex));
    }
    ROS_WARN_STREAM("xxx: ...stopped");
  }

  ros::CallbackQueue::IDInfoPtr id_info;
  ros::Timer timer;
};

Timer::Timer(const ros::Timer& rhs) : impl_(std::make_shared<Impl>(rhs)) {}
Timer& Timer::operator=(const ros::Timer& rhs) { return *this = Timer(rhs); }

void Timer::start() { if (impl_) impl_->timer.start(); }
void Timer::stop() { if (impl_) impl_->stop(); }
void Timer::blocking_stop() { if (impl_) impl_->blocking_stop(); }
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
    ROS_WARN_STREAM("xxx: running: " << timer.hasStarted());
    timer.stop();
    timer = foo::Timer();

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
