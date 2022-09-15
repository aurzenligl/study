#include <optional>
#include <thread>
#include <ros/ros.h>
#include <ros/timer_manager.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>
#include <sstream>
#include <mutex>
#include <foo/timeit.h>

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
    // access to: ros::Timer::impl_
    int32_t thandle = timer.impl_->timer_handle_;

    // rob to access: ros::TimerManager::timers_mutex_
    // rob to access: ros::TimerManager::TimerInfoPtr
    // rob to access: ros::TimerManager::findTimer
    auto &mgr = ros::TimerManager<ros::Time, ros::Duration, ros::TimerEvent>::global();
    auto info = (std::lock_guard(mgr.timers_mutex_), mgr.findTimer(thandle));
    uint64_t removal_id = (uint64_t)info.get();

    // derive to access: ros::CallbackQueue::IDInfoPtr
    // derive to access: ros::CallbackQueue::getIDInfo
    ros::CallbackQueue& queue = *ros::getGlobalCallbackQueue();
    ros::CallbackQueue::IDInfoPtr id_info = queue.getIDInfo(removal_id);

    ROS_WARN_STREAM("xxx: ids...: " << thandle << " " << removal_id << " " << id_info->id);
    (foo::Timeit("timer.stop()"), timer.stop());
    (foo::Timeit("calling_rw_mutex"), std::lock_guard(id_info->calling_rw_mutex));
    ROS_WARN_STREAM("xxx: ...stopped");
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
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ROS_WARN_STREAM("xxx: OnTick ...toc");
  }

  std::mutex mtx;
  ros::Timer quick_timer1;
  ros::Timer quick_timer2;
  ros::Timer quick_timer3;
  ros::Timer timer;
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
