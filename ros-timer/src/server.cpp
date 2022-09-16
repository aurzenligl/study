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
#include <foo/blocking_timer.h>

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
