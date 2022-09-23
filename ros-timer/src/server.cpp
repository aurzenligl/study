#include <optional>
#include <thread>
#include <ros/ros.h>
#include <ros/timer.h>
#include <ros/timer_manager.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <sstream>
#include <mutex>
#include <foo/blocking_timer.h>

namespace foo {

struct Server {
  Server() {
    ros::NodeHandle nh;
    timer = nh.createTimer(ros::Duration(0.0), &Server::OnTick, this, false, true);
    srvs = {
      nh.advertiseService("/srv", &Server::OnSrv, this),
      nh.advertiseService("/stop", &Server::OnStop, this),
      nh.advertiseService("/selfown", &Server::OnSelfown, this)
    };
  }

  void Stop() {
    ROS_WARN_STREAM("xxx: stopping...");
    timer.stop();
    ROS_WARN_STREAM("xxx: ...stopped");
  }

  bool OnSrv(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp) {
    return true;
  }

  bool OnStop(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp) {
    Stop();
    return true;
  }

  bool OnSelfown(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp) {
    struct Context {
      Context(int idx, int counter) : idx(idx), counter(counter) {}

      void operator()() {
        ROS_WARN_STREAM("xxx: callback id: " << idx << ", count: " << counter);
        if (!--counter) {
          self.stop();
        }
      }

      int idx;
      int counter;
      ros::Timer self;
    };

    static int idx = 0;
    ros::NodeHandle nh;
    auto ctx = std::make_shared<Context>(idx++, 10);
    ros::TimerCallback cb = [ctx](auto &) { (*ctx)(); };
    ctx->self = nh.createTimer(ros::Duration(0.5), cb, false, false);
    ctx->self.start();

    return true;
  }

  void OnTick(const ros::TimerEvent &event) {
    ROS_WARN_STREAM("xxx: OnTick tic...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ROS_WARN_STREAM("xxx: OnTick ...toc");
  }

  BlockingTimer timer;
  std::vector<ros::ServiceServer> srvs;
};

}  // namespace foo

int main(int argc, char **argv) {
  ros::init(argc, argv, "server");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  foo::Server srv;

#if 1
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  srv.Stop();
#else
  ros::waitForShutdown();
#endif

  return 0;
}
