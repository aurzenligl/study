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
  struct DetachableCtx {
    explicit DetachableCtx(int idx) : idx(idx) {}

    void operator()(const ros::TimerEvent &) {
      ROS_WARN_STREAM("xxx: DetachableCtx tic... " << idx);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      ROS_WARN_STREAM("xxx: DetachableCtx ...toc " << idx);
    }

    int idx;
  };

  struct SelfownCtx {
    SelfownCtx(int idx, int counter) : idx(idx), counter(counter) {}

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

  Server() {
    ros::NodeHandle nh;
    timer = nh.createTimer(ros::Duration(0.0), &Server::OnTick, this, false, true);
    srvs = {
      nh.advertiseService("/srv", &Server::OnSrv, this),
      nh.advertiseService("/stop", &Server::OnStop, this),
      nh.advertiseService("/detachable", &Server::OnDetachable, this),
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

  bool OnDetachable(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp) {
    static int idx = 0;
    detachable_ctx = boost::make_shared<DetachableCtx>(idx++);
    detachable_timer = ros::NodeHandle().createTimer(ros::Duration(0.0), &DetachableCtx::operator(), detachable_ctx);
    return true;
  }

  bool OnSelfown(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp) {
    static int idx = 0;
    auto ctx = std::make_shared<SelfownCtx>(idx++, 10);
    ros::TimerCallback cb = [ctx](auto &) { (*ctx)(); };
    ctx->self = ros::NodeHandle().createTimer(ros::Duration(0.5), cb, false, false);
    ctx->self.start();
    return true;
  }

  void OnTick(const ros::TimerEvent &event) {
    ROS_WARN_STREAM("xxx: OnTick tic...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ROS_WARN_STREAM("xxx: OnTick ...toc");
  }

  BlockingTimer timer;
  boost::shared_ptr<DetachableCtx> detachable_ctx;
  ros::Timer detachable_timer;
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
#endif

  ros::waitForShutdown();

  return 0;
}
