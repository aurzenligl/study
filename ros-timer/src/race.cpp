#include <memory>
#include <thread>

#include <ros/ros.h>

namespace foo {

struct Server {
  Server(const std::string &name) : name(std::make_unique<std::string>(name)) {
    timer = ros::NodeHandle().createTimer(ros::Duration(0.0), &Server::OnTick, this, false, true);
  }

  void OnTick(const ros::TimerEvent &event) {
    ROS_WARN_STREAM("foo::Server::OnTick tic...: " << *name);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ROS_WARN_STREAM("foo::Server::OnTick ...toc: " << *name);
  }

  std::unique_ptr<std::string> name;
  ros::Timer timer;
};

}  // namespace foo

int main(int argc, char **argv) {
  ros::init(argc, argv, "race");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_WARN_STREAM("foo::Server: constructing...");
  {
    foo::Server srv("the-name");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  ROS_WARN_STREAM("foo::Server: ...destroyed");

  return 0;
}
