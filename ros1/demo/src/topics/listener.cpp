#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);  // Force flush of the stdout buffer.

  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;

  auto t0 = std::chrono::steady_clock::now();
  int count = 0;

  boost::function<void(const std_msgs::String::ConstPtr &)> callback;
  callback = [&](const std_msgs::String::ConstPtr &) {
    ++count;
    auto t1 = std::chrono::steady_clock::now();
    const std::chrono::duration<double> d1 = t1 - t0;
    if (d1.count() >= 1) {
      double freq = count / d1.count();
      t0 = t1;
      count = 0;
      ROS_INFO("Frequency: %.3f [Hz]", freq);
    }

    // ROS_INFO("I heard: [%s]", msg.data.c_str());
  };

  ros::Subscriber sub = nh.subscribe("/chatter", 10, callback);
  ros::spin();
  return 0;
}
