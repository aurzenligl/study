#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);  // Force flush of the stdout buffer.

  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;

  size_t count = 1;
  ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter", 7);

  auto publish_message = [&]() {
    std_msgs::String msg;
    msg.data = "Hello World: " + std::to_string(count++);
    ROS_INFO("Publishing: '%s'", msg.data.c_str());
    pub.publish(std::move(msg));
  };

  while (ros::ok()) {
    publish_message();
  }

  return 0;
}
