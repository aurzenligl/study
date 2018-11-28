#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <dynrec/LoremConfig.h>

void callback(dynrec::LoremConfig &config, uint32_t level) {
  ROS_INFO("New values: [%d] - [%s]", config.int_param, config.str_param.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "node");

  dynamic_reconfigure::Server<dynrec::LoremConfig> server;
  dynamic_reconfigure::Server<dynrec::LoremConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::spin();

  return 0;
}
