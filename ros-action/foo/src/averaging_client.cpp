#include <thread>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <foo/AveragingAction.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "test_averaging");

  actionlib::SimpleActionClient<foo::AveragingAction> ac("averaging");
  std::thread spinner([]() {
    ros::spin();
  });

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  foo::AveragingGoal goal;
  goal.samples = 2;
  ac.sendGoal(goal);

  if (ac.waitForResult(ros::Duration(30.0))) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }

  ros::shutdown();
  spinner.join();
  return 0;
}
