#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <foo/FibonacciAction.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "test_fibonacci");
  actionlib::SimpleActionClient<foo::FibonacciAction> ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  foo::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal, {}, {}, [](const foo::FibonacciFeedbackConstPtr &f) {
    std::cout << "Feedback: " << f->sequence.size() << '\n';
  });

  if (ac.waitForResult(ros::Duration(30.0))) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }
  return 0;
}
