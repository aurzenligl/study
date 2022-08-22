#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

#include <foo/DoDishesAction.h>

using Client = actionlib::SimpleActionClient<foo::DoDishesAction>;

int main(int argc, char **argv) try {
  ros::init(argc, argv, "client");

  Client client("do_dishes", true);  // spins internally thanks to "true"
  if (!client.waitForServer(ros::Duration(5.0))) {
    throw std::runtime_error("action server is not available");
  }

  client.sendGoal([]() {
    foo::DoDishesGoal goal;
    goal.dishwasher_id = 42;
    return goal;
  }());
  client.waitForResult(ros::Duration(5.0));

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    std::cout << "The dishes are now clean" << '\n';
  } else {
    std::cout << "Dishes are not fully clean, state: " << client.getState().toString() << '\n';
  }

  return 0;
} catch (const std::exception &exc) {
  std::cout << "error: " << exc.what() << '\n';
  return 1;
}
