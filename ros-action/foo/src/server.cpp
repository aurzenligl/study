#include <thread>

#include <actionlib/server/simple_action_server.h>
#include <foo/DoDishesAction.h>
#include <ros/ros.h>

using Server = actionlib::SimpleActionServer<foo::DoDishesAction>;

int main(int argc, char **argv) {
  ros::init(argc, argv, "server");
  ros::NodeHandle nh;
  Server server(
      nh, "do_dishes",
      [&server](const foo::DoDishesGoalConstPtr &goal) {
        std::cout << "server.isActive: " << server.isActive() << '\n';
        std::cout << "server.isNewGoalAvailable: " << server.isNewGoalAvailable() << '\n';
        std::cout << "server.isPreemptRequested: " << server.isPreemptRequested() << '\n';

        // Do lots of awesome groundbreaking robot stuff here
        std::cout << "Cleaning dishes..." << '\n';
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "...and cleaned." << '\n';
        server.setSucceeded();
      },
      false);
  server.start();
  ros::spin();
  return 0;
}
