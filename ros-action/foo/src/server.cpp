#include <thread>

#include <actionlib/server/simple_action_server.h>
#include <foo/DoDishesAction.h>
#include <ros/ros.h>

using Server = actionlib::SimpleActionServer<foo::DoDishesAction>;

int main(int argc, char **argv) {
  ros::init(argc, argv, "server");

  ros::Timer timer;
  Server server("do_dishes", false);
  server.registerGoalCallback([&server, &timer]() {
    std::cout << "Goal callback... "
              << "active:" << server.isActive() << ' '
              << "new_goal:" << server.isNewGoalAvailable() << ' '
              << "preempt:" << server.isPreemptRequested() << '\n';

    auto goal = server.acceptNewGoal();
    std::cout << "Cleaning dishes... " << goal->dishwasher_id << '\n';
    timer = ros::NodeHandle().createTimer(ros::Duration(2.0), [&server](const ros::TimerEvent&) {
      std::cout << "...and cleaned." << '\n';
      server.setSucceeded();
    }, true);
  });
  server.registerPreemptCallback([&server, &timer]() {
    std::cout << "Preempt callback... "
              << "active:" << server.isActive() << ' '
              << "new_goal:" << server.isNewGoalAvailable() << ' '
              << "preempt:" << server.isPreemptRequested() << '\n';
    timer.stop();
    server.setPreempted([]() {
      foo::DoDishesResult r;
      r.total_dishes_cleaned = 13;
      return r;
    }());
  });
  server.start();

  ros::MultiThreadedSpinner spinner(4);
  ros::spin(spinner);
  return 0;
}
