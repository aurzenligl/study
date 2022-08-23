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
    timer = ros::NodeHandle().createTimer(ros::Duration(0.3), [i = 5, &server, &timer](const ros::TimerEvent&) mutable {
      if (--i) {
        std::cout << "cleaning..." << '\n';
        server.publishFeedback([&]() {
          foo::DoDishesFeedback f;
          f.percent_complete = 1 - i/5.;
          return f;
        }());
      } else {
        std::cout << "...cleaned." << '\n';
        server.setSucceeded();
        timer.stop();
      }
    });
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
