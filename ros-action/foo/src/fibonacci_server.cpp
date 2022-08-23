#include <actionlib/server/simple_action_server.h>
#include <foo/FibonacciAction.h>
#include <ros/ros.h>

class FibonacciActionServer {
 public:
  explicit FibonacciActionServer(std::string name)
      : as_(name, boost::bind(&FibonacciActionServer::OnExecute, this, _1), false) {
    as_.start();
  }

  // run in unique thread, unrelated with spinner
  void OnExecute(const foo::FibonacciGoalConstPtr &goal) {
    ros::Rate r(10);

    foo::FibonacciFeedback feedback;
    feedback.sequence.clear();
    feedback.sequence.push_back(0);
    feedback.sequence.push_back(1);

    ROS_INFO("Executing, creating fibonacci sequence of order %i with seeds %i, %i",
             goal->order, feedback.sequence[0], feedback.sequence[1]);

    for (int i = 1; i <= goal->order; i++) {
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("Preempted");
        as_.setPreempted();
        return;
      }

      feedback.sequence.push_back(feedback.sequence[i] + feedback.sequence[i - 1]);
      as_.publishFeedback(feedback);
      r.sleep();
    }

    ROS_INFO("Succeeded");
    foo::FibonacciResult result;
    result.sequence = feedback.sequence;
    as_.setSucceeded(result);
  }

 private:
  actionlib::SimpleActionServer<foo::FibonacciAction> as_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "fibonacci");
  FibonacciActionServer fibonacci("fibonacci");
  ros::spin();
  return 0;
}
