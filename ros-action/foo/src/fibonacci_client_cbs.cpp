#include <thread>

#include <actionlib/client/simple_action_client.h>
#include <foo/FibonacciAction.h>
#include <ros/ros.h>

class FibonacciClient {
 public:
  FibonacciClient() : ac("fibonacci", false) {
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
  }

  void Goal(int order) {
    ROS_INFO("Sending goal.");
    foo::FibonacciGoal goal;
    goal.order = order;
    ac.sendGoal(goal,
                boost::bind(&FibonacciClient::OnDone, this, _1, _2),
                boost::bind(&FibonacciClient::OnActive, this),
                boost::bind(&FibonacciClient::OnFeedback, this, _1));
  }

  void Cancel() {
    ROS_INFO("Cancelling goal.");
    ac.cancelGoal();
  }

  void OnActive() {
    ROS_INFO("Goal just went active");
  }

  void OnFeedback(const foo::FibonacciFeedbackConstPtr &feedback) {
    ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
  }

  void OnDone(const actionlib::SimpleClientGoalState &state,
              const foo::FibonacciResultConstPtr &result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %i", result->sequence.empty() ? 0 : result->sequence.back());
    ros::shutdown();
  }

 private:
  actionlib::SimpleActionClient<foo::FibonacciAction> ac;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_fibonacci_callback");
  std::thread spinner([]() { ros::spin(); });
  FibonacciClient as;
  as.Goal(10);

  if (getenv("CANCEL")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    as.Cancel();
  }

  spinner.join();
  return 0;
}
