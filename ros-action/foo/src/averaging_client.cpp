#include <thread>

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

#include <foo/AveragingAction.h>

class AveragingClient {
 public:
  AveragingClient() : ac("averaging", false) {
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
  }

  void Goal(int n) {
    ROS_INFO("Sending goal.");
    foo::AveragingGoal goal;
    goal.samples = n;
    ac.sendGoal(goal,
                boost::bind(&AveragingClient::OnDone, this, _1, _2),
                boost::bind(&AveragingClient::OnActive, this),
                boost::bind(&AveragingClient::OnFeedback, this, _1));
  }

  void Cancel() {
    ROS_INFO("Cancelling goal.");
    ac.cancelGoal();
  }

  void OnActive() {
    ROS_INFO("Goal just went active");
  }

  void OnFeedback(const foo::AveragingFeedbackConstPtr &f) {
    ROS_INFO("Feedback: %i %f %f", f->sample, f->mean, f->std_dev);
  }

  void OnDone(const actionlib::SimpleClientGoalState &state,
              const foo::AveragingResultConstPtr &result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Result: %f %f", result->mean, result->std_dev);
    ros::shutdown();
  }

 private:
  actionlib::SimpleActionClient<foo::AveragingAction> ac;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_averaging");
  std::thread spinner([]() { ros::spin(); });
  AveragingClient as;
  as.Goal(10);

  if (getenv("CANCEL")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    as.Cancel();
  }

  spinner.join();
  return 0;
}
