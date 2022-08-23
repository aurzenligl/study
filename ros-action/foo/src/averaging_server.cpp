#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <foo/AveragingAction.h>
#include <foo/reject_action_server.h>

class AveragingActionServer {
 public:
  explicit AveragingActionServer(std::string name) : as_(name) {
    ros::NodeHandle nh;
    sub_ = nh.subscribe("/random_number", 1, &AveragingActionServer::OnNumber, this);
    as_.registerGoalCallback([this]() { OnGoal(); });
    as_.registerPreemptCallback([this]() { OnPreempt(); });
    as_.registerRejectCallback([this](auto goal) { OnReject(goal); });
    as_.start();
  }

  void OnGoal() {
    data_count_ = 0;
    sum_ = 0;
    sum_sq_ = 0;
    goal_ = as_.acceptNewGoal()->samples;
  }

  void OnPreempt() {
    ROS_INFO("Preempted");
    as_.setPreempted();
  }

  void OnReject(const foo::AveragingGoalConstPtr &goal) {
    ROS_INFO("Rejected");
  }

  void OnNumber(const std_msgs::Float32::ConstPtr& msg) {
    if (!as_.isActive()) {
      return;
    }

    data_count_++;
    sum_ += msg->data;
    sum_sq_ += pow(msg->data, 2);

    foo::AveragingFeedback feedback;
    feedback.sample = data_count_;
    feedback.data = msg->data;
    feedback.mean = sum_ / data_count_;
    feedback.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(feedback.mean, 2)));
    as_.publishFeedback(feedback);

    if (data_count_ >= goal_) {
      foo::AveragingResult result;
      result.mean = feedback.mean;
      result.std_dev = feedback.std_dev;
      if (result.mean < 5.0) {
        ROS_INFO("Aborted");
        as_.setAborted(result);
      } else {
        ROS_INFO("Succeeded");
        as_.setSucceeded(result);
      }
    }
  }

 private:
  ros::Subscriber sub_;
  foo::RejectActionServer<foo::AveragingAction> as_;
  int data_count_, goal_;
  float sum_, sum_sq_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "averaging");
  AveragingActionServer averaging(ros::this_node::getName());
  ros::spin();
  return 0;
}
