#pragma once

#include <string>

#include <ros/ros.h>

namespace foo {

template <class ActionSpec>
RejectActionServer<ActionSpec>::RejectActionServer(std::string name)
    : new_goal_(false),
      preempt_request_(false),
      as_({}, name,
          [this](GoalHandle goal) { goalCallback(goal); },
          [this](GoalHandle goal) { preemptCallback(goal); },
          false) {}

template <class ActionSpec>
typename RejectActionServer<ActionSpec>::GoalConstPtr RejectActionServer<ActionSpec>::acceptNewGoal() {
  std::lock_guard lock(lock_);

  if (!new_goal_ || !current_goal_.getGoal()) {
    ROS_ERROR_NAMED("actionlib", "Attempting to accept a goal when a new goal is not available");
    return boost::shared_ptr<const Goal>();
  }

  ROS_DEBUG_NAMED("actionlib", "Accepting a new goal");
  new_goal_ = false;
  current_goal_.setAccepted("This goal has been accepted by the simple action server");
  return current_goal_.getGoal();
}

template <class ActionSpec>
bool RejectActionServer<ActionSpec>::isNewGoalAvailable() {
  return new_goal_;
}

template <class ActionSpec>
bool RejectActionServer<ActionSpec>::isPreemptRequested() {
  return preempt_request_;
}

template <class ActionSpec>
bool RejectActionServer<ActionSpec>::isActive() {
  if (!current_goal_.getGoal()) {
    return false;
  }
  unsigned int status = current_goal_.getGoalStatus().status;
  return status == actionlib_msgs::GoalStatus::ACTIVE || status == actionlib_msgs::GoalStatus::PREEMPTING;
}

template <class ActionSpec>
void RejectActionServer<ActionSpec>::setSucceeded(const Result &result, const std::string &text) {
  std::lock_guard lock(lock_);
  ROS_DEBUG_NAMED("actionlib", "Setting the current goal as succeeded");
  current_goal_.setSucceeded(result, text);
  current_goal_ = {};
}

template <class ActionSpec>
void RejectActionServer<ActionSpec>::setAborted(const Result &result, const std::string &text) {
  std::lock_guard lock(lock_);
  ROS_DEBUG_NAMED("actionlib", "Setting the current goal as aborted");
  current_goal_.setAborted(result, text);
  current_goal_ = {};
}

template <class ActionSpec>
void RejectActionServer<ActionSpec>::setPreempted(const Result &result, const std::string &text) {
  std::lock_guard lock(lock_);
  ROS_DEBUG_NAMED("actionlib", "Setting the current goal as canceled");
  current_goal_.setCanceled(result, text);
  current_goal_ = {};
}

template <class ActionSpec>
void RejectActionServer<ActionSpec>::registerGoalCallback(std::function<void()> cb) {
  goal_callback_ = cb;
}

template <class ActionSpec>
void RejectActionServer<ActionSpec>::registerPreemptCallback(std::function<void()> cb) {
  preempt_callback_ = cb;
}

template <class ActionSpec>
void RejectActionServer<ActionSpec>::registerRejectCallback(RejectCallback cb) {
  reject_callback_ = cb;
}

template <class ActionSpec>
void RejectActionServer<ActionSpec>::publishFeedback(const FeedbackConstPtr &feedback) {
  current_goal_.publishFeedback(*feedback);
}

template <class ActionSpec>
void RejectActionServer<ActionSpec>::publishFeedback(const Feedback &feedback) {
  current_goal_.publishFeedback(feedback);
}

template <class ActionSpec>
void RejectActionServer<ActionSpec>::goalCallback(GoalHandle goal) {
  std::lock_guard lock(lock_);
  ROS_DEBUG_NAMED("actionlib", "A new goal has been recieved by the reject action server");

  if (!current_goal_.getGoal()) {
    current_goal_ = goal;
    new_goal_ = true;
    preempt_request_ = false;

    // if the user has defined a goal callback, we'll call it now
    if (goal_callback_) {
      goal_callback_();
    }
  } else {
    goal.setRejected(Result(), "This goal was rejected because another goal is active in reject action server");

    // if the user has defined a reject callback, we'll call it now
    if (reject_callback_) {
      reject_callback_(goal.getGoal());
    }
  }
}

template <class ActionSpec>
void RejectActionServer<ActionSpec>::preemptCallback(GoalHandle preempt) {
  std::lock_guard lock(lock_);
  ROS_DEBUG_NAMED("actionlib", "A preempt has been received by the RejectActionServer");

  // if the preempt is for the current goal, then we'll set the preemptRequest flag and call the user's preempt callback
  if (preempt == current_goal_) {
    ROS_DEBUG_NAMED("actionlib", "Setting preempt_request bit for the current goal to TRUE and invoking callback");
    preempt_request_ = true;

    // if the user has registered a preempt callback, we'll call it now
    if (preempt_callback_) {
      preempt_callback_();
    }
  }
}

template <class ActionSpec>
void RejectActionServer<ActionSpec>::start() {
  as_.start();
}

}  // namespace foo
