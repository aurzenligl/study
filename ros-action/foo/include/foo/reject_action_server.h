#pragma once

#include <functional>
#include <mutex>
#include <string>

#include <actionlib/action_definition.h>
#include <actionlib/server/action_server.h>
#include <ros/ros.h>

namespace foo {

/**
 * @class RejectActionServer @brief The RejectActionServer
 * implements a single goal policy on top of the ActionServer class.
 * Only one goal can be either pending/active at a time, new goals get rejected as long
 * as there is a goal pending/active. Explicit cancel preempts the pending/active goal.
 */
template <class ActionSpec>
class RejectActionServer {
 public:
  ACTION_DEFINITION(ActionSpec)

  using GoalHandle = typename actionlib::ActionServer<ActionSpec>::GoalHandle;
  using RejectCallback = std::function<void (const GoalConstPtr &)>;

  /**
   * @param name A name for the action server
   */
  explicit RejectActionServer(std::string name);

  /**
   * @brief Accepts a new goal when one is pending. The status of this
   * goal is set to active upon acceptance.
   * @return A shared_ptr to the new goal.
   */
  GoalConstPtr acceptNewGoal();

  /**
   * @brief Allows polling implementations to query about the availability of a new goal
   * @return True if a new goal is available, false otherwise
   */
  bool isNewGoalAvailable();

  /**
   * @brief Allows polling implementations to query about preempt requests
   * @return True if a preempt is requested, false otherwise
   */
  bool isPreemptRequested();

  /**
   * @brief Allows polling implementations to query about the status of the current goal
   * @return True if a goal is active, false otherwise
   */
  bool isActive();

  /**
   * @brief Sets the status of the active goal to succeeded
   * @param result An optional result to send back to any clients of the goal
   * @param result An optional text message to send back to any clients of the goal
   */
  void setSucceeded(const Result & result = Result(), const std::string & text = std::string(""));

  /**
   * @brief Sets the status of the active goal to aborted
   * @param result An optional result to send back to any clients of the goal
   * @param result An optional text message to send back to any clients of the goal
   */
  void setAborted(const Result & result = Result(), const std::string & text = std::string(""));

  /**
  * @brief Publishes feedback for a given goal
  * @param feedback Shared pointer to the feedback to publish
  */
  void publishFeedback(const FeedbackConstPtr & feedback);

  /**
  * @brief Publishes feedback for a given goal
  * @param feedback The feedback to publish
  */
  void publishFeedback(const Feedback & feedback);

  /**
   * @brief Sets the status of the active goal to preempted
   * @param result An optional result to send back to any clients of the goal
   * @param result An optional text message to send back to any clients of the goal
   */
  void setPreempted(const Result & result = Result(), const std::string & text = std::string(""));

  /**
   * @brief Allows users to register a callback to be invoked when a new goal is available
   * @param cb The callback to be invoked
   */
  void registerGoalCallback(std::function<void()> cb);

  /**
   * @brief Allows users to register a callback to be invoked when a new preempt request is available
   * @param cb The callback to be invoked
   */
  void registerPreemptCallback(std::function<void()> cb);

  /**
   * @brief Allows users to register a callback to be invoked when a new goal is rejected
   * @param cb The callback to be invoked
   */
  void registerRejectCallback(RejectCallback cb);

  /**
   * @brief Explicitly start the action server
   */
  void start();

private:
  /**
   * @brief Callback for when the ActionServer receives a new goal and passes it on
   */
  void goalCallback(GoalHandle goal);

  /**
   * @brief Callback for when the ActionServer receives a new preempt and passes it on
   */
  void preemptCallback(GoalHandle preempt);

  GoalHandle current_goal_;

  bool new_goal_, preempt_request_;

  std::recursive_mutex lock_;

  std::function<void()> goal_callback_;
  std::function<void()> preempt_callback_;
  RejectCallback reject_callback_;

  actionlib::ActionServer<ActionSpec> as_;
};

}  // namespace foo

#include <foo/reject_action_server_imp.h>
