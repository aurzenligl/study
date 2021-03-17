#include <behaviortree_cpp_v3/action_node.h>

#include <iostream>

#include "utils.h"
#include "types.h"

namespace foo {

inline BT::NodeStatus CheckBattery() {
  std::cout << "[ Battery: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

class SaySomething : public BT::SyncActionNode {
 public:
  using BT::SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("message") };
  }

  BT::NodeStatus tick() override {
    BT::Optional<std::string> msg = getInput<std::string>("message");
    if (!msg) {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }

    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

class MoveBaseAction : public BT::AsyncActionNode {
 public:
  using BT::AsyncActionNode::AsyncActionNode;

  static BT::PortsList providedPorts() {
    return { BT::InputPort<Pose2D>("goal") };
  }

  BT::NodeStatus tick() override {
    Pose2D goal;
    if (!getInput<Pose2D>("goal", goal)) {
      throw BT::RuntimeError("missing required input [goal]");
    }

    printf("[ MoveBase: STARTED ]. goal: x=%.f y=%.1f theta=%.2f\n",
           goal.x, goal.y, goal.theta);

    halt_requested_.store(false);
    int count = 0;
    while (!halt_requested_ && count++ < 25) {
      SleepMs(10);
    }

    std::cout << "[ MoveBase: FINISHED ]" << std::endl;
    return halt_requested_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

  void halt() override {
    std::cout << "[ MoveBase: HALTING ]" << std::endl;
    halt_requested_.store(true);
  }

 private:
  std::atomic_bool halt_requested_;
};

}  // namespace foo
