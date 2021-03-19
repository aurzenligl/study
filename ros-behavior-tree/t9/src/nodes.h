#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "behaviortree_cpp_v3/bt_factory.h"

#include "types.h"

namespace foo {

BT::NodeStatus CheckBattery();

class SaySomething : public BT::SyncActionNode {
 public:
  using BT::SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("message") };
  }

  BT::NodeStatus tick() override;
};

class MoveBase : public BT::AsyncActionNode {
 public:
  using BT::AsyncActionNode::AsyncActionNode;

  static BT::PortsList providedPorts() {
    return { BT::InputPort<Pose2D>("goal") };
  }

  BT::NodeStatus tick() override;
  void halt() override;

 private:
  std::atomic_bool halt_requested_;
};

// parametrization: the builder way
class Action_A : public BT::SyncActionNode {
 public:
  Action_A(const std::string& name, const BT::NodeConfiguration& config,
           int arg1, double arg2, std::string arg3):
      SyncActionNode(name, config),
      arg1_(arg1),
      arg2_(arg2),
      arg3_(arg3) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    printf("A: %d %f %s\n", arg1_, arg2_, arg3_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

 private:
  int arg1_;
  double arg2_;
  std::string arg3_;
};

// parametrization: the init way
class Action_B: public BT::SyncActionNode {
 public:
  using BT::SyncActionNode::SyncActionNode;

  void init(int arg1, double arg2, const std::string& arg3) {
    arg1_ = arg1;
    arg2_ = arg2;
    arg3_ = arg3;
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    printf("B: %d %f %s\n", arg1_, arg2_, arg3_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

 private:
  int arg1_;
  double arg2_;
  std::string arg3_;
};

void RegisterNodes(BT::BehaviorTreeFactory& factory);

}  // namespace foo
