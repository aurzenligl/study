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

void RegisterNodes(BT::BehaviorTreeFactory& factory);

}  // namespace foo
