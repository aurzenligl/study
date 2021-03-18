#include <stdlib.h>

#include <iostream>

#include <behaviortree_cpp_v3/bt_factory.h>
#include "nodes.h"
#include "legacy.h"
#include "utils.h"
#include "xdoor.h"

void Main() {
  lgc::MyLegacyMoveTo move_to;

  // Here we use a lambda that captures the reference of move_to
  auto MoveToWrapperWithLambda = [&move_to](BT::TreeNode& self) -> BT::NodeStatus {
    lgc::Point3D goal;
    self.getInput("goal", goal);
    bool res = move_to.go(goal);
    return res ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  };

  BT::BehaviorTreeFactory factory;

  BT::PortsList ports = { BT::InputPort<lgc::Point3D>("goal") };
  factory.registerSimpleAction("MoveTo", MoveToWrapperWithLambda, ports);

  BT::Tree tree = factory.createTreeFromFile(getenv("XML"));

  tree.tickRoot();
}

int main() {
  try {
    Main();
  } catch (const std::exception& e) {
    std::cout << "error: " << e.what() << '\n';
  }
}
