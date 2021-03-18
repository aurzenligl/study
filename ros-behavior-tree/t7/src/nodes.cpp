#include "nodes.h"

#include <iostream>

#include "utils.h"

namespace foo {

BT::NodeStatus CheckBattery() {
  std::cout << "[ Battery: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SaySomething::tick() {
  BT::Optional<std::string> msg = getInput<std::string>("message");
  if (!msg) {
    throw BT::RuntimeError("missing required input [message]: ", msg.error());
  }

  std::cout << "Robot says: " << msg.value() << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveBase::tick() {
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

void MoveBase::halt() {
  std::cout << "[ MoveBase: HALTING ]" << std::endl;
  halt_requested_.store(true);
}

void RegisterNodes(BT::BehaviorTreeFactory& factory) {
  factory.registerSimpleCondition("BatteryOK", std::bind(foo::CheckBattery));
  factory.registerNodeType<foo::MoveBase>("MoveBase");
  factory.registerNodeType<foo::SaySomething>("SaySomething");
}

}  // namespace foo
