#include <behaviortree_cpp_v3/bt_factory.h>

#include <iostream>

#include "nodes.h"

void Main() {
  BT::BehaviorTreeFactory factory;

  // register via derived class
  factory.registerNodeType<t1::ApproachObject>("ApproachObject");

  // register via function
  factory.registerSimpleCondition("CheckBattery", std::bind(t1::CheckBattery));

  // register via methods
  t1::GripperInterface gripper;
  factory.registerSimpleAction("OpenGripper", std::bind(&t1::GripperInterface::Open, &gripper));
  factory.registerSimpleAction("CloseGripper", std::bind(&t1::GripperInterface::Close, &gripper));

  // execute tree
  BT::Tree tree = factory.createTreeFromFile("./foo.xml");
  tree.tickRoot();
}

int main() {
  try {
    Main();
  } catch (const std::exception& e) {
    std::cout << "error: " << e.what() << '\n';
  }
}
