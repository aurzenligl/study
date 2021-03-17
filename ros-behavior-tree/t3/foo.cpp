#include <behaviortree_cpp_v3/bt_factory.h>

#include <iostream>

#include "nodes.h"

void Main() {
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<t3::CalculateGoal>("CalculateGoal");
  factory.registerNodeType<t3::PrintTarget>("PrintTarget");

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
