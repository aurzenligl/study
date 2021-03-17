#include <behaviortree_cpp_v3/bt_factory.h>

#include <stdlib.h>

#include <iostream>

#include "nodes.h"

void Main() {
  BT::BehaviorTreeFactory factory;

  factory.registerSimpleCondition("BatteryOK", std::bind(foo::CheckBattery));
  factory.registerNodeType<foo::MoveBaseAction>("MoveBase");
  factory.registerNodeType<foo::SaySomething>("SaySomething");

  BT::Tree tree = factory.createTreeFromFile(getenv("XML"));

  BT::NodeStatus status;

  std::cout << "--- 1st executeTick() ---\n";
  status = tree.tickRoot();
  std::cout << '\n';

  foo::SleepMs(150);

  std::cout << "--- 2nd executeTick() ---\n";
  status = tree.tickRoot();
  std::cout << '\n';

  foo::SleepMs(150);

  std::cout << "--- 3rd executeTick() ---\n";
  status = tree.tickRoot();
  std::cout << '\n';
}

int main() {
  try {
    Main();
  } catch (const std::exception& e) {
    std::cout << "error: " << e.what() << '\n';
  }
}
