#include <stdlib.h>

#include <iostream>

#include <behaviortree_cpp_v3/bt_factory.h>
#include "nodes.h"
#include "utils.h"
#include "coro.h"

void Main() {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<foo::MyAsyncAction>("MyAsyncAction");

  BT::Tree tree = factory.createTreeFromFile(getenv("XML"));

  while (tree.tickRoot() == BT::NodeStatus::RUNNING) {
    foo::SleepMs(10);
  }
}

int main() {
  try {
    Main();
  } catch (const std::exception& e) {
    std::cout << "error: " << e.what() << '\n';
  }
}
