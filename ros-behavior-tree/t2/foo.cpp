#include <behaviortree_cpp_v3/bt_factory.h>

#include <iostream>

#include "nodes.h"

void Main() {
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<t2::SaySomething>("SaySomething");
  factory.registerNodeType<t2::ThinkWhatToSay>("ThinkWhatToSay");

  factory.registerSimpleAction("SaySomething2", t2::SaySomethingSimple,
                               {BT::InputPort<std::string>("message")});

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
