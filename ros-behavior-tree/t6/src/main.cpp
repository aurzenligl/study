#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>

#include <stdlib.h>

#include <iostream>

#include "nodes.h"
#include "utils.h"
#include "xdoor.h"

void Main() {
  BT::BehaviorTreeFactory factory;

  foo::RegisterNodes(factory);
  foo::RegisterCrossdoorNodes(factory);

  BT::Tree tree = factory.createTreeFromFile(getenv("XML"));

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    foo::SleepMs(1);  // avoids "busy loops"
  }

  // let's visualize some information about the current state of the blackboards.
  std::cout << "--------------" << std::endl;
  tree.blackboard_stack[0]->debugMessage();
  std::cout << "--------------" << std::endl;
  tree.blackboard_stack[1]->debugMessage();
  std::cout << "--------------" << std::endl;
}

int main() {
  try {
    Main();
  } catch (const std::exception& e) {
    std::cout << "error: " << e.what() << '\n';
  }
}
