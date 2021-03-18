#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>

#include <stdlib.h>

#include <iostream>

#include "utils.h"
#include "xdoor.h"

void Main() {
  BT::BehaviorTreeFactory factory;

  foo::RegisterCrossdoorNodes(factory);

  BT::Tree tree = factory.createTreeFromFile(getenv("XML"));  // tree from file

  BT::StdCoutLogger logger_cout(tree);  // state changes to console
  BT::FileLogger logger_file(tree, "bt_trace.fbl");  // state changes to file
  BT::MinitraceLogger logger_minitrace(tree, "bt_trace.json");  // node execution time

  printTreeRecursively(tree.rootNode());

  //while (1)
  {
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while (status == BT::NodeStatus::RUNNING) {
      status = tree.tickRoot();
      foo::SleepMs(1);   // optional sleep to avoid "busy loops"
    }
    foo::SleepMs(1000);
  }
}

int main() {
  try {
    Main();
  } catch (const std::exception& e) {
    std::cout << "error: " << e.what() << '\n';
  }
}
