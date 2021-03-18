#include <stdlib.h>

#include <iostream>

#include <behaviortree_cpp_v3/bt_factory.h>
#include "nodes.h"
#include "legacy.h"
#include "utils.h"
#include "xdoor.h"

void Main() {
  BT::BehaviorTreeFactory factory;

  BT::NodeBuilder builder_A =
      [](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<foo::Action_A>(name, config, 42, 3.14, "hello world");
      };

  factory.registerBuilder<foo::Action_A>("Action_A", builder_A);
  factory.registerNodeType<foo::Action_B>("Action_B");

  // Create the whole tree
  BT::Tree tree = factory.createTreeFromFile(getenv("XML"));

  for (BT::TreeNode::Ptr& node : tree.nodes) {
    if (foo::Action_B* n = dynamic_cast<foo::Action_B*>(node.get())) {
      n->init(42, 3.14, "hello world");
    }
  }

  tree.tickRoot();
}

int main() {
  try {
    Main();
  } catch (const std::exception& e) {
    std::cout << "error: " << e.what() << '\n';
  }
}
