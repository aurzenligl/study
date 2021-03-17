#include <behaviortree_cpp_v3/action_node.h>

#include <iostream>

namespace t2 {

class SaySomething : public BT::SyncActionNode {
 public:
  using BT::SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("message") };
  }

  BT::NodeStatus tick() override {
    BT::Optional<std::string> msg = getInput<std::string>("message");
    if (!msg) {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }

    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

inline BT::NodeStatus SaySomethingSimple(BT::TreeNode& self) {
  BT::Optional<std::string> msg = self.getInput<std::string>("message");
  if (!msg) {
    throw BT::RuntimeError("missing required input [message]: ", msg.error());
  }

  std::cout << "Robot says: " << msg.value() << std::endl;
  return BT::NodeStatus::SUCCESS;
}

class ThinkWhatToSay : public BT::SyncActionNode {
 public:
  using BT::SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts() {
    return { BT::OutputPort<std::string>("text") };
  }

  BT::NodeStatus tick() override {
    setOutput("text", "The answer is 42" );
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace t2
