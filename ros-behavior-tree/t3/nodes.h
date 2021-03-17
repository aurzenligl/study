#include <behaviortree_cpp_v3/action_node.h>

#include <iostream>

namespace t3 {

struct Position2D {
  double x, y;
};

class CalculateGoal : public BT::SyncActionNode {
 public:
  using BT::SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts() {
    return { BT::OutputPort<Position2D>("goal") };
  }

  BT::NodeStatus tick() override {
    Position2D mygoal = {1.1, 2.3};
    setOutput<Position2D>("goal", mygoal);
    return BT::NodeStatus::SUCCESS;
  }
};

class PrintTarget : public BT::SyncActionNode {
 public:
  using BT::SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts() {
    const char* description = "Simply print the goal on console...";
    return { BT::InputPort<Position2D>("target", description) };
  }

  BT::NodeStatus tick() override {
    auto res = getInput<Position2D>("target");
    if (!res) {
      throw BT::RuntimeError("error reading port [target]:", res.error());
    }

    Position2D target = res.value();
    printf("Target positions: [ %.1f, %.1f ]\n", target.x, target.y);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace t3

namespace BT {

template <> t3::Position2D convertFromString(StringView str) {
  printf("Converting string: \"%s\"\n", str.data());

  std::vector<StringView> parts = splitString(str, ';');
  if (parts.size() != 2) {
    throw RuntimeError("invalid input)");
  }

  t3::Position2D out;
  out.x = convertFromString<double>(parts[0]);
  out.y = convertFromString<double>(parts[1]);
  return out;
}

}  // namespace BT
