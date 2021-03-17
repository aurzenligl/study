#include <behaviortree_cpp_v3/action_node.h>

#include <iostream>

namespace t1 {

class ApproachObject : public BT::SyncActionNode {
 public:
  explicit ApproachObject(const std::string& name) :
      BT::SyncActionNode(name, {}) {}

  BT::NodeStatus tick() override {
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

BT::NodeStatus CheckBattery() {
  std::cout << "[ Battery: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

class GripperInterface {
 public:
  BT::NodeStatus Open() {
    open_ = true;
    std::cout << "GripperInterface::open" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus Close() {
    std::cout << "GripperInterface::close" << std::endl;
    open_ = false;
    return BT::NodeStatus::SUCCESS;
  }

 private:
  bool open_ = true;  // shared information
};

}  // namespace t1
