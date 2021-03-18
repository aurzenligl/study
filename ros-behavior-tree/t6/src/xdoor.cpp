#include "xdoor.h"

#include "utils.h"

namespace foo {

// For simplicity, in this example the status of the door is not shared
// using ports and blackboards
static bool door_open_ = false;
static bool door_locked_ = true;

BT::NodeStatus IsDoorOpen() {
  SleepMs(500);
  return door_open_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsDoorLocked() {
  SleepMs(500);
  return door_locked_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus UnlockDoor() {
  if (door_locked_) {
    SleepMs(2000);
    door_locked_ = false;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PassThroughDoor() {
  SleepMs(1000);
  return door_open_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus PassThroughWindow() {
  SleepMs(1000);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus OpenDoor() {
  if (door_locked_) {
    return BT::NodeStatus::FAILURE;
  }
  SleepMs(2000);
  door_open_ = true;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CloseDoor() {
  if (door_open_) {
    SleepMs(1500);
    door_open_ = false;
  }
  return BT::NodeStatus::SUCCESS;
}

// Register at once all the Actions and Conditions in this file
void RegisterCrossdoorNodes(BT::BehaviorTreeFactory& factory) {
  factory.registerSimpleCondition("IsDoorOpen", std::bind(IsDoorOpen));
  factory.registerSimpleAction("PassThroughDoor", std::bind(PassThroughDoor));
  factory.registerSimpleAction("PassThroughWindow", std::bind(PassThroughWindow));
  factory.registerSimpleAction("OpenDoor", std::bind(OpenDoor));
  factory.registerSimpleAction("CloseDoor", std::bind(CloseDoor));
  factory.registerSimpleCondition("IsDoorLocked", std::bind(IsDoorLocked));
  factory.registerSimpleAction("UnlockDoor", std::bind(UnlockDoor));
}

}  // namespace foo
