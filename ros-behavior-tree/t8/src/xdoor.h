#pragma once

#include "behaviortree_cpp_v3/bt_factory.h"

namespace foo {

BT::NodeStatus IsDoorOpen();

BT::NodeStatus IsDoorLocked();

BT::NodeStatus UnlockDoor();

BT::NodeStatus PassThroughDoor();

BT::NodeStatus PassThroughWindow();

BT::NodeStatus OpenDoor();

BT::NodeStatus CloseDoor();

void RegisterCrossdoorNodes(BT::BehaviorTreeFactory& factory);

}  // namespace foo
