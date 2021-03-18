#pragma once

#include <iostream>

#include <behaviortree_cpp_v3/action_node.h>
#include "legacy.h"

namespace foo {

struct Position2D {
  double x, y;
};

struct Pose2D {
  double x, y, theta;
};

}  // namespace foo

namespace BT {

template <> foo::Position2D convertFromString(StringView str);
template <> foo::Pose2D convertFromString(StringView str);
template <> lgc::Point3D convertFromString(StringView key);

}  // namespace BT
