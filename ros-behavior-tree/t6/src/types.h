#include <behaviortree_cpp_v3/action_node.h>

#include <iostream>

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

}  // namespace BT
