#include "types.h"

namespace BT {

template <>
foo::Position2D convertFromString(StringView str) {
  std::vector<StringView> parts = splitString(str, ';');
  if (parts.size() != 2) {
    throw RuntimeError("invalid input)");
  }

  foo::Position2D out;
  out.x = convertFromString<double>(parts[0]);
  out.y = convertFromString<double>(parts[1]);
  return out;
}

template <>
foo::Pose2D convertFromString(StringView str) {
  std::vector<StringView> parts = splitString(str, ';');
  if (parts.size() != 3) {
    throw RuntimeError("invalid input)");
  }

  foo::Pose2D out;
  out.x = convertFromString<double>(parts[0]);
  out.y = convertFromString<double>(parts[1]);
  out.theta = convertFromString<double>(parts[2]);
  return out;
}

}  // namespace BT
