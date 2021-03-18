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

template <>
lgc::Point3D convertFromString(StringView key) {
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3) {
    throw RuntimeError("invalid input)");
  }

  lgc::Point3D output;
  output.x  = convertFromString<double>(parts[0]);
  output.y  = convertFromString<double>(parts[1]);
  output.z  = convertFromString<double>(parts[2]);
  return output;
}

}  // namespace BT
