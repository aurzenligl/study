#pragma once

#include <iostream>

namespace lgc {

// This is my custom type.
struct Point3D { double x,y,z; };

// We want to create an ActionNode to calls method MyLegacyMoveTo::go
class MyLegacyMoveTo {
 public:
  bool go(Point3D goal) {
    printf("Going to: %f %f %f\n", goal.x, goal.y, goal.z);
    return true; // true means success in my legacy code
  }
};

}  // namespace lgc
