#pragma once

#include <chrono>

#include <ros/ros.h>

namespace foo {

class Timeit {
 public:
  explicit Timeit(const char *name) : name(name) {}

  ~Timeit() {
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> d(t1 - t0);
    ROS_WARN_STREAM("Timeit: " << name << ": " << d.count() << " [ms]");
  }

 private:
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  const char *name;
};

}  // namespace foo
