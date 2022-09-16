#pragma once

#include <memory>

#include <ros/duration.h>
#include <ros/timer.h>

namespace foo {

class Timer {
 public:
  Timer() = default;
  Timer(const ros::Timer& rhs);
  Timer& operator=(const ros::Timer& other);

  // see ros::Timer for api documentation
  void start();
  void stop();
  bool hasPending();
  void setPeriod(const ros::Duration &period, bool reset = true);
  bool hasStarted() const;
  bool isValid() const;
  explicit operator bool() const;
  bool operator<(const Timer& rhs);
  bool operator==(const Timer& rhs);
  bool operator!=(const Timer& rhs);

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace foo
