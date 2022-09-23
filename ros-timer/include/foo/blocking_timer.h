#pragma once

#include <memory>

#include <ros/duration.h>
#include <ros/timer.h>

namespace foo {

class BlockingTimer {
 public:
  BlockingTimer() = default;
  BlockingTimer(const ros::Timer& rhs);
  BlockingTimer& operator=(const ros::Timer& other);

  // see ros::Timer for api documentation
  void start();
  void stop();
  bool hasPending();
  void setPeriod(const ros::Duration &period, bool reset = true);
  bool hasStarted() const;
  bool isValid() const;
  explicit operator bool() const;
  bool operator<(const BlockingTimer& rhs);
  bool operator==(const BlockingTimer& rhs);
  bool operator!=(const BlockingTimer& rhs);

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace foo
