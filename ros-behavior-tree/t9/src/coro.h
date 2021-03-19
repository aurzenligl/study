#pragma once

#include "behaviortree_cpp_v3/bt_factory.h"

#include <iostream>

namespace foo {

using Milliseconds = std::chrono::milliseconds;

inline BT::TimePoint Now() {
  return std::chrono::high_resolution_clock::now();
};

class Resource {
 public:
  explicit Resource(std::string name) : name(name) {
    std::cout << name << ": Resource::ctor" << '\n';
  }

  ~Resource() {
    if (std::uncaught_exceptions()) {
      std::cout << name << ": Resource::exception\n";
    } else {
      std::cout << name << ": Resource::dtor\n";
    }
    std::cout << '\n';
  }

  std::string name;
};

class MyAsyncAction: public BT::CoroActionNode {
 public:
  explicit MyAsyncAction(const std::string& name) : BT::CoroActionNode(name, {}) {}

 private:
  // This is the ideal skeleton/template of an async action:
  //  - A request to a remote service provider.
  //  - A loop where we check if the reply has been received.
  //  - You may call setStatusRunningAndYield() to "pause".
  //  - Code to execute after the reply.
  //  - A simple way to handle halt().
  BT::NodeStatus tick() override {
    std::cout << name() << ": Started. Send Request to server." << std::endl;

    Resource res{name()};

    BT::TimePoint initial_time = Now();
    BT::TimePoint reply_time = initial_time + Milliseconds(100);

    int count = 0;
    bool reply_received = false;

    while (!reply_received) {
      if (count++ == 0) {
        std::cout << name() << ": Waiting Reply..." << std::endl;
      }

      if (Now() >= reply_time) {
        reply_received = true;
      }

      if (!reply_received) {
        std::cout << name() << ": Yielding..." << std::endl;
        setStatusRunningAndYield();
      }
    }

    // This part of the code is never reached if halt() is invoked,
    // only if reply_received == true;
    std::cout << name() <<": Done. 'Waiting Reply' loop repeated "
              << count << " times" << std::endl;
    cleanup(false);
    return BT::NodeStatus::SUCCESS;
  }

  // you might want to cleanup differently if it was halted or successful
  void cleanup(bool halted) {
    if (halted) {
      std::cout << name() <<": cleaning up after a halt()" << std::endl;
    } else {
      std::cout << name() <<": cleaning up after SUCCESS" << std::endl;
    }
  }

  void halt() override {
    std::cout << name() <<": Halted." << std::endl;
    cleanup(true);
    // Do not forget to call this at the end.
    CoroActionNode::halt();
  }
};

}  // namespace foo
