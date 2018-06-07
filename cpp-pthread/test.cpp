#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "odr.h"

int main() {
  std::thread thr([&]() {
    std::cout << "Notify\n";
    Notify();
  });

  std::cout << "Before GetWithin\n";
  Get();
  std::cout << "After GetWithin\n";

  thr.join();
}
