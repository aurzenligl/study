#include "odr.h"

#include <condition_variable>
#include <mutex>
#include <thread>

std::mutex mtx;
std::condition_variable cv;
bool ready = false;

void Notify() {
  std::chrono::milliseconds(100);
  std::unique_lock<std::mutex> lock(mtx);
  ready = true;
  cv.notify_one();
}

void Get() {
  std::unique_lock<std::mutex> lock(mtx);
  cv.wait_for(lock, std::chrono::milliseconds(300));
}
