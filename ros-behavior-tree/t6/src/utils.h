#include <thread>

namespace foo {

inline void SleepMs(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

}  // namespace foo
