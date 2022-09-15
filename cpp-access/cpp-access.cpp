#include <functional>
#include <iostream>
#include <memory>
#include <mutex>

template <class X>
class TimerManager {
 private:
  struct Impl {
    Impl(int x) : x(x) {}
    int x;
  };
  using ImplPtr = std::shared_ptr<Impl>;

 public:
  TimerManager() {
    impl_ = std::make_shared<Impl>(111);
  }

 private:
  int findTimer(int x) { return 33 + x; }
  ImplPtr findTimer2(int x) { return std::make_shared<Impl>(33 + 3 * x); }
  ImplPtr impl_;
  std::mutex mtx_;
};

struct MyStruct {
  MyStruct(int secret) : secret_(secret) {}

 private:
  int secret_;
};

using SecretAccessor = int MyStruct::*;
SecretAccessor get_secret_accessor();
template <SecretAccessor Instance>
struct Robber { friend SecretAccessor get_secret_accessor() { return Instance; } };
template struct Robber<&MyStruct::secret_>;

using TimerManagerInt = TimerManager<int>;

using FindTimerAccessor = int (TimerManagerInt::*)(int);
FindTimerAccessor get_find_timer_accessor();
template <FindTimerAccessor Instance>
struct Robber2 { friend FindTimerAccessor get_find_timer_accessor() { return Instance; } };
template struct Robber2<&TimerManagerInt::findTimer>;

int RobHandle(TimerManagerInt &x);
template <typename Ret, Ret TimerManagerInt::*DataPtr>
struct RobHandleT { friend int RobHandle(TimerManagerInt &x) { return std::invoke(DataPtr, x)->x; } };
template struct RobHandleT<decltype(TimerManagerInt::impl_), &TimerManagerInt::impl_>;

std::mutex &RobMutex(TimerManagerInt &x);
template <std::mutex TimerManagerInt::* DataPtr>
struct RobMutexT { friend std::mutex &RobMutex(TimerManagerInt &x) { return std::invoke(DataPtr, x); } };
template struct RobMutexT<&TimerManagerInt::mtx_>;

int RobTimerHandle(TimerManagerInt &x, int y, std::mutex &mtx);
template <typename Ret, Ret (TimerManagerInt::*FunPtr)(int)>
struct RobTimerHandleT { friend int RobTimerHandle(TimerManagerInt &x, int y, std::mutex &mtx) { return std::invoke(FunPtr, x, y)->x; } };
template struct RobTimerHandleT<TimerManagerInt::ImplPtr, &TimerManagerInt::findTimer2>;

int RobTimerHandle2(TimerManagerInt &x, int y);
template <typename Ret, Ret (TimerManagerInt::*FindPtr)(int), std::mutex TimerManagerInt::* MtxPtr>
struct RobTimerHandleT2 {
  friend int RobTimerHandle2(TimerManagerInt &x, int y) {
    std::lock_guard lock(std::invoke(MtxPtr, x));
    return std::invoke(FindPtr, x, y)->x;
  }
};
template struct RobTimerHandleT2<TimerManagerInt::ImplPtr, &TimerManagerInt::findTimer2, &TimerManagerInt::mtx_>;

int main() {
  MyStruct my_struct(42);
  std::cout << "Proof: " << std::invoke(get_secret_accessor(), my_struct) << '\n';

  TimerManagerInt m;
  std::cout << "xxx: " << std::invoke(get_find_timer_accessor(), m, 42) << '\n';
  std::cout << "yyy: " << RobHandle(m) << '\n';

  std::mutex &mtx = RobMutex(m);
  std::cout << "zzz: " << RobTimerHandle(m, 100, mtx) << '\n';

  std::cout << "www: " << RobTimerHandle2(m, 100) << '\n';

  // int32_t timer_handle = rob::GetHandle(timer);
  // uint64_t removal_id = rob::GetRemovalId(TimerManager::global(), timer_handle);
  // if (auto info = rob::GetIdInfo(ros::getGlobalCallbackQueue())) {
  //   std::lock_guard(info->rw_lock);
  // }
}
