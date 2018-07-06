#include <type_traits>
#include <iostream>
#include <tr2/type_traits>

// since name has to be hardcoded when checking for type members,
// we need to prepare macros to automatically generate those for different names

#define HAS_METHOD(NAME)                                                                  \
  template <typename, typename T>                                                         \
  struct has_method_##NAME {                                                              \
    static_assert(std::integral_constant<T, false>::value,                                \
                  "Second template parameter needs to be of function type.");             \
  };                                                                                      \
                                                                                          \
  template <typename C, typename Ret, typename... Args>                                   \
  struct has_method_##NAME<C, Ret(Args...)> {                                                    \
   private:                                                                               \
    template <typename T>                                                                 \
    static constexpr auto check(T *) ->                                                   \
        typename std::is_same<decltype(std::declval<T>(). NAME (std::declval<Args>()...)),\
                              Ret>::type;                                                 \
                                                                                          \
    template <typename>                                                                   \
    static constexpr std::false_type check(...);                                          \
                                                                                          \
   public:                                                                                \
    typedef decltype(check<C>(0)) type;                                                   \
    static constexpr bool value = type::value;                                            \
  };

#define HAS_DATA(NAME)                                                         \
  template <typename C, typename T>                                            \
  struct has_data_##NAME {                                                     \
   private:                                                                    \
    struct fallback {                                                          \
      int NAME;                                                                \
    };                                                                         \
    struct derived : C, fallback {                                             \
      using inner = C;                                                         \
    };                                                                         \
                                                                               \
    template <typename U, U>                                                   \
    struct test;                                                               \
                                                                               \
    template <typename U>                                                      \
    static constexpr auto check(...) ->                                        \
        typename std::is_convertible<decltype(U::inner::NAME), T>::type;       \
                                                                               \
    template <typename U>                                                      \
    static constexpr std::false_type check(test<int fallback::*, &U::NAME> *); \
                                                                               \
   public:                                                                     \
    typedef decltype(check<derived>(0)) type;                                  \
    static constexpr bool value = type::value;                                 \
  };

// this is user code, that's everything that user has to write

class First {
 public:
  static constexpr const char *kName = "first";
  static constexpr const char *kLabel = "the_first_label";

  static void Load();
  static void Unload();

  First() {}
  ~First() {}
};

class Second {
 public:
  static constexpr const char *kName = "second";

  static void Load();
  static void Unload();

  Second() {}
  ~Second() {}

  void Pause() {
    std::cout << "Pause\n";
  }
  void Resume() {
    std::cout << "Resume\n";
  }
};

// simple logic reacting to presence or lack of certain members

template <typename Svc>
void AddLabel(std::false_type) {
  std::cout << "Detected no label in '" << Svc::kName << "'.\n";
}

template <typename Svc>
void AddLabel(std::true_type) {
  std::cout << "Detected label '" << Svc::kLabel << "' in '" << Svc::kName << "'.\n";
}

template <typename Svc>
void AddPauseResume(std::false_type) {
  std::cout << "Detected neither Pause/Resume in '" << Svc::kName << "'.\n";
}

template <typename Svc>
void AddPauseResume(std::true_type) {
  std::cout << "Detected both Pause/Resume for '" << Svc::kName << "', let's call them:\n";
  Svc svc;
  svc.Pause();
  svc.Resume();
}

// define checkers for members with particular names
HAS_DATA(kName);
HAS_DATA(kLabel);
HAS_METHOD(Pause);
HAS_METHOD(Resume);

// the introspective implementation, which may detect members, their types
// and act accordingly - including failure to compile if service definition
// has internal contradictions
template <typename Svc>
void RegisterService() {
  using name = typename has_data_kName<Svc, std::string>::type;
  using label = typename has_data_kLabel<Svc, std::string>::type;
  using default_ctor = typename std::is_default_constructible<Svc>::type;
  using pause = typename has_method_Pause<Svc, void()>::type;
  using resume = typename has_method_Resume<Svc, void()>::type;

  static_assert(name::value, "must be named");
  static_assert(default_ctor::value, "must be default constructible");
  static_assert(pause::value == resume::value, "both pause/resume or none must be defined");

  AddLabel<Svc>(label());
  AddPauseResume<Svc>(pause());
}

int main() {
  RegisterService<First>();
  RegisterService<Second>();
}
