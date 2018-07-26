#include <iostream>
#include <memory>

template <typename T>
struct Wrap {
  Wrap(T t): arg(std::move(t)) {}  // must be implicit
  T arg;
};

template <typename T>
void Foo(const T&) { std::cout << "value" << '\n'; }

template <typename T>
void Foo(Wrap<std::shared_ptr<T>>) { std::cout << "shptr" << '\n'; }

struct X{};

int main() {
  Foo<X>({});
  Foo<X>(X());
  Foo<X>(std::make_shared<X>());
}
