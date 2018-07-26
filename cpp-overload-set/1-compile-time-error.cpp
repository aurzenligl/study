#include <iostream>
#include <memory>

template <typename T>
void Foo(const T&) { std::cout << "value" << '\n'; }

template <typename T, typename SharedPtrT = std::shared_ptr<T>>
void Foo(SharedPtrT) { std::cout << "shptr" << '\n'; }

struct X{};

int main() {
  Foo<X>({});  // call of overloaded ambiguous
  Foo<X>(X());  // call of overloaded ambiguous
  Foo<X>(std::make_shared<X>());
}
