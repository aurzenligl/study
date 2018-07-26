#include <iostream>
#include <memory>

template <typename T>
void Foo(const T&) { std::cout << "value" << '\n'; }

template <typename T>
void Foo(std::shared_ptr<T>) { std::cout << "shptr" << '\n'; }

struct X{};

int main() {
  Foo<X>({});  // this instantiates a shared_ptr! I'd prefer a value
  Foo<X>(X());
  Foo<X>(std::make_shared<X>());
}
