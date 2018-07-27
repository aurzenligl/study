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

struct B { virtual void f() = 0; };
struct X : B { void f() override {} };

int main() {
  Foo<X>({});  // this works nicely, shared_ptr overload is a worse match, as it should!
  Foo<X>(X());
  Foo<X>(std::make_shared<X>());

  // Foo<B>(std::make_shared<X>());  // but this doesn't work!
}
