#include <iostream>
#include <memory>
#include <type_traits>

template <typename From, typename To>
struct EnableIfSharedPointersConvert : std::enable_if<std::is_convertible<std::shared_ptr<From>, std::shared_ptr<To>>::value>
{};

template <typename T>
void Foo(const T& = T()) { std::cout << "value" << '\n'; }

template <typename T, typename MaybeT, typename = typename EnableIfSharedPointersConvert<MaybeT, T>::type>
void Foo(std::shared_ptr<MaybeT> m) { std::shared_ptr<T> t(std::move(m)); std::cout << "shptr" << '\n'; }

struct B { virtual void f() = 0; };
struct X : B { X(int = 0) {} void f() override {} };
using XPtr = std::shared_ptr<X>;

int main() {
  X ref;
  XPtr shref = std::make_shared<X>();
  const X &cref = X();
  const XPtr& shcref = std::make_shared<X>();

  // finally! all use cases seem to work.

  Foo<X>();  // value
  Foo<X>({});  // value
  Foo<X>(1);  // value
  Foo<X>(X());  // value
  Foo<X>(ref);  // value
  Foo<X>(cref);  // value
  Foo<X>(std::make_shared<X>());  // shared
  Foo<X>(shref);  // shared
  Foo<X>(shcref);  // shared

  Foo<B>(std::make_shared<X>());  // shared
  Foo<B>(shref);  // shared
  Foo<B>(shcref);  // shared  

  Foo<XPtr>();  // value
  Foo<XPtr>({});  // value
  Foo<XPtr>(std::make_shared<X>());  // value
  Foo<XPtr>(XPtr());  // value
  Foo<XPtr>(shref);  // value
  Foo<XPtr>(shcref);  // value
}
