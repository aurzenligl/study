#include <iostream>
#include <memory>
#include <type_traits>

template <typename T, typename MaybeT>
struct EnableIfMatch : std::enable_if<std::is_convertible<MaybeT, T>::value, int>
{};

template <typename T, typename MaybeSharedPtrT>
struct EnableIfMatchSharedPtr : std::enable_if<std::is_convertible<MaybeSharedPtrT, std::shared_ptr<T>>::value, int>
{};

template <typename T, typename U, typename = typename EnableIfMatch<T, U>::type>
void FooImpl(U&& u) { std::cout << "value" << '\n'; }

template <typename T, typename U, typename _ = typename EnableIfMatchSharedPtr<T, U>::type>
void FooImpl(U&& u, _ = _()) { std::cout << "shared" << '\n'; }

template <typename T, typename U = T>
void Foo(U&& u) {
  FooImpl<T>(std::forward<U>(u));
}

struct X{};
using XPtr = std::shared_ptr<X>;

int main() {
  X ref;
  XPtr shref = std::make_shared<X>();
  const X &cref = X();
  const XPtr& shcref = std::make_shared<X>();

  Foo<X>({});  // value
  Foo<X>(X());  // value
  Foo<X>(ref);  // value
  Foo<X>(cref);  // value
  Foo<X>(std::make_shared<X>());  // shared
  Foo<X>(shref);  // shared
  Foo<X>(shcref);  // shared

  Foo<XPtr>({});  // value
  Foo<XPtr>(std::make_shared<X>());  // value
  Foo<XPtr>(XPtr());  // value
  Foo<XPtr>(shref);  // value
  Foo<XPtr>(shcref);  // value
}
