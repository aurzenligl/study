#include <iostream>
#include <memory>
#include <type_traits>

template <class T, template <class...> class Template>
struct IsSpecialization : std::false_type {};

template <template <class...> class Template, class... Args>
struct IsSpecialization<Template<Args...>, Template> : std::true_type {};

template <typename MaybeSharedPtr>
struct DisableIfSharedPtr : std::enable_if<!IsSpecialization<MaybeSharedPtr, std::shared_ptr>::value>
{};

template <typename T>
void Foo(const T&) { std::cout << "value" << '\n'; }

template <typename T, typename = typename DisableIfSharedPtr<T>::type>
void Foo(std::shared_ptr<T>) { std::cout << "shptr" << '\n'; }

struct X{};
using XPtr = std::shared_ptr<X>;

int main() {
  Foo<XPtr>({});  // works nicely! blocks shared_ptr<shared_ptr<T>> overload
  Foo<X>({});  // doesn't work at all, since X is not a shared_ptr in the first place
}
