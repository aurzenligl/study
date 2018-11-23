#include <string>

namespace tag {
struct Abc {
  using Type = std::string;
};

struct Def {
  using Arg1 = std::string;
  using Type = std::string;
};

struct Ghi {
  using Arg1 = std::string;
  using Arg2 = int;
  using Type = int;
};
}

template <class Tag>
typename Tag::Type Get();

template <class Tag>
void Set(const typename Tag::Type& value);

template <class Tag>
typename Tag::Type Get(const typename Tag::Arg1 &arg);

template <class Tag>
void Set(const typename Tag::Arg1 &arg, const typename Tag::Type& value);

template <class Tag>
typename Tag::Type Get(const typename Tag::Arg1 &arg1, const typename Tag::Arg2 &arg2);

template <class Tag>
void Set(const typename Tag::Arg1 &arg1, const typename Tag::Arg2 &arg2, const typename Tag::Type& value);
