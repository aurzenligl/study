#include <memory>
#include <iostream>
#include <string>

// metaprogramming utility

template<typename... Args>
class args_container
{};

template<typename _Functor>
class args_of;

template<typename Ret, typename... _ArgTypes>
struct args_of<Ret(*)(_ArgTypes...)>
{
    typedef args_container<_ArgTypes...> type;
};

// adapter utility

template <typename Handler, typename ArgsContainer>
struct Adapter;

template <typename Handler, typename... Args>
struct Adapter<Handler, args_container<Args...>>
{
    explicit Adapter(Handler h) : h(h)
    {}

    void operator()(std::string x, Args&&... args)
    {
        std::cout << "pre " << x << '\n';
        h(std::forward<Args>(args)...);
        std::cout << "post " << x << '\n';
    }

    Handler h;
};

template <typename Handler>
Adapter<Handler, typename args_of<Handler>::type> adapter(Handler handler)
{
    return Adapter<Handler, typename args_of<Handler>::type>(handler);
}

// library code

struct X
{
    int x;
};

void bar(std::function<void(std::string, int, double, const X&)> f)
{
    f("blawatek", 1, 2, {3});
}

void bar(std::function<void(std::string, int, int)> f)
{
    f("hortensja", 1, 2);
}

// library proxy

template <typename Handler>
void bar_proxy(Handler handler)
{
    bar(adapter(handler));
}

// user handlers

void fun(int x, double y, const X& z)
{
    std::cout << x << ' ' << y << ' ' << z.x << '\n';
}

void gun(int x, int y)
{
    std::cout << x << ' ' << y << '\n';
}

int main()
{
    bar_proxy(fun);
    bar_proxy(gun);
}
