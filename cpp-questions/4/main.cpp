#include <iostream>
#include <stdexcept>

struct X
{
    std::string x;
    X(std::string xx): x(xx){}
    ~X()
    {
        throw std::runtime_error("X error: " + x);
    }
};

void f()
{
    X x("a");
    X y("b");
}

int main()
{
    try { f(); } catch (...) {}
    return 0;
} 
