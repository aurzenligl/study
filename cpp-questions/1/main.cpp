#include <iostream>

void f(int x)
{
    std::cout << x << '\n';
}

void f(int = 2);

int main()
{
    f(1);
    f();
    void f(int = 3);
    f();
}
