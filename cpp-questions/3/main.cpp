#include <iostream>

void f(double)
{
    std::cout << "double\n";
}

void f(float)
{
    std::cout << "float\n";
}

int main()
{
    f(.0);
    void f(float);
    f(.0);
    void f(double);
    f(.0);
    void f(float);
    f(.0);
}
