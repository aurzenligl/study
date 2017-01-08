#include <iostream>

#include "messages.hpp"

int main()
{
    Bar x;
    x.y.foo = 987;
    x.x = 42;
    x.z = {1, 2, 3};
    std::cout << x.print();

    std::vector<uint8_t> data(1000);
    size_t a = x.encode(data.data());
    printf("size: %d \n", int(a));
}
