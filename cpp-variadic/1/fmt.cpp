#include <stdio.h>
#include "fmt.hpp"

template <typename ...Ts>
void print(Ts... values)
{
    printf(fmt<Ts...>::value, values...);
}

int main()
{
    print(0);
    print(1, 2, 3);
    print(10, 42, "hello", 192.32);
    return 0;
}
