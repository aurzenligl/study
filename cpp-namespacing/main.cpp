#include <stdio.h>
#include "x.h"

int main() {
    // use ADL, Luke! :) https://en.cppreference.com/w/cpp/language/adl
    printf("GetInt %d\n", GetInt(some::nested::ns::One));
    printf("GetInt %d\n", GetInt(some::nested::ns::Two));
    SetInt(some::nested::ns::Two, 20);
    printf("GetInt %d\n", GetInt(some::nested::ns::Two));
}
