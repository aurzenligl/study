#include <stdint.h>
#include <stdio.h>
#if __cplusplus >= 201103L
    #include <type_traits>
#endif

class Padding16
{
    uint16_t x;
};

struct X
{
    uint16_t x;
    Padding16 _padding0;
    uint32_t y;
};

template <typename T>
void print(const char* msg, const T& t)
{
    printf("%s: ", msg);
    for (size_t i = 0; i < sizeof(T); i++)
    {
        printf("%02X", ((unsigned char*)&t)[i]);
    }
    printf("\n");
}

int main()
{
    X x;
    x.x = 1;
    x.y = 2;

#if __cplusplus >= 201103L
    printf("is_pod<X>: %d\n", std::is_pod<X>::value);
#endif

    print("bytes: ", x);

#ifdef ERRORS
    X error1 = {1, 2};
    X error2{1, 2};
#endif
}

