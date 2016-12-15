#include <stdint.h>
#include <stdio.h>
#if __cplusplus >= 201103L
    #include <type_traits>
#endif

#define PROPHY_STRUCT(N) struct __attribute__((__aligned__(N), __packed__))

PROPHY_STRUCT(2) Padding16_RetainsPodInCpp11
{
    uint16_t x;
};

PROPHY_STRUCT(2) Padding16 : public Padding16_RetainsPodInCpp11
{
};

PROPHY_STRUCT(4) X
{
    uint16_t x;
    Padding16 _padding0;
    uint32_t y;
};

template <typename T>
void print(const char* msg, const T& t)
{
    printf("%s", msg);
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
    (void) error1;
#if __cplusplus >= 201103L
    X error2{1, 2};
    (void) error2;
#endif
#endif
}

