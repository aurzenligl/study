#include <string>

#define DLL_PUBLIC __attribute__ ((visibility ("default")))

struct X
{
//    const char* x = "0123456789012";
//    const char* x;
    std::string x = "0123456789012";
//    std::string x;
};

//X x[50000000];
X x[1000000];

template <int N>
void fill(X (&xs)[N])
{
    for (X& x : xs)
    {
        x.x = "a";
    }
}

DLL_PUBLIC void fun()
{
    fill(x);
}

DLL_PUBLIC void gun()
{
}

DLL_PUBLIC void hun()
{
}
