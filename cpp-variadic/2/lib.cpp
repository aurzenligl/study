#include <cstdio>

const char magic[] = MAGIC;

extern "C"
{

#pragma GCC visibility push(default)

void lib_global_init()
{
    printf("%s:%s\n", magic, __FUNCTION__);
}

void lib_local_init(int x)
{
    printf("%s:%s %d\n", magic, __FUNCTION__, x);
}

void lib_nano_init(double x, float y, int z, void* p)
{
    printf("%s:%s %f %f %d %p\n", magic, __FUNCTION__, x, y, z, p);
}

#pragma GCC visibility pop

}

struct global_object_t
{
    global_object_t()
    {
        printf("%s:%s\n", magic, __FUNCTION__);
    }
    ~global_object_t()
    {
        printf("%s:%s\n", magic, __FUNCTION__);
    }
}
global_object;
