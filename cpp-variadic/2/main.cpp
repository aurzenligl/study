#include <dlfcn.h>
#include <link.h>
#include <vector>
#include <cstdio>

namespace detail
{

typedef std::vector<const char*> names_t;

names_t get_shared_names()
{
    auto cb = [](dl_phdr_info* info, size_t, void* arg)
    {
        if (*info->dlpi_name)
        {
            static_cast<names_t*>(arg)->push_back(info->dlpi_name);
        }
        return 0;
    };

    names_t names;
    dl_iterate_phdr(cb, &names);
    return names;
}

}  // namespace detail

template <typename FunPtr, typename ...Args>
void multicall(const char* symbol_name, Args... args)
{
    for (const char* shared_name : detail::get_shared_names())
    {
        void* obj = dlopen(shared_name, RTLD_LAZY);
        if (void* sym = dlsym(obj, symbol_name))
        {
            ((FunPtr)sym)(args...);
        }
    }
}

extern "C" typedef void (*lib_global_init_t)();
extern "C" typedef void (*lib_local_init_t)(int x);
extern "C" typedef void (*lib_nano_init_t)(double x, float y, int z, void* p);

int main()
{
    multicall<lib_global_init_t>("lib_global_init");
    multicall<lib_local_init_t>("lib_local_init", 42);
    multicall<lib_nano_init_t>("lib_nano_init", 12.34, 56.78f, 90, nullptr);
    return 0;
}
