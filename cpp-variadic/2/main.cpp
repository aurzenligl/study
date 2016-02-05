#include <dlfcn.h>
#include <link.h>
#include <vector>
#include <cstdio>

namespace detail
{

typedef std::vector<const char*> shared_names_t;

shared_names_t get_shared_names()
{
    auto cb = [](dl_phdr_info* info, size_t, void* arg)
    {
        shared_names_t& names = *static_cast<shared_names_t*>(arg);
        if (*info->dlpi_name)
        {
            names.push_back(info->dlpi_name);
        }
        return 0;
    };

    shared_names_t names;
    dl_iterate_phdr(cb, &names);
    return names;
}

}  // namespace detail

template <typename ...Args>
void multicall(const char* symbol_name, Args... args)
{
    for (const char* shared_name : detail::get_shared_names())
    {
        void* obj = dlopen(shared_name, RTLD_LAZY);
        if (void* sym = dlsym(obj, symbol_name))
        {
            typedef void (*fun_t)(Args...);
            ((fun_t)sym)(args...);
        }
    }
}

int main()
{
    multicall("lib_global_init");
    multicall("lib_local_init", 42);
    multicall("lib_nano_init", 12.34, 56.78f, 90, &printf);
    return 0;
}
