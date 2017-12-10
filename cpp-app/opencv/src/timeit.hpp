#ifndef CPP_APP_SRC_TIMEIT_HPP_
#define CPP_APP_SRC_TIMEIT_HPP_

#include <chrono>

namespace app
{

struct timeit
{
    timeit(const char* name): name(name)
    {
        t0 = std::chrono::high_resolution_clock::now();
    }

    ~timeit()
    {
        using namespace std::chrono;
        auto t1 = high_resolution_clock::now();
        int t = duration_cast<microseconds>(t1-t0).count();
        printf("%s: %d[usec] \n", name, t);
    }

    const char* name;
    std::chrono::system_clock::time_point t0;
};

}

#endif /* CPP_APP_SRC_TIMEIT_HPP_ */
