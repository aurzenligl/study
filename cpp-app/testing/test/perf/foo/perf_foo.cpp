#include <benchmark/benchmark.h>
#include <app.hpp>

struct provider : public benchmark::Fixture
{
};

BENCHMARK_F(provider, foo_null)(benchmark::State& st)
{
    while (st.KeepRunning())
    {
        app::foo(0);
    }
}

BENCHMARK_F(provider, foo_42)(benchmark::State& st)
{
    while (st.KeepRunning())
    {
        app::foo(42);
    }
}

BENCHMARK_MAIN();
