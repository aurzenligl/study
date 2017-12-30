#include <benchmark/benchmark.h>
#include <app.hpp>

struct provider : public benchmark::Fixture
{
};

BENCHMARK_F(provider, bar_null)(benchmark::State& st)
{
    while (st.KeepRunning())
    {
        app::bar(0);
    }
}

BENCHMARK_F(provider, bar_42)(benchmark::State& st)
{
    while (st.KeepRunning())
    {
        app::bar(42);
    }
}

BENCHMARK_MAIN();
