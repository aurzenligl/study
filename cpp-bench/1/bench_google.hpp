#include <benchmark/benchmark.h>

template <class Div>
void test(std::vector<int>& xs, std::vector<int>& ys)
{
    int res = 0;
    critical_loop<Div, 1>(res, xs, ys);
}

struct test_data
{
    test_data() : xs(NUMBERS), ys(NUMBERS)
    {
        randomize(xs, 1*1000*1000*1000, 2*1000*1000*1000);
        randomize(ys, 1, 15);
    }

    std::vector<int> xs;
    std::vector<int> ys;
};

static test_data data;

struct provider : public benchmark::Fixture
{
    test_data& d {data};
};

BENCHMARK_F(provider, bmf_warm_up)(benchmark::State& st)
{
    while (st.KeepRunning())
    {
        test<warm_up>(d.xs, d.ys);
    }
}

BENCHMARK_F(provider, bmf_div_operator)(benchmark::State& st)
{
    while (st.KeepRunning())
    {
        test<div_operator>(d.xs, d.ys);
    }
}

BENCHMARK_F(provider, bmf_div_switchocasilla)(benchmark::State& st)
{
    while (st.KeepRunning())
    {
        test<div_switchocasilla>(d.xs, d.ys);
    }
}

BENCHMARK_F(provider, bmf_div_binsearch)(benchmark::State& st)
{
    while (st.KeepRunning())
    {
        test<div_binsearch>(d.xs, d.ys);
    }
}

BENCHMARK_F(provider, bmf_div_virtual)(benchmark::State& st)
{
    while (st.KeepRunning())
    {
        test<div_virtual>(d.xs, d.ys);
    }
}

BENCHMARK_MAIN();
