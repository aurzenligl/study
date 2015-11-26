struct timer
{
    timer(const char* fmt): fmt(fmt)
    {
        t0 = std::chrono::high_resolution_clock::now();
    }

    ~timer()
    {
        using namespace std::chrono;
        auto t1 = high_resolution_clock::now();
        int t = duration_cast<microseconds>(t1-t0).count();
        printf("%s: %d[usec] \n", fmt, t);
    }

    const char* fmt;
    std::chrono::system_clock::time_point t0;
};

enum { LOOPS = 1000 * 1000 / NUMBERS };

template <class Div>
void test(std::vector<int>& xs, std::vector<int>& ys)
{
    int res = 0;
    {
        timer _(Div::name);
        critical_loop<Div, LOOPS>(res, xs, ys);
    }
    printf("result: %d\n", res);
}

int main()
{
    std::vector<int> xs(NUMBERS);
    std::vector<int> ys(NUMBERS);
    randomize(xs, 1*1000*1000*1000, 2*1000*1000*1000);
    randomize(ys, 1, 15);

    test<warm_up>(xs, ys);
    test<div_operator>(xs, ys);
    test<div_switchocasilla>(xs, ys);
    test<div_binsearch>(xs, ys);
    test<div_virtual>(xs, ys);

    return 0;
}
