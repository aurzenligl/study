#include <cstdio>
#include <chrono>
#include <random>
#include <algorithm>
#include <vector>

struct timer
{
    timer(): timer("")
    {}
    timer(const char* fmt): fmt(fmt)
    {
        t0 = std::chrono::high_resolution_clock::now();
    }

    ~timer()
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        int t = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
        printf("%s: %d[usec] \n", fmt, t);
    }

    const char* fmt;
    std::chrono::system_clock::time_point t0;
};

enum { NUMBERS = 10 };
enum { LOOPS = 1000 * 1000 / NUMBERS };

static void clobber()
{
    asm volatile("" : : : "memory");
}

struct randint
{
    randint(int min, int max): dist(min, max) {}
    int operator()()
    {
        return dist(re);
    }
    std::random_device re;
    std::uniform_int_distribution<> dist;
};

template <typename C>
void randomize(C& c, int min, int max)
{
    randint r(min, max);
    std::generate(c.begin(), c.end(), [&] () { return r(); } );
}

template <class Div>
void __attribute__ ((noinline)) critical_loop(int& res, std::vector<int>& xs, std::vector<int>& ys)
{
    for (int j=0; j<LOOPS; j++)
    {
        for (int i=0; i<NUMBERS; i++)
        {
            res += Div::div(xs[i], ys[i]);
            clobber();
        }
    }
}

template <class Div>
void test(std::vector<int>& xs, std::vector<int>& ys)
{
    int res = 0;
    {
        timer _(Div::name);
        critical_loop<Div>(res, xs, ys);
    }
    printf("result: %d\n", res);
}

struct warm_up
{
    static constexpr const char* name = "warm_up";
    static int div(int x, int y)
    {
        return x + y;
    }
};

struct div_operator
{
    static constexpr const char* name = "div_operator";
    static int div(int x, int y)
    {
        return x / y;
    }
};

struct div_switchocasilla
{
    static constexpr const char* name = "div_switchocasilla";
    static int div(int x, int y)
    {
        switch (y)
        {
            case 1: return x / 1;
            case 2: return x / 2;
            case 3: return x / 3;
            case 4: return x / 4;
            case 5: return x / 5;
            case 6: return x / 6;
            case 7: return x / 7;
            case 8: return x / 8;
            case 9: return x / 9;
            case 10: return x / 10;
            case 11: return x / 11;
            case 12: return x / 12;
            case 13: return x / 13;
            case 14: return x / 14;
            case 15: return x / 15;
            default: return x / y;
        }
    }
};

struct div_binsearch
{
    static constexpr const char* name = "div_binsearch";
    static int div(int x, int y)
    {
        if (y < 16)
            if (y < 8)
                if (y < 4)
                    if (y < 2)
                        return x / 1;
                    else
                        if (y < 3) return x / 2; else return x / 3;
                else
                    if (y < 6)
                        if (y < 5) return x / 4; else return x / 5;
                    else
                        if (y < 7) return x / 6; else return x / 7;
            else
                if (y < 12)
                    if (y < 10)
                        if (y < 9) return x / 8; else return x / 9;
                    else
                        if (y < 11) return x / 10; else return x / 11;
                else
                    if (y < 14)
                        if (y < 13) return x / 12; else return x / 13;
                    else
                        if (y < 15) return x / 14; else return x / 15;
        else
            return x / y;
    }
};

struct div_virtual
{
    struct dividor
    {
        virtual int div(int x) const = 0;
    };

    template <int N>
    struct dividor_impl : public dividor
    {
        virtual int div(int x) const
        {
            return x / N;
        }
        static dividor_impl inst;
    };

    static const dividor* divs[];

    static constexpr const char* name = "div_virtual";
    static int div(int x, int y)
    {
        if (y < 16)
        {
            return divs[y]->div(x);
        }
        return x / y;
    }
};

template <int N>
div_virtual::dividor_impl<N> div_virtual::dividor_impl<N>::inst;

const div_virtual::dividor* div_virtual::divs[] = {
    &dividor_impl<0>::inst,
    &dividor_impl<1>::inst,
    &dividor_impl<2>::inst,
    &dividor_impl<3>::inst,
    &dividor_impl<4>::inst,
    &dividor_impl<5>::inst,
    &dividor_impl<6>::inst,
    &dividor_impl<7>::inst,
    &dividor_impl<8>::inst,
    &dividor_impl<9>::inst,
    &dividor_impl<10>::inst,
    &dividor_impl<11>::inst,
    &dividor_impl<12>::inst,
    &dividor_impl<13>::inst,
    &dividor_impl<14>::inst,
    &dividor_impl<15>::inst
};

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

