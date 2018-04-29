#include <gtest/gtest.h>
#include <app.hpp>

using namespace testing;

struct pow_t
{
    int input;
    int expected;
};

template <typename Fixture>
struct AlgoTest : public TestWithParam<pow_t>
{
    AlgoTest()
    {
        const pow_t& p = TestWithParam<pow_t>::GetParam();
        static_cast<Fixture*>(this)->input = p.input;
        static_cast<Fixture*>(this)->expected = p.expected;
    }
};

struct FooTest : public AlgoTest<FooTest>
{
    int input;
    int expected;


    // You can implement all the usual fixture class members here.
    // To access the test parameter, call GetParam() from class
    // TestWithParam<T>.
};

TEST_P(FooTest, DoesBlah)
{
    EXPECT_EQ(app::square(input), expected);
}

pow_t data[] =
{
    {1, 1},
    {2, 4},
    {3, 9},
};

INSTANTIATE_TEST_CASE_P(InstantiationName,
                        FooTest,
                        ::testing::ValuesIn(data));
