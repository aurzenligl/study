#include "algo.hpp"
#include "app.hpp"

struct FooTest : public TestAlgo<FooTest>
{
    int input;
    int expected;
};

TEST_P(FooTest, test)
{
    EXPECT_EQ(app::square(input), expected);
}

INSTANTIATE_TEST_CASE_P(, FooTest, testing::ValuesIn(glob_test_dir("foo/vectors/foo-*")));
