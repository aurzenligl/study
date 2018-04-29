#include "algo.hpp"
#include "app.hpp"

struct FooTest : public TestAlgo<FooTest>
{
    int input;
    int expected;
};

TEST_ALGO(FooTest, test, "foo/vectors/foo-*")
{
    EXPECT_EQ(app::square(input), expected);
}
