#include "algo.hpp"
#include "app.hpp"

namespace foo
{

struct foo_test : public test_algo<foo_test>
{
    int input;
    int expected;
};

TEST_ALGO(foo_test, test, "foo/vectors/foo-*")
{
    EXPECT_EQ(app::foo(input), expected);
}

struct foo_big_test : public test_algo<foo_big_test>
{
    int input;
    int expected;
};

TEST_ALGO(foo_big_test, test, "foo/vectors/foo_big-*")
{
    EXPECT_EQ(app::foo(input), expected);
}

struct bar_test : public test_algo<bar_test>
{
    int input;
    int expected;
};

TEST_ALGO(bar_test, test, "foo/vectors/bar-*")
{
    EXPECT_EQ(app::bar(input), expected);
}

}
