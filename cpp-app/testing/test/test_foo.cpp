#include <gtest/gtest.h>
#include "foo.hpp"

using namespace testing;

TEST(FooTest, foo_null)
{
    EXPECT_EQ(0, app::foo(0));
}

TEST(FooTest, foo_42)
{
    EXPECT_EQ(42, app::foo(42));
}
