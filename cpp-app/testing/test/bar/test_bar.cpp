#include <gtest/gtest.h>
#include <app.hpp>

using namespace testing;

TEST(BarTest, foo_null)
{
    EXPECT_EQ(2, app::bar(0));
}

TEST(BarTest, foo_42)
{
    EXPECT_EQ(44, app::bar(42));
}
