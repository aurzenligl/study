#include <gtest/gtest.h>
#include <app.hpp>
#include <string>
#include <fstream>

#include <unistd.h>
#include <libgen.h>

using namespace testing;

std::string read_link(const char* path)
{
    enum { n_buf = 1024 };
    char buf[n_buf];
    ssize_t len = readlink(path, buf, n_buf - 1);
    if (len == -1)
    {
        return {};
    }
    buf[len] = '\0';
    return std::string(buf);
}

std::string dir_name(const std::string& path)
{
    return std::string(dirname(const_cast<char*>(path.c_str())));
}

std::string self_dir()
{
    return dir_name(read_link("/proc/self/exe"));
}

std::string test_dir()
{
    return dir_name(self_dir()) + "/test/";
}

const std::string c_test_dir = test_dir();

template <typename Fixture>
struct AlgoTest : public TestWithParam<std::string>
{
    AlgoTest()
    {
        const std::string& path = c_test_dir + AlgoTest::GetParam();
        std::fstream f(path);
        if (!f)
        {
            throw std::runtime_error("file \'" + path + "\' not found");
        }
        f >> derived().input >> derived().expected;
    }

private:
    Fixture& derived()
    {
        return *static_cast<Fixture*>(this);
    }
};

struct FooTest : public AlgoTest<FooTest>
{
    int input;
    int expected;
};

TEST_P(FooTest, test)
{
    EXPECT_EQ(app::square(input), expected);
}

INSTANTIATE_TEST_CASE_P(InstantiationName,
                        FooTest,
                        ::testing::Values("foo/vectors/foo-one.txt",
                                          "foo/vectors/foo-two.txt",
                                          "foo/vectors/foo-three.txt"));
