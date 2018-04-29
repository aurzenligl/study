#ifndef CPP_APP_GTESTFSPARAM_TEST_ALGO_HPP_
#define CPP_APP_GTESTFSPARAM_TEST_ALGO_HPP_

#include <gtest/gtest.h>
#include <fstream>
#include "fsutils.hpp"

template <typename Fixture>
struct TestAlgo : public testing::TestWithParam<std::string>
{
    TestAlgo()
    {
        const std::string& path = TestAlgo::GetParam();
        std::fstream f(path);
        if (!f)
        {
            throw std::runtime_error("file \'" + path + "\' not found");
        }

        Fixture& fixture = *static_cast<Fixture*>(this);
        f >> fixture.input >> fixture.expected;
    }
};

#define TEST_ALGO(test_case_name, test_name, vector_pattern) \
    INSTANTIATE_TEST_CASE_P(, \
                            test_case_name, \
                            testing::ValuesIn(glob_test_dir(vector_pattern))); \
    TEST_P(test_case_name, test_name)

#endif /* CPP_APP_GTESTFSPARAM_TEST_ALGO_HPP_ */
