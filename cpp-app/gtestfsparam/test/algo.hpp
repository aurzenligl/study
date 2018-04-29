#ifndef CPP_APP_GTESTFSPARAM_TEST_ALGO_HPP_
#define CPP_APP_GTESTFSPARAM_TEST_ALGO_HPP_

#include <gtest/gtest.h>
#include "fsutils.hpp"

void read_vector(const std::string& path, int* input, int* expected);

template <typename Fixture>
struct test_algo : public testing::TestWithParam<std::string>
{
    test_algo()
    {
        Fixture& fixture = *static_cast<Fixture*>(this);
        read_vector(test_algo::GetParam(), &fixture.input, &fixture.expected);
    }
};

#define TEST_ALGO(test_case_name, test_name, vector_pattern) \
    INSTANTIATE_TEST_CASE_P(, \
                            test_case_name, \
                            testing::ValuesIn(glob_test_dir(vector_pattern))); \
    TEST_P(test_case_name, test_name)

#endif /* CPP_APP_GTESTFSPARAM_TEST_ALGO_HPP_ */
