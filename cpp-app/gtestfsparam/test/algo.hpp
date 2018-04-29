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

#endif /* CPP_APP_GTESTFSPARAM_TEST_ALGO_HPP_ */
