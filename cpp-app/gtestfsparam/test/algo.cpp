#include "algo.hpp"
#include <fstream>

void read_vector(const std::string& path, int* input, int* expected)
{
    std::fstream f(path);
    if (!f)
    {
        throw std::runtime_error("file \'" + path + "\' not found");
    }

    f >> *input >> *expected;
    if (!f)
    {
        throw std::runtime_error("failure while reading \'" + path + "\'");
    }
}
