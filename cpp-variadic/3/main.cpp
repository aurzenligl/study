#include "algorithm_pack.hpp"
#include <iostream>
#include <vector>

struct Printer
{
    template <typename T>
    void operator()(const T& t)
    {
        std::cout << t << '\n';
    }
};

void onlyInt(int x)
{}

int main()
{
    std::cout
        << "count arithmetic: "
        << meta_count_if<std::is_arithmetic, int, const char*, unsigned, std::string, float>::value << '\n'
        << "count literal: "
        << meta_count_if<std::is_literal_type, int, const char*, unsigned, std::string, float>::value << '\n';

    for_args_if<std::is_arithmetic>(Printer(), 1, "foo", 2u, std::string("bar"), 3.f);
    for_args_if<std::is_literal_type>(Printer(), 1, "foo", 2u, std::string("bar"), 3.f);

    for_args_if<std::is_arithmetic>(onlyInt, 1, "foo");

    std::cout
        << "std::vector<int>: "
        << is_specialization<std::vector<int>, std::vector>::value << '\n'
        << "int: "
        << is_specialization<int, std::vector>::value << '\n'
        << "pair<int, int>: "
        << is_specialization<std::pair<int, int>, std::pair>::value << '\n';
}
