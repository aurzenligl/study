#include <foo/bar.hpp>

#include <set>

std::string bar(std::string x, std::string y) {
    std::set<std::string> m{x, y, x + y};
    std::string cat;
    for (auto x : m) {
        cat += x;
    }
    return cat;
}
