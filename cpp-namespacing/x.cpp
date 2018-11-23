#include "x.h"

template<> std::string Get<tag::Abc>() {
    return "";
}

template<> void Set<tag::Abc>(const std::string &x) {
}

template<> std::string Get<tag::Def>(const std::string &arg) {
    return "";
}

template<> void Set<tag::Def>(const std::string &arg, const std::string &x) {
}

template<> int Get<tag::Ghi>(const std::string &arg1, const int &arg2) {
    return 42;
}

template<> void Set<tag::Ghi>(const std::string &arg1, const int &arg2, const int &x) {
}
