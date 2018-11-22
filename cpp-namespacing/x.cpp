#include "x.h"
#include <string>
#include <stdexcept>
#include <unordered_map>

static std::unordered_map<std::string, int> vars{
    {"One_some_suffix", 1},
    {"Two_some_suffix", 2},
};

namespace some {
namespace nested {
namespace ns {

std::string KindStr(Kind kind)
{
    if (kind == One) return "One";
    if (kind == Two) return "Two";
    throw std::runtime_error("unknown kind");
}

int GetInt(Kind kind) {
    return vars[KindStr(kind) + "_some_suffix"];
}

void SetInt(Kind kind, int x) {
    vars[KindStr(kind) + "_some_suffix"] = x;
}

}
}
}
