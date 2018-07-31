#include <bar/bar.h>

#include <vector>
#include <foo/foo.h>

namespace bar {

int process(const std::vector<int>& x) {
    int result = 0;
    for (auto y : x) {
        result += foo::square(y);
    }
    return result;
}

}  // namespace bar
