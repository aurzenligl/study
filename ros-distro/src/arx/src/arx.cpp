#include <vector>
#include <string>
#include <bar/bar.h>

int main(int argc, char** argv) {
    std::vector<std::string> args(argv + 1, argv + argc);
    std::vector<int> numbers;
    for (auto x : args) {
        numbers.push_back(std::stoi(x));
    }
    printf("%d\n", bar::process(numbers));
    return 0;
}
