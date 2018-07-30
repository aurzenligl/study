#include <foo/foo.h>
#include <string>

int main(int argc, char** argv) {
    if (argc > 1) {
        printf("%d\n", foo::square(std::stoi(argv[1])));
        return 0;
    }
    return 1;
}
