#include <stdio.h>

namespace libcwrap
{

int system(const char* cmd)
{
    printf("library implementation of system, called system with command: %s\n", cmd);
    return 0;
}

}
