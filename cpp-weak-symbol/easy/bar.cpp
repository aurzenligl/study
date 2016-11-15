#include <stdio.h>

extern "C"
{

int libcwrap_system(const char* cmd)
{
    printf("library implementation of system, called system with command: %s\n", cmd);
    return 0;
}

}
