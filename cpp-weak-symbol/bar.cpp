#include <stdio.h>

extern "C"
{

#ifndef NO_OVERRIDE
int libcwrap_system(const char* cmd)
{
    printf("library implementation of system, called system with command: %s\n", cmd);
    return 0;
}
#endif

}
