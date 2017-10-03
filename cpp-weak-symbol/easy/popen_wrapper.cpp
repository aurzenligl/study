#include <cstdio>

extern "C" {

FILE* __real_popen(const char *command, const char *type);

FILE* __wrap_popen(const char *command, const char *type)
{
    printf ("DEBUG: popen called with for %s\n", command);
    return __real_popen(command, type);
}

}
