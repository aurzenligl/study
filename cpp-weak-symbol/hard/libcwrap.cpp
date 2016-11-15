#include "libcwrap.hpp"
#include <stdlib.h>

namespace libcwrap
{

int system(const char* cmd)
{
    if (libcwrap_system)
    {
        return libcwrap_system(cmd);
    }
    return ::system(cmd);
}

}  // namespace libcwrap
