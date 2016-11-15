#include "libcwrap.hpp"
#include <stdlib.h>

namespace libcwrap
{

int system(const char* cmd)
{
#ifdef USE_LIBCWRAP
    return libcwrap_system(cmd);
#else
    return ::system(cmd);
#endif
}

}  // namespace libcwrap
