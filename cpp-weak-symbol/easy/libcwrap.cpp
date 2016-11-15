#include "libcwrap.hpp"
#include <stdlib.h>

namespace libcwrap
{

int system(const char* cmd)
{
    return ::system(cmd);
}

}  // namespace libcwrap
