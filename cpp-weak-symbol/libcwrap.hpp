#ifndef LIBCWRAP_HPP_
#define LIBCWRAP_HPP_

// libcwrap::* symbols have default implementations
// which can be substituted at run-time by dynamic libraries
// providing libcwrap_* symbols

namespace libcwrap
{

int system(const char* cmd);

}  // namespace libcwrap

extern "C"
{

// may be delivered by some dynamic library
int libcwrap_system(const char* cmd) __attribute__((weak));

}

#endif  // LIBCWRAP_HPP_
