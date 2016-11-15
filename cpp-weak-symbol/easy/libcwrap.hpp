#ifndef LIBCWRAP_HPP_
#define LIBCWRAP_HPP_

// libcwrap::* functions have default implementations, but
// if flag USE_LIBCWRAP is used during compilation of this module
// libcwrap::* functions will be implemented by libcwrap_* functions
// and external dynamic library will be expected to ship them
 
namespace libcwrap
{

int system(const char* cmd);

}  // namespace libcwrap

#endif  // LIBCWRAP_HPP_
