#ifndef CPP_APP_SRC_OPTS_HPP_
#define CPP_APP_SRC_OPTS_HPP_

#include <string>
#include <boost/optional.hpp>

namespace app
{

struct cmdline_opts
{
    std::string filename;
    int operation;
};

struct cmdline_status
{
    cmdline_status()
    {}

    cmdline_status(int return_code): exit(true), return_code(return_code)
    {}

    bool exit = false;
    int return_code = 0;

    explicit operator bool()
    {
        return exit;
    }
};

cmdline_status parse(int ac, char* av[], cmdline_opts& opts);

}

#endif /* CPP_APP_SRC_OPTS_HPP_ */
