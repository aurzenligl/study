#ifndef CPP_APP_GTESTFSPARAM_TEST_FSUTILS_HPP_
#define CPP_APP_GTESTFSPARAM_TEST_FSUTILS_HPP_

#include <unistd.h>
#include <libgen.h>
#include <glob.h>
#include <string>
#include <vector>

std::string read_link(const char* path)
{
    enum { n_buf = 1024 };
    char buf[n_buf];
    ssize_t len = readlink(path, buf, n_buf - 1);
    if (len == -1)
    {
        return {};
    }
    buf[len] = '\0';
    return std::string(buf);
}

std::string dir_name(const std::string& path)
{
    std::string copy(path);
    char* c_str = const_cast<char*>(copy.c_str());
    return std::string(dirname(c_str));
}

std::string self_dir()
{
    return dir_name(read_link("/proc/self/exe"));
}

std::vector<std::string> glob_files(const std::string& pattern)
{
    glob_t gout;
    int ec = glob(pattern.c_str(), 0, NULL, &gout);
    if (ec)
    {
        throw std::runtime_error("globbing pattern \'" + pattern + "\' failed");
    }
    std::vector<std::string> out;
    while (*gout.gl_pathv)
    {
        out.emplace_back(*gout.gl_pathv++);
    }
    return out;
}

const std::string c_test_dir = dir_name(self_dir()) + "/test/";

std::vector<std::string> glob_test_dir(const std::string& pattern)
{
    const std::string test_pattern = c_test_dir + pattern;

    glob_t gout;
    int ec = glob(test_pattern.c_str(), 0, NULL, &gout);
    if (ec)
    {
        throw std::runtime_error("globbing pattern \'" + test_pattern + "\' failed");
    }
    std::vector<std::string> out;
    while (*gout.gl_pathv)
    {
        out.emplace_back(*gout.gl_pathv++);
    }
    return out;
}

#endif /* CPP_APP_GTESTFSPARAM_TEST_FSUTILS_HPP_ */
