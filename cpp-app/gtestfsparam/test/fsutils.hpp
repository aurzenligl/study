#ifndef CPP_APP_GTESTFSPARAM_TEST_FSUTILS_HPP_
#define CPP_APP_GTESTFSPARAM_TEST_FSUTILS_HPP_

#include <string>
#include <vector>

std::string read_link(const std::string& path);
std::string dir_name(const std::string& path);
std::string self_dir();
std::vector<std::string> glob_files(const std::string& pattern);

std::vector<std::string> glob_test_dir(const std::string& pattern);

#endif /* CPP_APP_GTESTFSPARAM_TEST_FSUTILS_HPP_ */
