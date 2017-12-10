#ifndef CPP_APP_SRC_PROCESS_HPP_
#define CPP_APP_SRC_PROCESS_HPP_

#include <string>

namespace app
{

bool show_image(const std::string& filepath);

bool convert_image(const std::string& filepath);

bool reduce_color_space(const std::string& filepath);

bool sharpen(const std::string& filepath);

}

#endif /* CPP_APP_SRC_PROCESS_HPP_ */
