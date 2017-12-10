#ifndef CPP_APP_SRC_ALGO_HPP_
#define CPP_APP_SRC_ALGO_HPP_

#include <opencv2/opencv.hpp>

namespace app
{
namespace algo
{

void show(cv::Mat& image);
void reduce_color_space(cv::Mat& image);
void reduce_color_space_lut(cv::Mat& image);

}
}

#endif /* CPP_APP_SRC_ALGO_HPP_ */
