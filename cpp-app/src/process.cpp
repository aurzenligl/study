#include <opencv2/opencv.hpp>
#include "algo.hpp"
#include "timeit.hpp"
#include "process.hpp"

namespace app
{

bool show_image(const std::string& filepath)
{
    cv::Mat image = cv::imread(filepath);
    if (!image.data)
    {
        return false;
    }
    algo::show(image);
    return true;
}

bool convert_image(const std::string& filepath)
{
    cv::Mat image = cv::imread(filepath, cv::IMREAD_COLOR);
    if (!image.data)
    {
        return false;
    }
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    cv::imwrite('_' + filepath, gray_image);
    algo::show(gray_image);
    return true;
}

bool reduce_color_space(const std::string& filepath)
{
    cv::Mat image = cv::imread(filepath, cv::IMREAD_COLOR);
    if (!image.data)
    {
        return false;
    }

    {
        cv::Mat cloned = image.clone();
        timeit _("manual lut");
        algo::reduce_color_space(cloned);
    }

    {
        cv::Mat cloned = image.clone();
        timeit _("opencv lut");
        algo::reduce_color_space_lut(cloned);
    }

    algo::reduce_color_space(image);
    algo::show(image);
    return true;
}

}
