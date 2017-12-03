#include <opencv2/opencv.hpp>
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
    cv::namedWindow("image");
    cv::imshow("image", image);
    cv::waitKey();
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

    cv::namedWindow(filepath, cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Gray image", cv::WINDOW_AUTOSIZE);
    imshow(filepath, image);
    imshow("Gray image", gray_image);
    cv::waitKey();
    return true;
}

}
