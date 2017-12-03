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

}
