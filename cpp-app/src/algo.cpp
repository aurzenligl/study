#include "seq.hpp"
#include "algo.hpp"

namespace app
{
namespace algo
{

void show(cv::Mat& image)
{
    cv::namedWindow("image");
    cv::imshow("image", image);
    cv::waitKey();
}

struct color_space_reduction
{
    constexpr uint8_t operator()(uint8_t x) const
    {
        return x / 30 * 30;
    }
};

static constexpr std::array<uint8_t, 256> redlut = make_lut<256, color_space_reduction>();
static cv::Mat redlut_mat(1, 256, CV_8U, (void*)redlut.data(), sizeof(uint8_t));

void reduce_color_space(cv::Mat& image)
{
    int nRows = image.rows;
    int nCols = image.cols * image.channels();
    if (image.isContinuous())
    {
        nCols *= nRows;
        nRows = 1;
    }

    for (int i = 0; i < nRows; ++i)
    {
        uint8_t* p = image.ptr<uint8_t>(i);
        for (int j = 0; j < nCols; ++j)
        {
            p[j] = redlut[p[j]];
        }
    }
}

void reduce_color_space_lut(cv::Mat& image)
{
    cv::LUT(image, redlut_mat, image);
}

}
}
