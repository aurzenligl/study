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
    cv::LUT(image, {redlut.data(), redlut.size()}, image);
}

void sharpen(const cv::Mat& in, cv::Mat& out)
{
    const int nChannels = in.channels();
    out.create(in.size(), in.type());
    for (int j = 1 ; j < in.rows - 1; ++j)
    {
        const uchar* previous = in.ptr<uchar>(j - 1);
        const uchar* current = in.ptr<uchar>(j    );
        const uchar* next = in.ptr<uchar>(j + 1);
        uchar* output = out.ptr<uchar>(j);
        for (int i = nChannels; i < nChannels * (in.cols - 1); ++i)
        {
            *output++ = cv::saturate_cast<uchar>(
                5 * current[i]
                - current[i - nChannels]
                - current[i + nChannels]
                - previous[i]
                - next[i]
            );
        }
    }
    out.row(0).setTo(cv::Scalar(0));
    out.row(out.rows - 1).setTo(cv::Scalar(0));
    out.col(0).setTo(cv::Scalar(0));
    out.col(out.cols - 1).setTo(cv::Scalar(0));
}

void sharpen_kernel(const cv::Mat& in, cv::Mat& out)
{
    cv::Mat kernel = (cv::Mat_<char>(3,3) <<  0, -5,  0,
                                             -5,  21, -5,
                                              0, -5,  0);

    cv::filter2D(in, out, in.depth(), kernel);
}

}
}
