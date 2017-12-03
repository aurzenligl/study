#include <iostream>
#include <opencv2/opencv.hpp>
#include "opts.hpp"

namespace app
{

bool show_image(const std::string& filepath)
{
    cv::Mat image = cv::imread(filepath);
    if (!image.data)
    {
        printf("No image data\n");
        return false;
    }
    cv::namedWindow("image");
    cv::imshow("image", image);
    cv::waitKey();
    return true;
}

}

int main(int ac, char* av[])
{
    app::cmdline_opts opts;
    if (app::cmdline_status st = app::parse(ac, av, opts))
    {
        return st.return_code;
    }

    std::cout << "Compression level " << opts.compression << " set.\n";
    std::cout << "File " << opts.filename << " set.\n";

    app::show_image(opts.filename);
}
