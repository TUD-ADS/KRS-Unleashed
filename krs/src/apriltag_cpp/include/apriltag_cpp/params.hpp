#pragma once

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>

namespace apriltag
{
    static constexpr int image_width = 1920;
    static constexpr int image_height = 1080;
    static constexpr bool do_blur = false;
    static constexpr double blur_sigma = 1.999;
    // 0.999              3
    // 1.499              5
    // 1.999              7
    static constexpr ushort decimate_factor = 1;
    static constexpr bool debug_images = true;
    static constexpr double center_factor = (1 / apriltag::decimate_factor) * 0.5;
}

static const std::vector<cv::Scalar> &getCColors()
{
    static std::vector<cv::Scalar> ccolors;
    if (ccolors.empty())
    {
        ccolors.push_back(CV_RGB(0, 255, 255));
        ccolors.push_back(CV_RGB(255, 0, 0));
        ccolors.push_back(CV_RGB(0, 191, 255));
        ccolors.push_back(CV_RGB(255, 63, 0));
        ccolors.push_back(CV_RGB(0, 127, 255));
        ccolors.push_back(CV_RGB(255, 127, 0));
        ccolors.push_back(CV_RGB(0, 63, 255));
        ccolors.push_back(CV_RGB(255, 191, 0));
        ccolors.push_back(CV_RGB(0, 0, 255));
        ccolors.push_back(CV_RGB(255, 255, 0));
        ccolors.push_back(CV_RGB(63, 0, 255));
        ccolors.push_back(CV_RGB(191, 255, 0));
        ccolors.push_back(CV_RGB(127, 0, 255));
        ccolors.push_back(CV_RGB(127, 255, 0));
        ccolors.push_back(CV_RGB(191, 0, 255));
        ccolors.push_back(CV_RGB(63, 255, 0));
        ccolors.push_back(CV_RGB(255, 0, 255));
        ccolors.push_back(CV_RGB(0, 255, 0));
        ccolors.push_back(CV_RGB(255, 0, 191));
        ccolors.push_back(CV_RGB(0, 255, 63));
        ccolors.push_back(CV_RGB(255, 0, 127));
        ccolors.push_back(CV_RGB(0, 255, 127));
        ccolors.push_back(CV_RGB(255, 0, 63));
        ccolors.push_back(CV_RGB(0, 255, 191));
    }
    return ccolors;
}
