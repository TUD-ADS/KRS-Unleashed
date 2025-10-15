#ifndef _DECIMATE_CONFIG_H_
#define _DECIMATE_CONFIG_H_

#include "hls_stream.h"
#include "common/xf_common.hpp"
#include "common/xf_utility.hpp"
#include "imgproc/xf_cvt_color.hpp"
#include "imgproc/xf_cvt_color_1.hpp"
#include "hls_utils/params.hpp"

namespace decimate
{
    constexpr int WIDTH = hls_utils::imWidth;
    constexpr int HEIGHT = hls_utils::imHeight;

    constexpr int XF_CV_DEPTH_IN = 2;
    constexpr int XF_CV_DEPTH_OUT = 2;

    constexpr int IN_TYPE = XF_8UC3;
    constexpr int OUT_TYPE = XF_8UC1;

    constexpr int INPUT_CH_TYPE = 3;
    constexpr int OUTPUT_CH_TYPE = 1;

    constexpr int NPPCX = XF_NPPC1;

    constexpr int _XF_SYNTHESIS_ = 1;

#if SPC // Single Pixel per Clock operation
    static constexpr int NPC1 = XF_NPPC1;
    static constexpr int NPC2 = XF_NPPC1;
#else // Multiple Pixels per Clock operation
    static constexpr int NPC1 = XF_NPPC8;
    static constexpr int NPC2 = XF_NPPC8;
#endif

    constexpr int INPUT_PTR_WIDTH = 32 * decimate::NPC1;
    constexpr int OUTPUT_PTR_WIDTH = 8 * decimate::NPC1;

    extern "C" void cvtcolor_rgb2gray(
        ap_uint<decimate::INPUT_PTR_WIDTH> *img_inp, ap_uint<decimate::OUTPUT_PTR_WIDTH> *img_out);
}
#endif //_DECIMATE_CONFIG_H_