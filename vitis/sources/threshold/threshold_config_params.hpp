#ifndef THRESHOLD_CONFIG_H
#define THRESHOLD_CONFIG_H

#include "hls_utils/params.hpp"
#include "common/xf_common.hpp"
#include "common/xf_utility.hpp"
#include "core/xf_arithm.hpp"

#include "ap_int.h"
#include "hls_stream.h"
#include "imgproc/xf_box_filter.hpp"
#include "imgproc/xf_duplicateimage.hpp"

#include "math.h"

// THRESH_BINARY_INV, ADAPTIVE_THRESH_MEAN_C
namespace threshold
{

    constexpr int width = hls_utils::imWidth;
    constexpr int height = hls_utils::imHeight;
    constexpr int adaptive_threshold_delta = 5;
    constexpr int adaptive_threshold_radius = 7; // should actually be 9
    constexpr int ADAPTIVE_MAX_VALUE = 255;

    constexpr int NPPCX = XF_NPPC1;

    constexpr int IN_TYPE = XF_8UC1;
    constexpr int OUT_TYPE = XF_8UC1;

    constexpr int XF_CV_DEPTH_IN = 2;
    constexpr int XF_CV_DEPTH_IN_2 = 15360; // MAX delay
    constexpr int XF_CV_DEPTH_OUT = 2;

    constexpr int INPUT_CH_TYPE = 1;
    constexpr int OUTPUT_CH_TYPE = 1;
    constexpr int XF_USE_URAM = 0;
    constexpr int _XF_SYNTHESIS_ = 1;

#if SPC // Single Pixel per Clock operation
    static constexpr int NPC1 = XF_NPPC1;
    static constexpr int NPC2 = XF_NPPC1;
#else // Multiple Pixels per Clock operation
    static constexpr int NPC1 = XF_NPPC8;
    static constexpr int NPC2 = XF_NPPC8;
#endif

    constexpr int INPUT_PTR_WIDTH = 8 * threshold::NPC1;
    constexpr int OUTPUT_PTR_WIDTH = 8 * threshold::NPC1;

    extern "C" void adaptive_threshold(ap_uint<threshold::INPUT_PTR_WIDTH> *img_inp, ap_uint<threshold::OUTPUT_PTR_WIDTH> *img_out);
    void init_threshold_table(uint8_t threshold_table[768]);
    void apply_threshold(xf::cv::Mat<threshold::IN_TYPE, threshold::height, threshold::width, threshold::NPPCX, threshold::XF_CV_DEPTH_IN_2> &_src_mat,
                         xf::cv::Mat<threshold::IN_TYPE, threshold::height, threshold::width, threshold::NPPCX, threshold::XF_CV_DEPTH_IN> &_mean_mat,
                         xf::cv::Mat<threshold::OUT_TYPE, threshold::height, threshold::width, threshold::NPPCX, threshold::XF_CV_DEPTH_OUT> &_dst_mat);

}

#endif // THRESHOLD_CONFIG_H