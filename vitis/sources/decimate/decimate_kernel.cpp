#include "decimate/decimate_kernel.hpp"

static constexpr int __XF_DEPTH_INP = ((decimate::HEIGHT) * (decimate::WIDTH) * (XF_PIXELWIDTH(decimate::IN_TYPE, decimate::NPC1))) / (decimate::INPUT_PTR_WIDTH);
static constexpr int __XF_DEPTH_OUT = ((decimate::HEIGHT) * (decimate::WIDTH) * (XF_PIXELWIDTH(decimate::OUT_TYPE, decimate::NPC1))) / (decimate::OUTPUT_PTR_WIDTH);

namespace decimate
{
    extern "C"
    {
        void cvtcolor_rgb2gray(ap_uint<decimate::INPUT_PTR_WIDTH> *img_inp, ap_uint<decimate::OUTPUT_PTR_WIDTH> *img_out)
        {
            // clang-format off
        #pragma HLS INTERFACE m_axi      port=img_inp    offset=slave  bundle=gmem_in0  depth=__XF_DEPTH_INP
        #pragma HLS INTERFACE m_axi      port=img_out   offset=slave  bundle=gmem_out0 depth=__XF_DEPTH_OUT
        #pragma HLS INTERFACE s_axilite  port=return
            // clang-format on

            xf::cv::Mat<decimate::IN_TYPE, decimate::HEIGHT, decimate::WIDTH, decimate::NPC1, decimate::XF_CV_DEPTH_IN> _img_inp(decimate::HEIGHT, decimate::WIDTH);
            xf::cv::Mat<decimate::OUT_TYPE, decimate::HEIGHT, decimate::WIDTH, decimate::NPC1, decimate::XF_CV_DEPTH_OUT> _img_out(decimate::HEIGHT, decimate::WIDTH);

            // clang-format off
        #pragma HLS DATAFLOW
            // clang-format on

            xf::cv::Array2xfMat<decimate::INPUT_PTR_WIDTH, decimate::IN_TYPE, decimate::HEIGHT, decimate::WIDTH, decimate::NPC1, decimate::XF_CV_DEPTH_IN>(img_inp, _img_inp);

            xf::cv::rgb2gray<decimate::IN_TYPE, decimate::OUT_TYPE, decimate::HEIGHT, decimate::WIDTH, decimate::NPC1, decimate::XF_CV_DEPTH_IN, decimate::XF_CV_DEPTH_OUT>(_img_inp, _img_out);

            xf::cv::xfMat2Array<decimate::OUTPUT_PTR_WIDTH, decimate::OUT_TYPE, decimate::HEIGHT, decimate::WIDTH, decimate::NPC1, decimate::XF_CV_DEPTH_OUT>(_img_out, img_out);
        }
    }
}