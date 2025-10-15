#include "blur/blur_kernel.hpp"

static constexpr int __XF_DEPTH = (blur::HEIGHT * blur::WIDTH * (XF_PIXELWIDTH(blur::IN_TYPE, blur::NPPCX)) / 8) / (blur::INPUT_PTR_WIDTH / 8);

namespace blur
{

    extern "C"
    {
        void gaussian_filter_accel(
            ap_uint<blur::INPUT_PTR_WIDTH> *img_inp, ap_uint<blur::OUTPUT_PTR_WIDTH> *img_out, int rows, int cols, float sigma)
        {
            // clang-format off
    #pragma HLS INTERFACE m_axi     port=img_inp  offset=slave bundle=gmem1 depth=__XF_DEPTH
    #pragma HLS INTERFACE m_axi     port=img_out  offset=slave bundle=gmem2 depth=__XF_DEPTH
    #pragma HLS INTERFACE s_axilite port=sigma    
    #pragma HLS INTERFACE s_axilite port=rows     
    #pragma HLS INTERFACE s_axilite port=cols     
    #pragma HLS INTERFACE s_axilite port=return
            // clang-format on

            xf::cv::Mat<blur::IN_TYPE, blur::HEIGHT, blur::WIDTH, blur::NPPCX, blur::XF_CV_DEPTH_IN_1> in_mat(rows, cols);
            xf::cv::Mat<blur::OUT_TYPE, blur::HEIGHT, blur::WIDTH, blur::NPPCX, blur::XF_CV_DEPTH_OUT_1> out_mat(rows, cols);
            // clang-format off
    #pragma HLS DATAFLOW
            // clang-format on

            xf::cv::Array2xfMat<blur::INPUT_PTR_WIDTH, blur::IN_TYPE, blur::HEIGHT, blur::WIDTH, blur::NPPCX, blur::XF_CV_DEPTH_IN_1>(img_inp, in_mat);

            xf::cv::GaussianBlur<FILTER_WIDTH, XF_BORDER_CONSTANT, blur::OUT_TYPE, blur::HEIGHT, blur::WIDTH, blur::NPPCX, blur::XF_CV_DEPTH_IN_1,
                                 blur::XF_CV_DEPTH_OUT_1>(in_mat, out_mat, sigma);

            xf::cv::xfMat2Array<blur::OUTPUT_PTR_WIDTH, blur::OUT_TYPE, blur::HEIGHT, blur::WIDTH, blur::NPPCX, blur::XF_CV_DEPTH_OUT_1>(out_mat, img_out);
        }
    }
}