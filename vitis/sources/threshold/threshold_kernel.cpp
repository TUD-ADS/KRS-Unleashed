#include "threshold/threshold_kernel.hpp"

static constexpr int __XF_DEPTH = (threshold::height * threshold::width * (XF_PIXELWIDTH(threshold::IN_TYPE, threshold::NPPCX)) / 8) / (threshold::INPUT_PTR_WIDTH / 8);
// https://github.com/opencv/opencv/blob/2.3.1/modules/imgproc/src/thresh.cpp line 551 ff.
// delta == adaptive_threshold_delta; blocksize == adaptive_threshold_radius

namespace threshold
{

    extern "C"
    {
        void adaptive_threshold(
            ap_uint<threshold::INPUT_PTR_WIDTH> *img_inp, ap_uint<threshold::OUTPUT_PTR_WIDTH> *img_out)
        {
            // clang-format off
    #pragma HLS INTERFACE m_axi     port=img_inp  offset=slave bundle=gmem1 depth=__XF_DEPTH
    #pragma HLS INTERFACE m_axi     port=img_out  offset=slave bundle=gmem2 depth=__XF_DEPTH   
    #pragma HLS INTERFACE s_axilite port=return
            // clang-format on

            xf::cv::Mat<threshold::IN_TYPE, threshold::height, threshold::width, threshold::NPPCX, threshold::XF_CV_DEPTH_IN> in_mat(threshold::height, threshold::width);
            xf::cv::Mat<threshold::IN_TYPE, threshold::height, threshold::width, threshold::NPPCX, threshold::XF_CV_DEPTH_IN> in_mat1(threshold::height, threshold::width);
            xf::cv::Mat<threshold::IN_TYPE, threshold::height, threshold::width, threshold::NPPCX, threshold::XF_CV_DEPTH_IN_2> in_mat2(threshold::height, threshold::width);
            xf::cv::Mat<threshold::IN_TYPE, threshold::height, threshold::width, threshold::NPPCX, threshold::XF_CV_DEPTH_IN> mean_mat(threshold::height, threshold::width);
            xf::cv::Mat<threshold::OUT_TYPE, threshold::height, threshold::width, threshold::NPPCX, threshold::XF_CV_DEPTH_OUT> out_mat(threshold::height, threshold::width);
            // clang-format off
    #pragma HLS DATAFLOW
            // clang-format on

            xf::cv::Array2xfMat<threshold::INPUT_PTR_WIDTH, threshold::IN_TYPE, threshold::height, threshold::width, threshold::NPPCX, threshold::XF_CV_DEPTH_IN>(img_inp, in_mat);

            xf::cv::duplicateMat<threshold::IN_TYPE, threshold::height, threshold::width, threshold::NPPCX, threshold::XF_CV_DEPTH_IN, threshold::XF_CV_DEPTH_IN, threshold::XF_CV_DEPTH_IN_2>(
                in_mat, in_mat1, in_mat2);

            // boxFilter(src, mean, src.type(), Size(blockSize, blockSize),
            //           Point(-1, -1), true, BORDER_REPLICATE); //XF_BORDER_REPLICATE
            xf::cv::boxFilter<XF_BORDER_CONSTANT, threshold::adaptive_threshold_radius, threshold::IN_TYPE, threshold::height, threshold::width, threshold::NPPCX, threshold::XF_USE_URAM, threshold::XF_CV_DEPTH_IN,
                              threshold::XF_CV_DEPTH_OUT>(in_mat1, mean_mat);

            apply_threshold(in_mat2, mean_mat, out_mat);

            xf::cv::xfMat2Array<threshold::OUTPUT_PTR_WIDTH, threshold::OUT_TYPE, threshold::height, threshold::width, threshold::NPPCX, threshold::XF_CV_DEPTH_OUT>(out_mat, img_out);
        }
    }

    void init_threshold_table(uint8_t threshold_table[768])
    {
        for (uint16_t i = 0; i < 768; i++)
        {
            threshold_table[i] = (uint8_t)(i - 255 <= -threshold::adaptive_threshold_delta ? threshold::ADAPTIVE_MAX_VALUE : 0);
        }
    }

    void apply_threshold(xf::cv::Mat<threshold::IN_TYPE, threshold::height, threshold::width, threshold::NPPCX, threshold::XF_CV_DEPTH_IN_2> &_src_mat,
                         xf::cv::Mat<threshold::IN_TYPE, threshold::height, threshold::width, threshold::NPPCX, threshold::XF_CV_DEPTH_IN> &_mean_mat,
                         xf::cv::Mat<threshold::OUT_TYPE, threshold::height, threshold::width, threshold::NPPCX, threshold::XF_CV_DEPTH_OUT> &_dst_mat)
    {
        uint8_t threshold_table[768];
        init_threshold_table(threshold_table);

        for (uint16_t h_index = 0; h_index < threshold::height; h_index++)
        {
            // step - Number of bytes each matrix row occupies
            for (uint16_t w_index = 0; w_index < threshold::width; w_index++)
            {
                uint8_t index = threshold::width * h_index + w_index;
                _dst_mat.write(index, threshold_table[_src_mat.read(index) - _mean_mat.read(index) + 255]);
            }
        }
    }
}