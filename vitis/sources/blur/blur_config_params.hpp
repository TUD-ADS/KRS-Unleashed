/*
 * Copyright 2022 Xilinx, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _XF_GAUSSIAN_FILTER_CONFIG_H_
#define _XF_GAUSSIAN_FILTER_CONFIG_H_

#include "hls_stream.h"
#include "common/xf_common.hpp"
#include "common/xf_utility.hpp"
#include "imgproc/xf_gaussian_filter.hpp"
#include "hls_utils/params.hpp"

namespace blur
{

    typedef unsigned short int uint16_t;

    constexpr int WIDTH = hls_utils::imWidth;
    constexpr int HEIGHT = hls_utils::imHeight;

    constexpr int XF_CV_DEPTH_IN_1 = 2;
    constexpr int XF_CV_DEPTH_OUT_1 = 2;

#define FILTER_SIZE_3 0
#define FILTER_SIZE_5 0
#define FILTER_SIZE_7 1

#if FILTER_SIZE_3
#define FILTER_WIDTH 3
#define FILTER 3
#elif FILTER_SIZE_5
#define FILTER_WIDTH 5
#define FILTER 5
#elif FILTER_SIZE_7
#define FILTER_WIDTH 7
#define FILTER 7
#endif

    constexpr int NPPCX = XF_NPPC1;
    constexpr int IN_TYPE = XF_8UC1;
    constexpr int OUT_TYPE = XF_8UC1;

    constexpr int INPUT_CH_TYPE = 1;
    constexpr int OUTPUT_CH_TYPE = 1;

    constexpr int INPUT_PTR_WIDTH = 8;
    constexpr int OUTPUT_PTR_WIDTH = 8;

    extern "C" void gaussian_filter_accel(
        ap_uint<blur::INPUT_PTR_WIDTH> *img_inp, ap_uint<blur::OUTPUT_PTR_WIDTH> *img_out, int rows, int cols, float sigma);

}
#endif
//_XF_GAUSSIAN_FILTER_CONFIG_H_
