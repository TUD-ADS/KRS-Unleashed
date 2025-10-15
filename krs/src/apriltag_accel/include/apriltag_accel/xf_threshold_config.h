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
#include "ap_int.h"
#include "math.h"

#include "common/xf_common.hpp" //vitis_common needs to be removed
#include "common/xf_utility.hpp"
#include "core/xf_arithm.hpp"

#include "imgproc/xf_box_filter.hpp"
#include "imgproc/xf_duplicateimage.hpp"
#include "xf_config_params.h"

namespace apriltag_accel
{

    typedef unsigned short int uint16_t;

    /* config width and height */

    extern "C" void adaptive_threshold(ap_uint<INPUT_PTR_WIDTH> *img_inp, ap_uint<OUTPUT_PTR_WIDTH> *img_out);

} // namespace apriltag_accel

#endif
//_XF_GAUSSIAN_FILTER_CONFIG_H_
