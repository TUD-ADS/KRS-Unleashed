#ifndef HLS_UTILS_H_
#define HLS_UTILS_H_

#include <stdint.h>

namespace hls_utils
{

    constexpr int min_white_black_diff = 5;
    constexpr int imWidth = 1920;
    constexpr int imHeight = 1080;
    constexpr int decimate_factor = 1; // currently not supported

}

#define SPC 1

#endif // HLS_UTILS_H_