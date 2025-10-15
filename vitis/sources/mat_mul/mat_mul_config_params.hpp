#ifndef MATMUL_CONFIG_H
#define MATMUL_CONFIG_H

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "ap_int.h"
#include "hls_stream.h"
#include <ap_axi_sdata.h>

#include "math.h"

// THRESH_BINARY_INV, ADAPTIVE_THRESH_MEAN_C
namespace mat_mul
{
    typedef int data_t;
    constexpr int DIMENSION = 64;
    constexpr int TILE_SIZE = 16;
    constexpr int SIZE = DIMENSION * DIMENSION;
    constexpr int U = 4;
    constexpr int TI = 5;
    constexpr int TD = 5;
#define MCR_SIZE 1024

    extern "C" void standalone_mat_mul(mat_mul::data_t a[mat_mul::DIMENSION][mat_mul::DIMENSION],
                                       mat_mul::data_t b[mat_mul::DIMENSION][mat_mul::DIMENSION],
                                       mat_mul::data_t out[mat_mul::DIMENSION][mat_mul::DIMENSION]);

}

#endif // MATMUL_CONFIG_H