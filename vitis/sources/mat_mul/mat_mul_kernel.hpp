#ifndef MATMUL_KERNEL_H
#define MATMUL_KERNEL_H
#include "mat_mul/mat_mul_config_params.hpp"

namespace mat_mul
{
    template <typename T, int DIM>
    void mmult_hw(T a[DIM][DIM], T b[DIM][DIM], T out[DIM][DIM]);
}

#endif // MATMUL_KERNEL_H
