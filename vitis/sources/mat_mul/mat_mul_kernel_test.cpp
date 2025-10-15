#include "mat_mul/mat_mul_kernel_test.hpp"

void mmult_sw(mat_mul::data_t a[mat_mul::DIMENSION][mat_mul::DIMENSION],
              mat_mul::data_t b[mat_mul::DIMENSION][mat_mul::DIMENSION],
              mat_mul::data_t out[mat_mul::DIMENSION][mat_mul::DIMENSION])
{
    // matrix multiplication of a A*B matrix
    for (int ia = 0; ia < mat_mul::DIMENSION; ++ia)
        for (int ib = 0; ib < mat_mul::DIMENSION; ++ib)
        {

            float sum = 0;

            for (int id = 0; id < mat_mul::DIMENSION; ++id)

                sum += a[ia][id] * b[id][ib];

            out[ia][ib] = sum;
        }
}

int main(void)
{

    int i, j, err;

    mat_mul::data_t matOp1[mat_mul::DIMENSION][mat_mul::DIMENSION];
    mat_mul::data_t matOp2[mat_mul::DIMENSION][mat_mul::DIMENSION];
    mat_mul::data_t matmul_sw[mat_mul::DIMENSION][mat_mul::DIMENSION];
    mat_mul::data_t matmul_hw[mat_mul::DIMENSION][mat_mul::DIMENSION];

    assert(sizeof(mat_mul::data_t) * 8 == 32);

    /** Matrix Initiation */
    for (i = 0; i < mat_mul::DIMENSION; i++)
        for (j = 0; j < mat_mul::DIMENSION; j++)
            matOp1[i][j] = (mat_mul::data_t)(i + j);

    for (i = 0; i < mat_mul::DIMENSION; i++)
        for (j = 0; j < mat_mul::DIMENSION; j++)
            matOp2[i][j] = (mat_mul::data_t)(i * j);
    /** End of Initiation */

    mat_mul::standalone_mat_mul(matOp1, matOp2, matmul_hw);

    mmult_sw(matOp1, matOp2, matmul_sw);

    /** Matrix comparison */
    err = 0;
    for (i = 0; (i < mat_mul::DIMENSION && !err); i++)
        for (j = 0; (j < mat_mul::DIMENSION && !err); j++)
            if (matmul_sw[i][j] != matmul_hw[i][j])
                err++;

    if (err == 0)
        printf("Matrixes identical ... Test successful!\r\n");
    else
        printf("Test failed!\r\n");

    return err;
}