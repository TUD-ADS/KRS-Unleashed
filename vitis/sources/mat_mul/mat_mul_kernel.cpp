#include "mat_mul/mat_mul_kernel.hpp"

namespace mat_mul
{

    void standalone_mat_mul(mat_mul::data_t a[mat_mul::DIMENSION][mat_mul::DIMENSION],
                            mat_mul::data_t b[mat_mul::DIMENSION][mat_mul::DIMENSION],
                            mat_mul::data_t out[mat_mul::DIMENSION][mat_mul::DIMENSION])
    {
#pragma HLS INTERFACE m_axi port = a offset = slave bundle = gmem_A depth = 4096
#pragma HLS INTERFACE m_axi port = b offset = slave bundle = gmem_B depth = 4096
#pragma HLS INTERFACE m_axi port = out offset = slave bundle = gmem_Out depth = 4096

        static data_t tileA[mat_mul::TILE_SIZE][mat_mul::TILE_SIZE];
        static data_t tileB[mat_mul::TILE_SIZE][mat_mul::TILE_SIZE];
        static data_t tileOut[mat_mul::TILE_SIZE][mat_mul::TILE_SIZE];

#pragma HLS ARRAY_PARTITION variable = tileA dim = 2 complete
#pragma HLS ARRAY_PARTITION variable = tileB dim = 1 complete
#pragma HLS ARRAY_PARTITION variable = tileOut dim = 0 complete

        // Initialize output matrix
        for (int i = 0; i < mat_mul::DIMENSION; i++)
            for (int j = 0; j < mat_mul::DIMENSION; j++)
#pragma HLS PIPELINE II = 1
                out[i][j] = 0;

        // Tile-based computation
        for (int ti = 0; ti < mat_mul::DIMENSION; ti += mat_mul::TILE_SIZE)
            for (int tj = 0; tj < mat_mul::DIMENSION; tj += mat_mul::TILE_SIZE)
                for (int tk = 0; tk < mat_mul::DIMENSION; tk += mat_mul::TILE_SIZE)
                {
                    // Load tiles
                    for (int i = 0; i < mat_mul::TILE_SIZE && (ti + i) < mat_mul::DIMENSION; i++)
                        for (int k = 0; k < mat_mul::TILE_SIZE && (tk + k) < mat_mul::DIMENSION; k++)
#pragma HLS PIPELINE II = 1
                            tileA[i][k] = a[ti + i][tk + k];

                    for (int k = 0; k < mat_mul::TILE_SIZE && (tk + k) < mat_mul::DIMENSION; k++)
                        for (int j = 0; j < mat_mul::TILE_SIZE && (tj + j) < mat_mul::DIMENSION; j++)
#pragma HLS PIPELINE II = 1
                            tileB[k][j] = b[tk + k][tj + j];

                    for (int i = 0; i < mat_mul::TILE_SIZE && (ti + i) < mat_mul::DIMENSION; i++)
                        for (int j = 0; j < mat_mul::TILE_SIZE && (tj + j) < mat_mul::DIMENSION; j++)
#pragma HLS PIPELINE II = 1
                            tileOut[i][j] = out[ti + i][tj + j];

                    // Compute tile multiplication
                    for (int i = 0; i < mat_mul::TILE_SIZE && (ti + i) < mat_mul::DIMENSION; i++)
                        for (int j = 0; j < mat_mul::TILE_SIZE && (tj + j) < mat_mul::DIMENSION; j++)
                        {
#pragma HLS PIPELINE II = 1
                            data_t sum = tileOut[i][j];
                            for (int k = 0; k < mat_mul::TILE_SIZE && (tk + k) < mat_mul::DIMENSION; k++)
                            {
#pragma HLS UNROLL factor = 2
                                sum += tileA[i][k] * tileB[k][j];
                            }
                            tileOut[i][j] = sum;
                        }

                    // Store tile C back
                    for (int i = 0; i < mat_mul::TILE_SIZE && (ti + i) < mat_mul::DIMENSION; i++)
                        for (int j = 0; j < mat_mul::TILE_SIZE && (tj + j) < mat_mul::DIMENSION; j++)
#pragma HLS PIPELINE II = 1
                            out[ti + i][tj + j] = tileOut[i][j];
                }
    }

    // --------------------------------------------------------
    // function to be accelerated in HW
    template <typename T, int DIM>
    void mmult_hw(T a[DIM][DIM], T b[DIM][DIM], T out[DIM][DIM])
    {
#pragma HLS INLINE

    // matrix multiplication of a A*B matrix
    L1:
        for (int i = 0; i < DIM; ++i)
        {
        L2:
            for (int j = 0; j < DIM; ++j)
            {
#pragma HLS PIPELINE II = 1
                T sum = 0;
            L3:
                for (int k = 0; k < DIM; ++k)
                {
                    sum += a[i][k] * b[k][j];
                }
                out[i][j] = sum;
            }
        }

        return;
    }

}