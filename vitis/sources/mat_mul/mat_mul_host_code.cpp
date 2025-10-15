#include "hls_utils/xcl2.hpp"
#include "mat_mul/mat_mul_config_params.hpp"

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

int main(int argc, char **argv)
{

    int i, j, matrix_err;

    mat_mul::data_t matOp1[mat_mul::DIMENSION][mat_mul::DIMENSION];
    mat_mul::data_t matOp2[mat_mul::DIMENSION][mat_mul::DIMENSION];
    mat_mul::data_t matmul_sw[mat_mul::DIMENSION][mat_mul::DIMENSION];
    mat_mul::data_t matmul_hw[mat_mul::DIMENSION][mat_mul::DIMENSION];

    std::string xclbinFilename = "smarobix_matmul.xclbin";

    /** Matrix Initiation */
    for (i = 0; i < mat_mul::DIMENSION; i++)
        for (j = 0; j < mat_mul::DIMENSION; j++)
            matOp1[i][j] = (mat_mul::data_t)(i + j);

    for (i = 0; i < mat_mul::DIMENSION; i++)
        for (j = 0; j < mat_mul::DIMENSION; j++)
            matOp2[i][j] = (mat_mul::data_t)(i * j);
    /** End of Initiation */

    // SW version
    mmult_sw(matOp1, matOp2, matmul_sw);

    /////////////////////////////////////// CL ////////////////////////

    cl_int err;
    std::cout << "INFO: Running OpenCL section." << std::endl;

    // Get the device:
    std::vector<cl::Device> devices = xcl::get_xil_devices();
    cl::Device device = devices[0];

    // Context, command queue and device name:
    OCL_CHECK(err, cl::Context context(device, NULL, NULL, NULL, &err));
    OCL_CHECK(err, cl::CommandQueue q(context, device, CL_QUEUE_PROFILING_ENABLE, &err));
    OCL_CHECK(err, std::string device_name = device.getInfo<CL_DEVICE_NAME>(&err));

    std::cout << "INFO: Device found - " << device_name << std::endl;
    std::cout << "Matrix Dimension:" << mat_mul::DIMENSION << std::endl;
    std::cout << "Matrix Multiplication Parameterisation :" << mat_mul::U << ", " << mat_mul::TI << ", " << mat_mul::TD << std::endl;

    // Load binary:
    unsigned fileBufSize; // pass-by-reference, updated in read_binary_file()
    char *buf = xcl::read_binary_file(xclbinFilename, fileBufSize);

    // Creating Program from Binary File
    cl::Program::Binaries bins{{buf, fileBufSize}};

    // std::string binaryFile = xcl::find_binary_file(device_name, "krnl_gaussian_filter");
    // cl::Program::Binaries bins = xcl::import_binary_file(binaryFile);
    devices.resize(1);
    OCL_CHECK(err, cl::Program program(context, devices, bins, NULL, &err));

    // Create a kernel:
    OCL_CHECK(err, cl::Kernel kernel(program, "standalone_mat_mul", &err));

    // Allocate the buffers:
    OCL_CHECK(err, cl::Buffer mat1_buf(context, CL_MEM_READ_ONLY, (sizeof(matOp1)), NULL,
                                       &err));
    OCL_CHECK(err, cl::Buffer mat2_buf(context, CL_MEM_READ_ONLY, (sizeof(matOp2)), NULL,
                                       &err));
    OCL_CHECK(err, cl::Buffer mat_out_buf(context, CL_MEM_WRITE_ONLY, (sizeof(matmul_hw)), NULL,
                                          &err));

    // Set kernel arguments:
    OCL_CHECK(err, err = kernel.setArg(0, mat1_buf));
    OCL_CHECK(err, err = kernel.setArg(1, mat2_buf));
    OCL_CHECK(err, err = kernel.setArg(2, mat_out_buf));

    // Initialize the buffers:
    cl::Event event;

    OCL_CHECK(err, q.enqueueWriteBuffer(mat1_buf,         // buffer on the FPGA
                                        CL_TRUE,          // blocking call
                                        0,                // buffer offset in bytes
                                        (sizeof(matOp1)), // Size in bytes
                                        *matOp1,          // Pointer to the data to copy
                                        nullptr, &event));

    OCL_CHECK(err, q.enqueueWriteBuffer(mat2_buf,         // buffer on the FPGA
                                        CL_TRUE,          // blocking call
                                        0,                // buffer offset in bytes
                                        (sizeof(matOp2)), // Size in bytes
                                        *matOp2,          // Pointer to the data to copy
                                        nullptr, &event));

    // Profiling Objects
    cl_ulong start = 0;
    cl_ulong end = 0;
    double diff_prof = 0.0f;
    cl::Event event_sp;

    // Execute the kernel:
    OCL_CHECK(err, err = q.enqueueTask(kernel, NULL, &event_sp));

    clWaitForEvents(1, (const cl_event *)&event_sp);
    event_sp.getProfilingInfo(CL_PROFILING_COMMAND_START, &start);
    event_sp.getProfilingInfo(CL_PROFILING_COMMAND_END, &end);
    diff_prof = end - start;
    std::cout << (diff_prof / 1000000) << "ms" << std::endl;

    // Copy Result from Device Global Memory to Host Local Memory
    q.enqueueReadBuffer(mat_out_buf, // This buffers data will be read
                        CL_TRUE,     // blocking call
                        0,           // offset
                        (sizeof(matmul_hw)),
                        *matmul_hw, // Data will be stored here
                        nullptr, &event_sp);

    q.finish();
    /////////////////////////////////////// end of CL ////////////////////////

    //////////////////  Compute Absolute Difference ////////////////////
    matrix_err = 0;
    for (i = 0; (i < mat_mul::DIMENSION && !matrix_err); i++)
        for (j = 0; (j < mat_mul::DIMENSION && !matrix_err); j++)
            if (matmul_sw[i][j] != matmul_hw[i][j])
                matrix_err++;

    if (matrix_err == 0)
        printf("Matrixes identical ... Test successful!\r\n");
    else
        printf("Test failed!\r\n");

    return matrix_err;
}