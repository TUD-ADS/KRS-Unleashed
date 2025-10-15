#include "hls_utils/xcl2.hpp"
#include "blur/blur_config_params.hpp"
#include "common/xf_headers.hpp"

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        fprintf(stderr, "Usage: <executable> <bitstream>\n");
        return -1;
    }

    cv::Mat in_img, out_img, ocv_ref;
    cv::Mat diff;

    std::string xclbinFilename = argv[1];

    std::string image_path = "blur_in.png";
    in_img = cv::imread(image_path, 0); // reading in the color image
    if (!in_img.data)
    {
        fprintf(stderr, "Failed to load the image ... !!!\n ");
        return -1;
    }

    out_img.create(in_img.rows, in_img.cols, CV_8UC1); // create memory for output image
    diff.create(in_img.rows, in_img.cols, CV_8UC1);    // create memory for OCV-ref image
    ocv_ref.create(in_img.rows, in_img.cols, CV_8UC1); // create memory for OCV-ref image

#if FILTER_WIDTH == 3
    float sigma = 0.5f;
#endif
#if FILTER_WIDTH == 7
    float sigma = 1.16666f;
#endif
#if FILTER_WIDTH == 5
    float sigma = 0.8333f;
#endif

    // OpenCV Gaussian filter function
    cv::GaussianBlur(in_img, ocv_ref, cv::Size(FILTER_WIDTH, FILTER_WIDTH), FILTER_WIDTH / 6.0, FILTER_WIDTH / 6.0,
                     cv::BORDER_CONSTANT);

    imwrite("output_ocv.png", ocv_ref);

    /////////////////////////////////////// CL ////////////////////////

    int height = in_img.rows;
    int width = in_img.cols;
    std::cout << "Input image height : " << height << std::endl;
    std::cout << "Input image width  : " << width << std::endl;

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
    std::cout << "Input Image Bit Depth:" << XF_DTPIXELDEPTH(blur::IN_TYPE, blur::NPPCX) << std::endl;
    std::cout << "Input Image Channels:" << XF_CHANNELS(blur::IN_TYPE, blur::NPPCX) << std::endl;
    std::cout << "NPPC:" << blur::NPPCX << std::endl;

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
    OCL_CHECK(err, cl::Kernel kernel(program, "gaussian_filter_accel", &err));

    // Allocate the buffers:
    OCL_CHECK(err, cl::Buffer imageToDevice(context, CL_MEM_READ_ONLY, (height * width * blur::INPUT_CH_TYPE), NULL,
                                            &err)); //,in_img.data);
    OCL_CHECK(err, cl::Buffer imageFromDevice(context, CL_MEM_WRITE_ONLY, (height * width * blur::OUTPUT_CH_TYPE), NULL,
                                              &err)); //,(ap_uint<OUTPUT_PTR_WIDTH>*)out_img.data);

    // Set kernel arguments:
    OCL_CHECK(err, err = kernel.setArg(0, imageToDevice));
    OCL_CHECK(err, err = kernel.setArg(1, imageFromDevice));
    OCL_CHECK(err, err = kernel.setArg(2, height));
    OCL_CHECK(err, err = kernel.setArg(3, width));
    OCL_CHECK(err, err = kernel.setArg(4, sigma));

    // Initialize the buffers:
    cl::Event event;

    OCL_CHECK(err, q.enqueueWriteBuffer(imageToDevice,                          // buffer on the FPGA
                                        CL_TRUE,                                // blocking call
                                        0,                                      // buffer offset in bytes
                                        (height * width * blur::INPUT_CH_TYPE), // Size in bytes
                                        in_img.data,                            // Pointer to the data to copy
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
    q.enqueueReadBuffer(imageFromDevice, // This buffers data will be read
                        CL_TRUE,         // blocking call
                        0,               // offset
                        (height * width * blur::OUTPUT_CH_TYPE),
                        out_img.data, // Data will be stored here
                        nullptr, &event_sp);

    q.finish();
    /////////////////////////////////////// end of CL ////////////////////////

    cv::imwrite("hw_out.png", out_img);

    //////////////////  Compute Absolute Difference ////////////////////
    cv::absdiff(ocv_ref, out_img, diff);
    cv::imwrite("out_error.png", diff);

    float err_per;

    xf::cv::analyzeDiff(diff, 0, err_per);

    if (err_per > 1)
    {
        fprintf(stderr, "\nTest Failed.\n ");
        return -1;
    }
    else
    {
        std::cout << "Test Passed " << std::endl;
        return 0;
    }
}