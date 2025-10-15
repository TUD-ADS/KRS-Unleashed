/*
# Copyright (C) 2023, Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: X11

* taken from https://github.com/Xilinx/Vitis-Tutorials/tree/2024.1/Hardware_Acceleration/Design_Tutorials/01-convolution-tutorial/src
*/

#include <CL/cl_platform.h>
#include <unistd.h>
#include <limits.h>
#include <sys/stat.h>
#include "hls_utils/xcl2.hpp"

namespace xcl
{
    std::vector<cl::Device> get_devices(const std::string &vendor_name)
    {

        size_t i;
        cl_int err;
        std::vector<cl::Platform> platforms;
        OCL_CHECK(err, err = cl::Platform::get(&platforms));
        cl::Platform platform;
        for (i = 0; i < platforms.size(); i++)
        {
            platform = platforms[i];
            OCL_CHECK(err, std::string platformName = platform.getInfo<CL_PLATFORM_NAME>(&err));
            if (platformName == vendor_name)
            {
                std::cout << "Found platform: " << platformName.c_str() << std::endl;
                break;
            }
        }
        if (i == platforms.size())
        {
            std::cout << "Error: Failed to find Xilinx platform" << std::endl;
            exit(EXIT_FAILURE);
        }

        // Getting ACCELERATOR Devices and selecting 1st such device
        std::vector<cl::Device> devices;
        OCL_CHECK(err, err = platform.getDevices(CL_DEVICE_TYPE_ACCELERATOR, &devices));
        return devices;
    }

    std::vector<cl::Device> get_xil_devices()
    {
        return get_devices("Xilinx");
    }

    char *read_binary_file(const std::string &xclbin_file_name, unsigned &nb)
    {
        if (access(xclbin_file_name.c_str(), R_OK) != 0)
        {
            printf("ERROR: %s xclbin not available please build\n", xclbin_file_name.c_str());
            exit(EXIT_FAILURE);
        }
        // Loading XCL Bin into char buffer
        std::cout << "Loading: '" << xclbin_file_name.c_str() << "'\n";
        std::ifstream bin_file(xclbin_file_name.c_str(), std::ifstream::binary);
        bin_file.seekg(0, bin_file.end);
        nb = bin_file.tellg();
        bin_file.seekg(0, bin_file.beg);
        char *buf = new char[nb];
        bin_file.read(buf, nb);
        return buf;
    }

    bool is_emulation()
    {
        bool ret = false;
        char *xcl_mode = getenv("XCL_EMULATION_MODE");
        if (xcl_mode != NULL)
        {
            ret = true;
        }
        return ret;
    }

    bool is_hw_emulation()
    {
        bool ret = false;
        char *xcl_mode = getenv("XCL_EMULATION_MODE");
        if ((xcl_mode != NULL) && !strcmp(xcl_mode, "hw_emu"))
        {
            ret = true;
        }
        return ret;
    }

    bool is_xpr_device(const char *device_name)
    {
        const char *output = strstr(device_name, "xpr");

        if (output == NULL)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    /////////////////////////////////////////////////////////////////
// Helper function to print vector elements
/////////////////////////////////////////////////////////////////
void printVector(const std::string arrayName,
const u_char * arrayData,
const unsigned int length)
{   int numElementsToPrint = (256 < length) ? 256 : length;
    std::cout << std::endl << arrayName << ":" << std::endl;
    for(int i = 0; i < numElementsToPrint; ++i)
    std::cout << arrayData[i] << " ";
    std::cout << std::endl;
}
};
