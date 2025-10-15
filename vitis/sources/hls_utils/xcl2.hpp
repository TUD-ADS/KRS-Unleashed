/*
# Copyright (C) 2023, Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: X11

* taken from https://github.com/Xilinx/Vitis-Tutorials/tree/2024.1/Hardware_Acceleration/Design_Tutorials/01-convolution-tutorial/src
*/

#pragma once

#define CL_HPP_CL_1_2_DEFAULT_BUILD
#define CL_HPP_TARGET_OPENCL_VERSION 120
#define CL_HPP_MINIMUM_OPENCL_VERSION 120
#define CL_HPP_ENABLE_PROGRAM_CONSTRUCTION_FROM_ARRAY_COMPATIBILITY 1
#define CL_USE_DEPRECATED_OPENCL_1_2_APIS

// OCL_CHECK doesn't work if call has templatized function call
#define OCL_CHECK(error, call)                                   \
  call;                                                          \
  if (error != CL_SUCCESS)                                       \
  {                                                              \
    printf("%s:%d Error calling " #call ", error code is: %d\n", \
           __FILE__, __LINE__, error);                           \
    exit(EXIT_FAILURE);                                          \
  }

#include <CL/opencl.hpp>
#include <iostream>
#include <fstream>
#include <CL/cl_ext_xilinx.h>
// When creating a buffer with user pointer (CL_MEM_USE_HOST_PTR), under the hood
// User ptr is used if and only if it is properly aligned (page aligned). When not
// aligned, runtime has no choice but to create its own host side buffer that backs
// user ptr. This in turn implies that all operations that move data to and from
// device incur an extra memcpy to move data to/from runtime's own host buffer
// from/to user pointer. So it is recommended to use this allocator if user wish to
// Create Buffer/Memory Object with CL_MEM_USE_HOST_PTR to align user buffer to the
// page boundary. It will ensure that user buffer will be used when user create
// Buffer/Mem Object with CL_MEM_USE_HOST_PTR.
template <typename T>
struct aligned_allocator
{
  using value_type = T;
  T *allocate(std::size_t num)
  {
    void *ptr = nullptr;
    if (posix_memalign(&ptr, 4096, num * sizeof(T)))
      throw std::bad_alloc();
    return reinterpret_cast<T *>(ptr);
  }
  void deallocate(T *p, std::size_t num)
  {
    free(p);
  }
};

namespace xcl
{
  std::vector<cl::Device> get_xil_devices();
  std::vector<cl::Device> get_devices(const std::string &vendor_name);
  char *read_binary_file(const std::string &xclbin_file_name, unsigned &nb);
  bool is_emulation();
  bool is_hw_emulation();
  bool is_xpr_device(const char *device_name);
  void printVector(const std::string arrayName,const u_char * arrayData, const unsigned int length);
}

// const char *printErrorString(cl_int error_code)
// {
// switch(error_code){
//     // run-time and JIT compiler errors
//     case 0: return "CL_SUCCESS";
//     case -1: return "CL_DEVICE_NOT_FOUND";
//     case -2: return "CL_DEVICE_NOT_AVAILABLE";
//     case -3: return "CL_COMPILER_NOT_AVAILABLE";
//     case -4: return "CL_MEM_OBJECT_ALLOCATION_FAILURE";
//     case -5: return "CL_OUT_OF_RESOURCES";
//     case -6: return "CL_OUT_OF_HOST_MEMORY";
//     case -7: return "CL_PROFILING_INFO_NOT_AVAILABLE";
//     case -8: return "CL_MEM_COPY_OVERLAP";
//     case -9: return "CL_IMAGE_FORMAT_MISMATCH";
//     case -10: return "CL_IMAGE_FORMAT_NOT_SUPPORTED";
//     case -11: return "CL_BUILD_PROGRAM_FAILURE";
//     case -12: return "CL_MAP_FAILURE";
//     case -13: return "CL_MISALIGNED_SUB_BUFFER_OFFSET";
//     case -14: return "CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST";
//     case -15: return "CL_COMPILE_PROGRAM_FAILURE";
//     case -16: return "CL_LINKER_NOT_AVAILABLE";
//     case -17: return "CL_LINK_PROGRAM_FAILURE";
//     case -18: return "CL_DEVICE_PARTITION_FAILED";
//     case -19: return "CL_KERNEL_ARG_INFO_NOT_AVAILABLE";

//     // compile-time errors
//     case -30: return "CL_INVALID_VALUE";
//     case -31: return "CL_INVALID_DEVICE_TYPE";
//     case -32: return "CL_INVALID_PLATFORM";
//     case -33: return "CL_INVALID_DEVICE";
//     case -34: return "CL_INVALID_CONTEXT";
//     case -35: return "CL_INVALID_QUEUE_PROPERTIES";
//     case -36: return "CL_INVALID_COMMAND_QUEUE";
//     case -37: return "CL_INVALID_HOST_PTR";
//     case -38: return "CL_INVALID_MEM_OBJECT";
//     case -39: return "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR";
//     case -40: return "CL_INVALID_IMAGE_SIZE";
//     case -41: return "CL_INVALID_SAMPLER";
//     case -42: return "CL_INVALID_BINARY";
//     case -43: return "CL_INVALID_BUILD_OPTIONS";
//     case -44: return "CL_INVALID_PROGRAM";
//     case -45: return "CL_INVALID_PROGRAM_EXECUTABLE";
//     case -46: return "CL_INVALID_KERNEL_NAME";
//     case -47: return "CL_INVALID_KERNEL_DEFINITION";
//     case -48: return "CL_INVALID_KERNEL";
//     case -49: return "CL_INVALID_ARG_INDEX";
//     case -50: return "CL_INVALID_ARG_VALUE";
//     case -51: return "CL_INVALID_ARG_SIZE";
//     case -52: return "CL_INVALID_KERNEL_ARGS";
//     case -53: return "CL_INVALID_WORK_DIMENSION";
//     case -54: return "CL_INVALID_WORK_GROUP_SIZE";
//     case -55: return "CL_INVALID_WORK_ITEM_SIZE";
//     case -56: return "CL_INVALID_GLOBAL_OFFSET";
//     case -57: return "CL_INVALID_EVENT_WAIT_LIST";
//     case -58: return "CL_INVALID_EVENT";
//     case -59: return "CL_INVALID_OPERATION";
//     case -60: return "CL_INVALID_GL_OBJECT";
//     case -61: return "CL_INVALID_BUFFER_SIZE";
//     case -62: return "CL_INVALID_MIP_LEVEL";
//     case -63: return "CL_INVALID_GLOBAL_WORK_SIZE";
//     case -64: return "CL_INVALID_PROPERTY";
//     case -65: return "CL_INVALID_IMAGE_DESCRIPTOR";
//     case -66: return "CL_INVALID_COMPILER_OPTIONS";
//     case -67: return "CL_INVALID_LINKER_OPTIONS";
//     case -68: return "CL_INVALID_DEVICE_PARTITION_COUNT";

//     // extension errors
//     case -1000: return "CL_INVALID_GL_SHAREGROUP_REFERENCE_KHR";
//     case -1001: return "CL_PLATFORM_NOT_FOUND_KHR";
//     case -1002: return "CL_INVALID_D3D10_DEVICE_KHR";
//     case -1003: return "CL_INVALID_D3D10_RESOURCE_KHR";
//     case -1004: return "CL_D3D10_RESOURCE_ALREADY_ACQUIRED_KHR";
//     case -1005: return "CL_D3D10_RESOURCE_NOT_ACQUIRED_KHR";
//     default: return "Unknown OpenCL error";
//     }
// }