#!/bin/bash

workspaceFolder=../../../..
run_dir=`dirname $0`
cd $run_dir

export XCL_EMULATION_MODE=sw_emu
export ENABLE_KERNEL_DEBUG=true
export XCL_BIN_PATH=${workspaceFolder}/apriltag_system/build/sw_emu/package
export XRT_INI_PATH=${workspaceFolder}/apriltag_system/apriltag_system_host/runtime/sw_emu_xrt.ini
export OPENCV_INCLUDE=/home/paul/.local/include/opencv4
export OPENCV_LIB=/home/paul/.local/lib
export LD_LIBRARY_PATH=/home/paul/.local/lib:$LD_LIBRARY_PATH
export XILINX_XRT=/opt/xilinx/xrt

cp ${workspaceFolder}/blur_host_code/build/x86sim/emconfig.json .

# Command to launch application. Format: <host_exe_path> <cmd_line_args>

${workspaceFolder}/blur_host_code/build/x86sim/blur_host_code ${workspaceFolder}/apriltag_system/build/sw_emu/hw_link/blur_container.xclbin
