#    ____  ____
#   /   /\/   /
#  /___/  \  /   Copyright (c) 2021, Xilinx®.
#  \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
#   \   \
#   /   /
#  /___/   /\
#  \   \  /  \
#   \___\/\___\
#
cmake_minimum_required(VERSION 3.5)
project(krs_firmware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
include(cmake/firmware_setup.cmake)  # define only once, here

# NOTE:
# The following scripts avoid the use of CMake EXISTS
# due to undefined behavior. Refer to the development
# notes for more details https://gitlab.com/xilinxrobotics/docs/-/issues/9#note_505935065
# For more details, refer to the xilinx_vitis package.

# To execute logic after the install() CMake command, embed
# follow-up actions into CMake scripts which get added to the
# "cmake_install.cmake" script in order
#     See https://stackoverflow.com/a/10000327
install(SCRIPT "${CMAKE_SOURCE_DIR}/cmake/firmware_setup.cmake")
# Generate toolchain, first install template, then execute
install(
  FILES
    "${CMAKE_SOURCE_DIR}/templates/kr260.mixin.template"
    "${CMAKE_SOURCE_DIR}/templates/index.yaml.template"
  DESTINATION
    ${CMAKE_BINARY_DIR}
)
install(SCRIPT "${CMAKE_SOURCE_DIR}/cmake/generate_mixin.cmake")

# kept so that other packages can depend on it
ament_package()
