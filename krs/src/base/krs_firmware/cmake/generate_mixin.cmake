# Copyright (c) 2022, Acceleration Robotics
# Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
#
# generate and add mixins at build-time to further simplify embedded flows
#
# NOTE: this logic is specific to KR260

message(STATUS "Creating mixins for the overlay ROS 2 workspace")

set(TEMPLATE_YAML "${CMAKE_BINARY_DIR}/index.yaml.template")
set(MIXIN_DIR "${FIRMWARE_DIR}/mixin") # just to get some artifacts back for debugging, actual are installed
set(INDEX_YAML "${MIXIN_DIR}/index.yaml")

if(KRS_DEVICE STREQUAL "kr260")
  set(TEMPLATE_MIXIN "${CMAKE_BINARY_DIR}/kr260.mixin.template")
  set(MIXIN "${MIXIN_DIR}/kr260.mixin")
endif()


# arguments
# - ARG1_*: firmware directory, resulting toolchain should be here
# - ARG2_*: OS specific toolchain
# - ARG3_*: root of the install directory
# - set SYSROOT PATH via export SYSROOT_PATH=<absolute path>

set(ARG1_FIRMWARE_DIR ${FIRMWARE_DIR})

if(DEFINED KRS_OS)
  if(KRS_OS STREQUAL "ubuntu")
    set(ARG2_OS_TOOLCHAIN aarch64-linux-gnu)
    set(ARG4_OS_SYSROOT_PATH aarch64-xilinx-linux)
    set(ARG5_ROS_PATH_PREFIX ${ARG4_OS_SYSROOT_PATH}/opt/ros/humble) #default installation for Ubuntu under /opt/ros/humble
    set(ARG6_ROS_LIB_PATH ${ARG4_OS_SYSROOT_PATH}/usr/lib)
  elseif(KRS_OS STREQUAL "petalinux")
    set(ARG2_OS_TOOLCHAIN aarch64-xilinx-linux)
    set(ARG4_OS_SYSROOT_PATH cortexa72-cortexa53-xilinx-linux)
    set(ARG5_ROS_PATH_PREFIX ${ARG4_OS_SYSROOT_PATH}/usr)
    set(ARG6_ROS_LIB_PATH ${ARG4_OS_SYSROOT_PATH}/usr/lib)
  else()
    message(FATAL_ERROR "KRS_OS wrongly configured" ${KRS_OS})
  endif()
endif()

set(ARG3_INSTALL_DIR ${CMAKE_INSTALL_PREFIX})
message(STATUS "ARG1_FIRMWARE_DIR: " ${ARG1_FIRMWARE_DIR})
message(STATUS "ARG2_OS_TOOLCHAIN: " ${ARG2_OS_TOOLCHAIN})
message(STATUS "ARG3_INSTALL_DIR: " ${ARG3_INSTALL_DIR})
message(STATUS "ARG4_OS_SYSROOT_PATH: " ${ARG4_OS_SYSROOT_PATH})
message(STATUS "ARG5_ROS_PATH_PREFIX: " ${ARG5_ROS_PATH_PREFIX})
message(STATUS "ARG6_ROS_LIB_PATH: " ${ARG6_ROS_LIB_PATH})

# deploy in firmware
if (EXISTS ${MIXIN_DIR})
  run("rm -rf ${MIXIN_DIR}")  
endif()
run("mkdir -p ${MIXIN_DIR}")
run("mv ${TEMPLATE_MIXIN} ${MIXIN}")
run("mv ${TEMPLATE_YAML} ${INDEX_YAML}")
# replace placeholders
set(SEDEXP_ARG1 "s:ARG1_FIRMWARE_DIR:${ARG1_FIRMWARE_DIR}:g")
set(SEDEXP_ARG2 "s:ARG2_OS_TOOLCHAIN:${ARG2_OS_TOOLCHAIN}:g")
set(SEDEXP_ARG3 "s:ARG3_INSTALL_DIR:${ARG3_INSTALL_DIR}:g")
set(SEDEXP_ARG4 "s:ARG4_OS_SYSROOT_PATH:${ARG4_OS_SYSROOT_PATH}:g")
set(SEDEXP_ARG5 "s:ARG5_ROS_PATH_PREFIX:${ARG5_ROS_PATH_PREFIX}:g")
set(SEDEXP_ARG6 "s:ARG6_ROS_LIB_PATH:${ARG6_ROS_LIB_PATH}:g")
run("sed -i ${SEDEXP_ARG1} ${MIXIN}")
run("sed -i ${SEDEXP_ARG2} ${MIXIN}")
run("sed -i ${SEDEXP_ARG3} ${MIXIN}")
run("sed -i ${SEDEXP_ARG4} ${MIXIN}")
run("sed -i ${SEDEXP_ARG5} ${MIXIN}")
run("sed -i ${SEDEXP_ARG6} ${MIXIN}")

# enable mixins
set(REDIRECT_STDERR_STR "2> /dev/null")
run("colcon mixin remove kr260 2> /dev/null")  # clean up prior stuff
set(ADD_MIXIN_PATH "file://${FIRMWARE_DIR}/mixin/index.yaml")
run("colcon mixin add kr260 ${ADD_MIXIN_PATH}")
run("colcon mixin update kr260")
