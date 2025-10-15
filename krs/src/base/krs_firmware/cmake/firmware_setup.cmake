# Copyright (c) 2022, Acceleration Robotics
# Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
#
# Set up the firmware variables for this cmake level and define macros

set(KRS_DEVICE kr260) #currently the only tested one
set(KRS_OS ubuntu) #supported values are currently ['ubuntu', 'petalinux']
set(FIRMWARE_DIR ${CMAKE_INSTALL_PREFIX}/../../../firmwares/firmware_kr260_ubuntu/firmware)  # <ws>/../firmware_kria_ubuntu

# CMake checks
## KRS_DEVICE
if(DEFINED KRS_DEVICE)
  if(KRS_DEVICE STREQUAL "kr260")
    message(STATUS "KRS_DEVICE: " ${KRS_DEVICE})
  else()
    message(FATAL_ERROR "KRS_DEVICE wrongly configured" ${KRS_DEVICE})
  endif()
endif()

## KRS_OS
if(DEFINED KRS_OS)
  if(KRS_OS STREQUAL "ubuntu")
    message(STATUS "KRS_OS: " ${KRS_OS})
  elseif(KRS_OS STREQUAL "petalinux")
    message(STATUS "KRS_OS: " ${KRS_OS})
  else()
    message(FATAL_ERROR "KRS_OS wrongly configured" ${KRS_OS})
  endif()
endif()

## FIRMWARE_DIR
if(NOT DEFINED FIRMWARE_DIR)
  message(FATAL_ERROR "FIRMWARE_DIR not defined")
endif()

# run() macro
#  runs the CMD passed as an argument
macro(run CMD)
  execute_process(COMMAND bash -c ${CMD})
endmacro()
