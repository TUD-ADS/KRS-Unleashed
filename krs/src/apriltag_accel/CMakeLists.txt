cmake_minimum_required(VERSION 3.5)
project(apriltag_accel)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas -Wno-attributes)
endif()

#################
#    Dependencies
#################

set(VITIS_LIBRARY_PATH /opt/xilinx/Vitis-Libraries/vision/L1/include/)


find_package(ament_vitis) # sets the ROS_VITIS

if (ROS_VITIS)
  set(SYSROOT_INCLUDE_DIRS 
    ${CMAKE_SYSROOT}/usr/include
    ${CMAKE_SYSROOT}/usr/include/xrt
    )
  set(SYSROOT_LINK_LIBRARIES
    -lxilinxopencl
    xrt_coreutil
    )
  message(STATUS "SYSROOT_INCLUDE_DIR: ${SYSROOT_INCLUDE_DIR}")

  include_directories(${SYSROOT_INCLUDE_DIRS} include)

else()
  include_directories(include)
endif() # ROS_VITIS

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Threads REQUIRED)
find_package(adaptive_component REQUIRED)
find_package(tracetools_kernels REQUIRED)  
if (ROS_VITIS)
  find_package(vitis_common REQUIRED)
  find_package(OpenCL REQUIRED)
endif()  # ROS_VITIS



# create ament index resource which references the libraries in the binary dir
set(node_plugins "")


#################
#    Components
#################


set(COMPONENT_AMENT_DEPENDENCIES
  rclcpp
  rclcpp_components 
  std_msgs 
  cv_bridge 
  image_transport
  tracetools_kernels
)

if (ROS_VITIS)

  # Decimate Component
  add_library(decimate_fpga SHARED src/decimate_fpga.cpp)
  target_link_libraries(decimate_fpga  
    ${OpenCL_LIBRARY}
    ${SYSROOT_LINK_LIBRARIES} 
    pthread
  )
  ament_target_dependencies(decimate_fpga ${COMPONENT_AMENT_DEPENDENCIES} vitis_common )
  target_include_directories(decimate_fpga PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${SYSROOT_INCLUDE_DIRS}
    $ENV{XILINX_HLS}/common/technology/autopilot
    $ENV{XILINX_HLS}/include
    ${VITIS_LIBRARY_PATH}
  )
    
  rclcpp_components_register_nodes(
    decimate_fpga 
    PLUGIN "apriltag_accel::DecimateFPGA"
    EXECUTABLE apriltag_decimate_fpga
  )
  ament_export_targets(export_apriltag_decimate_fpga)

  # Blur Component
  add_library(blur_fpga SHARED src/blur_fpga.cpp)
  target_link_libraries(blur_fpga  
    ${OpenCL_LIBRARY}
    ${SYSROOT_LINK_LIBRARIES} 
    pthread
  )
  ament_target_dependencies(blur_fpga ${COMPONENT_AMENT_DEPENDENCIES} vitis_common )
  target_include_directories(blur_fpga PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${SYSROOT_INCLUDE_DIRS}
    $ENV{XILINX_HLS}/common/technology/autopilot
    $ENV{XILINX_HLS}/include
    ${VITIS_LIBRARY_PATH}
  )
  
  rclcpp_components_register_nodes(
    blur_fpga 
    PLUGIN "apriltag_accel::BlurFPGA"
    EXECUTABLE apriltag_blur_fpga
  )
  ament_export_targets(export_apriltag_blur_fpga)

  # Threshold Component
  add_library(threshold_fpga SHARED src/threshold_fpga.cpp)
  target_link_libraries(threshold_fpga  
    ${OpenCL_LIBRARY}
    ${SYSROOT_LINK_LIBRARIES} 
    pthread
  )
  ament_target_dependencies(threshold_fpga ${COMPONENT_AMENT_DEPENDENCIES} vitis_common )
  target_include_directories(threshold_fpga PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${SYSROOT_INCLUDE_DIRS}
    $ENV{XILINX_HLS}/common/technology/autopilot
    $ENV{XILINX_HLS}/include
    ${VITIS_LIBRARY_PATH}
  )
  
  rclcpp_components_register_nodes(
    threshold_fpga 
    PLUGIN "apriltag_accel::ThresholdFPGA"
    EXECUTABLE apriltag_threshold_fpga
  )
  ament_export_targets(export_apriltag_threshold_fpga)

  # Streamed Component
  add_library(streamed_fpga SHARED src/streamed_fpga.cpp)
  target_link_libraries(streamed_fpga  
    ${OpenCL_LIBRARY}
    ${SYSROOT_LINK_LIBRARIES} 
    pthread
  )
  ament_target_dependencies(streamed_fpga ${COMPONENT_AMENT_DEPENDENCIES} vitis_common )
  target_include_directories(streamed_fpga PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${SYSROOT_INCLUDE_DIRS}
    $ENV{XILINX_HLS}/common/technology/autopilot
    $ENV{XILINX_HLS}/include
    ${VITIS_LIBRARY_PATH}
  )
    
  rclcpp_components_register_nodes(
    streamed_fpga 
    PLUGIN "apriltag_accel::StreamedFPGA"
    EXECUTABLE apriltag_streamed_fpga
  )
  ament_export_targets(export_apriltag_streamed_fpga)



endif() # ROS_VITIS


#################
#    Nodes FPGA Node built with a "component"
#################

if (ROS_VITIS)
  # Decimate
  add_executable(decimate_fpga_node src/decimate_fpga_node.cpp)
  target_link_libraries(decimate_fpga_node decimate_fpga)
  ament_target_dependencies(decimate_fpga_node rclcpp class_loader)

  # Blur
  add_executable(blur_fpga_node src/blur_fpga_node.cpp)
  target_link_libraries(blur_fpga_node blur_fpga)
  ament_target_dependencies(blur_fpga_node rclcpp class_loader)

  # Threshold
  add_executable(threshold_fpga_node src/threshold_fpga_node.cpp)
  target_link_libraries(threshold_fpga_node threshold_fpga)
  ament_target_dependencies(threshold_fpga_node rclcpp class_loader)

   # Streamed
   add_executable(streamed_fpga_node src/streamed_fpga_node.cpp)
   target_link_libraries(streamed_fpga_node streamed_fpga)
   ament_target_dependencies(streamed_fpga_node rclcpp class_loader)
endif()

#################
#    Vitis HLS
#################

# if (ROS_VITIS) old approach
#   # resize kernel
#   vitis_acceleration_kernel(
#     NAME gaussian_filter_accel
#     FILE src/xf_blur_accel.cpp
#     CONFIG cfg/kr260.cfg
#     INCLUDE
#       include
#       ${CMAKE_INSTALL_PREFIX}/include
#       ${CMAKE_INSTALL_PREFIX}/include/vitis_common
#     TYPE
#       # sw_emu
#       # hw_emu
#       hw
#     LINK
#     PACKAGE
#     )

# endif()  # ROS_VITIS

#################
#    Install
#################

# install

if (DEFINED ROS_VITIS)
  
  install(TARGETS decimate_fpga
    EXPORT export_apriltag_decimate_fpga
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(TARGETS blur_fpga
    EXPORT export_apriltag_blur_fpga
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(TARGETS threshold_fpga
    EXPORT export_apriltag_threshold_fpga
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(TARGETS streamed_fpga
    EXPORT export_apriltag_streamed_fpga
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  set (install_targets ${install_targets} 
    decimate_fpga_node
    blur_fpga_node
    threshold_fpga_node
    streamed_fpga_node
  )
endif()  # ROS_VITIS

# install targets
install(TARGETS
  ${install_targets}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


ament_package()
