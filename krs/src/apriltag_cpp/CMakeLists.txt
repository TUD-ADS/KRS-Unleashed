cmake_minimum_required(VERSION 3.8)
project(apriltag_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tracetools_kernels REQUIRED)
find_package(apriltag_utils REQUIRED)


find_package( CGAL QUIET COMPONENTS  )

if (CGAL_FOUND)
  include( ${CGAL_USE_FILE} )
  add_definitions(-DHAVE_CGAL)
  find_package( Boost REQUIRED )
else()
  message("CGAL not found, can not use new quad detector")
endif()

 set(COMPONENT_AMENT_DEPENDENCIES
 rclcpp
 rclcpp_components 
 std_msgs 
 cv_bridge 
 image_transport
 tracetools_kernels
 OpenCV
 sensor_msgs
 apriltag_utils
)

file(GLOB apriltag_SRC
"src/apriltag/*.cpp"
)

# Apriltag Base Library
file(GLOB apriltag_files
"src/apriltag/*.cpp"
"src/apriltag/*.h"
)

add_library(apriltag_base
  ${apriltag_files}
)

target_include_directories(apriltag_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/apriltag>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

install(TARGETS apriltag_base
        EXPORT export_apriltag_base
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_export_targets(export_apriltag_base)

# Single Node full Apriltag CPP Algorithm
add_executable(apriltag_cpp_node src/apriltag_cpp_node.cpp ${apriltag_SRC})
ament_target_dependencies(apriltag_cpp_node ${COMPONENT_AMENT_DEPENDENCIES})

target_include_directories(apriltag_cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_link_libraries(apriltag_cpp_node apriltag_base)

install(TARGETS apriltag_cpp_node
  DESTINATION lib/${PROJECT_NAME})

# Decimate
## Node
add_executable(apriltag_decimate_node src/apriltag_decimate_node.cpp ${apriltag_SRC})
ament_target_dependencies(apriltag_decimate_node ${COMPONENT_AMENT_DEPENDENCIES})

target_include_directories(apriltag_decimate_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(apriltag_decimate_node apriltag_base)

install(TARGETS apriltag_decimate_node
  DESTINATION lib/${PROJECT_NAME})

## Component
add_library(apriltag_decimate_component SHARED src/apriltag_decimate_component.cpp ${apriltag_SRC})
target_link_libraries(apriltag_decimate_component apriltag_base)
ament_target_dependencies(apriltag_decimate_component ${COMPONENT_AMENT_DEPENDENCIES})

rclcpp_components_register_nodes(
  apriltag_decimate_component 
  PLUGIN "apriltag_cpp::ApriltagDecimatorComponent"
  EXECUTABLE apriltag_decimate
)
ament_export_targets(export_apriltag_decimate_component)

target_include_directories(apriltag_decimate_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  
install(TARGETS apriltag_decimate_component
        EXPORT export_apriltag_decimate_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)


# Blur
## Node

add_executable(apriltag_blur_node src/apriltag_blur_node.cpp ${apriltag_SRC})
ament_target_dependencies(apriltag_blur_node ${COMPONENT_AMENT_DEPENDENCIES})
  
target_include_directories(apriltag_blur_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(apriltag_blur_node apriltag_base)
  
install(TARGETS apriltag_blur_node
  DESTINATION lib/${PROJECT_NAME})

## Component
add_library(apriltag_blur_component SHARED src/apriltag_blur_component.cpp ${apriltag_SRC})
target_link_libraries(apriltag_blur_component apriltag_base)
ament_target_dependencies(apriltag_blur_component ${COMPONENT_AMENT_DEPENDENCIES})

rclcpp_components_register_nodes(
  apriltag_blur_component 
  PLUGIN "apriltag_cpp::ApriltagBlurComponent"
  EXECUTABLE apriltag_blur
)
ament_export_targets(export_apriltag_blur_component)

target_include_directories(apriltag_blur_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  
install(TARGETS apriltag_blur_component
        EXPORT export_apriltag_blur_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Threshold
## Node

add_executable(apriltag_threshold_node src/apriltag_threshold_node.cpp ${apriltag_SRC})
ament_target_dependencies(apriltag_threshold_node ${COMPONENT_AMENT_DEPENDENCIES})

target_include_directories(apriltag_threshold_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(apriltag_threshold_node apriltag_base)

install(TARGETS apriltag_threshold_node
  DESTINATION lib/${PROJECT_NAME})

## Component
add_library(apriltag_threshold_component SHARED src/apriltag_threshold_component.cpp ${apriltag_SRC})
target_link_libraries(apriltag_threshold_component apriltag_base)
ament_target_dependencies(apriltag_threshold_component ${COMPONENT_AMENT_DEPENDENCIES})

rclcpp_components_register_nodes(
  apriltag_threshold_component 
  PLUGIN "apriltag_cpp::ApriltagThresholdComponent"
  EXECUTABLE apriltag_threshold
)
ament_export_targets(export_apriltag_threshold_component)

target_include_directories(apriltag_threshold_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  
install(TARGETS apriltag_threshold_component
        EXPORT export_apriltag_threshold_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Contours
## Node
add_executable(apriltag_contours_node src/apriltag_contours_node.cpp ${apriltag_SRC})
ament_target_dependencies(apriltag_contours_node ${COMPONENT_AMENT_DEPENDENCIES})

target_include_directories(apriltag_contours_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(apriltag_contours_node apriltag_base)

install(TARGETS apriltag_contours_node
  DESTINATION lib/${PROJECT_NAME})

## Component
add_library(apriltag_contours_component SHARED src/apriltag_contours_component.cpp ${apriltag_SRC})
target_link_libraries(apriltag_contours_component apriltag_base)
ament_target_dependencies(apriltag_contours_component ${COMPONENT_AMENT_DEPENDENCIES})

rclcpp_components_register_nodes(
  apriltag_contours_component 
  PLUGIN "apriltag_cpp::ApriltagContoursComponent"
  EXECUTABLE apriltag_contours
)
ament_export_targets(export_apriltag_contours_component)

target_include_directories(apriltag_contours_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  
install(TARGETS apriltag_contours_component
        EXPORT export_apriltag_contours_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Hulls
## Node
add_executable(apriltag_hulls_node src/apriltag_hulls_node.cpp ${apriltag_SRC})
ament_target_dependencies(apriltag_hulls_node ${COMPONENT_AMENT_DEPENDENCIES})

target_include_directories(apriltag_hulls_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(apriltag_hulls_node apriltag_base)

install(TARGETS apriltag_hulls_node
  DESTINATION lib/${PROJECT_NAME})

## Component
add_library(apriltag_hulls_component SHARED src/apriltag_hulls_component.cpp ${apriltag_SRC})
target_link_libraries(apriltag_hulls_component apriltag_base)
ament_target_dependencies(apriltag_hulls_component ${COMPONENT_AMENT_DEPENDENCIES})

rclcpp_components_register_nodes(
  apriltag_hulls_component 
  PLUGIN "apriltag_cpp::ApriltagHullsComponent"
  EXECUTABLE apriltag_hulls
)
ament_export_targets(export_apriltag_hulls_component)

target_include_directories(apriltag_hulls_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  
install(TARGETS apriltag_hulls_component
        EXPORT export_apriltag_hulls_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Quads
## Node
add_executable(apriltag_quads_node src/apriltag_quads_node.cpp ${apriltag_SRC})
ament_target_dependencies(apriltag_quads_node ${COMPONENT_AMENT_DEPENDENCIES})

target_include_directories(apriltag_quads_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(apriltag_quads_node apriltag_base)

install(TARGETS apriltag_quads_node
  DESTINATION lib/${PROJECT_NAME})

## Component
add_library(apriltag_quads_component SHARED src/apriltag_quads_component.cpp ${apriltag_SRC})
target_link_libraries(apriltag_quads_component apriltag_base)
ament_target_dependencies(apriltag_quads_component ${COMPONENT_AMENT_DEPENDENCIES})

rclcpp_components_register_nodes(
  apriltag_quads_component 
  PLUGIN "apriltag_cpp::ApriltagQuadsComponent"
  EXECUTABLE apriltag_quads
)
ament_export_targets(export_apriltag_quads_component)

target_include_directories(apriltag_quads_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  
install(TARGETS apriltag_quads_component
        EXPORT export_apriltag_quads_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Tag Decode
## Node
add_executable(apriltag_tag_decode_node src/apriltag_tag_decode_node.cpp ${apriltag_SRC})
ament_target_dependencies(apriltag_tag_decode_node ${COMPONENT_AMENT_DEPENDENCIES})

target_include_directories(apriltag_tag_decode_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(apriltag_tag_decode_node apriltag_base)

install(TARGETS apriltag_tag_decode_node
  DESTINATION lib/${PROJECT_NAME})

## Component
add_library(apriltag_tag_decode_component SHARED src/apriltag_tag_decode_component.cpp ${apriltag_SRC})
target_link_libraries(apriltag_tag_decode_component apriltag_base)
ament_target_dependencies(apriltag_tag_decode_component ${COMPONENT_AMENT_DEPENDENCIES})

rclcpp_components_register_nodes(
  apriltag_tag_decode_component 
  PLUGIN "apriltag_cpp::ApriltagTagDecodeComponent"
  EXECUTABLE apriltag_tag_decode
)
ament_export_targets(export_apriltag_tag_decode_component)

target_include_directories(apriltag_tag_decode_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  
install(TARGETS apriltag_tag_decode_component
        EXPORT export_apriltag_tag_decode_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
                
# Launch Files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
