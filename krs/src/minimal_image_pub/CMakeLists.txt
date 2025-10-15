cmake_minimum_required(VERSION 3.8)
project(minimal_image_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Threads REQUIRED)


file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/input.png
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})


add_executable(minimal_image_pub src/minimal_image_pub.cpp)
target_link_libraries(minimal_image_pub
)
target_include_directories(minimal_image_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(minimal_image_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(minimal_image_pub rclcpp std_msgs cv_bridge image_transport)

install(TARGETS minimal_image_pub
  DESTINATION lib/${PROJECT_NAME})

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
