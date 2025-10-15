# tracetools_kernels

LTTng tracing provider wrapprer for image_pipeline ROS 2 meta-package. `tracetools_kernels` is a fork of [tracetools](https://github.com/ros2/ros2_tracing/tree/humble/tracetools), refer to this package for the original work.

Additionally, this package is inspired by [tracetools_image_pipeline](https://github.com/ros-acceleration/image_pipeline/tree/ros2/tracetools_image_pipeline)

## Usage
* defines 3 tracepoints:
  * `kernel_register` (defines one-time lookup information linked to the ID)
  * `kernel_start` (starts a kernel trace instance)
  * `kernel_end` (stops a kernel trace instance)