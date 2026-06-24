# tracetools_kernels

Low-overhead LTTng tracepoints for timing **arbitrary code regions** inside a ROS 2 application. Wrap any segment with a start/end hook and measure it. Useful on its own for any ROS 2 project (no FPGA required); here it is used to compare hardware- and software-based kernels.

`tracetools_kernels` is a fork of [tracetools](https://github.com/ros2/ros2_tracing/tree/humble/tracetools) (refer to it for the original work) and is inspired by [tracetools_image_pipeline](https://github.com/ros-acceleration/image_pipeline/tree/ros2/tracetools_image_pipeline).

## Usage
* defines 3 tracepoints:
  * `kernel_register` (defines one-time lookup information linked to the ID)
  * `kernel_start` (starts a kernel trace instance)
  * `kernel_end` (stops a kernel trace instance)