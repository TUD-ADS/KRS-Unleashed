# KRS Base Acceleration

This folder contains helpful utilities from the original KRS:

- [adaptive_component](https://github.com/KRS-AMD/adaptive_component)
- [ament_acceleration](https://github.com/KRS-AMD/ament_acceleration)
- [ament_vitis](https://github.com/KRS-AMD/ament_vitis)
- [vitis_common](https://github.com/KRS-AMD/vitis_common)


## Adaptive Component
* not altered
* contains utilities to combine hardware-accelerated and SW-based nodes as shared library inside a single component node
* this allows to dynamically switch between hardware-accelerated and sw-based version via the parameter server
* nice for adaptive systems, but switch occurs some latency overhead

## Ament Acceleration
* only removed functionality here
* originally used via `colcon acceleration select xxxx`, which required firmware to be present in the installed directory
* this was removed; but we kept the functionality to enable capabilities for hardware-accelerated packages at build-time

## Ament Vitis
* not altered
* original Vitis HLS flow
* kept to define some cmake variables

## Vitis Common
* removed outdated Vitis Library includes
* refactored to contain only OpenCL/XRT setup code 



In this simple project number of technologies are utilized:
- [AMD Kria 260](https://xilinx.github.io/kria-apps-docs/home/build/html/index.html), a board which is well-suited for robotics.
- [AMD Vitis :tm:](https://www.amd.com/en/products/software/adaptive-socs-and-fpgas/vitis.html), a platform to develop solutions for FPGA.
- [Kria Robotics stack](https://xilinx.github.io/KRS/sphinx/build/html/index.html), a ROS 2 set of tools, nodes, and libraries to deploy hardware-accelerated solutions to Kria SOMs.


