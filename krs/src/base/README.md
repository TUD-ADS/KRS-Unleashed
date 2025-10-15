# KRS Unleashed Base Repository

* this folder wraps the ROS 2 workspace of KRS Unleashed including all necessary utilities and libraries
* everything outside this folder contains application code or other utilities which are not part of the core KRS
* it is based upon the original [Kria Robotics stack](https://xilinx.github.io/KRS/sphinx/build/html/index.html), a ROS 2 set of tools, nodes, and libraries to deploy hardware-accelerated solutions to Kria SOMs.

In this project number of technologies are utilized:
- [AMD Kria 260](https://xilinx.github.io/kria-apps-docs/home/build/html/index.html), a board which is well-suited for robotics.
- [AMD Vitis :tm:](https://www.amd.com/en/products/software/adaptive-socs-and-fpgas/vitis.html), a platform to develop solutions for FPGA.


## Prerequisites (tested)
- Host machine (X86) with Ubuntu 22.04 and plenty of free space (~300 Gb), primarily for Vitis. This machine will also run potential HIL simulation.

1. OS Workspace
2. ROS 2 Workspace

### 1. OS Workspace
* see top-level README for instructions on how to handle OS Workspace


### 2. ROS 2 Workspace
* internal: clone this repo and follow via `git submodule update --init --recursive` to fetch all the referenced repositories
* afterwards, navigate inside the `src/base/krs_firmware` repo and configure the sysroot links (check out [README](krs_firmware/README.md))

## Build
* the build is organized in 2 phases:
  1. native system build, creates `build/` and `install/` folders which respective files; you can run normal ros2 commands afterwards
  2. cross-compile build, creates `build-kr260/` and `install-kr260/` folders; only the `install-kr260/` needs to be copied over to the board

### 1. Native System Build
* `unset RMW_IMPLEMENTATION`
* `source /opt/ros/humble.setup.<shell>` and `<Xilinx Installation>/Vitis/<version>/settings64.sh` (default is 2024.1 for KRS)
* run `export PATH="/usr/bin":$PATH` to override Xilinx cmake version back to default version (necessary for ROS)
* (Optionally:) in case you run multiple OpenCV versions, export `LD_LIBRARY_PATH`, `OPENCV_LIB` and `OPENCV_INCLUDE` accordingly

* run the first build command `colcon build --merge-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON-DCMAKE_BUILD_TYPE=Release` (cmake-args are optional here for VSCode environment; Release tells compiler to optimize code further)
  * takes now only ~40s
* this will prepare a native build environment without `ROS_VITIS`, you can now run everything locally via `source install/setup.<shell>`

#### Build workspace for X86 (Native)
* previous build should have generated all packages in native environment
* if wanted, simply leave out `--mixin kr260` to build without preparing the sysroot linking


### 2. Cross-Compile Build
* `source install/setup.<shell>`
* run `colcon build --build-base=build-kr260 --install-base=install-kr260 --merge-install --mixin kr260 --cmake-args -DNOKERNELS=true -DCMAKE_BUILD_TYPE=Release` to create cross-compiled version in specific directories

#### Old Style Kernel Synthesis
* the command to trigger synthesis is: `colcon build --build-base=build-kr260 --install-base=install-kr260 --merge-install --mixin kr260 --cmake-args -DNOKERNELS=false -DCMAKE_BUILD_TYPE=Release --packages-select <package>`
  * package select necessary if multiple packages included (increases build time a lot)
* this is not tested and not the currently recommended way

#### HLS Accelerator Integration (Preferred Way)
* instead, we left the same references the code but use the `../../vitis` Workspace to synthesize kernels once and simply copy them over to the board
  * make sure that filenames,.. match (inside `apriltag_accel`)
  * make sure you use the right XSA + Sysroot 
  * once done, copy over the respective `exports/folder`(`.xclbin`, `shell.json` and the device-tree overlay)

#### Hints:
* `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` for vscode syntax highlighting
* `-DCMAKE_BUILD_TYPE=Release` faster c code (like -O3)
* ` --merge-install` necessary to allow --build-base,.. in the next step, otherwise files are always put into `install/`
* `--build-base=build-kr260` `--install-base=install-kr260` just specfiy the paths; they need to match the path in the mixin file generated, check
* `--mixin kr260` loads the mixin values into cmake
* `-DNOKERNELS=true` special flag for `ament_vitis` to skip synthesis


## Supported Launch Files
* all launch files expect an image to be published under `/image_raw`

* `ros2 launch apriltag3 trace_apriltag.py` -> multi_node version of Apriltag3
* `ros2 launch apriltag3 tracing_single_node.py` -> single_node version
* `ros2 launch apriltag3 trace_apriltag_nodes.py` _. unoptimized, node version

* `ros2 launch apriltag_cpp trace_apriltag.py` -> multi_node version of ApriltagCPP
* `ros2 launch apriltag_cpp tracing_single_node_apriltag_launch.py` -> single_node version
* `ros2 launch apriltag_cpp trace_apriltag_nodes.py` _. unoptimized, node version
* the other files are just different configurations..

* `ros2 launch apriltag_accel trace_apriltag_accel[0-4].py` different combinations of hardware accelerators, only works on the FPGA after xmutil command was run
* `ros2 launch apriltag_accel final_trace[0-1].py` contains the most optimal setup


## FAQ
* Current Measurements indicate the DDS configuration is highly important

### OpenCV needs to be brought into path
* run `export LD_LIBRARY_PATH=/usr/lib:$LD_LIBRARY_PATH`
* run `export OPENCV_LIB=/usr/lib`
* run `export OPENCV_INCLUDE=/usr/include/opencv4`


### Tracing
* follow installation instructions
* on Ubuntu 22.04 -> the current version for kernel packages is corrupted -> use instead the ppa
  * https://askubuntu.com/questions/1513837/lttng-modules-dkms-install-error/1514304
  * `sudo apt-add-repository ppa:lttng/stable-2.13`
* `sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev`

* Ensure non-root user to be in `tracing` group
  * Create group if it doesn't exist

```bash
$ sudo groupadd -r tracing

# Add <username> to the group
$ sudo usermod -aG tracing <username>
```

* trace manually via or simply check the launch files for easier life
```bash
ros2 trace -k 'sched_switch', 'kmem_mm_page_alloc', 'kmem_mm_page_free', 'power_cpu_frequency' --list
```
* expected output:
```log                                                                               
UST tracing enabled (28 events)
kernel tracing disabled
context (3 names)
writing tracing session to: /home/paul/.ros/tracing/session-20250110201321
press enter to start...
press enter to stop...
stopping & destroying tracing session
```

* single command to add all relevant userland traces
```bash
ros2 trace -u 'ros2:rcl_init', 'ros2:rcl_node_init', 'ros2:rmw_publisher_init', 'ros2:rcl_publisher_init', 'ros2:rclcpp_publish', 'ros2:rclcpp_intra_publish', 'ros2:rcl_publish', 'ros2:rmw_publish', 'ros2:rmw_subscription_init', 'ros2:rcl_subscription_init', 'ros2:rclcpp_subscription_init', 'ros2:rclcpp_subscription_callback_added', 'ros2:rmw_take', 'ros2:rcl_take', 'ros2:rclcpp_take', 'ros2:rcl_service_init', 'ros2:rclcpp_service_callback_added', 'ros2:rmw_take_request', 'ros2:rmw_send_response', 'ros2:rmw_client_init', 'ros2:rcl_client_init', 'ros2:rmw_send_request', 'ros2:rmw_take_response', 'ros2:rcl_timer_init', 'ros2:rclcpp_timer_callback_added', 'ros2:rclcpp_timer_link_node', 'ros2:rclcpp_callback_register', 'ros2:callback_start', 'ros2:callback_end', 'ros2:rcl_lifecycle_state_machine_init', 'ros2:rcl_lifecycle_transition', 'ros2:rclcpp_executor_get_next_ready', 'ros2:rclcpp_executor_wait_for_work', 'ros2:rclcpp_executor_execute', 'ros2:rclcpp_ipb_to_subscription', 'ros2:rclcpp_buffer_to_ipb', 'ros2:rclcpp_construct_ring_buffer', 'ros2:rclcpp_ring_buffer_enqueue', 'ros2:rclcpp_ring_buffer_dequeue', 'ros2:rclcpp_ring_buffer_clear', 'ros2_kernels:kernel_register', 'ros2_kernels:kernel_start', 'ros2_kernels:kernel_end' --list
```

### Current Petalinux Issue

#### OpenCV version missmatch due to not actually cross-compiling against sysroot
dont question...
```bash
sudo ln -s /usr/lib/libopencv_imgproc.so.4.6.0 /usr/lib/libopencv_imgproc.so.4.5d
sudo ln -s /usr/lib/libopencv_core.so.4.6.0 /usr/lib/libopencv_core.so.4.5d
sudo ln -s /usr/lib/libopencv_imgcodecs.so.4.6.0 /usr/lib/libopencv_imgcodecs.so.4.5d
sudo ln -s /usr/lib/libopencv_calib3d.so.4.6.0 /usr/lib/libopencv_calib3d.so.4.5d
```

#### LTTng provides Python bindings for liblttng-ctl, but Petalinux lacks them

* Error message:
```
writing tracing session to: /home/petalinux/.ros/tracing/session-20221108064057
press enter to start...
lttng module not found, but still tried to use it
```

* these are only build with `SWIG` on Ubuntu and installed via `apt install python3-lttng`
* see https://github.com/lttng/lttng-tools/blob/master/doc/python-howto.txt
* as a workaround, we utilize the ARM binaries from the Ubuntu image under:
  * `firmware_kria_ubuntu/sysroots/aarch64-xilinx-linux/usr/lib/python3/dist-packages`
  * and put them on the board under `/usr/lib/python3.10/` on the board