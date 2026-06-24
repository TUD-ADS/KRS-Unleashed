# KRS Unleashed Base

This folder holds the **core KRS Unleashed packages** — the utilities and libraries the framework needs. Everything *outside* this folder (in `krs/src/`) is application code or extra tooling, not part of the core.

It is based on the original [Kria Robotics Stack](https://xilinx.github.io/KRS/sphinx/build/html/index.html), a ROS 2 set of tools, nodes, and libraries for deploying hardware-accelerated solutions to Kria SOMs.

## What's in here

| Package | Purpose |
| --- | --- |
| [`acceleration/`](acceleration/README.md) | Slimmed-down KRS acceleration utilities (`adaptive_component`, `ament_acceleration`, `ament_vitis`, `vitis_common`). |
| [`krs_firmware/`](krs_firmware/README.md) | Lightweight, board/OS-independent cross-compilation wrapper that links the ROS 2 workspace against the OS workspace (sysroot + colcon mixins). |
| [`tracetools_kernels/`](tracetools_kernels/README.md) | LTTng tracepoints for timing arbitrary code regions inside ROS 2 (no FPGA required). |

## Building

The build steps for the whole ROS 2 workspace live in the parent **[`krs/README.md`](../../README.md)** — there's no separate build flow for this folder. Before your first build, set the sysroot links in [`krs_firmware/`](krs_firmware/README.md).
