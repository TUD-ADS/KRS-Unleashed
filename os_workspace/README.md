# OS Workspace

This workspace builds the **firmware that runs on the board's CPU** and the **sysroot** used to cross-compile the ROS 2 application against it. It is the first step of the flow.

Two flows are supported — pick one:

| Flow                 | When to use                                                                                         | Start here                                                        |
| -------------------- | --------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------- |
| **Ubuntu** (default) | Fastest path; familiar for ROS 2 development and prototyping. Run `./prepare_sysroot.sh` (~10 min). | [`firmware_kr260_ubuntu/`](firmware_kr260_ubuntu/README.md)       |
| **Petalinux**        | Full control of the OS for a hardened, minimal image. (full system build takes considerable longer) | [`firmware_kr260_petalinux/`](firmware_kr260_petalinux/README.md) |

> Even if you only target Petalinux, you currently still need the Ubuntu sysroot for cross-compilation (meta-ros bug, v2024.1).
> Updates are internally developed for pure Embedded Linux and Ubuntu 24.04, currently not released 
