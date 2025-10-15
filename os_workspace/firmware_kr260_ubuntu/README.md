# firmware_kr260_ubuntu Repo (Inofficial Fork of acceleration robotics repo)

* forked from [Acceleration Firmware KR260](https://github.com/ros-acceleration/acceleration_firmware_kr260/)

| Board                                                                                                       | Picture                                                                            | Description                                                                                                                                                                                           |
| ----------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Kria KR260 Robotics Starter Kit](https://www.xilinx.com/products/som/kria/kr260-robotics-starter-kit.html) | ![](https://www.xilinx.com/content/dam/xilinx/imgs/products/som/kr260-angel-1.png) | The Kria™ KR260 is built for robotics and industrial applications, complete with high performance interfaces and native ROS 2 support for ease of development by roboticists and software developers. |


This repository provides AMD/Xilinx's firmware artifacts for the KR260 board using Ubuntu 22.04 meant to accelerate ROS 2 robotic applications.

*It extends the official support of Ubuntu and provides some refactorings to make it more lightweight.*

<ins>**NOTE on Ubuntu 22.04 integration**</ins>: *Ubuntu 22.04 sysroot is provided by Canonical and AMD at their own terms as available at https://ubuntu.com/download/amd-xilinx. Refer to this resource for more information. This repository leverages the resulting artifacts "as they are" and enables ROS 2 build flows while integrating hardware acceleration through Vitis and Vivado tools.*. 

## Use this Workspace

### Fetch from sources
To fetch it from sources, proceed as follows to include the Ubuntu 22.04 resources:

```bash
git clone repo
./prepare_sysroot.sh
```
* this downloads the Ubuntu SDK which we currently also use for cross-compiling against Petalinux
* takes roughly 10min to patch SDK

### Make Workspace Available
* simple alter the cmake path inside `krs_repo/src/krs_firmware/cmake/firmware_setup.cmake`
  * e.g. update `set(FIRMWARE_DIR ${CMAKE_INSTALL_PREFIX}/../../firmware_kr260_ubuntu/firmware)  # <ws>/../firmware_kria_ubuntu`


## FAQ

### Installing Dependencies on the SD Card
* as Ubuntu/Cannonical does not provide a way to create an Image yourself, you have to manually install everything on the board by connecting it to the internet
  * alternatively, you can also download the `.deb` files on your development machine and move them over via `scp`

### Altering the Sysroot
* to create a new patch file (to add missing dependencies,..) go to `customize.sh` script (old location was inside `acceleration/firmware/kr260/sysroots/bin/`, but should be inside every downloaded `sysroot.tar.gz`)
  * look under `firmware/sysroots/bin/customize.sh`
  * check to use an unpatched customize script (ideally export from tar.gz directly)
* apply your changes, then run `diff -u customize_old.sh customize_new.sh  > customize.patch`
* and replace patch file at `patches/customize.patch`
* you can also manually verify via `patch --dry-run <file> <patch>` 


## Current Manual Steps on SD Card to get it running
* Download Image from Website
* plug ethernet cable at bottom row
* update system `apt-get update && apt-get upgrade`
* activate swapfile
* install ros2
* install lttng (dont forget python3-lttng), cgal
* install vitis ai 
* unplug, plug ssh ethernet cable at top row
* update network config eth1 