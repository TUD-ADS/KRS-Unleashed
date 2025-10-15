# KRS Firmware
* originated from the repository [acceleration_firmware_kr260](https://github.com/KRS-AMD/acceleration_firmware_kr260)
* heavily adjusted to account for the separation of ROS 2 and OS workspaces:
  * renamed module from acceleration_firmware_kr260 into krs_firmware
  * now contains a lightweight cross-compilation wrapper which is board and OS indepedant
  * outsourced sysroot creation
  * adjusted templates for Petalinux support
  * firmware_setup is now configurable and not automatic anymore

This repository provides now only cross-compilation preparation via colcon mixin's and firmware path setup to link against the OS workspace

## Usage
* navigate to `cmake/firmware_setup.cmake`
* update the following properties:
```CMake
set(KRS_DEVICE kr260) #currently the only tested one
set(KRS_OS ubuntu) #supported values are currently ['ubuntu', 'petalinux']
set(FIRMWARE_DIR ${CMAKE_INSTALL_PREFIX}/../../../firmwares/firmware_kr260_ubuntu/firmware)  # <ws>/../firmware_kria_ubuntu
```
* `KRS_DEVICE` and `KRS_OS` are for future features
* `FIRMWARE_DIR` needs to point to the folder that contains the directory `sysroots`
  * this is relative to your install folder e.g. `../` is `ros2`, `../..` is the `project root`,..
  * currently, we only use the ubuntu version with kr260, even when targeting petalinux due to issues in the meta-ros layer for the sysroot 