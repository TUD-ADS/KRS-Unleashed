# Apriltag Demo Application

* Apriltag demo using the Kria Robotic Stack
* accelerators for gray conversion, blur and adaptive thresholding, based on Vitis Vision library (2024.1)
* internal: after check out run `git submodule update --init --recursive` to fetch all the code inside the submodules

## Folder
* `Vitis` contains the Vitis Workspace with the accelerators
* `krs` contains the ROS 2 workspace
* public: the `os_workspace` contains the Ubuntu and Petalinux
* internal: the `os_workspace` repo is expected to be in the same parent directory as this folder e.g. under `../firmwares`

## Prerequisites (tested)
- Host machine (X86) with Ubuntu 22.04 and plenty of free space (~300 Gb), primarily for Vitis. This machine will also run potential HIL simulation.

1. OS Workspace and SD Card preparation on the FPGA board (KR260):
   1. Ubuntu
   2. Petalinux
2. Prepare the ROS 2 Workspace
3. Generate the Kernels in the Vitis Workspace
4. Load Everything onto the board and execute

### 1.1 Ubuntu on KR260
* even when only using the Ubuntu OS, it is recommend to clone the Petalinux environment as well, as it is still necessary for the Vitis Flow
- KR260 board with Ubuntu 22.04, which you set up following [this guide](https://www.amd.com/en/products/system-on-modules/kria/k26/kr260-robotics-starter-kit/getting-started/setting-up-the-sd-card-image.html).
- public: firmware should be in the `os_workspace/firmware_kr260_ubuntu` folder
- internal: clone the [Ubuntu Firmware](https://git-ads.inf.tu-dresden.de/krs/firmware_kr260_ubuntu) outside of this repo and follow readme to configure the sysroot
* Prepare system for cross compilation (right now only works for Ubuntu, but Petalinux can be build with same dependencies)
  * this will create the missing python on your development system from the used sysroot (make sure to use full paths not relative)
```bash 
sudo ln -s <firmware_kr260_ubuntu>/firmware/sysroots/aarch64-xilinx-linux/usr/lib/aarch64-linux-gnu/libpython3.10.so.1.0 /usr/lib/aarch64-linux-gnu/libpython3.10.so -f
```

### 1.2 Petalinux on KR260
- public: firmware should be in the `os_workspace/kr260_petalinux` folder
- internal: build Petalinux-based firmware yourself via [ROS 2 Petalinux Firmware](https://git-ads.inf.tu-dresden.de/krs/firmware_kr260)
- currently still requires the Ubuntu Firmware sysroot due to a meta-ros bug (v2024.1)


### 2. ROS 2 Workspace
* follow instructions inside `krs`
* afterwards, navigate inside the `src/base/krs_firmware` repo and configure the sysroot links (check out [README](krs_firmware/README.md))

### 3. Vitis Workspace
* follow instructions inside `vitis`
* you should end up with a `export_xxx` folder containing all the necessary files to be put on the board

### 4. Run the Board

## FPGA Preparation
* adjust the paths to your system name, user and IPs accordingly 

1. copy the compiled ROS 2 modules over to run ros nodes via:

```bash
cd $KRS_WS
scp -r install-kr260  ubuntu@192.168.2.2:/home/ubuntu/
```
* Next on the board (after `ssh ubuntu`/`ssh petalinux`):
2. prepare xmutil commands to load HLS kernels

Ubuntu
```bash
sudo cp -r /home/ubuntu/<exports_folder> /usr/lib/firmware/xilinx #preparation for xmutil command
```
Petalinux
```bash
sudo cp -r /home/ubuntu/<exports_folder> /lib/firmware/xilinx #preparation for xmutil command
```

3. run the xmutil commands: (`<accel>` needs to be the folder name, you can see accepted values via `listapps` cmd)
```bash
sudo xmutil listapps #Queries on target FW resource manager daemon of pre-built app bitstreams available on the platform and provides summary to CLI.
sudo xmutil unloadapp #Removes application bitstream. (Takes slot number, default 0)
sudo xmutil loadapp <accel> #Loads requested application configuration bitstream to programmable logic if the device is available.
```
4. source KRS and run accelerated node:
Ubuntu
```bash
source /home/ubuntu/install-kr260/setup.bash
source /opt/ros/humble/setup.bash
ros2 <run/launch> <package> <node/launch_file>
```

Petalinux 2024.1
```bash
source /usr/bin/ros_setup.sh
source /home/petalinux/install-kr260/setup.sh
ros2 <run/launch> <package> <node/launch_file>
```