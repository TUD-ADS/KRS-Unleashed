# KRS Unleashed

<img width="726" height="429" alt="High-level architecture overview of KRS Unleashed" src="https://github.com/user-attachments/assets/0175b876-3983-4232-aed0-95284224bef6" />

*Architecture overview of KRS Unleashed. The original KRS's single ROS 2 workspace was separated into three workspaces.*

This repository implements an example Apriltag Demo Application using the newly developed KRS Unleashed flow in Ubuntu 22.04 (ROS 2 humble).
The Code is an architectural redesign of the original [Kria Robotics Stack](https://xilinx.github.io/KRS/sphinx/build/html/index.html) into 3 separate workspaces represented here via the 3 top-level folders:
* `krs`: the original, slimmed-down ROS 2 workspace containing the application, the host code for FPGA acceleration inside nodes and hardware acceleration utilities necessary for cross-compilation,..
* `os_workspace`: the firmware (OS to be put on the FPGA CPU) including scripts to generate a sysroot (target file system) for cross-compilation; Currently contains Ubuntu 22.04 and Petalinux 2024.1
* `Vitis`: a newly devised Vitis Workspace Flow automating many steps of the Vitis Unified Software Platform via Python to generate the partial bitstream (.xclbin) via the Vitis Flow (.xo kernels linked via v++). The flow can be run semi-automatically and the Vitis Workspace opened and manually inspected and modified at any given time.

---
## Main Benefits
* decoupling cuts recurrent build times by up to 80 times (for reusable artifacts like sysroot creation)
* allows individual components of the stack to be used in other solutions without using the whole flow:
  * `kernel_tracetools`: a low-overhead LTTng tracing module that extends ros-tracing with user/application code tracing capabilities for dedicated regions, regions can be defined arbitrarily (no FPGA required)
  * `Vitis` an automated, scripted Vitis HLS -> .xclbin + pl.dtbo flow that allows to easily automate mass kernel synthesis via Python CLI but still gives you a fully built Vitis Workspace for customization (Useful for any Vitis HLS developer)
  * `os_workspace` + `krs/src/firmware`: scripted cross-compilation flow including sysroot and colcon mixin generation (portable to other devices, requires minimal adjustments)


<img width="728" height="301" alt="Proposed development workflow of KRS Unleashed across the three workspaces" src="https://github.com/user-attachments/assets/623906d6-2416-47d6-969e-ff68eb25eb6a" />

*Proposed development flow of KRS Unleashed. The workspaces have a logical order, but intermediary results can also be used to enable parallel development. Fat arrows highlight new capabilities.*

The proposed workflow is explained in detail in a hackster series:
* [1. Getting Started](https://www.hackster.io/paul-gottschaldt/krs-unleashed-1-getting-started-5c38f3)
* [2. OS Workspace](https://www.hackster.io/paul-gottschaldt/krs-unleashed-2-os-workspace-13c0d9)
* [3. ROS Workspace](https://www.hackster.io/paul-gottschaldt/krs-unleashed-3-krs-workspace-807e60)
* [4. Vitis Flow](https://www.hackster.io/paul-gottschaldt/krs-unleashed-4-vitis-workspace-fe4a0d)
* [5. Putting Everything together on the board](https://www.hackster.io/paul-gottschaldt/krs-unleashed-5-running-on-the-board-4a49e0)

---

## Author/Citation

If you use this work, please cite the original paper:

P. Gottschaldt and D. Goehringer, "KRS Unleashed: Towards a Robotics FPGA Development Environment for Rapid Prototyping," 2025 IEEE Nordic Circuits and Systems Conference (NorCAS), Riga, Latvia, 2025, pp. 1-7, doi: [10.1109/NorCAS66540.2025.11231288](https://doi.org/10.1109/NorCAS66540.2025.11231288). 

**Paul Gottschaldt**  
TU Dresden, Chair of Adaptive Dynamic Systems  
[ORCID](https://orcid.org/0000-0002-4878-8656)

---

## AprilTag Demo Application
* AprilTag visual detection algorithm based on [Swarthmore College Robotics Lab](https://github.com/swatbotics/apriltags-cpp)
* accelerators for gray conversion, blur and adaptive thresholding, based on Vitis Vision library (2024.1)


### Prerequisites (tested)
- Host machine (X86) with Ubuntu 22.04 and plenty of free space (~300 Gb), primarily for Vitis. This machine will also run potential HIL simulation.

1. OS Workspace and SD Card preparation on the FPGA board (KR260):
   1. Ubuntu
   2. Petalinux
2. Prepare the ROS 2 Workspace
3. Generate the Kernels in the Vitis Workspace
4. Load Everything onto the board and execute

#### 1.1 Ubuntu on KR260
* even when only using the Ubuntu OS, it is recommend to clone the Petalinux environment as well, as it is still necessary for the Vitis Flow
- KR260 board with Ubuntu 22.04, which you set up following [this guide](https://www.amd.com/en/products/system-on-modules/kria/k26/kr260-robotics-starter-kit/getting-started/setting-up-the-sd-card-image.html).
- firmware should be in the `os_workspace/firmware_kr260_ubuntu` folder
* Prepare system for cross compilation (right now only works for Ubuntu, but Petalinux can be built with same dependencies)
  * this will create the missing python on your development system from the used sysroot (make sure to use full paths not relative)
```bash 
sudo ln -s <firmware_kr260_ubuntu>/firmware/sysroots/aarch64-xilinx-linux/usr/lib/aarch64-linux-gnu/libpython3.10.so.1.0 /usr/lib/aarch64-linux-gnu/libpython3.10.so -f
```

#### 1.2 Petalinux on KR260
- firmware should be in the `os_workspace/kr260_petalinux` folder
- currently still requires the Ubuntu Firmware sysroot due to a meta-ros bug (v2024.1)


#### 2. ROS 2 Workspace
* follow instructions inside `krs`
* afterwards, navigate inside the `src/base/krs_firmware` repo and configure the sysroot links (check out [README](krs_firmware/README.md))

#### 3. Vitis Workspace
* follow instructions inside `Vitis`
* you should end up with a `export_xxx` folder containing all the necessary files to be put on the board

#### 4. Run the Board

### FPGA Preparation
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
