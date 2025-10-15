#!/bin/bash -exu

set -o pipefail

if [ -d "firmware" ]; then
  echo "firmware does already exist; skip download and extracting assuming everything is fine."
else
  wget https://people.canonical.com/~platform/images/xilinx/kria-ubuntu-22.04/iot-limerick-kria-classic-desktop-2204-x06-20220614-78-sysroot.tar.xz -P firmware
  tar -xf firmware/iot-limerick-kria-classic-desktop-2204-x06-20220614-78-sysroot.tar.xz -C firmware
fi

echo "Ubuntu sysroot deployed."

# apply patch
patch firmware/sysroots/bin/customize.sh patches/customize.patch

# install ROS 2, LTTng and net-tools inside of the sysroot
sudo firmware/sysroots/bin/customize.sh firmware/sysroots/aarch64-xilinx-linux jammy http://packages.ros.org/ros2/ubuntu '' net-tools > /dev/null

echo "Sysroot cooked for cross-compilation."
echo "Can be linked from other workspace under $(pwd)/firmware/sysroots"