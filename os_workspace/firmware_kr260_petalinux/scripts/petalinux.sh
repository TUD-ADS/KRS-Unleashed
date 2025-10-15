#!/bin/bash
set -Eeuo pipefail

CURRENT_DIR="$(pwd)"
BSP_FILE=$CURRENT_DIR"/artifacts/xilinx-kr260-starterkit-v2024.1-05230256.bsp"
XSA_FILE=$CURRENT_DIR"/artifacts/kria_base_platform.xsa"
PETALINUX_CONFIG_DIR=$CURRENT_DIR"/petalinx_config"
DIR_NAME="build_petalinux"


#TODO add variable check for ${PETALINUX_VER} to be 2024.1

if [ -d "$DIR_NAME" ]; then
	echo "$DIR_NAME already exist."
	rm -Rf $DIR_NAME;
fi

echo "
Step: 1/9	#
"
petalinux-create project -s ${BSP_FILE} --name build_petalinux 

echo "
Step: 2/9	##
"
cd build_petalinux
petalinux-config --get-hw-description ${XSA_FILE} --silentconfig

echo "
Step: 3/9	###
"
petalinux-config -c rootfs --silentconfig

echo "
Step: 4/9	####
"
petalinux-build

echo "Default configuration build successfully, continue with platform specific adjustments"
echo "
Step: 5/9	#####
"
cp -f ${PETALINUX_CONFIG_DIR}/config project-spec/configs/
cp -f ${PETALINUX_CONFIG_DIR}/rootfs_config project-spec/configs/
cp -f ${PETALINUX_CONFIG_DIR}/user-rootfsconfig project-spec/meta-user/conf/
petalinux-config -c rootfs --silentconfig

echo "
Step: 6/9	######
"
petalinux-build

echo "
Step: 7/9	#######
"
petalinux-build --sdk

echo "
Step: 8/9	########
"
petalinux-package boot --u-boot

echo "
Step: 9/9	#########
"
petalinux-package wic --images-dir images/linux/ --bootfiles "ramdisk.cpio.gz.u-boot,boot.scr,Image,system.dtb,system-zynqmp-sck-kr-g-revB.dtb" --size ,8G
echo "
DONE!
"
echo "Linux OS Creation was successful. Current Image can be inspected and flashed under 'images/linux/petalinux-sdimage.wic'. Simply run 'dd if=petalinux-sdimage.wic of=/dev/sdX conv=fsync status=progress' to flash sd card"