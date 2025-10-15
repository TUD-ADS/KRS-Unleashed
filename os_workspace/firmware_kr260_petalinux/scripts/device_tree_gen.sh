#!/bin/bash
set -Eeuo pipefail

DIR_NAME="../../build/platform/dtg_output"
EXPORT_FOLDER="../../build/export/template"
SHELL_JSON_PATH="../resources/shell.json"

if [ -d "$DIR_NAME" ]; then
	echo "$DIR_NAME already exist."
	rm -Rf $DIR_NAME;
fi

if [ -d "$EXPORT_FOLDER" ]; then
	echo "$EXPORT_FOLDER already exist."
	rm -Rf $EXPORT_FOLDER;
fi



echo "
Step: 1/2	#
"
export PLATFORM_NAME="kria_kr260_base_accel"
xsct ../scripts/xsct.tcl $PLATFORM_NAME $DIR_NAME

echo "
Step: 2/2	#
"

dtc -@ -O dtb -o ./$DIR_NAME/$DIR_NAME/$PLATFORM_NAME/psu_cortexa53_0/device_tree_domain/bsp/pl.dtbo ./$DIR_NAME/$DIR_NAME/$PLATFORM_NAME/psu_cortexa53_0/device_tree_domain/bsp/pl.dtsi

mkdir -p $EXPORT_FOLDER
cp $DIR_NAME/$DIR_NAME/$PLATFORM_NAME/psu_cortexa53_0/device_tree_domain/bsp/pl.dtbo $EXPORT_FOLDER/pl.dtbo
cp $SHELL_JSON_PATH $EXPORT_FOLDER/shell.json
echo "DONE! Successfully generated device tree overlay"