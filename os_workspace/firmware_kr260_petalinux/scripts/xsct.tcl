set xsa_file "../resources/kria_kr260.xsa"

# platform name assumed to be passed as first argument
lassign $argv platform_name dir_name
hsi::open_hw_design $xsa_file 
createdts -hw $xsa_file -zocl -platform-name $platform_name -git-branch xlnx_rel_v2024.1 -overlay -compile -out ./$dir_name
exit