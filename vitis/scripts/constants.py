import os


class Constants:
    """
    A class that stores multiple important values
    """

    def __init__(self, yaml_parsed=None):
        self._yaml_parsed = yaml_parsed

        print(f"Current directory: {os.getcwd()}")

        self._parent_dir = os.getcwd()
        # self._parent_dir = os.path.abspath( # used when still running inside script dir
        #     os.path.join(os.getcwd(), os.pardir))

        print(f"Parent directory: {self._parent_dir}")
        self._workspace_name = "workdir"
        self._work_dir = self._parent_dir + "/build/" + self._workspace_name
        self._sources_dir = self._parent_dir + "/sources/"
        self._template_dir = self._parent_dir + "/template/"

        self._component_name = yaml_parsed.get("Project").get("name")
        print(f"Project: \t{self._component_name}")
        self._export_dir = self._parent_dir + "/exports/export_"+self._component_name
        self._kernels = [x['name']
                         for x in yaml_parsed.get("Project").get("kernels")]
        print(f"Kernels: \t{self._kernels}")

        board = yaml_parsed.get("Project").get("board")
        if board == "KR260":
            self._part = "xck26-sfvc784-2LV-c"
        elif board == "Ultra96-v2":
            self._part = "xczu3eg-sbva484-1-i"
        elif board == "PYNQ-Z1":
            self._part = "xc7z020clg400-1"
        else:
            print("Unsupported board name")
        print(f"Part: \t\t{self._part}")
        self._platform_name = yaml_parsed.get(
            "Project").get("platform").get('name')
        if yaml_parsed.get("Project").get("platform").get('create_new'):
            self._new_platform = True
            self._os = "linux"
            self._cpu_name = "psu_cortexa53"
            self._domain_name = self._os + "_" + self._cpu_name
            self._xsa = yaml_parsed.get("Project").get(
                "platform").get('xsa_file_path')
            self._petalinux_path = yaml_parsed.get("Project").get(
                "platform").get('petalinux_dir')
            self._petalinux_output_path = self._petalinux_path + "/build_petalinux/images/linux"
            self._sdk_path = self._petalinux_path + \
                "/sysroots/cortexa72-cortexa53-xilinx-linux"
            self._dtb_path = self._petalinux_output_path + "/system.dtb"
            self._bif_path = ""
            self._fsbl_path = self._petalinux_output_path + "/zynqmp_fsbl.elf"
            self._pmu_fw_path = self._petalinux_output_path + "/pmufw.elf"
            self._sd_path = self._parent_dir + "/platform/"
            print(
                f"Platform: \tDomain: {self._domain_name}, XSA: {self._xsa} Petalinux: {self._petalinux_path}")

        # only allowed values are ['vitis', 'vivado']
        self._hls_flow_target = "vitis"
        self._target_clock = yaml_parsed.get("Project").get("target_clock")
        print(f"clock: \t{self._target_clock}")

        self._opencv_include = yaml_parsed.get(
            "Project").get("opencv_include_path")
        self._opencv_lib = yaml_parsed.get(
            "Project").get("opencv_library_path")

        self._system_project_name = self._component_name + "_system"
        self._system_template = "empty_accelerated_application"
        self._package_conf_dir = self._parent_dir + "/build/workdir/" + \
            self._system_project_name+"/package/package.cfg"
        self._system_kernel_image_path = self._petalinux_output_path + "/Image"
        self._system_rootfs_path = self._petalinux_output_path + "/rootfs.ext4"

        self._kernel_configs = {}
        for kernel in yaml_parsed.get("Project").get("kernels"):
            name = kernel.get("name")
            config = {}
            config['top_level_function'] = kernel.get('top_level_function')
            config['hls_file_path'] = name + "/" + name + "_kernel"

            include_list = []
            host_include_list = []
            if 'vitis_library' in kernel and kernel.get('vitis_library'):
                include_list.append(yaml_parsed.get("Project").get(
                    "vitis_library_path") + "/"+kernel.get('vitis_library')+"/L1/include/")
                host_include_list.append(yaml_parsed.get("Project").get(
                    "vitis_library_path") + "/"+kernel.get('vitis_library')+"/L1/include/")

            config["include_list"] = include_list

            testbench_include_list = []
            testbench_misc_file_list = []
            sim_flags = ""
            host_linker_libraries = []
            config["opencv"] = False
            if 'use_opencv' in kernel and kernel.get('use_opencv'):
                config["opencv"] = True
                testbench_include_list.append(self._opencv_include)
                host_include_list.append(self._opencv_include)
                testbench_misc_file_list.append(name+"_in.png")
                testbench_misc_file_list.append(name+"_result.png")
                sim_flags = "-L " + self._opencv_lib + \
                    " -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lopencv_highgui -lopencv_flann -lopencv_features2d"
                host_linker_libraries = ["opencv_imgcodecs", "opencv_imgproc",
                                         "opencv_core", "opencv_highgui", "opencv_flann", "opencv_features2d"]
            config["tb_include_list"] = testbench_include_list
            config["tb_misc_include_list"] = testbench_misc_file_list
            config["cflags"] = "-D__SDSVHLS__"
            config["sim_flags"] = sim_flags

            config["host_code_name"] = name+"_host_code"
            config["host_code_files"] = [name + "/" + name+"_host_code.cpp", "hls_utils/xcl2.cpp",
                                         "hls_utils/cmd_line_parser.cpp"]
            config["host_include_list"] = host_include_list
            config["host_link_libraries"] = host_linker_libraries
            config["host_flags"] = "-Wno-unknown-pragmas"

            self._kernel_configs[name] = config
        print(f"Kernel configs: \t{self._kernel_configs}")

    def get(self, key):
        """
        Generalized getter method that retrieves processed attributes by key.

        Args:
            key (str): The attribute key to retrieve

        Returns:
            Processed attribute value

        Raises:
            KeyError: If the key is not recognized
        """
        processors = {
            'name': lambda: self._component_name,
            'workspace': lambda: self._workspace_name,
            'parent_dir': lambda: self._parent_dir,
            'work_dir': lambda: self._work_dir,
            'export_dir': lambda: self._export_dir,
            'template_dir': lambda: self._template_dir,
            'kernels': lambda: self._kernels,
            'part': lambda: self._part,
            'platform_name': lambda: self._platform_name,
            'new_platform': lambda: self._new_platform,
            'platform_os': lambda: self._os,
            'platform_cpu_name': lambda: self._cpu_name,
            'platform_domain_name': lambda: self._domain_name,
            'platform_xsa': lambda: self._xsa,
            'petalinux_path': lambda: self._petalinux_path,
            'petalinux_out_path': lambda: self._petalinux_output_path,
            'petalinux_sdk_path': lambda: self._sdk_path,
            'platform_dtb_path': lambda: self._dtb_path,
            'platform_bif_path': lambda: self._bif_path,
            'platform_fsbl_path': lambda: self._fsbl_path,
            'platform_pmu_fw_path': lambda: self._pmu_fw_path,
            'platform_sd_path': lambda: self._sd_path,
            'hls_flow_target': lambda: self._hls_flow_target,
            'hls_clock_target': lambda: self._target_clock,
            'opencv_include': lambda: self._opencv_include,
            'opencv_lib_path': lambda: self._opencv_lib,
            'src_dir': lambda: self._sources_dir,
            'system_name': lambda: self._system_project_name,
            'system_template': lambda: self._system_template,
            'system_package_conf_dir': lambda: self._package_conf_dir,
            'petalinux_kernel_image_path': lambda: self._system_kernel_image_path,
            'petalinux_rootfs_path': lambda: self._system_rootfs_path
        }

        if key not in processors:
            available_keys = ', '.join(sorted(processors.keys()))
            raise KeyError(
                f"Unknown key '{key}'. Available keys: {available_keys}")

        return processors[key]()

    def get_config_for_kernel(self, kernel):

        if kernel not in self._kernels:
            available_kernels = ', '.join(sorted(self._kernels))
            raise KeyError(
                f"Unknown kernel '{kernel}'. Available kernels: {available_kernels}")

        return self._kernel_configs[kernel]


def check_error(status, msg="Unspecified Error"):
    if (not status):
        print(msg)
        vitis.dispose()
        exit("Found Error, stop execution")

# #############################################
# #             General Options               #
# #############################################
# # IP export config
# VENDOR_NAME = "smarobix"
# COMPONENT_NAME = VENDOR_NAME + "_apriltag"  # IP_LIBRARY_NAME

# # HLS Kernel Name + top level function
# KERNEL_NAMES = {
#     "decimate": "cvtcolor_rgb2gray",
#     "blur": "gaussian_filter_accel",
#     "threshold": "adaptive_threshold"
# }

# # Part name
# # "xczu3eg-sbva484-1-i" for Ultra96-v2
# # "xc7z020clg400-1 for" PYNQ-Z1
# # "xck26-sfvc784-2LV-c" for K26
# PART_NAME = "xck26-sfvc784-2LV-c"

# #############################################
# #    Platform Configuration Options         #
# #############################################
# PLATFORM_CREATE_NEW = True
# PLATFORM_NAME = "kria_kr260_base_accel"
# OS = "linux"
# CPU_NAME = "psu_cortexa53"
# DOMAIN_NAME = OS + "_" + CPU_NAME
# XSA_FILE_PATH = "/artifacts/kria_kr260.xsa"
# PETALINUX_OUTPUT_DIR = "/home/paul/Documents/HomDoc/Projects/firmware_kria_petalinux/build_petalinux/images/linux"
# DTB_PATH = PETALINUX_OUTPUT_DIR + "/system.dtb"
# # BIF_PATH = PETALINUX_OUTPUT_DIR + "/bootgen.bif"
# BIF_PATH = ""
# FSBL_PATH = PETALINUX_OUTPUT_DIR + "/zynqmp_fsbl.elf"
# PMU_FW_PATH = PETALINUX_OUTPUT_DIR + "/pmufw.elf"

# SD_CARD_PATH = "/platform/"


# #############################################
# #        HLS Configuration Options          #
# #############################################

# # without extension, needs to be the same for both source (.cpp) and testbench (same name + suffix _test.cpp)
# HLS_FILE_NAMES = {x: x + "/" + x+"_kernel" for x in list(KERNEL_NAMES.keys())}

# HLS_FLOW_TARGET = "vitis"  # only allowed values are ['vitis', 'vivado']

# # Target clk
# TARGET_CLOCK_FREQ = "5"  # ns

# RUN_CSIM = False
# RUN_PACKAGE = False
# RUN_IMPLEMENTATION = False

# # extra include list (for libraries,..)
# INCLUDE_FOLDER_LIST = ["/opt/xilinx/Vitis-Libraries/vision/L1/include/"]

# # includes purely for Testbenches
# OPENCV_INCLUDE = "/home/paul/.local/include/opencv4"  # "/usr/include/opencv4"
# OPENCV_LIB = "/home/paul/.local/lib"    # "/usr/lib/x86_64-linux-gnu"
# TB_INCLUDE_FOLDER_LIST = [OPENCV_INCLUDE]
# TB_ADDITIONAL_MISC_FILES_ARRAY = {
#     x: [x+"_in.png", x+"_result.png"] for x in list(KERNEL_NAMES.keys())}

# # Extra flags
# EXTRA_CFLAGS = "-D__SDSVHLS__"

# # Extra Sim (csim, cosim and sim) Flags
# EXTRA_SIM_FLAGS = "-L "+OPENCV_LIB + \
#     " -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lopencv_highgui -lopencv_flann -lopencv_features2d"

# EXTRA_CONFIG_PARAMS = ["cosim.enable_dataflow_profiling=1",
#                        "cosim.enable_fifo_sizing=1",
#                        "cosim.trace_level=none",  # all,port
#                        "cosim.wave_debug=1"]

# #############################################
# #    Application Configuration Options      #
# #############################################
# APPLICATION_NAMES = {x: x+"_host_code" for x in list(KERNEL_NAMES.keys())}
# SDK_INSTALL_PATH = "/home/paul/Documents/HomDoc/Projects/firmware_kria_petalinux/sysroots/cortexa72-cortexa53-xilinx-linux"
# HOST_FILES_LOCATION = "/sources/"
# HOST_FILE_NAMES_LIST = {x: [x + "/" + x+"_host_code.cpp", "hls_utils/xcl2.cpp",
#                             "hls_utils/cmd_line_parser.cpp"] for x in list(KERNEL_NAMES.keys())}
# HOST_INCLUDES = ["/home/paul/.local/include/opencv4",
#                  "/opt/xilinx/Vitis-Libraries/vision/L1/include/"]  # , "/usr/include/CL"
# HOST_LINKER_LIBRARIES = ["opencv_imgcodecs", "opencv_imgproc",
#                          "opencv_core", "opencv_highgui", "opencv_flann", "opencv_features2d"]
# HOST_EXTRA_FLAGS = "-Wno-unknown-pragmas"

# #############################################
# #       System Configuration Options        #
# #############################################
# SYSTEM_PROJECT_NAME = "apriltag_system"
# SYSTEM_TEMPLATE = "empty_accelerated_application"
# PACKAGE_CONFIGURATION_PATH = "/build/workdir/apriltag_system/package/package.cfg"
# KERNEL_IMAGE_PATH = "/Image"
# ROOTFS_PATH = "/rootfs.ext4"
