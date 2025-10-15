# Add package: Vitis Python CLI
import vitis
import os
import shutil
from constants import Constants, check_error


def print_cfg_file(cfg_obj):
    """
    Helper method to print whole HLS config file
    """

    print("Printing whole config file: \n\n")
    sections = cfg_obj.get_sections()
    for entry in sections:
        sec = entry
        if sec == "":
            sec = "top-level"

        print(f"inside section [{sec}] found following lines:\n")
        for val in cfg_obj.get_values(entry):
            print(val)
    print("\n")


def hls_component_creation(client, dry, kernel, const_file):
    """HLS Component creation method (2. Step in Vitis Flow)

    creates kernel object (.xo) file from xpfm platform + HLS source files
    main configurations:
    * Flow: Vitis for .xo files /Vivado for old IP flow
    * Target_clock_freq: 5ns
    * CFlags for synthesis, testbench runs
    * include setup
    * if CSIM/IMPL stage is wanted to be executed
    """
    print("\nStarting HLS Project Creation Stage\n")

    workdir = const_file.get('work_dir')
    hls_dir = workdir + "/" + kernel
    parentdir = const_file.get('parent_dir')
    # Create hls component

    if (os.path.isdir(hls_dir)):
        shutil.rmtree(hls_dir)
    print(f"Deleted HLS Space {hls_dir}")
    hls_comp = client.create_hls_component(name=kernel)

    # Get config file object
    cfg_path = os.path.join(
        workdir, kernel, 'hls_config.cfg')
    cfg_obj = client.get_config_file(cfg_path)

    # configure part
    cfg_obj.set_value('', key='part', value=const_file.get('part'))

    cfg_obj.add_lines('hls')

    if const_file.get('hls_flow_target') == "vitis":
        cfg_obj.set_value('hls', key='flow_target', value="vitis")
        cfg_obj.set_value('hls', key='package.output.format', value="xo")
        print("Set HLS Flow Target to: Vitis Kernel Flow")
    else:
        cfg_obj.set_value('hls', key='flow_target', value="vivado")
        cfg_obj.set_value('hls', key='package.output.format',
                          value="ip_catalog")
        print("Set HLS Flow Target to: Vivado IP Flow")

    include_folder = parentdir + "/sources/"
    source_folder = parentdir + "/sources/"
    testbench_folder = parentdir + "/sources/"
    misc_folder = parentdir + "/sources/misc/"

    print(f"Set Include Dir to : {include_folder}")
    print(f"Set Source Dir to : {source_folder}")
    print(f"Set Testbench Dir to : {testbench_folder}")

    config = const_file.get_config_for_kernel(kernel)

    synth_file = source_folder + config['hls_file_path']+".cpp"
    cfg_obj.add_lines('hls', ['syn.file='+synth_file])
    print(f"Added Synthesis File: {synth_file}")

    # Testbenches can be added as folder
    tb_file = testbench_folder + config['hls_file_path']+"_test.cpp"
    cfg_obj.add_lines('hls', ['tb.file='+tb_file])

    for file in config['tb_misc_include_list']:
        tb_file = misc_folder + file
        cfg_obj.add_lines('hls', ['tb.file='+tb_file])
        print(f"Added Testbench File: {tb_file}")

    cflags = config['cflags'] + " -I" + include_folder
    for include in config['include_list']:
        cflags += " -I" + include

    synth_cflags = cflags + " -I" + source_folder
    tb_cflags = cflags + " -I" + testbench_folder + " -I" + misc_folder

    cfg_obj.add_lines('hls', ['syn.cflags='+synth_cflags])
    cfg_obj.add_lines('hls', ['syn.csimflags='+synth_cflags])
    print(f"Added Synth CFlags: {synth_cflags}")

    for include in config['tb_include_list']:
        tb_cflags += " -I" + include

    cfg_obj.add_lines('hls', ['tb.cflags='+tb_cflags])
    cfg_obj.add_lines('hls', ['tb.csimflags='+tb_cflags])
    print(f"Added Testbench CFlags: {tb_cflags}")

    cfg_obj.add_lines(
        'hls', ['syn.top='+config['top_level_function']])
    print(f"Set Top Level Function to: {config['top_level_function']}")

    cfg_obj.add_lines('hls', [f'clock={const_file.get("hls_clock_target")}'])
    print(
        f"Set Target Clock Frequency to: {const_file.get('hls_clock_target')}")

    if config['sim_flags']:
        cfg_obj.add_lines('hls', [f"csim.ldflags={config['sim_flags']}"])
        cfg_obj.add_lines('hls', [f"cosim.ldflags={config['sim_flags']}"])
        cfg_obj.add_lines('hls', [f"sim.ldflags={config['sim_flags']}"])
    print(f"Set Extra ldflags for simulation to: {config['sim_flags']}")

    print_cfg_file(cfg_obj)

    print("\nDone with HLS Project Creation Stage\n")

    if not dry:

        # Run c-simulation on the component
        hls_comp.run('C_SIMULATION')

        # Run synthesis on the component
        hls_comp.run('SYNTHESIS')
        hls_comp.report()

    print("HLS Synthesis was successful!\n")


def application_component_creation(client, dry, kernel, const_file):
    """Application (HostCode) Component creation method (3. Step in Vitis Flow)

    creates host code (.exe) file from xpfm platform + Petalinux sysroot (SDK) + host code (XRT source files)
    """

    print("\nStarting Application Creation Phase!\n")

    config = const_file.get_config_for_kernel(kernel)
    platform_xpfm = client.find_platform_in_repos(
        const_file.get('platform_name'))
    print(platform_xpfm)

    try:
        client.get_component(config["host_code_name"])
        client.delete_component(config["host_code_name"])
    except Exception:
        pass

    comp = client.create_app_component(
        name=config["host_code_name"], platform=platform_xpfm, domain=const_file.get('platform_domain_name'))
    comp = client.get_component(config["host_code_name"])
    check_error(comp.set_sysroot(sysroot=const_file.get('petalinux_sdk_path')),
                "failed to set Sysroot, was: {}".format(const_file.get('petalinux_sdk_path')))
    print(comp.get_sysroot())

    comp.import_files(
        from_loc=const_file.get('src_dir'), files=config['host_code_files'])
    includes = [const_file.get('src_dir')]
    includes += config['host_include_list']

    comp.append_app_config(key="USER_INCLUDE_DIRECTORIES",
                               values=includes)

    comp.append_app_config(key="USER_LINK_DIRECTORIES",
                           values=[const_file.get('opencv_lib_path')])

    comp.append_app_config(key="USER_LINK_LIBRARIES",
                           values=config['host_link_libraries'])

    comp.set_app_config(key="USER_COMPILE_OTHER_FLAGS",
                        values=config['host_flags'])

    comp.get_app_config()

    if not dry:
        comp.build(target="x86sim")
        comp.build(target="hw")
        comp.report()
    print("\nDone with Application Creation!\n")


def system_linking(client, const_file):
    """System Linking/Creation method (4. Step in Vitis Flow)

    links kernel object files (.xo) with host code (.exe) according to platform (.xpfm) into binary container (.xclbin)
    one binary container per application, possible to run multiple applications in parallel on FPGA

    creates kernel object (.xo) file from xpfm platform + HLS source files
    main configurations:
    * System Template: Empty Accelerated Application
    """

    print("\nSystem Design Phase!\n")
    platform_xpfm = client.find_platform_in_repos(
        const_file.get('platform_name'))
    print(platform_xpfm)

    proj = client.create_sys_project(
        name=const_file.get('system_name'), platform=platform_xpfm, template=const_file.get('system_template'))
    proj = client.get_sys_project(name=const_file.get('system_name'))
    check_error(proj.add_container(name=const_file.get('name')),
                "Binary Container couldnt be created for System Project with name: {}".format(const_file.get('name')))

    for kernel in const_file.get('kernels'):
        proj = proj.add_component(name=kernel,
                                  container_name=[const_file.get('name')])
    # Get config file object
    cfg_path = const_file.get('system_package_conf_dir')
    cfg_obj = client.get_config_file(cfg_path)

    # configure part
    cfg_obj.set_value('package', key='kernel_image',
                      value=const_file.get('petalinux_out_path'))
    cfg_obj.set_value('package', key='rootfs',
                      value=const_file.get('petalinux_rootfs_path'))

    print_cfg_file(cfg_obj)
    proj.report()

    proj.clean_all(target="hw")
    proj.build(target="hw")

    print("\nDone with System Design Phase\n")
