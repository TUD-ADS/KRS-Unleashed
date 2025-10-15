#!/usr/bin/env python3
"""
Multi-mode CLI tool with global and mode-specific arguments.

Usage examples:
  ./vitis_ws.py ws --common
  ./vitis_ws.py clean
  ./vitis_ws.py hls --multi "kernel1, kernel2"
  ./vitis_ws.py hls --multi "kernel1, kernel2" --dry
  ./vitis_ws.py hls kernel1
  ./vitis_ws.py system 
  ./vitis_ws.py full 
  ./vitis_ws.py --help
"""

import vitis
import os
from constants import Constants
from simple_yaml_parser import SimpleYAMLParser
from create_platform import platform_creation
from create_hls import hls_component_creation, application_component_creation, system_linking
import shutil
import argparse
from functools import partial


def ws_mode(args, client, const_file):
    print("Running Platform/WS Creation mode")
    try:
        common = args.common
    except AttributeError:
        common = False
    print(f"  Common flag: {common}")
    workdir = const_file.get("work_dir")
    if (os.path.isdir(workdir)):
        shutil.rmtree(workdir)
        print(f"Deleted workspace {workdir}")

    client.set_workspace(path=workdir)  # path="./workspace" //pwd
    print(f"Set Workspace to : {workdir}")
    platform_creation(client, const_file, common)


def clean_mode(args, const_file):
    print("Clean Workspace")
    workdir = const_file.get("work_dir")
    if (os.path.isdir(workdir)):
        shutil.rmtree(workdir)
        print(f"Deleted workspace {workdir}")


def hls_mode(args, client, const_file):
    print("Running HLS mode")
    client.set_workspace(path=const_file.get("work_dir"))
    kernels = []
    if args.multi:
        print(f"Kernels: {args.multi}")
        kernels = [k.strip() for k in args.multi.split(',')]
    elif args.kernel:
        print(f"Single kernel: {args.kernel}")
        kernels = [args.kernel]
    else:
        print("No kernel(s) specified")
        return

    print("Running HLS mode")
    print(f"  Dry run: {args.dry}")
    for kernel in kernels:
        hls_component_creation(client, args.dry, kernel, const_file)
        application_component_creation(client, args.dry, kernel, const_file)


def system_mode(args, client, const_file):
    client.set_workspace(path=const_file.get("work_dir"))
    system_linking(client, const_file)

    # Try to prepare a folder to copy over to the board

    export_path = const_file.get('export_dir')
    component_name = const_file.get('name')

    if (os.path.isdir(export_path)):
        shutil.rmtree(export_path)
        print(f"Deleted last exported results {export_path}")

    os.makedirs(export_path)
    print(f"Created directory '{export_path}' to contain final artifacts")

    # copy system container
    workdir = const_file.get('work_dir')
    xclbin_path = workdir + "/"+const_file.get('system_name') + \
        "/build/hw/hw_link/"+component_name+".xclbin"
    shutil.copyfile(xclbin_path, export_path + "/" +
                    component_name+".xclbin")
    shutil.copyfile(xclbin_path, export_path + "/" +
                    component_name+".bin")  # .bin necessary for the board

    for kernel in const_file.get('kernels'):
        config = const_file.get_config_for_kernel(kernel)
        application = config['host_code_name']
        x86_64_host_code = workdir + "/" + application + \
            "/build/x86sim/"+application
        arm_host_code = workdir + "/" + application + \
            "/build/hw/"+application
        shutil.copyfile(x86_64_host_code, export_path + "/" +
                        application+"_x86")
        shutil.copyfile(arm_host_code, export_path + "/" +
                        application+"_arm")

    for kernel in const_file.get('kernels'):
        config = const_file.get_config_for_kernel(kernel)
        kernel_object_file = workdir + "/" + kernel+"/" + \
            kernel+"/"+config['top_level_function']+".xo"
        shutil.copyfile(kernel_object_file, export_path + "/" +
                        config['top_level_function']+".xo")

    templates_dir = const_file.get('template_dir')
    shutil.copyfile(templates_dir+"pl.dtbo", export_path + "/" +
                    "pl.dtbo")
    shutil.copyfile(templates_dir+"shell.json", export_path + "/" +
                    "shell.json")
    print("Fully prepared new exported artifacts folder\n")


def full_mode(args, client, const_file):
    ws_mode(args, client, const_file)

    kernels = const_file.get("kernels")
    print("Running HLS mode")
    print(f"  Dry run: {args.dry}")
    for kernel in kernels:
        hls_component_creation(client, args.dry, kernel, const_file)
        application_component_creation(client, args.dry, kernel, const_file)

    system_mode(args, client, const_file)


def main():

    # Establishes a connection between the server and the client
    client = vitis.create_client()
    client.info()
    # Set the log level to info
    client.log_level('INFO')

    config_file = "scripts/config.yaml"
    yaml_parser = SimpleYAMLParser()
    yaml_parsed = yaml_parser.parse_file(config_file)
    const_file = Constants(yaml_parsed)

    parser = argparse.ArgumentParser(description="Multi-mode CLI tool with global and mode-specific arguments",
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     epilog="""
Examples:
  ./vitis_ws.py ws --common
  ./vitis_ws.py clean
  ./vitis_ws.py hls --multi "kernel1, kernel2"
  ./vitis_ws.py hls --multi "kernel1, kernel2" --dry
  ./vitis_ws.py hls kernel1
  ./vitis_ws.py system 
  ./vitis_ws.py full 
  ./vitis_ws.py --help
        """.strip()
    )

    print("\n\n")

    subparsers = parser.add_subparsers(
        dest='mode', required=True, help='Operation mode')

    # ws mode
    ws_parser = subparsers.add_parser('ws', help='Workspace mode')
    ws_parser.add_argument('--common', action='store_true',
                           help='Utilize the common image instead of custom petalinux')
    ws_handler = partial(ws_mode, client=client, const_file=const_file)
    ws_parser.set_defaults(func=ws_handler)
    # ws_parser.set_defaults(func=ws_mode)

    # clean mode
    clean_parser = subparsers.add_parser('clean', help='Clean Workspace mode')
    clean_handler = partial(clean_mode, const_file=const_file)
    clean_parser.set_defaults(func=clean_handler)
    # clean_parser.set_defaults(func=clean_mode)

    # hls mode
    hls_parser = subparsers.add_parser('hls', help='HLS/Kernel mode')

    # Mutually exclusive group: either --multi or a positional kernel
    group = hls_parser.add_mutually_exclusive_group(required=True)
    group.add_argument("-m", '--multi', type=str,
                       help='Comma-separated list of kernels')
    group.add_argument('kernel', nargs='?', help='Single kernel name')

    hls_parser.add_argument("-d", "--dry", action='store_true',
                            help='Run in dry mode without building/synthesizing (global option)')
    hls_handler = partial(hls_mode, client=client, const_file=const_file)
    hls_parser.set_defaults(func=hls_handler)
    # hls_parser.set_defaults(func=hls_mode)

    # system building mode
    system_parser = subparsers.add_parser(
        'system', help='System Linking/Export mode')
    system_handler = partial(system_mode, client=client, const_file=const_file)
    system_parser.set_defaults(func=system_handler)

    # full mode
    full_parser = subparsers.add_parser(
        'full', help='Full (all steps) mode')
    full_parser.add_argument('--common', action='store_true',
                             help='Utilize the common image instead of custom petalinux')
    full_parser.add_argument("-d", "--dry", action='store_true',
                             help='Run in dry mode without building/synthesizing (global option)')
    # Mutually exclusive group: either --multi or a positional kernel
    group = full_parser.add_mutually_exclusive_group(required=False)
    group.add_argument("-m", '--multi', type=str,
                       help='Comma-separated list of kernels')
    group.add_argument('kernel', nargs='?', help='Single kernel name')

    full_handler = partial(full_mode, client=client, const_file=const_file)
    full_parser.set_defaults(func=full_handler)

    # Use partial to bind extra parameters
    hls_handler = partial(hls_mode, client=client, const_file=const_file)
    hls_parser.set_defaults(func=hls_handler)

    args = parser.parse_args()
    args.func(args)

    print("\nDone, Vitis Flow finished successfully\n")
    print("You can now continue using the workspace from within Vitis 2024 to adjust files/configurations")
    # Close the client and terminate the server
    vitis.dispose()


if __name__ == "__main__":
    main()
