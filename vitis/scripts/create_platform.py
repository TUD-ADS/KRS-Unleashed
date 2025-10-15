import vitis
import os
from constants import Constants, check_error


def platform_creation(client, const_file, common):
    """Platform creation method (1. Step in Vitis Flow)

    creates platform (xpfm file) from XSA file + Petalinux output files
    main configurations:
    * OS: Linux
    * CPU: PSU_Cortexa53
    """

    print("\nPlatform creation phase!\n")
    print(const_file.get("platform_name"))
    if common:
        print("Currently not supported, continue with default!\n")
    if not const_file.get('new_platform'):
        print("Error: currently not supported")
    xsa_path = const_file.get("platform_xsa")
    sd_dir = const_file.get("platform_sd_path")

    platform = client.create_platform_component(name=const_file.get("platform_name"), hw_design=xsa_path,
                                                os=const_file.get(
                                                    "platform_os"),
                                                cpu=const_file.get(
                                                    "platform_cpu_name"),
                                                domain_name=const_file.get(
                                                    "platform_domain_name"), no_boot_bsp=True)
    platform = client.get_component(name=const_file.get("platform_name"))

    platform.generate_boot_bsp(target_processor="psu_cortexr5_0")

    # TODO renable if they are again working, need to regenerate files
    # check_error(platform.set_fsbl_elf(path=constants.FSBL_PATH),
    #             "failed to set FSBL, was: {}".format(constants.FSBL_PATH))
    # check_error(platform.set_pmufw_elf(path=constants.PMU_FW_PATH),
    #             "failed to set PMU_FW, was: {}".format(constants.PMU_FW_PATH))

    domain = platform.get_domain(name=const_file.get("platform_domain_name"))

    check_error(domain.set_dtb(path=const_file.get("platform_dtb_path")),
                "failed to set DTB, was: {}".format(
                    const_file.get("platform_dtb_path")))
    check_error(domain.set_sd_dir(path=sd_dir),
                "failed to set SD_CARD, was: {}".format(sd_dir))
    check_error(domain.set_boot_dir(path=const_file.get("petalinux_out_path")),
                "failed to set Boot dir, was: {}".format(const_file.get("petalinux_out_path")))

    bif_path = const_file.get("platform_bif_path")
    if not bif_path:
        domain.generate_bif()
        bif_path = const_file.get("work_dir") + '/' + const_file.get("platform_name") + \
            '/resources/' + \
            const_file.get("platform_domain_name") + '/linux.bif'
    check_error(domain.set_bif(path=bif_path),
                "failed to set BIF, was: {}".format(bif_path))

    # check_error(platform.build(), "failed to build platform!") currently always causes failure, ignore status
    platform.build()

    platform.report()
    print("\nPlatform Creation Successful!\n")
