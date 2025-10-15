^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package krs_firmware (acceleration_firmware_kr260)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

v2.0.0 (2025-10-14)
* renamed module from acceleration_firmware_kr260 into krs_firmware
* now contains a lightweight cross-compilation wrapper which is board and OS indepedant
* outsourced sysroot creation
* adjusted templates for Petalinux support
* firmware_setup is now configurable and not automatic anymore

v1.1.1 (2022-11-09)
-------------------
* Release v1.1.1
* Add platform of KV260 for testing kernel build
* Update README for KRS 1.1
* Update to v1.1.0 in README

v1.1.0 (2022-10-04)
-------------------
* Release v1.1.0
* Add missing files to build acceleration kernels
* Start addressing accelerator build error

v1.0.0 (2022-06-28)
-------------------
* Adjustments for enhancing sysroot with ROS 2
* Create kr260 ROS mixins, leverage Ubuntu 22.04 sysroot
* Initial commit
