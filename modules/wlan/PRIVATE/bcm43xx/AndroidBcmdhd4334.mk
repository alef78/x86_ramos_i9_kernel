# This makefile is included from vendor/intel/common/wifi/WifiRules.mk.
$(eval $(call build_kernel_module,$(call my-dir)/src_bcm4334,bcm4334,CONFIG_BCMDHD=m CONFIG_BCM4334=y DRIVER=bcm4334 CONFIG_DHD_USE_SCHED_SCAN=y))
