# This makefile is included from vendor/intel/common/wifi/WifiRules.mk.
$(eval $(call build_kernel_module,$(call my-dir)/src_bcm4334x,bcm4334x,CONFIG_BCMDHD=m CONFIG_BCM4334x=y DRIVER=bcm4334x CONFIG_DHD_USE_SCHED_SCAN=y))
