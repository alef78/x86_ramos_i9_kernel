# This makefile is included from vendor/intel/common/wifi/WifiRules.mk.
$(eval $(call build_kernel_module,$(call my-dir)/src_bcm4335,bcm4335,CONFIG_BCMDHD=m CONFIG_BCM4335=y DRIVER=bcm4335 CONFIG_DHD_USE_SCHED_SCAN=y))
