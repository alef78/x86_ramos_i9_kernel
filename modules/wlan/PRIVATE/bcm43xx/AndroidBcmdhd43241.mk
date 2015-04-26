# This makefile is included from vendor/intel/common/wifi/WifiRules.mk.
$(eval $(call build_kernel_module,$(call my-dir)/src_bcm43241,bcm43241,CONFIG_BCMDHD=m CONFIG_BCM43241=y DRIVER=bcm43241 CONFIG_DHD_USE_SCHED_SCAN=y))
