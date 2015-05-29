# x86_ramos_i9_kernel
unofficial 3.10.20 kernel for Ramos i9 tablet, based on Dell Venue 8, Asus Zenphone / Asus Memopad and Acer Iconia GPL sources, patched to work after kexec

Ramos I9 tablet (and its rebrands, like Flipkart XT901) are violating GPL by not releasing its kernel sources.
This is a reimplementation of that kernel. Also it is patched in order to support booting it via kexec.
There were some issues with hardware reinitialization, that required patches to kexec and the kernel to be loaded.
Some of this work can be useful on other x86 Intel Mid tablets.

Current status:
kernel loads, boots to Android. usb, touchscreen, video, sound, mmc, sd card, power management,wifi,bluetooth,both cameras, accelerometer sensor works ok. adb works. charging seems to work but needs testing
known problems:
1) charging needs minor fixes (currently it is safe to use but may not start charging sometimes)

Note: lcd panel, camera and smb347 drivers here are created by partial reverse-engeneering (of linux kernel and  modules that marked as GPL, but Ramos failed to publish its source code) and partially based on sources of similar tablets. I do not have datasheets for this hardware. On my tablet it works, but be careful of possible bugs.
