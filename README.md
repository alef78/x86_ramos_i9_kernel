# x86_ramos_i9_kernel
unofficial 3.10.20 kernel for Ramos i9 tablet, based on Dell Venue 8 and Asus Zenphone 5 sources, patched to work after kexec

Ramos I9 tablet (and its rebrands, like Flipkart XT901) has good hardware, but locked bootloader.
Additionally, Ramos is violating GPL by not releasing its kernel sources.
Loading custom kernel is still possible via kexec.
There were some issues with hardware reinitialization, that required patches to kexec and the kernel to be loaded.
Some of this work can be useful on other x86 Intel Mid tablets.

Current status:
kernel loads, boots to Android. usb, touchscreen, video, sound, mmc, sd card, power management,wifi,bluetooth,accelerometer sensor works ok. adb works. charging seems to work but needs testing
known problems / todo list:
1) camera sensors drivers (for ov2675 and ov5645) are not there yet.
2) 2nd kexec from kexec'ed kernel crashes. seems to be some problem with pvr driver irq reinit.
