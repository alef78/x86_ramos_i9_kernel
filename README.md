# x86_ramos_i9_kernel
3.10.20, based on Dell Venue 8 and Asus Zenphone 5 sources, patched to work on Ramos i9 tablet with kexec

Ramos I9 tablet (and its rebrands, like Flipkart XT901) has good hardware, but locked bootloader.
Additionally, Ramos is violating GPL by not releasing its kernel sources.
Loading custom kernel is still possible via kexec.
There are some issues with hardware reinitialization, that required patches to kexec and the kernel to be loaded.
Some of this work can be useful on other x86 Intel Mid tablets.

Current status:
kernel loads, boots to Android. usb, touchscreen, video, mmc, power management,wifi works ok. adb works.
known problems / todo list:
1) no bluetooth drivers yet.
2) bma250e (accelerometer) detected but not working
3) camera drivers are not there yet.
