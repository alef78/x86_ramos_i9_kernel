# x86_ramos_i9_kernel
3.10.20, based on Dell Venue 8 and/or Asus Zenphone 5 sources, patched to work on Ramos i9 tablet with kexec

Ramos I9 tablet (and its rebrands, like Flipkart XT901) has good hardware, but locked bootloader.
Additionally, Ramos is violating GPL by not releasing its kernel sources.
Loading custom kernel is still possible via kexec.
There are some issues with hardware reinitialization, that required patches to kexec and the kernel to be loaded.
Some of this work can be useful on other x86 Intel Mid tablets.

Current status:
kernel loads, boots to Android. usb, touchscreen,mmc, power management works ok. adb works.
known problems / todo list:
1) partially distorted screen (some problem with vblank/vsync in video driver)
2) sound initiatization takes about 12 seconds during kernel boot (but sound works after that)
3) not tested bt/wifi yet, because of (1)
4) bma250 (accelerometer) and camera drivers are not there yet.
