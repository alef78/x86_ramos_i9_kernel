/*
 * platform_camera.h: CAMERA platform library header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_CAMERA_H_
#define _PLATFORM_CAMERA_H_

#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>

extern const struct intel_v4l2_subdev_id v4l2_ids[] __attribute__((weak));

#define IS_BYT (INTEL_MID_BOARD(1, PHONE, BYT) || \
	INTEL_MID_BOARD(1, TABLET, BYT))

/* MFLD iCDK camera sensor GPIOs */

/* Obsolete pin, maybe used by old MFLD iCDK */
#define GP_CAMERA_0_POWER_DOWN          "cam_pwdn"
/* Camera VDD 1.8V Switch */
#define GP_CAMERA_1P8			"camera_on_n"
/* Camera0 Standby pin control */
#define GP_CAMERA_0_STANDBY		"cam_en"
#define GP_CAMERA_1_POWER_DOWN          "cam_pwr_en"
#define GP_CAMERA_0_RESET               "cam_0_rst"
#define GP_CAMERA_1_RESET               "cam_1_rst"
#define GP_CAMERA_1_POWER_EN	"vga_ldo_en"
	
extern int camera_sensor_gpio(int gpio, char *name, int dir, int value);
extern int camera_sensor_csi(struct v4l2_subdev *sd, u32 port,
			u32 lanes, u32 format, u32 bayer_order, int flag);
extern void intel_register_i2c_camera_device(
				struct sfi_device_table_entry *pentry,
				struct devs_id *dev)
				__attribute__((weak));
#endif
