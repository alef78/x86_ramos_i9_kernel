/*
 * platform_ekth3250.c: ekth3250 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/ektf2k.h>
#include "platform_ekth3250.h"

void *ekthf2k_platform_data(void *info)
{
	static struct elan_ktf2k_i2c_platform_data ts_elan_ktf2k_data;

	ts_elan_ktf2k_data.version            = 10;
	ts_elan_ktf2k_data.abs_x_min          = 0;
	ts_elan_ktf2k_data.abs_x_max          = ELAN_X_MAX;
	ts_elan_ktf2k_data.abs_y_min          = 0;
	ts_elan_ktf2k_data.abs_y_max          = ELAN_Y_MAX;
	//ts_elan_ktf2k_data.intr_gpio            = get_gpio_by_name("TOUCH_INT_N");
	//ts_elan_ktf2k_data.reset            = get_gpio_by_name("TOUCH_RST_N");
	ts_elan_ktf2k_data.intr_gpio          = 183; // for dell
	//ts_elan_ktf2k_data.intr_gpio          = 66;    // for vv board
	ts_elan_ktf2k_data.reset              = 175;
	//ts_elan_ktf2k_data.reset              = 191;

	return &ts_elan_ktf2k_data;
}
