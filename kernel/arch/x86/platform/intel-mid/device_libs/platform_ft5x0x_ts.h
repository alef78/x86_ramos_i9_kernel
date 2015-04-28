/*
 * platform_mxt224.h: mxt224 platform data header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_FT5X0X_H_
#define _PLATFORM_FT5X0X_H_

extern void *ft5x0x_platform_data(void *info) __attribute__((weak));
extern void ft5x0x_platform_i2c_device_register(struct sfi_device_table_entry *pentry,
					struct devs_id *dev);

#endif
