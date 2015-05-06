/*
 * platform_ft5x0x_ts.c: ft5x0x_ts platform data initilization file
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
#include <linux/input/ft5x0x_ts.h>
#include "platform_ft5x0x_ts.h"

#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>

void *ft5x0x_platform_data(void *info)
{
	static struct ft5x0x_platform_data ft_pdata;

	//ft_pdata.numtouch       = 10;
	ft_pdata.x_max          = 1920;
	ft_pdata.y_max          = 1200;
	
	ft_pdata.reset          = get_gpio_by_name("ts_rst");
        if(ft_pdata.reset == 65535)
	    ft_pdata.reset          = 175;
	ft_pdata.irq            = get_gpio_by_name("ts_int");
        if(ft_pdata.irq == 65535)
	    ft_pdata.irq          = 183;


	//pr_info("ft5x0x_ts_platform_data(), reset-pin=%d, irq=%d\n", ft_pdata.reset, ft_pdata.irq);

	return &ft_pdata;
}
