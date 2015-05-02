/*
 * platform_mxt224.c: mxt224 platform data initilization file
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
#include <linux/input/bma250.h>
#include "platform_bma250.h"

#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>


#if 1

void *bma250_platform_data(void *info)
{
	static struct bma250_platform_data bma_pdata;

#if 0
	ft_pdata.reset          = get_gpio_by_name("ts_rst");
        if(ft_pdata.reset == 65535)
	    //ft5416_pdata.reset          = 175;
	    ft_pdata.reset          = 175;
	ft_pdata.irq            = get_gpio_by_name("ts_int");
        if(ft_pdata.irq == 65535)
	    ft_pdata.irq          = 183;
#endif 

	return &bma_pdata;
}
#endif
