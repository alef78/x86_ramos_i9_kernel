/*
 * platform_ov2675.c: ov2675 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipcutil.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include <linux/sfi.h>
#include "platform_camera.h"
#include "platform_ov2675.h"



#define VPROG1_VAL 2800000
static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;
static int camera_1p5_en;

static struct regulator *vprog1_reg;



static int ov2675_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					 GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = ret;
	}

	if (flag) {

		gpio_set_value(camera_reset, 0);
		msleep(20);
		gpio_set_value(camera_reset, 1);
	} else
		gpio_set_value(camera_reset, 0);

	return 0;
}

static int ov2675_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
}

static int ov2675_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
	if (camera_1p5_en < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_1P5,
					 GPIOF_DIR_OUT, 1);
		/*
		 * on some HW, this pin is not a connected pin,
		 * while on others, this indeed is avaiable.
		 * so just operate it when available and continue
		 * if it is failed.
		 */
		if (ret < 0)
			pr_debug("%s not available.", GP_CAMERA_1_1P5);
		camera_1p5_en = ret;
		}
	if (flag) {
		if (!camera_vprog1_on) {
			if (intel_mid_identify_cpu() ==
			   INTEL_MID_CPU_CHIP_TANGIER)
				ret = intel_scu_ipc_msic_vprog1(1);
			else
				ret = regulator_enable(vprog1_reg);
			if (!ret)
				camera_vprog1_on = 1;
			else
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
		}
		if (camera_1p5_en >= 0)
			gpio_set_value(camera_1p5_en, 1);
			msleep(20);
		
	} else {
		if (camera_1p5_en >= 0)
				gpio_set_value(camera_1p5_en, 0);
		if (camera_vprog1_on) {
			if (intel_mid_identify_cpu() ==
			   INTEL_MID_CPU_CHIP_TANGIER)
				ret = intel_scu_ipc_msic_vprog1(0);
			else
				ret = regulator_disable(vprog1_reg);
			if (!ret)
				camera_vprog1_on = 0;
			else
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
			return ret;
		}
	}

	return ret;
}

static int ov2675_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_YUV422_8, -1, flag);
}

static int ov2675_platform_init(struct i2c_client *client)
{
	int ret;
	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		if (&client->dev !=NULL)
		{
			dev_err(&client->dev, "regulator_get failed\n");
		}
		return PTR_ERR(vprog1_reg);
	}
	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {		
		if (&client->dev !=NULL)
		{
			dev_err(&client->dev, "regulator voltage set failed\n");
		}
		regulator_put(vprog1_reg);
	}
	return ret;
}

static int ov2675_platform_deinit(void)
{
	regulator_put(vprog1_reg);

	return 0;
}

static struct camera_sensor_platform_data ov2675_sensor_platform_data = {
	.gpio_ctrl	= ov2675_gpio_ctrl,
	.flisclk_ctrl	= ov2675_flisclk_ctrl,
	.power_ctrl	= ov2675_power_ctrl,
	.csi_cfg	= ov2675_csi_configure,
	.platform_init = ov2675_platform_init,
	.platform_deinit = ov2675_platform_deinit,

};

void *ov2675_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;
	camera_1p5_en = -1;

	return &ov2675_sensor_platform_data;
}
