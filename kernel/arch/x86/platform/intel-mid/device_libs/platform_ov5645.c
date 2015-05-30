/*
 * platform_ov5645.c: ov5645 platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include <linux/sfi.h>

#include "platform_camera.h"
#include "platform_ov5645.h"


static int camera_reset;
static int camera_vcm_power;
static int camera_vprog1_on;
static int camera_power_down;
static struct regulator *vprog1_reg;
#define VPROG1_VAL 2800000

/*
 * GRACELAND DV1 primary camera sensor - OV5645 platform data
 */

static int ov5645_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
					GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = ret;
	}

	if (flag) {
		gpio_set_value(camera_reset, 0);
		msleep(20);
		gpio_set_value(camera_reset, 1);
	} else {
		gpio_set_value(camera_reset, 0);
	}

	return 0;
}

static int ov5645_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
}

static int ov5645_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	#if 0
	if (camera_vcm_power < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_VCM_POWER,
					 GPIOF_DIR_OUT, 1);
		/*
		 * on some HW, this pin is not a connected pin,
		 * while on others, this indeed is avaiable.
		 * so just operate it when available and continue
		 * if it is failed.
		 */
		if (ret < 0)
			pr_debug("%s not available.", GP_CAMERA_0_VCM_POWER);
		camera_vcm_power = ret;
	}
	#endif
	if (camera_power_down < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_POWER_DOWN,
					 GPIOF_DIR_OUT, 1);
		/*
		 * on some HW, this pin is not a connected pin,
		 * while on others, this indeed is avaiable.
		 * so just operate it when available and continue
		 * if it is failed.
		 */
		if (ret < 0)
			pr_debug("%s not available.", GP_CAMERA_0_POWER_DOWN);
		camera_power_down = ret;
	}
	if (flag) {
	//	if (camera_vcm_power >= 0)
	//		gpio_set_value(camera_vcm_power, 1);
		if (camera_power_down >= 0)
			gpio_set_value(camera_power_down, 1);
		if (!camera_vprog1_on) {
			if (intel_mid_identify_cpu() ==
			   INTEL_MID_CPU_CHIP_TANGIER)
				ret = intel_scu_ipc_msic_vprog1(1);
			else
				{
				ret = regulator_enable(vprog1_reg);
				}
			if (!ret)
				camera_vprog1_on = 1;
			else
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
			/*
			 * delay 20ms to wait sensor power up stable.
			 */
			msleep(20);
			return ret;
		}
	} else {
		if (camera_vprog1_on) {
			if (intel_mid_identify_cpu() ==
			   INTEL_MID_CPU_CHIP_TANGIER)
				ret = intel_scu_ipc_msic_vprog1(0);
			else
				ret = regulator_disable(vprog1_reg);
			if (camera_power_down >= 0)
			{
			   gpio_set_value(camera_power_down, 0);
				}
		//	if (camera_vcm_power >= 0)
		//		gpio_set_value(camera_vcm_power, 0);
			if (!ret)
				camera_vprog1_on = 0;
			else
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
			return ret;
		}
	}

	return 0;
}

static int ov5645_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, 2,
		ATOMISP_INPUT_FORMAT_YUV422_8, -1, flag);
}


/*
 * Checking the SOC type is temporary workaround to enable ov5645
 * on Bodegabay (tangier) platform. Once standard regulator devices
 * (e.g. vprog1, vprog2) and control functions (pmic_avp) are added
 * for the platforms with tangier, then we can revert this change.
 * (dongwon.kim@intel.com)
 */
static int ov5645_platform_init(struct i2c_client *client)
{
	int ret;
	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_TANGIER)
		return 0;
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

/*
 * Checking the SOC type is temporary workaround to enable ov5645 on Bodegabay
 * (tangier) platform once standard regulator devices (e.g. vprog1, vprog2) and
 * control functions (pmic_avp) are added for the platforms with tangier, then
 * we can revert this change.(dongwon.kim@intel.com
 */
static int ov5645_platform_deinit(void)
{
	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_TANGIER)
		return 0;

	regulator_put(vprog1_reg);
	return 0;
}
static int ov5645_vcm_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	
	if (camera_vcm_power < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_VCM_POWER,
					 GPIOF_DIR_OUT, 1);
		/*
		 * on some HW, this pin is not a connected pin,
		 * while on others, this indeed is avaiable.
		 * so just operate it when available and continue
		 * if it is failed.
		 */
		if (ret < 0)
			pr_debug("%s not available.", GP_CAMERA_0_VCM_POWER);
		camera_vcm_power = ret;
		}
	if (flag) {
		if (camera_vcm_power >= 0)
			gpio_set_value(camera_vcm_power, 1);
		}
	else {
		if (camera_vcm_power >= 0)
				gpio_set_value(camera_vcm_power, 0);
	}
	return 0;
}
static struct camera_sensor_platform_data ov5645_sensor_platform_data = {
	.gpio_ctrl      = ov5645_gpio_ctrl,
	.flisclk_ctrl   = ov5645_flisclk_ctrl,
	.power_ctrl     = ov5645_power_ctrl,
	.csi_cfg        = ov5645_csi_configure,
	.platform_init  = ov5645_platform_init,
	.platform_deinit= ov5645_platform_deinit,
	.platform_vcm	= ov5645_vcm_power_ctrl,
};

void *ov5645_platform_data(void *info)
{
	camera_reset = -1;
	camera_vcm_power = -1;
	camera_power_down = -1;

	return &ov5645_sensor_platform_data;
}
