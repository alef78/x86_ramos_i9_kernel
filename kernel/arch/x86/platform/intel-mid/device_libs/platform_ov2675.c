/*
 * platform_ov2675.c: ov2675 platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#define DEBUG
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/atomisp_platform.h>
#include <linux/regulator/consumer.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include <linux/mfd/intel_mid_pmic.h>

#ifdef CONFIG_VLV2_PLAT_CLK
#include <linux/vlv2_plat_clock.h>
#endif

#include "platform_camera.h"
#include "platform_ov2675.h"

/* workround - pin defined for merrifield */
//#define CAMERA_1_RESET 10
#define CAMERA_1_RESET 173
//#define CAMERA_1_PWDN 7
#define CAMERA_1_PWDN 88
//#define CAMERA_1_EN   3
#define CAMERA_1_EN   172
#ifdef CONFIG_VLV2_PLAT_CLK
#define OSC_CAM1_CLK 0x1
#define CLK_19P2MHz 0x1
#define CLK_ON	0x1
#define CLK_OFF	0x2
#endif
static struct regulator *v1prog_reg;

static int camera_vprog1_on;
static int gp_camera1_power_down;
static int gp_camera1_reset;


static int camera_reset;
static int camera_power_down;
static int camera_power_en;
/*
 * OV2675 platform data
 */

static int ov2675_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	int pin;


	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, 
		/* GP_CAMERA_1_RESET */ "camera_1_reset",
				/*	GPIOF_DIR_OUT*/ 0, 1);
		if (ret < 0)
		  return ret;
		camera_reset = ret;
	}

	if (flag) {
			gpio_set_value(camera_reset, 0);
			msleep(20);
			gpio_set_value(camera_reset, 1);
		//	gpio_set_value(camera_power_down, 0);
		//	gpio_set_value(camera_power_en, 1);
		//	usleep_range(10000, 11000);
		
	} else {
			gpio_set_value(camera_reset, 0);
			/* disable 1.8v power */
		//	gpio_direction_output(camera_power_down, 0);
		//	gpio_set_value(camera_power_en, 0);	
	}
	return 0;
}

static int ov2675_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1 /* =2 ok */,
				     flag ? clock_khz : 0);
}

/*
 * The power_down gpio pin is to control OV2675's
 * internal power state.
 */
static int ov2675_power_ctrl(struct v4l2_subdev *sd, int flag)
{
  int ret;
  if (camera_power_en < 0) {
    ret = camera_sensor_gpio(-1, "camera_1_1p5", 0, 1);
    if (ret < 0) {
      pr_info("ov2675_power_ctrl err\n");
      return ret;
    }
    camera_power_en = ret;
  }

#ifdef CONFIG_CRYSTAL_COVE
	struct i2c_client *client = v4l2_get_subdevdata(sd);
#endif
	ret = 0;

	if (flag) {
		if (!camera_vprog1_on) {
			ret = regulator_enable(v1prog_reg);
			ret = intel_scu_ipc_msic_vprog1(1);
			__gpio_set_value(camera_power_en,1);
			msleep(20);
			//if (!ret)
				camera_vprog1_on = 1;
			
			return ret;
		}
	} else {
		if (camera_vprog1_on) {
			__gpio_set_value(camera_power_en,0);
			ret = intel_scu_ipc_msic_vprog1(0);
			regulator_disable(v1prog_reg);
			//if (!ret)
				camera_vprog1_on = 0;
			return ret;
		}
	}

	return 0;
}

static int ov2675_csi_configure(struct v4l2_subdev *sd, int flag)
{
//	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
//		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY /* ==0 ok */,
	        1 /* ok */,
		ATOMISP_INPUT_FORMAT_YUV422_8 /* = 3 */, 
		-1 /* = -1 - huh? */, flag);
}

static int ov2675_platform_init(struct i2c_client *client)
{
	v1prog_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(v1prog_reg)) {
		dev_err(&client->dev, "vprog1 regulator_get failed\n");
		return PTR_ERR(v1prog_reg);
	}

	int ret = regulator_set_voltage(v1prog_reg, 2800000, 2800000);
	if (ret!=0) {
	  regulator_put(v1prog_reg);
	}
//todo: error checking
	return ret;
	
}

static int ov2675_platform_deinit(void)
{ // ok == ramos version
	regulator_put(v1prog_reg);

	return 0;
}
//#endif

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
	gp_camera1_power_down = -1;
	gp_camera1_reset = -1;
#ifdef CONFIG_CRYSTAL_COVE
	pmic_id = PMIC_MAX;
#endif

	camera_reset = -1;
	camera_power_down = -1;
	camera_power_en = -1;
	
	return &ov2675_sensor_platform_data;
}
