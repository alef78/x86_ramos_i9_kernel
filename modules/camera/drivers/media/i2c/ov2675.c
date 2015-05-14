/*
 * Support for ov2675 Camera Sensor.
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/firmware.h>
#include <linux/videodev2.h>

#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>

#include "ov2675.h"

#define to_ov2675_sensor(sd) container_of(sd, struct ov2675_device, sd)

static int
ov2675_read_reg(struct i2c_client *client, u16 data_length, u16 reg, u32 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];

	if (!client->adapter) {
		dev_err(&client->dev, "%s error, no client->adapter\n",
			__func__);
		return -ENODEV;
	}

	if (data_length != MISENSOR_8BIT && data_length != MISENSOR_16BIT
					 && data_length != MISENSOR_32BIT) {
		dev_err(&client->dev, "%s error, invalid data length\n",
			__func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = MSG_LEN_OFFSET;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8)(reg >> 8);
	data[1] = (u8)(reg & 0xff);

	msg[1].addr = client->addr;
	msg[1].len = data_length;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err < 0) {
		dev_err(&client->dev,
			"read from offset 0x%x error %d", reg, err);
		return err;
	}

	*val = 0;
	/* high byte comes first */
	if (data_length == MISENSOR_8BIT)
		*val = (u8)data[0];
	else if (data_length == MISENSOR_16BIT)
		*val = be16_to_cpu(*(u16 *)&data[0]);
	else
		*val = be32_to_cpu(*(u32 *)&data[0]);

	return 0;
}

static int
ov2675_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u32 val)
{
	int num_msg;
	struct i2c_msg msg;
	unsigned char data[6] = {0};
	u16 *wreg;
	int retry = 0;

	if (!client->adapter) {
		dev_err(&client->dev, "%s error, no client->adapter\n",
			__func__);
		return -ENODEV;
	}

	if (data_length != MISENSOR_8BIT && data_length != MISENSOR_16BIT
					 && data_length != MISENSOR_32BIT) {
		dev_err(&client->dev, "%s error, invalid data_length\n",
			__func__);
		return -EINVAL;
	}

	memset(&msg, 0, sizeof(msg));

again:
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2 + data_length;
	msg.buf = data;

	/* high byte goes out first */
	wreg = (u16 *)data;
	*wreg = cpu_to_be16(reg);

	if (data_length == MISENSOR_8BIT) {
		data[2] = (u8)(val);
	} else if (data_length == MISENSOR_16BIT) {
		u16 *wdata = (u16 *)&data[2];
		*wdata = cpu_to_be16((u16)val);
	} else {
		u32 *wdata = (u32 *)&data[2];
		*wdata = cpu_to_be32(val);
	}

	num_msg = i2c_transfer(client->adapter, &msg, 1);
	//printk(" ov2675 write : wrote 0x%x to offset 0x%x pass %d\n",val, reg, num_msg);
	/*
	 * It is said that Rev 2 sensor needs some delay here otherwise
	 * registers do not seem to load correctly. But tests show that
	 * removing the delay would not cause any in-stablility issue and the
	 * delay will cause serious performance down, so, removed previous
	 * mdelay(1) here.
	 */

	if (num_msg >= 0)
		return 0;
	//printk("******num_msg  =%d **************happened error\n",num_msg);
	dev_err(&client->dev, "write error: wrote 0x%x to offset 0x%x error %d",
		val, reg, num_msg);
	if (retry <= I2C_RETRY_COUNT) {
		dev_err(&client->dev, "retrying... %d", retry);
		retry++;
		msleep(20);
		goto again;
	}

	return num_msg;
}

static int ov2675_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	int ret;
	int retry = 0;

again:
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);

	/*
	 * It is said that Rev 2 sensor needs some delay here otherwise
	 * registers do not seem to load correctly. But tests show that
	 * removing the delay would not cause any in-stablility issue and the
	 * delay will cause serious performance down, so, removed previous
	 * mdelay(1) here.
	 */

	if (ret == 1)
		return 0;

	if (retry <= I2C_RETRY_COUNT) {
		dev_dbg(&client->dev, "retrying i2c write transfer... %d",
			retry);
		retry++;
		msleep(20);
		goto again;
	}

	return ret;
}

/*
 * __ov2675_flush_reg_array() is internal function to make writing reg
 * faster and should be not used anywhere else.
 */
static int __ov2675_flush_reg_array(struct i2c_client *client,
				     struct ov2675_write_ctrl *ctrl)
{
	u16 size;

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return ov2675_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

/*
 * ov2675_write_reg_array - Initializes a list of MT9T111 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * Initializes a list of MT9T111 registers. The list of registers is
 * terminated by MISENSOR_TOK_TERM.
 */
static int ov2675_write_reg_array(struct i2c_client *client,
			    const struct misensor_reg *reglist)
{
	const struct misensor_reg *next = reglist;
	int err;

	for (; next->length != MISENSOR_TOK_TERM; next++) {
		if (next->length == MISENSOR_TOK_DELAY) {
			msleep(next->val);
		} else {
			err = ov2675_write_reg(client, next->length, next->reg,
						next->val);
			/* REVISIT: Do we need this delay? */
			udelay(10);
			if (err) {
				dev_err(&client->dev, "%s err. aborted\n",
					__func__);
				return err;
			}
		}
	}

	return 0;
}

static int ov2675_s_color_effect(struct v4l2_subdev *sd, int effect)
{
#if  0
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2675_device *dev = to_ov2675_sensor(sd);
	int err = 0;

	if (dev->color_effect == effect)
		return 0;

	switch (effect) {
	case V4L2_COLORFX_NONE:
		err = ov2675_write_reg_array(client, ov2675_normal_effect);
		break;
	case V4L2_COLORFX_SEPIA:
		err = ov2675_write_reg_array(client, ov2675_sepia_effect);
		break;
	case V4L2_COLORFX_NEGATIVE:
		err = ov2675_write_reg_array(client, ov2675_negative_effect);
		break;
	case V4L2_COLORFX_BW:
		err = ov2675_write_reg_array(client, ov2675_bw_effect);
		break;
	case V4L2_COLORFX_SKY_BLUE:
		err = ov2675_write_reg_array(client, ov2675_blue_effect);
		break;
	case V4L2_COLORFX_GRASS_GREEN:
		err = ov2675_write_reg_array(client, ov2675_green_effect);
		break;
	default:
		dev_err(&client->dev, "invalid color effect.\n");
		return -ERANGE;
	}
	if (err) {
		dev_err(&client->dev, "setting color effect fails.\n");
		return err;
	}

	dev->color_effect = effect;
#endif
	return 0;
}

static int ov2675_g_color_effect(struct v4l2_subdev *sd, int *effect)
{
	struct ov2675_device *dev = to_ov2675_sensor(sd);

	*effect = dev->color_effect;

	return 0;
}

/* calculate sysclk */
static int ov2675_get_sysclk(struct v4l2_subdev *sd, unsigned int *sysclk)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	// calculate sysclk
	int err;
	u32 temp1 = 0, temp2 = 0;
	u32 Indiv2x = 0, Bit8Div = 0, FreqDiv2x = 0, PllDiv = 0, SensorDiv = 0;
	u32 ScaleDiv = 0, DvpDiv = 0, ClkDiv = 0, VCO = 0;
	static	int Indiv2x_map[] = {2, 3, 4, 6, 4, 6, 8, 12};
	static 	int Bit8Div_map[] = {1, 1, 4, 5};
	static  int FreqDiv2x_map[] = {2, 3, 4, 6};
	static  int DvpDiv_map[] = {1, 2, 8, 16};

	err = ov2675_read_reg(client, MISENSOR_8BIT,
					OV2675_REG_PLL_CTRL_1, &temp1);
	if (err)
		return err;
	// bit[5:0] PllDiv
	PllDiv = 64 - (temp1 & 0x3f);
	err = ov2675_read_reg(client, MISENSOR_8BIT,
					OV2675_REG_PLL_CTRL_2, &temp1);
	if (err)
		return err;
	// bit[2:0] Indiv
	temp2 = temp1 & 0x07;
	Indiv2x = Indiv2x_map[temp2];
	// bit[5:4] Bit8Div
	temp2 = (temp1 >> 4) & 0x03;
	Bit8Div = Bit8Div_map[temp2];
	// bit[7:6] FreqDiv
	temp2 = temp1 >> 6;
	FreqDiv2x = FreqDiv2x_map[temp2];
	err = ov2675_read_reg(client, MISENSOR_8BIT,
					OV2675_REG_PLL_CTRL_3, &temp1);
	if (err)
		return err;
	//bit[3:0] ScaleDiv
	temp2 = temp1 & 0x0f;
	if(temp2==0) {
		ScaleDiv = 1;
		}
	else {
		ScaleDiv = temp2 * 2;
		}
	// bit[4] SensorDiv
	if(temp1 & 0x10) {
	SensorDiv = 2;
	}
	else {
	SensorDiv = 1;
	}
	// bit[5] LaneDiv
	// bit[7:6] DvpDiv
	temp2 = temp1 >> 6;
	DvpDiv = DvpDiv_map[temp2];
	err = ov2675_read_reg(client, MISENSOR_8BIT,
					OV2675_REG_CLK_DIVIDER, &temp1);
	if (err)
		return err;
	// bit[5:0] ClkDiv
	temp2 = temp1 & 0x3f;
	ClkDiv = temp2 + 1;
	VCO = OV2675_XVCLK * Bit8Div * FreqDiv2x * PllDiv / Indiv2x ;
	*sysclk = VCO / Bit8Div / SensorDiv / ClkDiv / 4;

	if (*sysclk < MIN_SYSCLK)
		return -EINVAL;

	return 0;	
}
/* read binning from register settings */
static int ov2675_get_binning(struct v4l2_subdev *sd, unsigned int *binning)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;
	u32 temp;

	err = ov2675_read_reg(client, MISENSOR_16BIT,
				OV2675_REG_TIMING_HTS, &temp);
	if (err)
		return err;
	if(temp==0x52) {
	// OV2650
		*binning = 2;
	}
	else {
	// OV2675
		*binning = 1;
	}
	return 0;
}

/* read HTS from register settings */
static int ov2675_get_hts(struct v4l2_subdev *sd, unsigned int *hts)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;
	u32 val,extra_HTS;

	err = ov2675_read_reg(client, MISENSOR_16BIT,
				OV2675_REG_TIMING_HTS, hts);
	if (err)
		return err;
	val = *hts;
	err = ov2675_read_reg(client, MISENSOR_8BIT,
				OV2675_REG_TIMING_EXTRA_HTS, &extra_HTS);
	if (err)
		return err;
	*hts = val + extra_HTS;
	if (*hts < MIN_HTS)
		return -EINVAL;

	return 0;
}

/* read VTS from register settings */
static int ov2675_get_vts(struct v4l2_subdev *sd, unsigned int *vts)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;
	u32 val,extra_VTS;
	err = ov2675_read_reg(client, MISENSOR_16BIT,
				OV2675_REG_TIMING_VTS, vts);
	if (err)
		return err;
	val = *vts;
	err = ov2675_read_reg(client, MISENSOR_16BIT,
				OV2675_REG_TIMING_EXTRA_VTS, &extra_VTS);
	if (err)
		return err;
	*vts = val + extra_VTS;
	if (*vts < MIN_VTS)
		return -EINVAL;

	return 0;
}

/* write VTS to registers */
static int ov2675_set_vts(struct v4l2_subdev *sd, unsigned int vts)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return ov2675_write_reg(client, MISENSOR_16BIT,
					OV2675_REG_TIMING_VTS, vts);
}

/* read shutter, in number of line period */
static int ov2675_get_shutter(struct v4l2_subdev *sd, int *shutter)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;
	u32 val, temp;

	err = ov2675_read_reg(client, MISENSOR_16BIT,
					OV2675_REG_EXPOSURE_0, &val);
	if (err)
		return err;
	temp = val;

	err = ov2675_read_reg(client, MISENSOR_16BIT,
					OV2675_REG_EXPOSURE_1, &val);
	if (err)
		return err;

	*shutter = temp + val;

	if (*shutter < MIN_SHUTTER)
		return -EINVAL;

	return 0;
}

/* write shutter, in number of line period */
static int ov2675_set_shutter(struct v4l2_subdev *sd, unsigned int shutter)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;
	
	shutter = shutter & 0xffff;

	err = ov2675_write_reg(client, MISENSOR_16BIT,
					OV2675_REG_EXPOSURE_0, shutter);
	if (err)
		return err;

	return 0;
}

/* read gain, 16 = 1x */
static int ov2675_get_gain16(struct v4l2_subdev *sd, unsigned int *gain16)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;
	u32 val;

	err = ov2675_read_reg(client, MISENSOR_8BIT,
					OV2675_REG_GAIN, &val);
	if (err)
		return err;

	*gain16 = (((val & 0xf0)>>4)+1) * ((val & 0x0f)+16);

	if (*gain16 < MIN_GAIN)
		return -EINVAL;

	return 0;
}

/* write gain, 16 = 1x */
static int ov2675_set_gain16(struct v4l2_subdev *sd, unsigned int capture_gain16)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int gain =0 ;
	if( capture_gain16 > 31)
	{
		capture_gain16 >>=1;
		gain |=0x10;
	}
	if( capture_gain16 > 31)
	{	
		capture_gain16 >>=1;
		gain |=0x20;
	}
	if( capture_gain16 > 31)
	{
		capture_gain16 >>=1;
		gain |=0x40;
	}
	if( capture_gain16 > 31)
	{
		capture_gain16 >>=1;
		gain |=0x80;	
	}
	gain = gain | (capture_gain16 -16);
	
	return ov2675_write_reg(client, MISENSOR_8BIT,OV2675_REG_GAIN, gain);
}

/*
 * This returns the exposure compensation value, which is expressed in
 * terms of EV. The default EV value is 0, and driver don't support
 * adjust EV value.
 */
static int ov2675_get_exposure_bias(struct v4l2_subdev *sd, s32 *value)
{
	*value = 0;

	return 0;
}

/*
 * This returns ISO sensitivity.
 */
static int ov2675_get_iso(struct v4l2_subdev *sd, s32 *value)
{
	u32 gain;
	int err;

	err = ov2675_get_gain16(sd, &gain);
	if (err)
		return err;

	*value = gain / 16 * 100;

	return 0;
}

static int ov2675_g_image_brightness(struct v4l2_subdev *sd, int *brightness)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* get target image luminance average */
//	return ov2675_read_reg(client, MISENSOR_8BIT,
//					OV2675_REG_AE_AVERAGE, brightness);
	return 0;
}

/* get banding filter value */
static int ov2675_get_light_frequency(struct v4l2_subdev *sd,
				unsigned int *light_frequency)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;
	u32 temp;

	err = ov2675_read_reg(client, MISENSOR_8BIT,
					OV2675_REG_LIGHT_CTRL_0, &temp);
	if (err)
		return err;

	if (temp & OV2675_AUTO_BAND) {
		/* auto */
		err = ov2675_read_reg(client, MISENSOR_8BIT,
					OV2675_REG_LIGHT_CTRL_2, &temp);
		if (err)
			return err;
		if (temp & 0x01)
			/* 50Hz */
			*light_frequency = OV2675_LIGHT_50HZ;
		else
			/* 60Hz */
			*light_frequency = OV2675_LIGHT_60HZ;

	} else {
		/* manual */
		if (temp & 0x80)
			/* 50Hz */
			*light_frequency = OV2675_LIGHT_50HZ;
		else
			/* 60Hz */
			*light_frequency = OV2675_LIGHT_60HZ;
	}

	return 0;
}

static int ov2675_set_bandingfilter(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2675_device *dev = to_ov2675_sensor(sd);
	u32 band_step60, max_band60, band_step50, max_band50;
	int err;

	/* read preview PCLK */
	err = ov2675_get_sysclk(sd, &dev->preview_sysclk);
	if (err)
		return err;

	/* read preview HTS */
	err = ov2675_get_hts(sd, &dev->preview_hts);
	if (err)
		return err;

	/* read preview VTS */
	err = ov2675_get_vts(sd, &dev->preview_vts);
	if (err)
		return err;

	/* calculate banding filter */
	/* 60Hz */
	band_step60 = dev->preview_sysclk * 100 / dev->preview_hts * 100 / 120;
	if (band_step60 == 0)
		return -EINVAL;

	err = ov2675_write_reg(client, MISENSOR_8BIT, OV2675_REG_B60_STEP_HIGH,
								(band_step60 >> 8));
	if (err)
		return err;
	err = ov2675_write_reg(client, MISENSOR_8BIT, OV2675_REG_B60_STEP_LOW,
								(band_step60 & 0xff));
	if (err)
		return err;

	max_band60 = dev->preview_vts / band_step60;
	err = ov2675_write_reg(client, MISENSOR_8BIT,
						OV2675_REG_B60_MAX, max_band60-1);
	if (err)
		return err;

	/* 50Hz */
	band_step50 = dev->preview_sysclk * 100 / dev->preview_hts;
	if (band_step50 == 0)
		return -EINVAL;

	err = ov2675_write_reg(client, MISENSOR_8BIT, OV2675_REG_B50_STEP_HIGH,
								(band_step50 >> 8));
	if (err)
		return err;
	err = ov2675_write_reg(client, MISENSOR_8BIT, OV2675_REG_B50_STEP_LOW,
								(band_step50 & 0xff));
	if (err)
		return err;

	max_band50 = dev->preview_vts / band_step50;
	return ov2675_write_reg(client, MISENSOR_8BIT,
						OV2675_REG_B50_MAX, max_band50-1);
}


static int ov2675_set_night_mode(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int temp;
	int err;

	err = ov2675_read_reg(client, MISENSOR_8BIT,
					OV2675_REG_LIGHT_CTRL_0, &temp);
	if (err)
		return err;

	if (enable) {
		temp = temp | 0x08;
		return ov2675_write_reg(client, MISENSOR_8BIT,
					OV2675_REG_LIGHT_CTRL_0, temp);
	} else {
		temp = temp & 0xf7;
		err = ov2675_write_reg(client, MISENSOR_8BIT,
						OV2675_REG_LIGHT_CTRL_0, temp);
		err = ov2675_write_reg(client, MISENSOR_8BIT,
						0x302d, 0);
		return ov2675_write_reg(client, MISENSOR_8BIT,
					0x302e, 0);
	}
}

static int ov2675_set_ag_ae(struct v4l2_subdev *sd, int enable)
{
     
	 struct i2c_client *client = v4l2_get_subdevdata(sd);
	 int temp;
	 int err;
 
	 err = ov2675_read_reg(client, MISENSOR_8BIT,
					 OV2675_REG_AE_MODE_CTRL, &temp);
	 if (err)
		 return err;

	if (enable) {	
		temp = temp | 0x05;	
		return ov2675_write_reg(client, MISENSOR_8BIT,
					OV2675_REG_AE_MODE_CTRL, temp);
	} else {		
		temp = temp & 0xfa;
	       return ov2675_write_reg(client, MISENSOR_8BIT,
						OV2675_REG_AE_MODE_CTRL, temp);
	}
}

static int ov2675_start_preview(struct v4l2_subdev *sd)
{
	struct ov2675_device *dev = to_ov2675_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	
	ret = ov2675_set_gain16(sd, dev->preview_gain16);
	if (ret)
		return ret;

	ret = ov2675_set_shutter(sd, dev->preview_shutter);
	if (ret)
		return ret;	

	/*turn on AE/AG */
	ret = ov2675_set_ag_ae(sd,1);
	if (ret)
		return ret;	

	ret = ov2675_set_bandingfilter(sd);
	if (ret)
		return ret;
       
	return ov2675_set_night_mode(sd, dev->night_mode);

}

static int ov2675_stop_preview(struct v4l2_subdev *sd)
{
	struct ov2675_device *dev = to_ov2675_sensor(sd);
	int err;

	/* read preview shutter */
	err = ov2675_get_shutter(sd, &dev->preview_shutter);
	if (err)
		return err;

	err = ov2675_get_gain16(sd, &dev->preview_gain16);
	if (err)
		return err;

	err = ov2675_get_hts(sd, &dev->preview_hts);
	if (err)
		return err;

	return 0;
}

static int ov2675_start_video(struct v4l2_subdev *sd)
{
	int err;

	err = ov2675_set_bandingfilter(sd);
	if (err)
		return err;

	//return ov2675_set_night_mode(sd, 0);
       return 0;
}

static int ov2675_start_capture(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2675_device *dev = to_ov2675_sensor(sd);
	u32 capture_sysclk, capture_hts, capture_vts, preview_binning;
	u32 capture_shutter, capture_gain16;
	u32 light_frequency, capture_bandingfilter, capture_max_band;
	long capture_gain16_shutter;
	int err,val,temp;
	/* turn off night mode for capture */
	ov2675_set_night_mode(sd, 0);

	/*turn off AE/AG */
	err = ov2675_set_ag_ae(sd,0);
	if (err)
		return err;
	
	/* read capture VTS */
	err = ov2675_get_vts(sd, &capture_vts);
	if (err)
		return err;

	err = ov2675_get_hts(sd, &capture_hts);
	if (err)
		return err;

	err = ov2675_get_binning(sd, &preview_binning);
	if (err)
		return err;

	err = ov2675_get_sysclk(sd, &capture_sysclk);
	if (err)
		return err;

	/* calculate capture banding filter */
	err = ov2675_get_light_frequency(sd, &light_frequency);
	if (err)
		return err;

	if (light_frequency == OV2675_LIGHT_60HZ) {
		/* 60Hz */
		capture_bandingfilter = capture_sysclk * 100 /
						capture_hts * 100 / 120;
	} else {
		/* 50Hz */
		capture_bandingfilter = capture_sysclk * 100 / capture_hts;
	}

	if (capture_bandingfilter == 0)
		return -EINVAL;

	capture_max_band = (int)((capture_vts) / capture_bandingfilter);
	if (capture_max_band == 0)
		return -EINVAL;
	/* calculate capture shutter/gain16 */
	capture_gain16_shutter = dev->preview_gain16 *
						dev->preview_shutter *
						capture_sysclk / dev->preview_sysclk *
						dev->preview_hts / capture_hts * preview_binning;

	/* gain to shutter */
	if (capture_gain16_shutter < (capture_bandingfilter * 16)) {
		/* shutter < 1/100 */
		capture_shutter = capture_gain16_shutter / 16;
		if (capture_shutter < 1)
			capture_shutter = 1;
		capture_gain16 = capture_gain16_shutter / capture_shutter;
		if (capture_gain16 < 16)
			capture_gain16 = 16;
	} else {
		if (capture_gain16_shutter >
			(capture_bandingfilter * capture_max_band * 16)) {
			/* exposure reach max */
			capture_shutter = capture_bandingfilter *
							capture_max_band;
			capture_gain16 = capture_gain16_shutter /
							capture_shutter;
		} else {
			/*
			 * 1/100 < capture_shutter =< max,
			 * capture_shutter = n/100
			 */
			capture_shutter = ((int)(capture_gain16_shutter / 16 /
						capture_bandingfilter)) *
						capture_bandingfilter;
			if (capture_shutter == 0)
				return -EINVAL;

			capture_gain16 = capture_gain16_shutter /
						capture_shutter;
		}
	}

	/* write capture gain */
	err = ov2675_set_gain16(sd, capture_gain16);
	if (err)
		return err;

	/* write capture shutter */
	if (capture_shutter > (capture_vts)) {
		capture_vts = capture_shutter;
		err = ov2675_set_vts(sd, capture_vts);
		if (err)
			return err;
	}

	return ov2675_set_shutter(sd, capture_shutter);
}

static int ov2675_standby(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return ov2675_write_reg_array(client, ov2675_standby_reg);
}

static int ov2675_wakeup(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return ov2675_write_reg_array(client, ov2675_wakeup_reg);
}

static int __ov2675_init(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = ov2675_write_reg_array(client, ov2675_init);
	if (ret)
		return ret;
	printk("ov2675_init finised \n");
	/*
	 * delay 5ms to wait for sensor initialization finish.
	 */
	usleep_range(5000, 6000);

	msleep(20);

	return ov2675_standby(sd);
	
}

static int power_up(struct v4l2_subdev *sd)
{
	struct ov2675_device *dev = to_ov2675_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	if (NULL == dev->platform_data) {
		dev_err(&client->dev, "no camera_sensor_platform_data");
		return -ENODEV;
	}

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret)
		goto fail_power;

	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret)
		goto fail_clk;

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret)
		dev_err(&client->dev, "gpio failed\n");

	/*
	 * according to DS, 20ms is needed between power up and first i2c
	 * commend
	 */
	msleep(20);
	printk("**********ov2675 power_up finised **** \n");
	return 0;

fail_clk:
	dev->platform_data->flisclk_ctrl(sd, 0);
fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct ov2675_device *dev = to_ov2675_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	if (NULL == dev->platform_data) {
		dev_err(&client->dev, "no camera_sensor_platform_data");
		return -ENODEV;
	}

	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio failed\n");

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	/*according to DS, 20ms is needed after power down*/
	msleep(20);
	printk("**********ov2675 power_down finised **** \n");
	return ret;
}

static int ov2675_s_power(struct v4l2_subdev *sd, int power)
{
	if (power == 0)
		return power_down(sd);

	if (power_up(sd))
		return -EINVAL;
	
		msleep(20);
		usleep_range(5000, 6000);
	return __ov2675_init(sd);;
}

static int ov2675_try_res(u32 *w, u32 *h)
{
	int i;

	/*
	 * The mode list is in ascending order. We're done as soon as
	 * we have found the first equal or bigger size.
	 */
	for (i = 0; i < N_RES; i++) {
		if (ov2675_res[i].width >= *w &&
		    ov2675_res[i].height >= *h)
			break;
	}

	/*
	 * If no mode was found, it means we can provide only a smaller size.
	 * Returning the biggest one available in this case.
	 */
	if (i == N_RES)
		i--;

	*w = ov2675_res[i].width;
	*h = ov2675_res[i].height;

	return 0;
}

static struct ov2675_res_struct *ov2675_to_res(u32 w, u32 h)
{
	int  index;

	for (index = 0; index < N_RES; index++) {
		if (ov2675_res[index].width == w &&
		    ov2675_res[index].height == h)
			break;
	}

	/* No mode found */
	if (index >= N_RES)
		return NULL;

	return &ov2675_res[index];
}

static int ov2675_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	return ov2675_try_res(&fmt->width, &fmt->height);
}

static int ov2675_res2size(unsigned int res, int *h_size, int *v_size)
{
	unsigned short hsize;
	unsigned short vsize;

	switch (res) {
	
	case OV2675_RES_VGA:
		hsize = OV2675_RES_VGA_SIZE_H;
		vsize = OV2675_RES_VGA_SIZE_V;
		break;
	case OV2675_RES_480P:
		hsize = OV2675_RES_480P_SIZE_H;
		vsize = OV2675_RES_480P_SIZE_V;
		break;
	case OV2675_RES_720P:
		hsize = OV2675_RES_720P_SIZE_H;
		vsize = OV2675_RES_720P_SIZE_V;
		break;
	case OV2675_RES_1M:
		hsize = OV2675_RES_1M_SIZE_H;
		vsize = OV2675_RES_1M_SIZE_V;
		break;
	case OV2675_RES_2M:
		hsize = OV2675_RES_2M_SIZE_H;
		vsize = OV2675_RES_2M_SIZE_V;
		break;
	default:
		/* QVGA mode is still unsupported */
		WARN(1, "%s: Resolution 0x%08x unknown\n", __func__, res);
		return -EINVAL;
	}

	if (h_size != NULL)
		*h_size = hsize;
	if (v_size != NULL)
		*v_size = vsize;

	return 0;
}

static int ov2675_g_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct ov2675_device *dev = to_ov2675_sensor(sd);
	int width, height;
	int ret;

	ret = ov2675_res2size(dev->res, &width, &height);
	if (ret)
		return ret;
	fmt->width = width;
	fmt->height = height;

	return 0;
}

static int ov2675_s_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	struct ov2675_device *dev = to_ov2675_sensor(sd);
	struct ov2675_res_struct *res_index;
	u32 width = fmt->width;
	u32 height = fmt->height;
	int ret;

	ov2675_try_res(&width, &height);
	res_index = ov2675_to_res(width, height);

	/* Sanity check */
	if (unlikely(!res_index)) {
		WARN_ON(1);
		return -EINVAL;
	}
	printk("*******ov2675 try width == %d ,height==%d \n",width, height);
	
	printk("*******ov2675 resolution res_index->res = %d\n",res_index->res);

	switch (res_index->res) {
	
	case OV2675_RES_VGA:
		ret = ov2675_write_reg_array(c, ov2675_vga_init);
		break;
	case OV2675_RES_480P:
		ret = ov2675_write_reg_array(c, ov2675_480p_init);
		break;
	case OV2675_RES_720P:
		ret = ov2675_write_reg_array(c, ov2675_720p_init);
		break;
	case OV2675_RES_1M:
		ret = ov2675_write_reg_array(c, ov2675_1M_init);
		break;
	case OV2675_RES_2M:
		ret = ov2675_write_reg_array(c, ov2675_2M_init);
		break;
	default:
		/* QVGA is not implemented yet */
		dev_err(&c->dev, "set resolution: %d failed!\n",
							res_index->res);
		return -EINVAL;
	}
	if (ret)
		return -EINVAL;
	printk("**********finished set fmt with ov2675\n");
	if (dev->res != res_index->res) {
		int index;

		/*
		 * Marked current sensor res as being "used"
		 *
		 * REVISIT: We don't need to use an "used" field on each mode
		 * list entry to know which mode is selected. If this
		 * information is really necessary, how about to use a single
		 * variable on sensor dev struct?
		 */
		for (index = 0; index < N_RES; index++) {
			if (width == ov2675_res[index].width &&
			    height == ov2675_res[index].height) {
				ov2675_res[index].used = 1;
				continue;
			}
			ov2675_res[index].used = 0;
		}
	}

	/*
	 * ov2675 - we don't poll for context switch
	 * because it does not happen with streaming disabled.
	 */
	dev->res = res_index->res;

	fmt->width = width;
	fmt->height = height;
       	fmt->code = V4L2_MBUS_FMT_UYVY8_1X16;
	msleep(20);
	printk("**********finished set fmt with ov2675 msleep 20\n");
        return 0;
}

static int ov2675_detect(struct i2c_client *client,  u16 *id, u8 *revision)
{
	struct i2c_adapter *adapter = client->adapter;
	u32 retvalue;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c error", __func__);
		return -ENODEV;
	}

	if (ov2675_read_reg(client, MISENSOR_16BIT,
		OV2675_REG_PID, &retvalue)) {
		dev_err(&client->dev, "sensor_id_high = 0x%x\n", retvalue);
		return -ENODEV;
	}
	dev_info(&client->dev, "sensor_id = 0x%x\n", retvalue);
	if (retvalue != OV2675_MOD_ID) {
		dev_err(&client->dev, "%s: failed: client->addr = %x\n",
			__func__, client->addr);
		return -ENODEV;
	}

	/* REVISIT: HACK: Driver is currently forcing revision to 0 */
	*revision = 0;

	return 0;
}

static int
ov2675_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct ov2675_device *dev = to_ov2675_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 sensor_revision;
	u16 sensor_id = 0;
	int ret;

	if (NULL == platform_data)
		return -ENODEV;

	dev->platform_data =
	    (struct camera_sensor_platform_data *)platform_data;
	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
	if (ret) {
			v4l2_err(client, "ov2675 platform init err\n");
			return ret;
		}
	}
	ret = ov2675_s_power(sd, 1);
	if (ret) {
		dev_err(&client->dev, "power_ctrl failed");
		return ret;
	}

	/* config & detect sensor */
	ret = ov2675_detect(client, &sensor_id, &sensor_revision);
	if (ret) {
		dev_err(&client->dev, "ov2675_detect err s_config.\n");
		goto fail_detect;
	}

	dev->sensor_id = sensor_id;
	dev->sensor_revision = sensor_revision;

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	ret = ov2675_s_power(sd, 0);
	if (ret)
		dev_err(&client->dev, "sensor power-gating failed\n");

	return ret;

fail_csi_cfg:
	dev->platform_data->csi_cfg(sd, 0);
fail_detect:
	ov2675_s_power(sd, 0);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}


static int ov2675_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (OV2675_FOCAL_LENGTH_NUM << 16) | OV2675_FOCAL_LENGTH_DEM;
	return 0;
}

static int ov2675_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/* const f number for OV2675 */
	*val = (OV2675_F_NUMBER_DEFAULT_NUM << 16) | OV2675_F_NUMBER_DEM;
	return 0;
}

static int ov2675_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (OV2675_F_NUMBER_DEFAULT_NUM << 24) |
		(OV2675_F_NUMBER_DEM << 16) |
		(OV2675_F_NUMBER_DEFAULT_NUM << 8) | OV2675_F_NUMBER_DEM;
	return 0;
}
static int ov2675_s_freq(struct v4l2_subdev *sd, s32  val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2675_device *dev = to_ov2675_sensor(sd);
	int ret;	
	unsigned char temp;
	if (val != OV2675_FLICKER_MODE_50HZ && val != OV2675_FLICKER_MODE_60HZ)
		return -EINVAL;		

	if (val == OV2675_FLICKER_MODE_50HZ) {
		ret = ov2675_read_reg(client, MISENSOR_8BIT,OV2675_REG_LIGHT_CTRL_0,&temp);
		temp=temp|0x80; //bit[7]=1, 50hz
		ret+=ov2675_write_reg(client, MISENSOR_8BIT,OV2675_REG_LIGHT_CTRL_0,temp);
		
		if (ret < 0)
			return ret;
			
	} else {		  
		ret = ov2675_read_reg(client, MISENSOR_8BIT,OV2675_REG_LIGHT_CTRL_0,&temp);
		temp=temp&0x7f; //bit[7]=0, 60hz		
		ret=ov2675_write_reg(client, MISENSOR_8BIT,OV2675_REG_LIGHT_CTRL_0,temp);
		if (ret < 0)
			return ret;			
	}
	return ret;
}


static struct ov2675_control ov2675_controls[] = {
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = ov2675_get_shutter,
	},
	{
		.qc = {
			.id = V4L2_CID_AUTO_EXPOSURE_BIAS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure bias",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = ov2675_get_exposure_bias,
	},
	{
		.qc = {
			.id = V4L2_CID_ISO_SENSITIVITY,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "iso",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = ov2675_get_iso,
	},
	{
		.qc = {
			.id = V4L2_CID_BRIGHTNESS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "target image luminance",
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = 0,
		},
		.query = ov2675_g_image_brightness,
	},
	{
		.qc = {
			.id = V4L2_CID_COLORFX,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "color effect",
			.minimum = 0,
			.maximum = 9,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov2675_s_color_effect,
		.query = ov2675_g_color_effect,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = 0,
			.maximum = OV2675_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = OV2675_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = ov2675_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = 0,
			.maximum = OV2675_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = OV2675_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = ov2675_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = 0,
			.maximum =  OV2675_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = OV2675_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = ov2675_g_fnumber_range,
	},
	{
		.qc = {
			.id = V4L2_CID_POWER_LINE_FREQUENCY,
			.type = V4L2_CTRL_TYPE_MENU,
			.name = "Light frequency filter",
			.minimum = 1,
			.maximum =  2, /* 1: 50Hz, 2:60Hz */
			.step = 1,
			.default_value = 1,
			.flags = 0,
		},
		.tweak = ov2675_s_freq,
	},
};
#define N_CONTROLS (ARRAY_SIZE(ov2675_controls))

static struct ov2675_control *ov2675_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++) {
		if (ov2675_controls[i].qc.id == id)
			return &ov2675_controls[i];
	}
	return NULL;
}

static int ov2675_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct ov2675_control *ctrl = ov2675_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int ov2675_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov2675_control *octrl = ov2675_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;

	ret = octrl->query(sd, &ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int ov2675_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov2675_control *octrl = ov2675_find_control(ctrl->id);
	int ret;

	if (!octrl || !octrl->tweak)
		return -EINVAL;

	ret = octrl->tweak(sd, ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int ov2675_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2675_device *dev = to_ov2675_sensor(sd);
	int err;
	printk("**********s_stream with ov2675 stream==%d  run_mode=%d\n",enable,dev->run_mode);
	if (enable) {
		switch (dev->run_mode) {
		case CI_MODE_PREVIEW:
			err = ov2675_start_preview(sd);
			break;
		case CI_MODE_VIDEO:
			err = ov2675_start_video(sd);
			break;
		case CI_MODE_STILL_CAPTURE:
			err = ov2675_start_capture(sd);
			break;
		default:
			dev_err(&client->dev,
				"invalid run mode.\n");
			return -EINVAL;
		}
		if (err)
			dev_warn(&client->dev,
				"fail to start preview/video/capture.\n");
		err =ov2675_wakeup(sd);
		if (err)
		return err;
		dev->streaming = true;
		printk("---ov2675_wakeup success -----\n");
	} else {
		if (dev->run_mode == CI_MODE_PREVIEW) {
			err = ov2675_stop_preview(sd);
			if (err)
				dev_warn(&client->dev,
					"fail to stop preview\n");
		}

		err = ov2675_standby(sd);
		if (err)
			return err;
		dev->streaming = false;
		printk("---ov2675_standby success -----\n");
	}

	return 0;
}

static int
ov2675_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;

	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = ov2675_res[index].width;
	fsize->discrete.height = ov2675_res[index].height;

	fsize->reserved[0] = ov2675_res[index].used;

	return 0;
}

static int ov2675_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	int i;

	if (index >= N_RES)
		return -EINVAL;

	/* find out the first equal or bigger size */
	for (i = 0; i < N_RES; i++) {
		if (ov2675_res[i].width >= fival->width &&
		    ov2675_res[i].height >= fival->height)
			break;
	}
	if (i == N_RES)
		i--;

	index = i;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = ov2675_res[index].fps;

	return 0;
}

static int
ov2675_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_OV2675, 0);
}

static int ov2675_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	return 0;
}

static int ov2675_enum_frame_size(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_frame_size_enum *fse)
{
	unsigned int index = fse->index;

	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = ov2675_res[index].width;
	fse->min_height = ov2675_res[index].height;
	fse->max_width = ov2675_res[index].width;
	fse->max_height = ov2675_res[index].height;

	return 0;
}

static int
ov2675_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ov2675_device *snr = to_ov2675_sensor(sd);

	switch (fmt->which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		fmt->format = *v4l2_subdev_get_try_format(fh, fmt->pad);
		break;
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		fmt->format = snr->format;
	}

	return 0;
}

static int
ov2675_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ov2675_device *snr = to_ov2675_sensor(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;

	return 0;
}


static int
ov2675_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct ov2675_device *dev = to_ov2675_sensor(sd);
	memset(param, 0, sizeof(*param));
	param->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (dev->res >= 0 && dev->res < N_RES) {
		param->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		param->parm.capture.timeperframe.numerator = 1;
		param->parm.capture.capturemode = dev->run_mode;
		param->parm.capture.timeperframe.denominator =
			ov2675_res[dev->res].fps;
	}
	return 0;
}

static int
ov2675_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct ov2675_device *dev = to_ov2675_sensor(sd);
	dev->run_mode = param->parm.capture.capturemode;
	return ov2675_g_parm(sd, param);
}

static int
ov2675_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	int index;
	struct ov2675_device *snr = to_ov2675_sensor(sd);

	for (index = 0; index < N_RES; index++) {
		if (ov2675_res[index].res == snr->res) {
			*frames = ov2675_res[index].skip_frames;
			return 0;
		}
	}
	return -EINVAL;
}

static const struct v4l2_subdev_video_ops ov2675_video_ops = {
	.try_mbus_fmt = ov2675_try_mbus_fmt,
	.g_mbus_fmt = ov2675_g_mbus_fmt,
	.s_mbus_fmt = ov2675_s_mbus_fmt,
	.s_parm = ov2675_s_parm,
	.g_parm = ov2675_g_parm,
	.s_stream = ov2675_s_stream,
	.enum_framesizes = ov2675_enum_framesizes,
	.enum_frameintervals = ov2675_enum_frameintervals,
};

static const struct v4l2_subdev_sensor_ops ov2675_sensor_ops = {
	.g_skip_frames	= ov2675_g_skip_frames,
};

static const struct v4l2_subdev_core_ops ov2675_core_ops = {
	.g_chip_ident = ov2675_g_chip_ident,
	.queryctrl = ov2675_queryctrl,
	.g_ctrl = ov2675_g_ctrl,
	.s_ctrl = ov2675_s_ctrl,
	.s_power = ov2675_s_power,
};

static const struct v4l2_subdev_pad_ops ov2675_pad_ops = {
	.enum_mbus_code = ov2675_enum_mbus_code,
	.enum_frame_size = ov2675_enum_frame_size,
	.get_fmt = ov2675_get_pad_format,
	.set_fmt = ov2675_set_pad_format,
};

static const struct v4l2_subdev_ops ov2675_ops = {
	.core = &ov2675_core_ops,
	.video = &ov2675_video_ops,
	.sensor = &ov2675_sensor_ops,
	.pad = &ov2675_pad_ops,
};

static const struct media_entity_operations ov2675_entity_ops;

static int ov2675_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2675_device *dev = container_of(sd,
					struct ov2675_device, sd);

	dev->platform_data->csi_cfg(sd, 0);

	v4l2_device_unregister_subdev(sd);
	if (dev->platform_data->platform_deinit)
	dev->platform_data->platform_deinit();
	media_entity_cleanup(&dev->sd.entity);
	kfree(dev);

	return 0;
}

static int ov2675_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct ov2675_device *dev;
	int ret;
	/* Setup sensor configuration structure */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	v4l2_i2c_subdev_init(&dev->sd, client, &ov2675_ops);
	if (client->dev.platform_data) {
		ret = ov2675_s_config(&dev->sd, client->irq,
				       client->dev.platform_data);
		if (ret) {
			v4l2_device_unregister_subdev(&dev->sd);
			kfree(dev);
			return ret;
		}
	}
	/*TODO add format code here*/
	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
//	dev->sd.entity.ops = &ov2675_entity_ops;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		ov2675_remove(client);
		return ret;
	}

	/* set res index to be invalid */
	dev->res = -1;


	/* set color_effect to be invalid */
	dev->color_effect = -1;
	dev->preview_gain16 = OV2675_DEFAULT_GAIN;
	dev->preview_shutter = OV2675_DEFAULT_SHUTTER;
	dev->night_mode = 1;

	return 0;
}

MODULE_DEVICE_TABLE(i2c, ov2675_id);
static struct i2c_driver ov2675_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = OV2675_NAME,
	},
	.probe = ov2675_probe,
	.remove = __exit_p(ov2675_remove),
	.id_table = ov2675_id,
};

static __init int ov2675_init_mod(void)
{
	return i2c_add_driver(&ov2675_driver);
}

static __exit void ov2675_exit_mod(void)
{
	i2c_del_driver(&ov2675_driver);
}

module_init(ov2675_init_mod);
module_exit(ov2675_exit_mod);

MODULE_DESCRIPTION("A low-level driver for Omnivision OV2675 sensors");
MODULE_LICENSE("GPL");
