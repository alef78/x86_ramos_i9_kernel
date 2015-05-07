/*
 * Support for gc0339 Camera Sensor.
 *
 * Copyright (c) 2013 ASUSTeK COMPUTER INC. All Rights Reserved.
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
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>

#include "gc0339.h"

#define to_gc0339_sensor(sd) container_of(sd, struct gc0339_device, sd)

static int
gc0339_read_reg(struct i2c_client *client, u16 data_length, u8 reg, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[2];

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != MISENSOR_8BIT && data_length != MISENSOR_16BIT
					 && data_length != MISENSOR_32BIT) {
		v4l2_err(client, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = MSG_LEN_OFFSET;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (reg & 0xff);

	msg[1].addr = client->addr;
	msg[1].len = data_length;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data+1;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err >= 0) {
		*val = 0;
		/* high byte comes first */
		if (data_length == MISENSOR_8BIT)
			*val = data[1];
		/*else if (data_length == MISENSOR_16BIT)
			*val = data[1] + (data[0] << 8);
		else
			*val = data[3] + (data[2] << 8) +
			    (data[1] << 16) + (data[0] << 24);*/

		return 0;
	}

	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int
gc0339_write_reg(struct i2c_client *client, u16 data_length, u8 reg, u8 val)
{
	int num_msg;
	struct i2c_msg msg;
	unsigned char data[4] = {0};
	u16 *wreg;
	int retry = 0;

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != MISENSOR_8BIT && data_length != MISENSOR_16BIT
					 && data_length != MISENSOR_32BIT) {
		v4l2_err(client, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	memset(&msg, 0, sizeof(msg));

again:

	data[0] = (u8) reg;
	data[1] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 1 + data_length;
	msg.buf = data;

	/* high byte goes out first 
	wreg = (u16 *)data;
	*wreg = cpu_to_be16(reg);*/

	if (data_length == MISENSOR_8BIT) {
		data[2] = (u8)(val);
	} /*else if (data_length == MISENSOR_16BIT) {
		u16 *wdata = (u16 *)&data[2];
		*wdata = be16_to_cpu((u16)val);
	} else {
		 MISENSOR_32BIT 
		u32 *wdata = (u32 *)&data[2];
		*wdata = be32_to_cpu(val);
	}*/

	num_msg = i2c_transfer(client->adapter, &msg, 1);

	/*
	 * HACK: Need some delay here for Rev 2 sensors otherwise some
	 * registers do not seem to load correctly.
	 */
	mdelay(1);

	if (num_msg >= 0)
		return 0;

	dev_err(&client->dev, "write error: wrote 0x%x to offset 0x%x error %d",
		val, reg, num_msg);
	if (retry <= I2C_RETRY_COUNT) {
		dev_dbg(&client->dev, "retrying... %d", retry);
		retry++;
		msleep(20);
		goto again;
	}

	return num_msg;
}
/**
 * misensor_rmw_reg - Read/Modify/Write a value to a register in the sensor
 * device
 * @client: i2c driver client structure
 * @data_length: 8/16/32-bits length
 * @reg: register address
 * @mask: masked out bits
 * @set: bits set
 *
 * Read/modify/write a value to a register in the  sensor device.
 * Returns zero if successful, or non-zero otherwise.
 */
static int
misensor_rmw_reg(struct i2c_client *client, u16 data_length, u16 reg,
		     u32 mask, u32 set)
{
	int err;
	u32 val;

	/* Exit when no mask */
	if (mask == 0)
		return 0;

	/* @mask must not exceed data length */
	switch (data_length) {
	case MISENSOR_8BIT:
		if (mask & ~0xff)
			return -EINVAL;
		break;
	case MISENSOR_16BIT:
		if (mask & ~0xffff)
			return -EINVAL;
		break;
	case MISENSOR_32BIT:
		break;
	default:
		/* Wrong @data_length */
		return -EINVAL;
	}

	err = gc0339_read_reg(client, data_length, reg, &val);
	if (err) {
		v4l2_err(client, "misensor_rmw_reg error exit, read failed\n");
		return -EINVAL;
	}

	val &= ~mask;

	/*
	 * Perform the OR function if the @set exists.
	 * Shift @set value to target bit location. @set should set only
	 * bits included in @mask.
	 *
	 * REVISIT: This function expects @set to be non-shifted. Its shift
	 * value is then defined to be equal to mask's LSB position.
	 * How about to inform values in their right offset position and avoid
	 * this unneeded shift operation?
	 */
	set <<= ffs(mask) - 1;
	val |= set & mask;

	err = gc0339_write_reg(client, data_length, reg, val);
	if (err) {
		v4l2_err(client, "misensor_rmw_reg error exit, write failed\n");
		return -EINVAL;
	}

	return 0;
}


static int __gc0339_flush_reg_array(struct i2c_client *client,
				     struct gc0339_write_ctrl *ctrl)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;
	int retry = 0;

	if (ctrl->index == 0)
		return 0;

again:
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2 + ctrl->index;
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	msg.buf = (u8 *)&ctrl->buffer;

	ret = i2c_transfer(client->adapter, &msg, num_msg);
	if (ret != num_msg) {
		if (++retry <= I2C_RETRY_COUNT) {
			dev_dbg(&client->dev, "retrying... %d\n", retry);
			msleep(20);
			goto again;
		}
		dev_err(&client->dev, "%s: i2c transfer error\n", __func__);
		return -EIO;
	}

	ctrl->index = 0;

	/*
	 * REVISIT: Previously we had a delay after writing data to sensor.
	 * But it was removed as our tests have shown it is not necessary
	 * anymore.
	 */

	return 0;
}

static int __gc0339_buf_reg_array(struct i2c_client *client,
				   struct gc0339_write_ctrl *ctrl,
				   const struct misensor_reg *next)
{
	u16 *data16;
	u32 *data32;
	int err;

	/* Insufficient buffer? Let's flush and get more free space. */
	if (ctrl->index + next->length >= GC0339_MAX_WRITE_BUF_SIZE) {
		err = __gc0339_flush_reg_array(client, ctrl);
		if (err)
			return err;
	}

	switch (next->length) {
	case MISENSOR_8BIT:
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case MISENSOR_16BIT:
		data16 = (u16 *)&ctrl->buffer.data[ctrl->index];
		*data16 = cpu_to_be16((u16)next->val);
		break;
	case MISENSOR_32BIT:
		data32 = (u32 *)&ctrl->buffer.data[ctrl->index];
		*data32 = cpu_to_be32(next->val);
		break;
	default:
		return -EINVAL;
	}

	/* When first item is added, we need to store its starting address */
	if (ctrl->index == 0)
		ctrl->buffer.addr = next->reg;

	ctrl->index += next->length;

	return 0;
}

static int
__gc0339_write_reg_is_consecutive(struct i2c_client *client,
				   struct gc0339_write_ctrl *ctrl,
				   const struct misensor_reg *next)
{
	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->reg;
}

static int gc0339_wait_state(struct i2c_client *client, int timeout)
{
	int ret;
	unsigned int val;

	while (timeout-- > 0) {
		ret = gc0339_read_reg(client, MISENSOR_16BIT, 0x0080, &val);
		if (ret)
			return ret;
		if ((val & 0x2) == 0)
			return 0;
		msleep(20);
	}

	return -EINVAL;
}

/*
 * gc0339_write_reg_array - Initializes a list of gc0339 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 * @poll: completion polling requirement
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __gc0339_flush_reg_array, __gc0339_buf_reg_array() and
 * __gc0339_write_reg_is_consecutive() are internal functions to
 * gc0339_write_reg_array() and should be not used anywhere else.
 *
 */
static int gc0339_write_reg_array(struct i2c_client *client,
				const struct misensor_reg *reglist,
				int poll)
{
	const struct misensor_reg *next = reglist;
	struct gc0339_write_ctrl ctrl;
	int err;

	if (poll == PRE_POLLING) {
		err = gc0339_wait_state(client, GC0339_WAIT_STAT_TIMEOUT);
		if (err)
			return err;
	}

	ctrl.index = 0;
	for (; next->length != MISENSOR_TOK_TERM; next++) {
		switch (next->length & MISENSOR_TOK_MASK) {
		case MISENSOR_TOK_DELAY:
			err = __gc0339_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			msleep(next->val);
			break;
		case MISENSOR_TOK_RMW:
			err = __gc0339_flush_reg_array(client, &ctrl);
			err |= misensor_rmw_reg(client,
						next->length &
							~MISENSOR_TOK_RMW,
						next->reg, next->val,
						next->val2);
			if (err) {
				dev_err(&client->dev, "%s read err. aborted\n",
					__func__);
				return -EINVAL;
			}
			break;
		default:
			/*
			 * If next address is not consecutive, data needs to be
			 * flushed before proceed.
			 */
			if (!__gc0339_write_reg_is_consecutive(client, &ctrl,
								next)) {
				err = __gc0339_flush_reg_array(client, &ctrl);
				if (err)
					return err;
			}
			err = __gc0339_buf_reg_array(client, &ctrl, next);
			if (err) {
				v4l2_err(client, "%s: write error, aborted\n",
					 __func__);
				return err;
			}
			break;
		}
	}

	err = __gc0339_flush_reg_array(client, &ctrl);
	if (err)
		return err;

	if (poll == POST_POLLING)
		return gc0339_wait_state(client, GC0339_WAIT_STAT_TIMEOUT);

	return 0;
}

static int gc0339_set_suspend(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	gc0339_write_reg(client, MISENSOR_8BIT,  0x60, 0x80); //10bit raw disable

	return 0;
}

static int gc0339_set_streaming(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	gc0339_write_reg(client, MISENSOR_8BIT,  0xFE, 0x50); //Wesley_Kao, enable per frame MIPI reset
	gc0339_write_reg(client, MISENSOR_8BIT,  0x60, 0x98); //10bit raw enable

	return 0;
}

static int gc0339_init_common(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	gc0339_write_reg(client, MISENSOR_8BIT, 0xFE, 0x80);
	gc0339_write_reg(client, MISENSOR_8BIT, 0xFC, 0x10);
	gc0339_write_reg(client, MISENSOR_8BIT, 0xFE, 0x00);
	gc0339_write_reg(client, MISENSOR_8BIT, 0xF6, 0x05);
	gc0339_write_reg(client, MISENSOR_8BIT, 0xF7, 0x01);
	gc0339_write_reg(client, MISENSOR_8BIT, 0xF7, 0x03);
	gc0339_write_reg(client, MISENSOR_8BIT, 0xFC, 0x16);

	gc0339_write_reg(client, MISENSOR_8BIT, 0x06, 0x00);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x08, 0x00);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x09, 0x01); //by Wesley
	gc0339_write_reg(client, MISENSOR_8BIT, 0x0A, 0xF2); //by Wesley
	gc0339_write_reg(client, MISENSOR_8BIT, 0x0B, 0x02); //by Wesley
	gc0339_write_reg(client, MISENSOR_8BIT, 0x0C, 0x94); //by Wesley

	gc0339_write_reg(client, MISENSOR_8BIT, 0x01, 0x90); //DummyHor 144
	gc0339_write_reg(client, MISENSOR_8BIT, 0x02, 0x31); //Wesley min:0x30

	gc0339_write_reg(client, MISENSOR_8BIT, 0x0F, 0x00); //DummyHor 144
	gc0339_write_reg(client, MISENSOR_8BIT, 0x14, 0x00);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x1A, 0x21);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x1B, 0x08);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x1C, 0x19);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x1D, 0xEA);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x20, 0xB0);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x2E, 0x00);

	gc0339_write_reg(client, MISENSOR_8BIT, 0x30, 0xB7);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x31, 0x7F);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x32, 0x00);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x39, 0x04);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x3A, 0x20);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x3B, 0x20);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x3C, 0x00);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x3D, 0x00);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x3E, 0x00);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x3F, 0x00);

	gc0339_write_reg(client, MISENSOR_8BIT, 0x62, 0x34);  //by Wesley
	gc0339_write_reg(client, MISENSOR_8BIT, 0x63, 0x03); // by Wesley
	gc0339_write_reg(client, MISENSOR_8BIT, 0x69, 0x03);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x60, 0x80);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x65, 0x20); //20 -> 21
	gc0339_write_reg(client, MISENSOR_8BIT, 0x6C, 0x40);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x6D, 0x01);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x6A, 0x34); //BOTH 0x34

	gc0339_write_reg(client, MISENSOR_8BIT, 0x4A, 0x50);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x4B, 0x40);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x4C, 0x40);
	gc0339_write_reg(client, MISENSOR_8BIT, 0xE8, 0x04);
	gc0339_write_reg(client, MISENSOR_8BIT, 0xE9, 0xBB);

	gc0339_write_reg(client, MISENSOR_8BIT, 0x42, 0x20);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x47, 0x10);

	gc0339_write_reg(client, MISENSOR_8BIT, 0x50, 0x80);

	gc0339_write_reg(client, MISENSOR_8BIT, 0xD0, 0x00);
	gc0339_write_reg(client, MISENSOR_8BIT, 0xD2, 0x00); //disable AE
	gc0339_write_reg(client, MISENSOR_8BIT, 0xD3, 0x50);

	gc0339_write_reg(client, MISENSOR_8BIT, 0x71, 0x01);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x72, 0x01);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x73, 0x05); //Nor:0x05 DOU:0x06 //clk zero 0x05 //data zero 0x7a=0x0a
	gc0339_write_reg(client, MISENSOR_8BIT, 0x74, 0x01);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x76, 0x02); //Nor:0x02 DOU:0x03
	gc0339_write_reg(client, MISENSOR_8BIT, 0x79, 0x01);
	gc0339_write_reg(client, MISENSOR_8BIT, 0x7A, 0x05); //data zero 0x7a default=0x0a
	gc0339_write_reg(client, MISENSOR_8BIT, 0x7B, 0x02); //Nor:0x02 DOU:0x03

	return 0;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct gc0339_device *dev = to_gc0339_sensor(sd);
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
		dev_err(&client->dev, "gpio failed 1\n");
	/*
	 * according to DS, 44ms is needed between power up and first i2c
	 * commend
	 */
	msleep(50);

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
	struct gc0339_device *dev = to_gc0339_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	if (NULL == dev->platform_data) {
		dev_err(&client->dev, "no camera_sensor_platform_data");
		return -ENODEV;
	}

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");

	msleep(2);

	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	/*according to DS, 20ms is needed after power down*/
	msleep(20);

	return ret;
}

static int gc0339_s_power(struct v4l2_subdev *sd, int power)
{
	if (power == 0)
		return power_down(sd);
	else {
		if (power_up(sd))
			return -EINVAL;

		return gc0339_init_common(sd);
	}
}

static int gc0339_try_res(u32 *w, u32 *h)
{
	int i;

	/*
	 * The mode list is in ascending order. We're done as soon as
	 * we have found the first equal or bigger size.
	 */
	for (i = 0; i < N_RES; i++) {
		if ((gc0339_res[i].width >= *w) &&
		    (gc0339_res[i].height >= *h))
			break;
	}

	/*
	 * If no mode was found, it means we can provide only a smaller size.
	 * Returning the biggest one available in this case.
	 */
	if (i == N_RES)
		i--;

	*w = gc0339_res[i].width;
	*h = gc0339_res[i].height;

	return 0;
}

static struct gc0339_res_struct *gc0339_to_res(u32 w, u32 h)
{
	int  index;

	for (index = 0; index < N_RES; index++) {
		if ((gc0339_res[index].width == w) &&
		    (gc0339_res[index].height == h))
			break;
	}

	/* No mode found */
	if (index >= N_RES)
		return NULL;

	return &gc0339_res[index];
}

static int gc0339_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	//fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;
	return gc0339_try_res(&fmt->width, &fmt->height);
}

static int gc0339_res2size(unsigned int res, int *h_size, int *v_size)
{
	unsigned short hsize;
	unsigned short vsize;

	switch (res) {
		case GC0339_RES_CIF:
			hsize = GC0339_RES_CIF_SIZE_H;
			vsize = GC0339_RES_CIF_SIZE_V;
			break;
		case GC0339_RES_VGA:
			hsize = GC0339_RES_VGA_SIZE_H;
			vsize = GC0339_RES_VGA_SIZE_V;
			break;
		default:
			WARN(1, "%s: Resolution 0x%08x unknown\n", __func__, res);
			return -EINVAL;
	}

	if (h_size != NULL)
		*h_size = hsize;
	if (v_size != NULL)
		*v_size = vsize;

	return 0;
}

static int gc0339_get_intg_factor(struct i2c_client *client,
				struct camera_mipi_info *info,
				const struct gc0339_res_struct *res)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct atomisp_sensor_mode_data *buf = &info->data;
	const unsigned int ext_clk_freq_hz = 19200000;
	u8 reg_val;
	u16 val;
	int ret;

	if (info == NULL)
		return -EINVAL;

	buf->vt_pix_clk_freq_mhz = ext_clk_freq_hz / 2;
	//printk("vt_pix_clk_freq_mhz = %d\n", buf->vt_pix_clk_freq_mhz);

	/* get integration time */
	buf->coarse_integration_time_min = GC0339_COARSE_INTG_TIME_MIN;
	buf->coarse_integration_time_max_margin =
					GC0339_COARSE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_min = GC0339_FINE_INTG_TIME_MIN;
	buf->fine_integration_time_max_margin =
					GC0339_FINE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_def = GC0339_FINE_INTG_TIME_MIN;

	buf->frame_length_lines = res->lines_per_frame;
	buf->line_length_pck = res->pixels_per_line;
	buf->read_mode = res->bin_mode;

	/* get the cropping and output resolution to ISP for this mode. */
	ret = gc0339_read_reg(client, MISENSOR_8BIT, REG_H_START, &reg_val);
	if(ret)
		return ret;
	buf->crop_horizontal_start = reg_val;
	//printk("crop_horizontal_start = %d\n", buf->crop_horizontal_start);

	ret = gc0339_read_reg(client, MISENSOR_8BIT, REG_V_START, &reg_val);
	if(ret)
		return ret;
	buf->crop_vertical_start = reg_val;
	//printk("crop_vertical_start = %d\n", buf->crop_vertical_start);

	val =0;
	ret = gc0339_read_reg(client, MISENSOR_8BIT, REG_WIDTH_H, &reg_val);
	val = reg_val;
	val = val << 8;
	ret |= gc0339_read_reg(client, MISENSOR_8BIT, REG_WIDTH_L, &reg_val);
	val += reg_val;
	if(ret)
		return ret;
	buf->output_width = val;
	buf->crop_horizontal_end = buf->crop_horizontal_start + val - 1;
	//printk("crop_horizontal_end = %d\n", buf->crop_horizontal_end);

	val =0;
	ret = gc0339_read_reg(client, MISENSOR_8BIT, REG_HEIGHT_H, &reg_val);
	val = reg_val;
	val = val << 8;
	ret |= gc0339_read_reg(client, MISENSOR_8BIT, REG_HEIGHT_L, &reg_val);
	val += reg_val;
	if(ret)
		return ret;
	buf->output_height = val;
	buf->crop_vertical_end = buf->crop_vertical_start + val - 1;
	//printk("crop_vertical_end = %d\n", buf->crop_vertical_end);

	val =0;
	ret = gc0339_read_reg(client, MISENSOR_8BIT, REG_DUMMY_H, &reg_val);
	val = reg_val;
	val = (val&0x0F) << 8;
	ret |= gc0339_read_reg(client, MISENSOR_8BIT, REG_H_DUMMY_L, &reg_val);
	val += reg_val;
	//printk("REG_H_DUMMY = %d\n", val);

	ret |= gc0339_read_reg(client, MISENSOR_8BIT, REG_SH_DELAY, &reg_val);
	if(ret)
		return ret;
	//printk("REG_SH_DELAY = %d\n", reg_val);
	val += reg_val;
	buf->line_length_pck = buf->output_width + val + 4;
	//printk("line_length_pck = %d\n", buf->line_length_pck);

	val =0;
	ret = gc0339_read_reg(client, MISENSOR_8BIT, REG_DUMMY_H, &reg_val);
	val = reg_val;
	val = (val&0xF0) << 4;
	ret |= gc0339_read_reg(client, MISENSOR_8BIT, REG_V_DUMMY_L, &reg_val);
	val += reg_val;
	if(ret)
		return ret;
	//printk("REG_V_DUMMY = %d\n", val);

	buf->frame_length_lines = buf->output_height + val;
	//printk("frame_length_lines = %d\n", buf->frame_length_lines);

	buf->read_mode = res->bin_mode;
	buf->binning_factor_x = 1;
	buf->binning_factor_y = 1;

	return 0;
}

static int gc0339_get_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct gc0339_device *dev = to_gc0339_sensor(sd);
	int width, height;
	int ret;

	fmt->code = V4L2_MBUS_FMT_SRGGB10_1X10;

	ret = gc0339_res2size(dev->res, &width, &height);
	if (ret)
		return ret;
	fmt->width = width;
	fmt->height = height;

	return 0;
}

static int gc0339_set_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	struct gc0339_device *dev = to_gc0339_sensor(sd);
	struct gc0339_res_struct *res_index;
	u32 width = fmt->width;
	u32 height = fmt->height;
	struct camera_mipi_info *gc0339_info = NULL;
	int ret;

	gc0339_info = v4l2_get_subdev_hostdata(sd);
	if (gc0339_info == NULL)
		return -EINVAL;

	gc0339_try_res(&width, &height);
	res_index = gc0339_to_res(width, height);

	/* Sanity check */
	if (unlikely(!res_index)) {
		WARN_ON(1);
		return -EINVAL;
	}

	switch (res_index->res) {
	case GC0339_RES_CIF:
		gc0339_write_reg(c, MISENSOR_8BIT, 0x15, 0x8A); //CIF
		gc0339_write_reg(c, MISENSOR_8BIT, 0x62, 0xCC); //LWC by verrill
		gc0339_write_reg(c, MISENSOR_8BIT, 0x63, 0x01);

		gc0339_write_reg(c, MISENSOR_8BIT, 0x06, 0x00);
		gc0339_write_reg(c, MISENSOR_8BIT, 0x08, 0x00);
		gc0339_write_reg(c, MISENSOR_8BIT, 0x09, 0x01);  // by Wesley
		gc0339_write_reg(c, MISENSOR_8BIT, 0x0A, 0xE2);  // by Wesley
		gc0339_write_reg(c, MISENSOR_8BIT, 0x0B, 0x02);  // by Wesley
		gc0339_write_reg(c, MISENSOR_8BIT, 0x0C, 0x54);  // by Wesley
		printk("gc0339_set_mbus_fmt: CIF\n");
		break;
	case GC0339_RES_VGA:
		gc0339_write_reg(c, MISENSOR_8BIT, 0x15, 0x0A);
		gc0339_write_reg(c, MISENSOR_8BIT, 0x62, 0x34); //by verrill
		gc0339_write_reg(c, MISENSOR_8BIT, 0x63, 0x03); //by verrill

		gc0339_write_reg(c, MISENSOR_8BIT, 0x06, 0x00);
		gc0339_write_reg(c, MISENSOR_8BIT, 0x08, 0x00);
		gc0339_write_reg(c, MISENSOR_8BIT, 0x09, 0x01); //by Wesley
		gc0339_write_reg(c, MISENSOR_8BIT, 0x0A, 0xF2); //by Wesley
		gc0339_write_reg(c, MISENSOR_8BIT, 0x0B, 0x02); //by Wesley
		gc0339_write_reg(c, MISENSOR_8BIT, 0x0C, 0x94); //by Wesley
		printk("gc0339_set_mbus_fmt: VGA\n");
		break;
	default:
		v4l2_err(sd, "set resolution: %d failed!\n", res_index->res);
		return -EINVAL;
	}

	ret = gc0339_get_intg_factor(c, gc0339_info,
					&gc0339_res[res_index->res]);
	if (ret) {
		dev_err(&c->dev, "failed to get integration_factor\n");
		return -EINVAL;
	}

	/*
	 * gc0339 - we don't poll for context switch
	 * because it does not happen with streaming disabled.
	 */
	dev->res = res_index->res;

	fmt->width = width;
	fmt->height = height;
	fmt->code = V4L2_MBUS_FMT_SRGGB10_1X10;

	return 0;
}

/* TODO: Update to SOC functions, remove exposure and gain */
static int gc0339_g_focal(struct v4l2_subdev *sd, s32 * val)
{
	*val = (GC0339_FOCAL_LENGTH_NUM << 16) | GC0339_FOCAL_LENGTH_DEM;
	return 0;
}

static int gc0339_g_fnumber(struct v4l2_subdev *sd, s32 * val)
{
	/*const f number for gc0339*/
	*val = (GC0339_F_NUMBER_DEFAULT_NUM << 16) | GC0339_F_NUMBER_DEM;
	return 0;
}

static int gc0339_g_fnumber_range(struct v4l2_subdev *sd, s32 * val)
{
	*val = (GC0339_F_NUMBER_DEFAULT_NUM << 24) |
		(GC0339_F_NUMBER_DEM << 16) |
		(GC0339_F_NUMBER_DEFAULT_NUM << 8) | GC0339_F_NUMBER_DEM;
	return 0;
}

/* Horizontal flip the image. */
static int gc0339_g_hflip(struct v4l2_subdev *sd, s32 * val)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	int ret;
	u32 data;
	ret = gc0339_read_reg(c, MISENSOR_16BIT,
			(u32)MISENSOR_READ_MODE, &data);
	if (ret)
		return ret;
	*val = !!(data & MISENSOR_HFLIP_MASK);

	return 0;
}

static int gc0339_g_vflip(struct v4l2_subdev *sd, s32 * val)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	int ret;
	u32 data;

	ret = gc0339_read_reg(c, MISENSOR_16BIT,
			(u32)MISENSOR_READ_MODE, &data);
	if (ret)
		return ret;
	*val = !!(data & MISENSOR_VFLIP_MASK);

	return 0;
}

static int gc0339_s_freq(struct v4l2_subdev *sd, s32  val)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	struct gc0339_device *dev = to_gc0339_sensor(sd);
	int ret;

	if (val != GC0339_FLICKER_MODE_50HZ &&
			val != GC0339_FLICKER_MODE_60HZ)
		return -EINVAL;

	if (val == GC0339_FLICKER_MODE_50HZ) {
		ret = gc0339_write_reg_array(c, gc0339_antiflicker_50hz,
					POST_POLLING);
		if (ret < 0)
			return ret;
	} else {
		ret = gc0339_write_reg_array(c, gc0339_antiflicker_60hz,
					POST_POLLING);
		if (ret < 0)
			return ret;
	}

	if (ret == 0)
		dev->lightfreq = val;

	return ret;
}

static int gc0339_g_2a_status(struct v4l2_subdev *sd, s32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	unsigned int status_exp, status_wb;
	*val = 0;

	ret = gc0339_read_reg(client, MISENSOR_16BIT,
			MISENSOR_AE_TRACK_STATUS, &status_exp);
	if (ret)
		return ret;

	ret = gc0339_read_reg(client, MISENSOR_16BIT,
			MISENSOR_AWB_STATUS, &status_wb);
	if (ret)
		return ret;

	if (status_exp & MISENSOR_AE_READY)
		*val = V4L2_2A_STATUS_AE_READY;

	if (status_wb & MISENSOR_AWB_STEADY)
		*val |= V4L2_2A_STATUS_AWB_READY;

	return 0;
}

static long gc0339_s_exposure(struct v4l2_subdev *sd,
			       struct atomisp_exposure *exposure)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned int coarse_integration = 0;
	unsigned int AnalogGain, DigitalGain;
	u8 expo_coarse_h,expo_coarse_l;
	int ret = 0;

	coarse_integration = exposure->integration_time[0];
	AnalogGain = exposure->gain[0];
	DigitalGain = exposure->gain[1];

	expo_coarse_h = (u8)(coarse_integration>>8);
	expo_coarse_l = (u8)(coarse_integration & 0xff);

	ret = gc0339_write_reg(client, MISENSOR_8BIT, REG_EXPO_COARSE, expo_coarse_h);
	ret |= gc0339_write_reg(client, MISENSOR_8BIT, REG_EXPO_COARSE + 1, expo_coarse_l);
	if (ret) {
	    	 v4l2_err(client, "%s: fail to set exposure time\n", __func__);
	    	 return -EINVAL;
	}

	//if (DigitalGain >= 16 || DigitalGain <= 1)
	//	DigitalGain = 1;

	ret = gc0339_write_reg(client, MISENSOR_8BIT, REG_GAIN, AnalogGain);
	if (ret) {
	     	v4l2_err(client, "%s: fail to set AnalogGainToWrite\n", __func__);
	     	return -EINVAL;
	}

	return ret;
}

static long gc0339_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return gc0339_s_exposure(sd, arg);
	default:
		return -EINVAL;
	}

	return 0;
}


/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int gc0339_g_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 coarse;
	u8 reg_val_h,reg_val_l;
	int ret;

	/* the fine integration time is currently not calculated */
	ret = gc0339_read_reg(client, MISENSOR_8BIT,
			       MISENSOR_COARSE_INTEGRATION_TIME_H, &reg_val_h);
	if (ret)
		return ret;

	coarse = ((u16)(reg_val_h & 0x0f)) << 8;

	ret = gc0339_read_reg(client, MISENSOR_8BIT,
			       MISENSOR_COARSE_INTEGRATION_TIME_L, &reg_val_l);
	if (ret)
		return ret;

	coarse |= reg_val_l;

	*value = coarse;

	return 0;
}

static struct gc0339_control gc0339_controls[] = {
/*
	{
		.qc = {
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Image v-Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.query = gc0339_g_vflip,
		.tweak = gc0339_t_vflip,
	},
	{
		.qc = {
			.id = V4L2_CID_HFLIP,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Image h-Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.query = gc0339_g_hflip,
		.tweak = gc0339_t_hflip,
	},
*/
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = GC0339_FOCAL_LENGTH_DEFAULT,
			.maximum = GC0339_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = GC0339_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = gc0339_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = GC0339_F_NUMBER_DEFAULT,
			.maximum = GC0339_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = GC0339_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = gc0339_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = GC0339_F_NUMBER_RANGE,
			.maximum =  GC0339_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = GC0339_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = gc0339_g_fnumber_range,
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
		.tweak = gc0339_s_freq,
	},
	{
		.qc = {
			.id = V4L2_CID_2A_STATUS,
			.type = V4L2_CTRL_TYPE_BITMASK,
			.name = "AE and AWB status",
			.minimum = 0,
			.maximum = V4L2_2A_STATUS_AE_READY |
				V4L2_2A_STATUS_AWB_READY,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = gc0339_g_2a_status,
	},
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
		.query = gc0339_g_exposure,
	},

};
#define N_CONTROLS (ARRAY_SIZE(gc0339_controls))

static struct gc0339_control *gc0339_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++) {
		if (gc0339_controls[i].qc.id == id)
			return &gc0339_controls[i];
	}
	return NULL;
}

static int gc0339_detect(struct gc0339_device *dev, struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	u8 retvalue;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c error", __func__);
		return -ENODEV;
	}

	gc0339_write_reg(client, MISENSOR_8BIT, 0xFE, 0x80); //sw reset
	gc0339_write_reg(client, MISENSOR_8BIT, 0xFC, 0x10); //clock enable
	gc0339_write_reg(client, MISENSOR_8BIT, 0xF6, 0x05); //pll mode
	gc0339_write_reg(client, MISENSOR_8BIT, 0xF7, 0x01);
	gc0339_write_reg(client, MISENSOR_8BIT, 0xF7, 0x03);
	gc0339_write_reg(client, MISENSOR_8BIT, 0xFC, 0x16);

	gc0339_read_reg(client, MISENSOR_8BIT, (u8)GC0339_PID, &retvalue);
	dev->real_model_id = retvalue;
	printk("detect module ID = %x\n",retvalue);

	if (retvalue != GC0339_MOD_ID) {
                dev_err(&client->dev, "%s: failed: client->addr = %x\n",
                                __func__, client->addr);
		return -ENODEV;
	}

	return 0;
}

static int
gc0339_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct gc0339_device *dev = to_gc0339_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	if (NULL == platform_data)
		return -ENODEV;

	dev->platform_data =
	    (struct camera_sensor_platform_data *)platform_data;

	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			v4l2_err(client, "gc0339 platform init err\n");
			return ret;
		}
	}
	ret = power_up(sd);
	if (ret) {
		v4l2_err(client, "gc0339 power-up err");
		return ret;
	}

	/* config & detect sensor */
	ret = gc0339_detect(dev, client);
	if (ret) {
		v4l2_err(client, "gc0339_detect err s_config.\n");
		goto fail_detect;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	ret = gc0339_set_suspend(sd);
	if (ret) {
		v4l2_err(client, "gc0339 suspend err");
		return ret;
	}

	ret = power_down(sd);
	if (ret) {
		v4l2_err(client, "gc0339 power down err");
		return ret;
	}

	return 0;

fail_csi_cfg:
	dev->platform_data->csi_cfg(sd, 0);
fail_detect:
	power_down(sd);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int gc0339_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct gc0339_control *ctrl = gc0339_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}


/* Horizontal flip the image. */
static int gc0339_t_hflip(struct v4l2_subdev *sd, int value)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	struct gc0339_device *dev = to_gc0339_sensor(sd);
	int err;

	/* set for direct mode */
	err = gc0339_write_reg(c, MISENSOR_16BIT, 0x098E, 0xC850);
	if (value) {
		/* enable H flip ctx A */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC850, 0x01, 0x01);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC851, 0x01, 0x01);
		/* ctx B */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC888, 0x01, 0x01);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC889, 0x01, 0x01);

		err += misensor_rmw_reg(c, MISENSOR_16BIT, MISENSOR_READ_MODE,
					MISENSOR_HFLIP_MASK, MISENSOR_FLIP_EN);

		dev->bpat = GC0339_BPAT_GRGRBGBG;
	} else {
		/* disable H flip ctx A */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC850, 0x01, 0x00);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC851, 0x01, 0x00);
		/* ctx B */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC888, 0x01, 0x00);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC889, 0x01, 0x00);

		err += misensor_rmw_reg(c, MISENSOR_16BIT, MISENSOR_READ_MODE,
					MISENSOR_HFLIP_MASK, MISENSOR_FLIP_DIS);

		dev->bpat = GC0339_BPAT_BGBGGRGR;
	}

	err += gc0339_write_reg(c, MISENSOR_8BIT, 0x8404, 0x06);
	udelay(10);

	return !!err;
}

/* Vertically flip the image */
static int gc0339_t_vflip(struct v4l2_subdev *sd, int value)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	int err;

	/* set for direct mode */
	err = gc0339_write_reg(c, MISENSOR_16BIT, 0x098E, 0xC850);
	if (value >= 1) {
		/* enable H flip - ctx A */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC850, 0x02, 0x01);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC851, 0x02, 0x01);
		/* ctx B */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC888, 0x02, 0x01);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC889, 0x02, 0x01);

		err += misensor_rmw_reg(c, MISENSOR_16BIT, MISENSOR_READ_MODE,
					MISENSOR_VFLIP_MASK, MISENSOR_FLIP_EN);
	} else {
		/* disable H flip - ctx A */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC850, 0x02, 0x00);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC851, 0x02, 0x00);
		/* ctx B */
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC888, 0x02, 0x00);
		err += misensor_rmw_reg(c, MISENSOR_8BIT, 0xC889, 0x02, 0x00);

		err += misensor_rmw_reg(c, MISENSOR_16BIT, MISENSOR_READ_MODE,
					MISENSOR_VFLIP_MASK, MISENSOR_FLIP_DIS);
	}

	err += gc0339_write_reg(c, MISENSOR_8BIT, 0x8404, 0x06);
	udelay(10);

	return !!err;
}

static int gc0339_s_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *param)
{
	return 0;
}

static int gc0339_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct gc0339_control *octrl = gc0339_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;

	ret = octrl->query(sd, &ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int gc0339_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct gc0339_control *octrl = gc0339_find_control(ctrl->id);
	int ret;

	if (!octrl || !octrl->tweak)
		return -EINVAL;

	ret = octrl->tweak(sd, ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int gc0339_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;

	if (enable) {
		ret = gc0339_set_streaming(sd);
	} else {
		ret = gc0339_set_suspend(sd);
	}

	return ret;
}

static int
gc0339_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;

	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = gc0339_res[index].width;
	fsize->discrete.height = gc0339_res[index].height;

	/* FIXME: Wrong way to know used mode */
	fsize->reserved[0] = gc0339_res[index].used;

	return 0;
}

static int gc0339_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	int i;

	if (index >= N_RES)
		return -EINVAL;

	/* find out the first equal or bigger size */
	for (i = 0; i < N_RES; i++) {
		if ((gc0339_res[i].width >= fival->width) &&
		    (gc0339_res[i].height >= fival->height))
			break;
	}
	if (i == N_RES)
		i--;

	index = i;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = gc0339_res[index].fps;

	return 0;
}

static int
gc0339_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_GC0339, 0);
}

static int gc0339_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_SRGGB10_1X10;

	return 0;
}

static int gc0339_enum_frame_size(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_frame_size_enum *fse)
{
	unsigned int index = fse->index;


	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = gc0339_res[index].width;
	fse->min_height = gc0339_res[index].height;
	fse->max_width = gc0339_res[index].width;
	fse->max_height = gc0339_res[index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__gc0339_get_pad_format(struct gc0339_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);

	if (pad != 0) {
		dev_err(&client->dev,  "%s err. pad %x\n", __func__, pad);
		return NULL;
	}

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->format;
	default:
		return NULL;
	}
}

static int
gc0339_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct gc0339_device *snr = to_gc0339_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__gc0339_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;
	fmt->format = *format;

	return 0;
}

static int
gc0339_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct gc0339_device *snr = to_gc0339_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__gc0339_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;

	return 0;
}

static int gc0339_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	int index;
	struct gc0339_device *snr = to_gc0339_sensor(sd);

	if (frames == NULL)
		return -EINVAL;

	for (index = 0; index < N_RES; index++) {
		if (gc0339_res[index].res == snr->res)
			break;
	}

	if (index >= N_RES)
		return -EINVAL;

	*frames = gc0339_res[index].skip_frames;

	return 0;
}

static const struct v4l2_subdev_video_ops gc0339_video_ops = {
	.try_mbus_fmt = gc0339_try_mbus_fmt,
	.s_mbus_fmt = gc0339_set_mbus_fmt,
	.g_mbus_fmt = gc0339_get_mbus_fmt,
	.s_stream = gc0339_s_stream,
	.enum_framesizes = gc0339_enum_framesizes,
	.enum_frameintervals = gc0339_enum_frameintervals,
	.s_parm = gc0339_s_parm,
};

static struct v4l2_subdev_sensor_ops gc0339_sensor_ops = {
	.g_skip_frames	= gc0339_g_skip_frames,
};

static const struct v4l2_subdev_core_ops gc0339_core_ops = {
	.g_chip_ident = gc0339_g_chip_ident,
	.queryctrl = gc0339_queryctrl,
	.g_ctrl = gc0339_g_ctrl,
	.s_ctrl = gc0339_s_ctrl,
	.s_power = gc0339_s_power,
	.ioctl = gc0339_ioctl,
};

/* REVISIT: Do we need pad operations? */
static const struct v4l2_subdev_pad_ops gc0339_pad_ops = {
	.enum_mbus_code = gc0339_enum_mbus_code,
	.enum_frame_size = gc0339_enum_frame_size,
	.get_fmt = gc0339_get_pad_format,
	.set_fmt = gc0339_set_pad_format,
};

static const struct v4l2_subdev_ops gc0339_ops = {
	.core = &gc0339_core_ops,
	.video = &gc0339_video_ops,
	.pad = &gc0339_pad_ops,
	.sensor = &gc0339_sensor_ops,
};

static const struct media_entity_operations gc0339_entity_ops = {
	.link_setup = NULL,
};

static int gc0339_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc0339_device *dev = to_gc0339_sensor(sd);

	dev->platform_data->csi_cfg(sd, 0);
	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&dev->sd.entity);
	kfree(dev);
	return 0;
}

static int gc0339_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct gc0339_device *dev;
	int ret;

	/* Setup sensor configuration structure */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	v4l2_i2c_subdev_init(&dev->sd, client, &gc0339_ops);

	if (client->dev.platform_data) {
		ret = gc0339_s_config(&dev->sd, client->irq,
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
	dev->format.code = V4L2_MBUS_FMT_SRGGB10_1X10;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	/* REVISIT: Do we need media controller? */
	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		gc0339_remove(client);
		return ret;
	}

	/* set res index to be invalid */
	dev->res = -1;

	return 0;
}

MODULE_DEVICE_TABLE(i2c, gc0339_id);

static struct i2c_driver gc0339_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "gc0339"
	},
	.probe = gc0339_probe,
	.remove = gc0339_remove,
	.id_table = gc0339_id,
};

static __init int init_gc0339(void)
{
	return i2c_add_driver(&gc0339_driver);
}

static __exit void exit_gc0339(void)
{
	i2c_del_driver(&gc0339_driver);
}

module_init(init_gc0339);
module_exit(exit_gc0339);

MODULE_AUTHOR("Shuguang Gong <Shuguang.gong@intel.com>");
MODULE_LICENSE("GPL");
