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

#ifndef __OV2675_H__
#define __OV2675_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/spinlock.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>

#define OV2675_NAME	"ov2675"

#define V4L2_IDENT_OV2675 1111
#define	LAST_REG_SETING	{0xffff, 0xff}

#define OV2675_FOCAL_LENGTH_NUM	439	/*4.39mm*/
#define OV2675_FOCAL_LENGTH_DEM	100
#define OV2675_F_NUMBER_DEFAULT_NUM	24
#define OV2675_F_NUMBER_DEM	10


#define OV2675_XVCLK		1920
#define OV2675_AE_TARGET	45
#define OV2675_DEFAULT_GAIN	50
#define OV2675_DEFAULT_SHUTTER	1000

/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define OV2675_FOCAL_LENGTH_DEFAULT 0xD00064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define OV2675_F_NUMBER_DEFAULT 0x18000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define OV2675_F_NUMBER_RANGE 0x180a180a

#define OV2675_FLICKER_MODE_50HZ   	1
#define OV2675_FLICKER_MODE_60HZ	       2


/* #defines for register writes and register array processing */
#define MISENSOR_8BIT		1
#define MISENSOR_16BIT		2
#define MISENSOR_32BIT		4

#define MISENSOR_TOK_TERM	0xf000	/* terminating token for reg list */
#define MISENSOR_TOK_DELAY	0xfe00	/* delay token for reg list */
#define MISENSOR_TOK_FWLOAD	0xfd00	/* token indicating load FW */
#define MISENSOR_TOK_POLL	0xfc00	/* token indicating poll instruction */

#define I2C_RETRY_COUNT		5
#define MSG_LEN_OFFSET		2

#define OV2675_REG_PID		0x300a
#define OV2675_REG_SYS_RESET	0x3000
#define OV2675_REG_FW_START	0x8000
#define OV2675_REG_FOCUS_MODE	0x3022	/* focus mode reg */
#define OV2675_REG_FOCUS_ZONE_X	0x3024	/* X coordinate of focus zone center */
#define OV2675_REG_FOCUS_ZONE_Y	0x3025	/* Y coordinate of focus zone center */
#define OV2675_REG_FOCUS_STATUS	0x3029	/* focus status reg */

/* system pll control reg */
#define OV2675_REG_PLL_CTRL_0	0x3034
#define OV2675_REG_PLL_CTRL_1	0x300e
#define OV2675_REG_PLL_CTRL_2	0x300f
#define OV2675_REG_PLL_CTRL_3	0x3010


/* pad clock divider for SCCB clock */
#define OV2675_REG_CLK_DIVIDER	0x3011

/* total horizontal size reg */
#define OV2675_REG_TIMING_HTS	0x3028
#define OV2675_REG_TIMING_EXTRA_HTS	0x302c

/* total vertical size reg */
#define OV2675_REG_TIMING_VTS	0x302a
#define OV2675_REG_TIMING_EXTRA_VTS	0x302d

/* exposure output reg */
#define OV2675_REG_EXPOSURE_0	0x3002
#define OV2675_REG_EXPOSURE_1	0x302d

/* gain reg */
#define OV2675_REG_GAIN	0x3000

/* light frequency control reg */
#define OV2675_REG_LIGHT_CTRL_0	0x3014
#define OV2675_REG_LIGHT_CTRL_2	0x508e

/* light frequency */
#define OV2675_LIGHT_50HZ	50
#define OV2675_LIGHT_60HZ	60

/* automatic banding filter */
#define OV2675_AUTO_BAND	0x40

/* 60HZ band step reg and 60HZ max bands */
#define OV2675_REG_B60_STEP_HIGH	0x3073
#define OV2675_REG_B60_STEP_LOW		0x3072

#define OV2675_REG_B60_MAX	0x301d

/* 50HZ band step reg and 50HZ max bands */
#define OV2675_REG_B50_STEP_HIGH	0x3071
#define OV2675_REG_B50_STEP_LOW 	0x3070

#define OV2675_REG_B50_MAX	0x301c

/* AEC domain control reg */
#define OV2675_REG_AE_STAB_IN_H	0x3a0f	/* stable in high */
#define OV2675_REG_AE_STAB_IN_L	0x3a10	/* stable in low */
#define OV2675_REG_AE_STAB_OUT_H	0x3a1b	/* stable out high */
#define OV2675_REG_AE_STAB_OUT_L	0x3a1e	/* stable out low */
#define OV2675_REG_AE_FAST_H	0x3a11	/* fast zone high */
#define OV2675_REG_AE_FAST_L	0x3a1f	/* fast zone low */

/* AEC mode control reg */
#define OV2675_REG_AE_MODE_CTRL	0x3013

#define OV2675_AUTO_AG_AE	0x00	/* auto AG&AE */
#define OV2675_MANUAL_AG_AE	0x03	/* manual AG&AE */

/* AEC system control reg */
#define OV2675_REG_AE_SYS_CTRL	0x3a00

/* image exposure average readout reg */
#define OV2675_REG_AE_AVERAGE	0x56a1

#define MIN_SYSCLK		10
#define MIN_VTS			8
#define MIN_HTS			8
#define MIN_SHUTTER		0
#define MIN_GAIN		0

/* OV2675_DEVICE_ID */
#define OV2675_MOD_ID		0x2656

/* Supported resolutions */
enum {
	OV2675_RES_VGA,
	OV2675_RES_480P,
	OV2675_RES_720P,
	OV2675_RES_1M,
	OV2675_RES_2M,
};
#define OV2675_RES_5M_SIZE_H		2560
#define OV2675_RES_5M_SIZE_V		1920
#define OV2675_RES_D5M_SIZE_H		2496
#define OV2675_RES_D5M_SIZE_V		1664
#define OV2675_RES_1M_SIZE_H		1024
#define OV2675_RES_1M_SIZE_V		768
#define OV2675_RES_2M_SIZE_H		1600
#define OV2675_RES_2M_SIZE_V		1200
#define OV2675_RES_1088P_SIZE_H		1920
#define OV2675_RES_1088P_SIZE_V		1088
#define OV2675_RES_1080P_SIZE_H		1920
#define OV2675_RES_1080P_SIZE_V		1080
#define OV2675_RES_720P_SIZE_H		1280
#define OV2675_RES_720P_SIZE_V		720
#define OV2675_RES_480P_SIZE_H		720
#define OV2675_RES_480P_SIZE_V		480
#define OV2675_RES_VGA_SIZE_H		640
#define OV2675_RES_VGA_SIZE_V		480
#define OV2675_RES_SVGA_SIZE_H	800
#define OV2675_RES_SVGA_SIZE_V	600
#define OV2675_RES_360P_SIZE_H		640
#define OV2675_RES_360P_SIZE_V		360
#define OV2675_RES_320P_SIZE_H		480
#define OV2675_RES_320P_SIZE_V		320
#define OV2675_RES_DVGA_SIZE_H		416
#define OV2675_RES_DVGA_SIZE_V		312
#define OV2675_RES_QVGA_SIZE_H		320
#define OV2675_RES_QVGA_SIZE_V		240

/*
 * struct misensor_reg - MI sensor  register format
 * @length: length of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 * Define a structure for sensor register initialization values
 */
struct misensor_reg {
	u16 length;
	u16 reg;
	u32 val;	/* value or for read/mod/write */
};

struct regval_list {
	u16 reg_num;
	u8 value;
};

struct ov2675_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	struct firmware *firmware;

	struct camera_sensor_platform_data *platform_data;
	int run_mode;
	int focus_mode;
	int night_mode;
	bool focus_mode_change;
	int color_effect;
	bool streaming;
	bool preview_ag_ae;
	u16 sensor_id;
	u8 sensor_revision;
	unsigned int ae_high;
	unsigned int ae_low;
	unsigned int preview_shutter;
	unsigned int preview_gain16;
	unsigned int average;
	unsigned int preview_sysclk;
	unsigned int preview_hts;
	unsigned int preview_vts;
	unsigned int res;
};

struct ov2675_priv_data {
	u32 port;
	u32 num_of_lane;
	u32 input_format;
	u32 raw_bayer_order;
};

struct ov2675_format_struct {
	u8 *desc;
	u32 pixelformat;
	struct regval_list *regs;
};

struct ov2675_res_struct {
	u8 *desc;
	int res;
	int width;
	int height;
	int fps;
	int skip_frames;
	bool used;
	struct regval_list *regs;
};

#define OV2675_MAX_WRITE_BUF_SIZE	32
struct ov2675_write_buffer {
	u16 addr;
	u8 data[OV2675_MAX_WRITE_BUF_SIZE];
};

struct ov2675_write_ctrl {
	int index;
	struct ov2675_write_buffer buffer;
};

struct ov2675_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, int value);
};

#define N_ov2675_FMTS ARRAY_SIZE(ov2675_formats)

/*
 * Modes supported by the mt9m114 driver.
 * Please, keep them in ascending order.
 */
static struct ov2675_res_struct ov2675_res[] = {
	
	{
	.desc	= "VGA",
	.res	= OV2675_RES_VGA,
	.width	= 640,
	.height	= 480,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 12,
	},
	{
	.desc	= "480P",
	.res	= OV2675_RES_480P,
	.width	= 720,
	.height	= 480,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 5,
	},
	{
	.desc	= "720p",
	.res	= OV2675_RES_720P,
	.width	= 1280,
	.height	= 720,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 6,
	},
	{
	.desc	= "1M",
	.res	= OV2675_RES_1M,
	.width	= 1024,
	.height	= 768,
	.fps	= 15,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 3,
	},
	{
	.desc	= "2M",
	.res	= OV2675_RES_2M,
	.width	= 1600,
	.height	= 1200,
	.fps	= 15,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 4,
	},
};
#define N_RES (ARRAY_SIZE(ov2675_res))

static const struct i2c_device_id ov2675_id[] = {
	{"ov2675", 0},
	{}
};

static struct misensor_reg const ov2675_standby_reg[] = {

	 {MISENSOR_8BIT,  0x30ad, 0x0a},	/* software powerdown */
	 {MISENSOR_8BIT,  0x3086, 0x0f},	/* software powerdown */
	 {MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const ov2675_wakeup_reg[] = {
	{MISENSOR_8BIT,  0x30ad, 0x00},
	{MISENSOR_8BIT,  0x3086, 0x00},
	{MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const ov2675_normal_effect[] = {
	{MISENSOR_8BIT, 0x308c, 0x88},	/* start group 3 */
	{MISENSOR_8BIT, 0x3391, 0x06},
	{MISENSOR_8BIT, 0x3390, 0x41},	/* sat U */
	{MISENSOR_8BIT, 0x30ff, 0xff},	/* end group 3 */
	{MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const ov2675_sepia_effect[] = {
	{MISENSOR_8BIT, 0x308c, 0x88},	/* start group 3 */
	{MISENSOR_8BIT, 0x3391, 0x1e},
	{MISENSOR_8BIT, 0x3396, 0x40},	/* sat U */
	{MISENSOR_8BIT, 0x3397, 0xa6},	
	{MISENSOR_8BIT, 0x30ff, 0xff},	/* end group 3 */
	{MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const ov2675_negative_effect[] = {
	{MISENSOR_8BIT, 0x308c, 0x88},	/* start group 3 */
	{MISENSOR_8BIT, 0x3391, 0x44},
	{MISENSOR_8BIT, 0x30ff, 0xff},	/* end group 3 */
	{MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const ov2675_bw_effect[] = {
	{MISENSOR_8BIT, 0x308c, 0x88},	/* start group 3 */
	{MISENSOR_8BIT, 0x3391, 0x24},
	{MISENSOR_8BIT, 0x30ff, 0xff},	/* end group 3 */
	{MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const ov2675_blue_effect[] = {
	{MISENSOR_8BIT, 0x308c, 0x88},	/* start group 3 */
	{MISENSOR_8BIT, 0x3391, 0x1e},
	{MISENSOR_8BIT, 0x3396, 0xa0},	/* sat U */
	{MISENSOR_8BIT, 0x3397, 0x40},	
	{MISENSOR_8BIT, 0x30ff, 0xff},	/* end group 3 */
	{MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const ov2675_green_effect[] = {
	{MISENSOR_8BIT, 0x308c, 0x88},	/* start group 3 */
	{MISENSOR_8BIT, 0x3391, 0x1e},
	{MISENSOR_8BIT, 0x3396, 0x60},	/* sat U */
	{MISENSOR_8BIT, 0x3397, 0x60},	
	{MISENSOR_8BIT, 0x30ff, 0xff},	/* end group 3 */
	{MISENSOR_TOK_TERM, 0, 0}
};


/* 2M, yuv422, 2lanes, mipi, 15fps */
static struct misensor_reg const ov2675_2M_init[] = {
{MISENSOR_8BIT, 0x3012, 0x00},  //; UXGA mode

{MISENSOR_8BIT, 0x302a, 0x05}, 	// VTS = 1236
{MISENSOR_8BIT, 0x302b, 0x34}, 	// VTS a14
{MISENSOR_8BIT, 0x306f, 0x54},  //; BLC target for short exposure of HDR mode
{MISENSOR_8BIT, 0x3070, 0x5d}, 	// B50
{MISENSOR_8BIT, 0x3072, 0x4d}, 	// B60
{MISENSOR_8BIT, 0x3020, 0x01}, 	// HS = 280
{MISENSOR_8BIT, 0x3021, 0x18}, 	// HS
{MISENSOR_8BIT, 0x3022, 0x00}, 	// VS = 10
{MISENSOR_8BIT, 0x3023, 0x0a}, 	// VS
{MISENSOR_8BIT, 0x3024, 0x06}, 	// HW = 1624
{MISENSOR_8BIT, 0x3025, 0x58}, 	// HW
{MISENSOR_8BIT, 0x3026, 0x04}, 	// VH = 1212
{MISENSOR_8BIT, 0x3027, 0xbc}, 	// VH
{MISENSOR_8BIT, 0x3088, 0x06}, 	// ISP XOUT = 1600
{MISENSOR_8BIT, 0x3089, 0x40}, 	// ISP XOUT
{MISENSOR_8BIT, 0x308a, 0x04}, 	// ISP YOUT = 1200
{MISENSOR_8BIT, 0x308b, 0xb0}, 	// ISP YOUT
{MISENSOR_8BIT, 0x3640, 0x09},  
{MISENSOR_8BIT, 0x3313, 0x00},  
{MISENSOR_8BIT, 0x3314, 0x00},  
{MISENSOR_8BIT, 0x3315, 0x00},  
{MISENSOR_8BIT, 0x3316, 0x64}, 	// Scale X input = 1600
{MISENSOR_8BIT, 0x3317, 0x4b}, 	// Scale Y input = 1200
{MISENSOR_8BIT, 0x3318, 0x00}, 	// Scale Y/X input
{MISENSOR_8BIT, 0x3319, 0x2c}, 	// Scale X offset = 44
{MISENSOR_8BIT, 0x331a, 0x64}, 	// Scale X output = 1600
{MISENSOR_8BIT, 0x331b, 0x4b}, 	// Scale Y output = 1200
{MISENSOR_8BIT, 0x331c, 0x00}, 	// Scale Y/X output
{MISENSOR_8BIT, 0x331d, 0x4c},  // Scale Y offset = 4, Scale X offset = 12
{MISENSOR_8BIT, 0x3302, 0x01}, 	// Scale off, UV Average off
{MISENSOR_8BIT, 0x300e, 0x31},  // PLL
{MISENSOR_8BIT, 0x300f, 0xa6},	// PLL
{MISENSOR_8BIT, 0x3010, 0x80},	// PLL
{MISENSOR_8BIT, 0x3011, 0x00},	// clock divider
{MISENSOR_8BIT, 0x302d, 0x00},	// EXVTS = 0
{MISENSOR_8BIT, 0x302e, 0x00},	// EXVTS
{MISENSOR_8BIT, 0x302c, 0x00},	// EXHTS = 0
{MISENSOR_8BIT, 0x3070, 0xbb},	// B50
{MISENSOR_8BIT, 0x3072, 0x9b},	// B60
{MISENSOR_8BIT, 0x301c, 0x06},	// max step 50
{MISENSOR_8BIT, 0x301d, 0x07},	// max step 60
//michael cong add for effect
{MISENSOR_8BIT, 0x3362, 0x80},

{MISENSOR_TOK_TERM, 0, 0}
};

/* 1M, yuv422, 2lanes, mipi, 15fps */
static struct misensor_reg const ov2675_1M_init[] = {
{MISENSOR_8BIT, 0x3012, 0x00}, 	// UXGA mode
{MISENSOR_8BIT, 0x302a, 0x04},	// VTS = 1236
{MISENSOR_8BIT, 0x302b, 0xd4},	// VTS
{MISENSOR_8BIT, 0x306f, 0x54},  //; BLC target for short exposure of HDR mode
{MISENSOR_8BIT, 0x3070, 0x5d},	// B50
{MISENSOR_8BIT, 0x3072, 0x4d},	// B60
{MISENSOR_8BIT, 0x3020, 0x01},	// HS = 280
{MISENSOR_8BIT, 0x3021, 0x18},	// HS
{MISENSOR_8BIT, 0x3022, 0x00},	// VS = 10
{MISENSOR_8BIT, 0x3023, 0x0a},	// VS
{MISENSOR_8BIT, 0x3024, 0x06},	// HW =1624
{MISENSOR_8BIT, 0x3025, 0x58},	// HW
{MISENSOR_8BIT, 0x3026, 0x04},	// VH = 1212
{MISENSOR_8BIT, 0x3027, 0xbc},	// VH
{MISENSOR_8BIT, 0x3088, 0x04},  // ISP XOUT = 1280
{MISENSOR_8BIT, 0x3089, 0x00},	// ISP XOUT
{MISENSOR_8BIT, 0x308a, 0x03},	// ISP YOUT = 960
{MISENSOR_8BIT, 0x308b, 0x00},  // ISP YOUT
{MISENSOR_8BIT, 0x3640, 0x09},  
{MISENSOR_8BIT, 0x3313, 0x00},  
{MISENSOR_8BIT, 0x3314, 0x00},  
{MISENSOR_8BIT, 0x3315, 0x00},  
{MISENSOR_8BIT, 0x3316, 0x64},	// Scale X input = 1600
{MISENSOR_8BIT, 0x3317, 0x4b},	// Scale Y input = 1200
{MISENSOR_8BIT, 0x3318, 0x00},	// Scale Y/X input
{MISENSOR_8BIT, 0x3319, 0x2c},	// Scale X offset = 44
{MISENSOR_8BIT, 0x331a, 0x64},	// Scale X output = 1600
{MISENSOR_8BIT, 0x331b, 0x4b},	// Scale Y output = 1200
{MISENSOR_8BIT, 0x331c, 0x00},	// Scale Y/X output
{MISENSOR_8BIT, 0x331d, 0x4c},  // Scale Y offset = 4, Scale X offset = 12
{MISENSOR_8BIT, 0x3302, 0x01},	// Scale off, UV average off
{MISENSOR_8BIT, 0x300e, 0x31},  // PLL
{MISENSOR_8BIT, 0x300f, 0xa6},	// PLL
{MISENSOR_8BIT, 0x3010, 0x80},	// PLL
{MISENSOR_8BIT, 0x3011, 0x00},	// clock divider
{MISENSOR_8BIT, 0x302d, 0x00},	// EXVTS = 0
{MISENSOR_8BIT, 0x302e, 0x00},	// EXVTS
{MISENSOR_8BIT, 0x302c, 0x00},	// EXHTS = 0
{MISENSOR_8BIT, 0x3070, 0x5c},	// B50
{MISENSOR_8BIT, 0x3072, 0x4d},	// B60
{MISENSOR_8BIT, 0x301c, 0x0c},	// max step 50
{MISENSOR_8BIT, 0x301d, 0x0f},	// max step 60
//michael cong add for effect
{MISENSOR_8BIT, 0x3362, 0x80},

{MISENSOR_TOK_TERM, 0, 0}
};

/* camera: 720p, yuv422, 2lanes, mipi, 30fps */
static struct misensor_reg const ov2675_720p_init[] = {
{MISENSOR_8BIT, 0x3012, 0x00},	// UXGA mode
{MISENSOR_8BIT, 0x302a, 0x05},	// VTS = 1236
{MISENSOR_8BIT, 0x302b, 0x34},	// VTS
{MISENSOR_8BIT, 0x306f, 0x54},  //; BLC target for short exposure of HDR mode
{MISENSOR_8BIT, 0x3070, 0x5d},	// B50
{MISENSOR_8BIT, 0x3072, 0x4d},	// B60
{MISENSOR_8BIT, 0x3020, 0x01},	// HS = 280
{MISENSOR_8BIT, 0x3021, 0x18},	// HS
{MISENSOR_8BIT, 0x3022, 0x00},	// VS = 10
{MISENSOR_8BIT, 0x3023, 0x0a},	// VS
{MISENSOR_8BIT, 0x3024, 0x06},	// HW = 1624
{MISENSOR_8BIT, 0x3025, 0x58},	// HW
{MISENSOR_8BIT, 0x3026, 0x04},	// VH = 1212
{MISENSOR_8BIT, 0x3027, 0xbc},	// VH
{MISENSOR_8BIT, 0x3088, 0x05},	// ISP XOUT = 1280
{MISENSOR_8BIT, 0x3089, 0x00},	// ISP XOUT
{MISENSOR_8BIT, 0x308a, 0x02},	// ISP YOUT = 720
{MISENSOR_8BIT, 0x308b, 0xd0},	// ISP YOUT
{MISENSOR_8BIT, 0x3313, 0x00},  
{MISENSOR_8BIT, 0x3314, 0x08},  
{MISENSOR_8BIT, 0x3315, 0x00},  
{MISENSOR_8BIT, 0x3316, 0x64},	// Scale X input = 1600
{MISENSOR_8BIT, 0x3317, 0x38},	// Scale Y input = 901
{MISENSOR_8BIT, 0x3318, 0x50},	// Scale Y/X input
{MISENSOR_8BIT, 0x3319, 0x2c},	// Scale X offset = 44
{MISENSOR_8BIT, 0x331a, 0x64},	// Scale X output = 1600
{MISENSOR_8BIT, 0x331b, 0x4b},	// Scale Y output = 1200
{MISENSOR_8BIT, 0x331c, 0x00},	// Scale Y/X output
{MISENSOR_8BIT, 0x331d, 0x4c},  // Scale Y offset = 4, Scale X offset = 12
{MISENSOR_8BIT, 0x3302, 0x11},	// Scale off, UV Average off
{MISENSOR_8BIT, 0x300e, 0x31},  // PLL
{MISENSOR_8BIT, 0x300f, 0xa6},	// PLL
{MISENSOR_8BIT, 0x3010, 0x80},	// PLL
{MISENSOR_8BIT, 0x3011, 0x00},	// clock divider
{MISENSOR_8BIT, 0x302d, 0x00},	// EXVTS = 0
{MISENSOR_8BIT, 0x302e, 0x00},	// EXTS
{MISENSOR_8BIT, 0x302c, 0x00},	// EXHTS = 0
{MISENSOR_8BIT, 0x3014, 0x8c},	// B60
{MISENSOR_8BIT, 0x3070, 0xbb},	// B50
{MISENSOR_8BIT, 0x3072, 0x9b},	// B60
{MISENSOR_8BIT, 0x301c, 0x06},	// max step 50
{MISENSOR_8BIT, 0x301d, 0x07},	// max step 60
//michael cong add for effect
{MISENSOR_8BIT, 0x3362, 0x80},

{MISENSOR_TOK_TERM, 0, 0}
};

/* camera: 480p, yuv422, 2lanes, mipi, 30fps */
static struct misensor_reg const ov2675_480p_init[] = {
{MISENSOR_8BIT, 0x3012, 0x10},	// SVGA Average mode
{MISENSOR_8BIT, 0x302a, 0x02},	// VTS = 618
{MISENSOR_8BIT, 0x302b, 0x6a},	// VTS
{MISENSOR_8BIT, 0x306f, 0x14},	// BLC target for short exposure of HDR mode
{MISENSOR_8BIT, 0x3020, 0x01},	// HS = 280
{MISENSOR_8BIT, 0x3021, 0x18},	// HS
{MISENSOR_8BIT, 0x3022, 0x00},	// VS = 6
{MISENSOR_8BIT, 0x3023, 0x06},	// VS
{MISENSOR_8BIT, 0x3024, 0x06},	// HW = 1624
{MISENSOR_8BIT, 0x3025, 0x58},	// HW
{MISENSOR_8BIT, 0x3026, 0x02},	// VH = 609
{MISENSOR_8BIT, 0x3027, 0x61},	// VH
{MISENSOR_8BIT, 0x3088, 0x03},	// ISP XOUT = 800
{MISENSOR_8BIT, 0x3089, 0x20},	// ISP XOUT
{MISENSOR_8BIT, 0x308a, 0x02},	// ISP YOUT = 600
{MISENSOR_8BIT, 0x308b, 0x58},	// ISP YOUT
{MISENSOR_8BIT, 0x3316, 0x64},	// Scale X input = 1600
{MISENSOR_8BIT, 0x3317, 0x1e},	// Scale Y input = 480
{MISENSOR_8BIT, 0x3318, 0x00},	// Scale Y/X input
{MISENSOR_8BIT, 0x3319, 0x08},	// Scale X offset = 8
{MISENSOR_8BIT, 0x331a, 0x64},	// Scale X output = 1600
{MISENSOR_8BIT, 0x331b, 0x4b},	// Scale Y output = 1200
{MISENSOR_8BIT, 0x331c, 0x00},	// Scale Y/X output
{MISENSOR_8BIT, 0x331d, 0x38},	// Scale Y offset = 3, Scale X offset = 8
{MISENSOR_8BIT, 0x3302, 0x11},	// Scale on, UV Average on
{MISENSOR_8BIT, 0x3088, 0x02}, 	// ISP X output = 800
{MISENSOR_8BIT, 0x3089, 0xd0}, 	// ISP X output
{MISENSOR_8BIT, 0x308a, 0x01},	// ISP Y output = 480
{MISENSOR_8BIT, 0x308b, 0xe0},	// ISP Y output
{MISENSOR_8BIT, 0x3313, 0x00},  
{MISENSOR_8BIT, 0x3314, 0x00},  
{MISENSOR_8BIT, 0x3315, 0x00},  
{MISENSOR_8BIT, 0x331a, 0x32},                                                  
{MISENSOR_8BIT, 0x331b, 0x1e},                                                  
{MISENSOR_8BIT, 0x331c, 0x00},  
{MISENSOR_8BIT, 0x300e, 0x31},  	//PLL
{MISENSOR_8BIT, 0x300f, 0xa6},	// PLL
{MISENSOR_8BIT, 0x3010, 0x81},	// PLL
{MISENSOR_8BIT, 0x3011, 0x00},	// clock divider
{MISENSOR_8BIT, 0x302d, 0x00},	// EXVTS = 0
{MISENSOR_8BIT, 0x302e, 0x00},	// EXVTS
{MISENSOR_8BIT, 0x302c, 0x00},	// EXHTS = 0
{MISENSOR_8BIT, 0x3014, 0x8c},	// B60
{MISENSOR_8BIT, 0x3070, 0xba},	// B50
{MISENSOR_8BIT, 0x3072, 0x9a},	// B60
{MISENSOR_8BIT, 0x301c, 0x02},	// max step 50
{MISENSOR_8BIT, 0x301d, 0x03},	// max step 60
//michael cong add for effect
{MISENSOR_8BIT, 0x3362, 0x80},

{MISENSOR_TOK_TERM, 0, 0}
};

/* camera vga 30fps, yuv, 2lanes */
static struct misensor_reg const ov2675_vga_init[] = {
{MISENSOR_8BIT, 0x3012, 0x10}, 	// SVGA Average mode

{MISENSOR_8BIT, 0x302a, 0x02}, 	// VTS = 696
{MISENSOR_8BIT, 0x302b, 0xb8}, 	// VTS
{MISENSOR_8BIT, 0x306f, 0x14}, 	// BLC target for short exposure of HDR mode
{MISENSOR_8BIT, 0x3020, 0x01}, 	// HS = 280
{MISENSOR_8BIT, 0x3021, 0x18}, 	// HS
{MISENSOR_8BIT, 0x3022, 0x00}, 	// VS = 6
{MISENSOR_8BIT, 0x3023, 0x06}, 	// VS
{MISENSOR_8BIT, 0x3024, 0x06}, 	// HW = 1624
{MISENSOR_8BIT, 0x3025, 0x58}, 	// HW
{MISENSOR_8BIT, 0x3026, 0x02}, 	// VH = 609
{MISENSOR_8BIT, 0x3027, 0x61}, 	// VH
{MISENSOR_8BIT, 0x3088, 0x02}, 	// ISP XOUT =640
{MISENSOR_8BIT, 0x3089, 0x80}, 	// ISP XOUT
{MISENSOR_8BIT, 0x308a, 0x01}, 	// ISP YOUT = 480
{MISENSOR_8BIT, 0x308b, 0xe0}, 	// ISP YOUT
{MISENSOR_8BIT, 0x3313, 0x00},  
{MISENSOR_8BIT, 0x3314, 0x00},  
{MISENSOR_8BIT, 0x3315, 0x00},  
{MISENSOR_8BIT, 0x3316, 0x64},	// Scale X input = 1600
{MISENSOR_8BIT, 0x3317, 0x25}, 	// Scale Y input = 600
{MISENSOR_8BIT, 0x3318, 0x80}, 	// Scale Y/X input
{MISENSOR_8BIT, 0x3319, 0x08}, 	// Scale X offset = 8
{MISENSOR_8BIT, 0x331a, 0x64}, 	// Scale X output = 1600
{MISENSOR_8BIT, 0x331b, 0x4b}, 	// Scale Y output = 1200
{MISENSOR_8BIT, 0x331c, 0x00}, 	// Scale Y/X output
{MISENSOR_8BIT, 0x331d, 0x38}, 	// Scale Y offset = 3, Scale X offset = 8
{MISENSOR_8BIT, 0x3302, 0x11}, 	// Scale on, UV Average on
{MISENSOR_8BIT, 0x3088, 0x02}, 	// ISP X output = 640
{MISENSOR_8BIT, 0x3089, 0x80}, 	// ISP X output
{MISENSOR_8BIT, 0x308a, 0x01}, 	// ISP Y output = 480
{MISENSOR_8BIT, 0x308b, 0xe0}, 	// ISP Y output
{MISENSOR_8BIT, 0x3640, 0x10},
{MISENSOR_8BIT, 0x331a, 0x28},
{MISENSOR_8BIT, 0x331b, 0x1e},
{MISENSOR_8BIT, 0x331c, 0x00},
{MISENSOR_8BIT, 0x300e, 0x2f},  // PLL
{MISENSOR_8BIT, 0x300f, 0xa6},	// PLL
{MISENSOR_8BIT, 0x3010, 0x81},	// PLL
{MISENSOR_8BIT, 0x3011, 0x00},	// clock divider
{MISENSOR_8BIT, 0x302d, 0x00},	// EXVTS = 0
{MISENSOR_8BIT, 0x302e, 0x00},	// EXVTS
{MISENSOR_8BIT, 0x302c, 0x00},	// EXHTS = 0
//{MISENSOR_8BIT, 0x3014, 0x8c},	// B60
{MISENSOR_8BIT, 0x3070, 0xd2},	// B50
{MISENSOR_8BIT, 0x3072, 0xaf},	// B60
{MISENSOR_8BIT, 0x301c, 0x02},	// max step 50
{MISENSOR_8BIT, 0x301d, 0x03},	// max step 60
//michael cong add for effect
{MISENSOR_8BIT, 0x3362, 0x90},

{MISENSOR_TOK_TERM, 0, 0}
};

/* camera svga 30fps, yuv, 2lanes */
static struct misensor_reg const ov2675_svga_init[] = {
{MISENSOR_8BIT, 0x3012, 0x10}, 	// SVGA Average mode
//{MISENSOR_8BIT, 0x3028, 0x07},	// HTS[15:8]
//{MISENSOR_8BIT, 0x3029, 0x94},	// HTS
{MISENSOR_8BIT, 0x302a, 0x08}, 	// VTS = 618
{MISENSOR_8BIT, 0x302b, 0x80}, 	// VTS
{MISENSOR_8BIT, 0x306f, 0x14}, 	// BLC target for short exposure in HDR mode
{MISENSOR_8BIT, 0x3020, 0x01}, 	// HS = 280
{MISENSOR_8BIT, 0x3021, 0x18}, 	// HS
{MISENSOR_8BIT, 0x3022, 0x00}, 	// VS = 6
{MISENSOR_8BIT, 0x3023, 0x06}, 	// VS
{MISENSOR_8BIT, 0x3024, 0x06}, 	// HW = 1624
{MISENSOR_8BIT, 0x3025, 0x58}, 	// HW
{MISENSOR_8BIT, 0x3026, 0x02}, 	// VH = 609
{MISENSOR_8BIT, 0x3027, 0x61}, 	// VH
{MISENSOR_8BIT, 0x3088, 0x03}, 	// ISP XOUT = 800
{MISENSOR_8BIT, 0x3089, 0x20}, 	// ISP XOUT
{MISENSOR_8BIT, 0x308a, 0x02}, 	// ISP YOUT = 600
{MISENSOR_8BIT, 0x308b, 0x58}, 	// ISP YOUT
{MISENSOR_8BIT, 0x3316, 0x64}, 	// Scale X input = 1600
{MISENSOR_8BIT, 0x3317, 0x25}, 	// Scale Y input = 600
{MISENSOR_8BIT, 0x3318, 0x80}, 	// Scale Y/X input
{MISENSOR_8BIT, 0x3319, 0x08}, 	// Scale offset = 8
{MISENSOR_8BIT, 0x331a, 0x64}, 	// Scale X output = 1600
{MISENSOR_8BIT, 0x331b, 0x4b}, 	// Scale Y output = 1200
{MISENSOR_8BIT, 0x331c, 0x00}, 	// Scale Y/X output
{MISENSOR_8BIT, 0x331d, 0x38}, 	// Scale Y offset = 3, Scale X offset = 8
{MISENSOR_8BIT, 0x3302, 0x11}, 	// Scale on, UV average on
{MISENSOR_8BIT, 0x3088, 0x03}, 	// ISP X output = 800
{MISENSOR_8BIT, 0x3089, 0x20}, 	// ISP X output
{MISENSOR_8BIT, 0x308a, 0x02}, 	// ISP Y output = 600
{MISENSOR_8BIT, 0x308b, 0x58}, 	// ISP Y output
{MISENSOR_8BIT, 0x3313, 0x00},  
{MISENSOR_8BIT, 0x3314, 0x00},  
{MISENSOR_8BIT, 0x3315, 0x00},  
{MISENSOR_8BIT, 0x331a, 0x32}, 	
{MISENSOR_8BIT, 0x331b, 0x25}, 	
{MISENSOR_8BIT, 0x331c, 0x80},  
{MISENSOR_8BIT, 0x300e, 0x31},  // PLL
{MISENSOR_8BIT, 0x300f, 0xa6},	// PLL
{MISENSOR_8BIT, 0x3010, 0x81},	// PLL
{MISENSOR_8BIT, 0x3011, 0x00},	// clock divider
{MISENSOR_8BIT, 0x302d, 0x00},	// EXVTS = 0
{MISENSOR_8BIT, 0x302e, 0x00},	// EXVTS
{MISENSOR_8BIT, 0x302c, 0x00},	// EXHTS = 0
{MISENSOR_8BIT, 0x3070, 0xba},	// B50
{MISENSOR_8BIT, 0x3072, 0x9a},	// B60
{MISENSOR_8BIT, 0x301c, 0x02},	// max step 50
{MISENSOR_8BIT, 0x301d, 0x03},	// max step 60
//michael cong add for effect
{MISENSOR_8BIT, 0x3362, 0x90},

{MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const ov2675_common[] = {
	 {MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const ov2675_iq[] = {
	{MISENSOR_TOK_TERM, 0, 0}
};

static struct misensor_reg const ov2675_init[] = {
	/* init software */
{MISENSOR_8BIT, 0x3012, 0x80},	//Soft Reset
{MISENSOR_TOK_DELAY, {0}, 5},

{MISENSOR_8BIT, 0x3086, 0x0f},	//stream off
{MISENSOR_8BIT, 0x308c, 0x80},	// timing control
{MISENSOR_8BIT, 0x308d, 0x0e},	// timing control
{MISENSOR_8BIT, 0x360b, 0x00},	// DVP control
{MISENSOR_8BIT, 0x30b0, 0xff},	// D[7:0] set to input
{MISENSOR_8BIT, 0x30b1, 0xff},	// Vsync, Strobe, PCLK, HREF, D[9:8] set to input
{MISENSOR_8BIT, 0x30b2, 0x24},	// IO control
{MISENSOR_8BIT, 0x300e, 0x31},  // PLL
{MISENSOR_8BIT, 0x300f, 0xa6},	// PLL
{MISENSOR_8BIT, 0x3010, 0x81},	// PLL
{MISENSOR_8BIT, 0x3082, 0x01},  
{MISENSOR_8BIT, 0x30f4, 0x01},  
{MISENSOR_8BIT, 0x3090, 0x3b},  
{MISENSOR_8BIT, 0x3091, 0xc0},  
{MISENSOR_8BIT, 0x30ac, 0x42},  
{MISENSOR_8BIT, 0x30d1, 0x08},  
{MISENSOR_8BIT, 0x30a8, 0x56},  
{MISENSOR_8BIT, 0x3015, 0x33},	// VAEC ceiling, 3 frames
{MISENSOR_8BIT, 0x3093, 0x00},  //
{MISENSOR_8BIT, 0x307e, 0xe5},	// apply digital gain if gain >=8x, digital gain from AGC[7:6]
{MISENSOR_8BIT, 0x3079, 0x00},  
{MISENSOR_8BIT, 0x30aa, 0x42},  
{MISENSOR_8BIT, 0x3017, 0x40},	// disable data drop
{MISENSOR_8BIT, 0x30f3, 0x82},  
{MISENSOR_8BIT, 0x306a, 0x0c},  
{MISENSOR_8BIT, 0x306d, 0x00},	// BLC control
{MISENSOR_8BIT, 0x336a, 0x3c},	// ISP control
{MISENSOR_8BIT, 0x3076, 0x6a},	// Vsync drop if frame data is dropped
{MISENSOR_8BIT, 0x30d9, 0x8c},  
{MISENSOR_8BIT, 0x3016, 0x82},	// max exposure time = Tframe - (2x2+1)
{MISENSOR_8BIT, 0x3601, 0x30},	// DVP output order D[9:0]
{MISENSOR_8BIT, 0x304e, 0x88},  
{MISENSOR_8BIT, 0x30f1, 0x82},  
{MISENSOR_8BIT, 0x3011, 0x02},	// PLL on, clock divider = 2+1=3
{MISENSOR_8BIT, 0x3013, 0xf7},	// fast AEC, big step, Banding filter on, auto banding disable under strong light, less than 1 line off
{MISENSOR_8BIT, 0x301c, 0x13},	// 50Hz max step
{MISENSOR_8BIT, 0x301d, 0x17},	// 60Hz max step
{MISENSOR_8BIT, 0x3070, 0x3e},	// B50
{MISENSOR_8BIT, 0x3072, 0x34},	// B60
{MISENSOR_8BIT, 0x30af, 0x00},  
{MISENSOR_8BIT, 0x3048, 0x1f},  
{MISENSOR_8BIT, 0x3049, 0x4e},  
{MISENSOR_8BIT, 0x304a, 0x20},  
{MISENSOR_8BIT, 0x304f, 0x20},  
{MISENSOR_8BIT, 0x304b, 0x02},  
{MISENSOR_8BIT, 0x304c, 0x00},  
{MISENSOR_8BIT, 0x304d, 0x02},  
{MISENSOR_8BIT, 0x304f, 0x20},  
{MISENSOR_8BIT, 0x30a3, 0x10},  
{MISENSOR_8BIT, 0x3013, 0xf7},	// fast AEC, big step, Banding filter on, auto banding disable under strong light, less than 1 line off
{MISENSOR_8BIT, 0x3014, 0x8c},  // manual 60Hz, band depend on 50/60 detect, night mode off, 50/60 smooth switch, 
{MISENSOR_8BIT, 0x3071, 0x00},	// BD50[15:8]
{MISENSOR_8BIT, 0x3070, 0xd2},	// BD50[7:0]
{MISENSOR_8BIT, 0x3073, 0x00},	// BD60[15:8]
{MISENSOR_8BIT, 0x3072, 0xaf},	// BD60[7:0]
{MISENSOR_8BIT, 0x301c, 0x02},	// 50Hz max band
{MISENSOR_8BIT, 0x301d, 0x03}, 	// 60Hz max band
{MISENSOR_8BIT, 0x304d, 0x42},    
{MISENSOR_8BIT, 0x304a, 0x40},  
{MISENSOR_8BIT, 0x304f, 0x40},  
{MISENSOR_8BIT, 0x3095, 0x07},  
{MISENSOR_8BIT, 0x3096, 0x16},  
{MISENSOR_8BIT, 0x3097, 0x1d},  
{MISENSOR_8BIT, 0x300e, 0x38},	// PLL
{MISENSOR_8BIT, 0x3020, 0x01},	// HS = 280
{MISENSOR_8BIT, 0x3021, 0x18},	// HS
{MISENSOR_8BIT, 0x3022, 0x00},	// VS = 10
{MISENSOR_8BIT, 0x3023, 0x0a},	// VS
{MISENSOR_8BIT, 0x3024, 0x06},	// HW = 1624
{MISENSOR_8BIT, 0x3025, 0x58},	// HW
{MISENSOR_8BIT, 0x3026, 0x04},	// VH = 1212
{MISENSOR_8BIT, 0x3027, 0xbc},	// VH

//{MISENSOR_8BIT, 0x3028, 0x07},	// HTS[15:8]
//{MISENSOR_8BIT, 0x3029, 0x94},	// HTS

{MISENSOR_8BIT, 0x3088, 0x06},	// ISP_XOUT = 1600
{MISENSOR_8BIT, 0x3089, 0x40},	// ISP_XOUT
{MISENSOR_8BIT, 0x308a, 0x04},	// ISP_YOUT = 1200
{MISENSOR_8BIT, 0x308b, 0xb0},	// ISP_YOUT
{MISENSOR_8BIT, 0x3316, 0x64},	// Scale H input size = 1600
{MISENSOR_8BIT, 0x3317, 0x4b},	// Scale V input size = 1200
{MISENSOR_8BIT, 0x3318, 0x00},	// Scale H/V input size
{MISENSOR_8BIT, 0x331a, 0x64},	// Scale H output size = 1600
{MISENSOR_8BIT, 0x331b, 0x4b},	// Scale V output size = 1200
{MISENSOR_8BIT, 0x331c, 0x00},	// Scale H/V output size
{MISENSOR_8BIT, 0x3100, 0x00},  
{MISENSOR_8BIT, 0x3320, 0xfa},	// Advanced AWB, one zone on
{MISENSOR_8BIT, 0x3321, 0x11},  
{MISENSOR_8BIT, 0x3322, 0x92},  
{MISENSOR_8BIT, 0x3323, 0x01},  
{MISENSOR_8BIT, 0x3324, 0x97},  
{MISENSOR_8BIT, 0x3325, 0x02},  
{MISENSOR_8BIT, 0x3326, 0xff},  
{MISENSOR_8BIT, 0x3327, 0x0c},	// S
{MISENSOR_8BIT, 0x3328, 0x10}, 	// EC
{MISENSOR_8BIT, 0x3329, 0x10},	// FC
{MISENSOR_8BIT, 0x332a, 0x58},	// X0
{MISENSOR_8BIT, 0x332b, 0x50},	// Y0
{MISENSOR_8BIT, 0x332c, 0xbe},	// Kx
{MISENSOR_8BIT, 0x332d, 0xe1},	// Ky
{MISENSOR_8BIT, 0x332e, 0x43},	// Day limit
{MISENSOR_8BIT, 0x332f, 0x36},	// A limit
{MISENSOR_8BIT, 0x3330, 0x4d},	// Day split
{MISENSOR_8BIT, 0x3331, 0x44},	// A split
{MISENSOR_8BIT, 0x3332, 0xf8},	// AWB top limit
{MISENSOR_8BIT, 0x3333, 0x0a},	// AWB bot limit
{MISENSOR_8BIT, 0x3334, 0xf0},	// Red limit
{MISENSOR_8BIT, 0x3335, 0xf0},	// Green limit
{MISENSOR_8BIT, 0x3336, 0xf0},	// Blue limit
{MISENSOR_8BIT, 0x3337, 0x40},  
{MISENSOR_8BIT, 0x3338, 0x40}, 
{MISENSOR_8BIT, 0x3339, 0x40},  
{MISENSOR_8BIT, 0x333a, 0x00},  
{MISENSOR_8BIT, 0x333b, 0x00},  

{MISENSOR_8BIT, 0x3380, 0x28},  // CMX1
{MISENSOR_8BIT, 0x3381, 0x48},  // CMX2
{MISENSOR_8BIT, 0x3382, 0x10},  // CMX3
{MISENSOR_8BIT, 0x3383, 0x23},  // CMX4
{MISENSOR_8BIT, 0x3384, 0xc0}, 	// CMX5
{MISENSOR_8BIT, 0x3385, 0xe5},  // CMX6
{MISENSOR_8BIT, 0x3386, 0xc2},  // CMX7
{MISENSOR_8BIT, 0x3387, 0xb3},  // CMX8
{MISENSOR_8BIT, 0x3388, 0xe },  // CMX9
{MISENSOR_8BIT, 0x3389, 0x98}, 	// sign[8:1]
{MISENSOR_8BIT, 0x338a, 0x1 },	// sign[9]

{MISENSOR_8BIT, 0x3340, 0x0e},	// YST1
{MISENSOR_8BIT, 0x3341, 0x1a},	// YST2
{MISENSOR_8BIT, 0x3342, 0x31},	// YST3
{MISENSOR_8BIT, 0x3343, 0x45},	// YST4
{MISENSOR_8BIT, 0x3344, 0x5a},	// YST5
{MISENSOR_8BIT, 0x3345, 0x69},	// YST6
{MISENSOR_8BIT, 0x3346, 0x75},	// YST7
{MISENSOR_8BIT, 0x3347, 0x7e},	// YST8
{MISENSOR_8BIT, 0x3348, 0x88},	// YST9
{MISENSOR_8BIT, 0x3349, 0x96},	// YST10
{MISENSOR_8BIT, 0x334a, 0xa3},	// YST11
{MISENSOR_8BIT, 0x334b, 0xaf},	// YST12
{MISENSOR_8BIT, 0x334c, 0xc4},	// YST13
{MISENSOR_8BIT, 0x334d, 0xd7},	// YST14
{MISENSOR_8BIT, 0x334e, 0xe8},	// YST15
{MISENSOR_8BIT, 0x334f, 0x20},	// Y slop, auto calculated if 0x06[7]=0

{MISENSOR_8BIT, 0x3350, 0x32},	// Red X center
{MISENSOR_8BIT, 0x3351, 0x25},  // Red Y center
{MISENSOR_8BIT, 0x3352, 0x80},  // Red X/Y center
{MISENSOR_8BIT, 0x3353, 0x1e},  // Red a1
{MISENSOR_8BIT, 0x3354, 0x00},  // Red b1
{MISENSOR_8BIT, 0x3355, 0x85},  // Red b2, a2
{MISENSOR_8BIT, 0x3356, 0x32},  // Green X center
{MISENSOR_8BIT, 0x3357, 0x25},  // Green Y center
{MISENSOR_8BIT, 0x3358, 0x80},  // Green X/Y center
{MISENSOR_8BIT, 0x3359, 0x1b},  // Green a1
{MISENSOR_8BIT, 0x335a, 0x00}, 	// Green b1
{MISENSOR_8BIT, 0x335b, 0x85},  // Green b2, a2
{MISENSOR_8BIT, 0x335c, 0x32},  // Blue X center
{MISENSOR_8BIT, 0x335d, 0x25},  // Blue Y center
{MISENSOR_8BIT, 0x335e, 0x80},  // Blue X/Y center
{MISENSOR_8BIT, 0x335f, 0x1b},  // Blue a1
{MISENSOR_8BIT, 0x3360, 0x00},  // Blue b1
{MISENSOR_8BIT, 0x3361, 0x85},  // Blue b2, a2
{MISENSOR_8BIT, 0x3363, 0x70},  // Lenc T1
{MISENSOR_8BIT, 0x3364, 0x7f},  // Lenc T2
{MISENSOR_8BIT, 0x3365, 0x00},  // Lenc T1/T2
{MISENSOR_8BIT, 0x3366, 0x00},  

{MISENSOR_8BIT, 0x3301, 0xff},	// SDE on, UV Adjust on, color matrix on, sharpen on, DNS on, CIP on, BPC on, WPC on
{MISENSOR_8BIT, 0x338B, 0x11},	// ADJ_offset
{MISENSOR_8BIT, 0x338c, 0x10},	// ADJ_Th1
{MISENSOR_8BIT, 0x338d, 0x40},	// ADJ_Th2

{MISENSOR_8BIT, 0x3370, 0xd0},  
{MISENSOR_8BIT, 0x3371, 0x00},  
{MISENSOR_8BIT, 0x3372, 0x00},	// Edge threshold manual
{MISENSOR_8BIT, 0x3373, 0x60},	// DNS offset
{MISENSOR_8BIT, 0x3374, 0x10},	// DNS threshold
{MISENSOR_8BIT, 0x3375, 0x10},	// DNS slop
{MISENSOR_8BIT, 0x3376, 0x04},	// Sharpness offset 1
{MISENSOR_8BIT, 0x3377, 0x00},	// Sharpness offset 2
{MISENSOR_8BIT, 0x3378, 0x04},	// Sharpness th1
{MISENSOR_8BIT, 0x3379, 0x80},	// Sharpness th2

{MISENSOR_8BIT, 0x3069, 0x84},	// BLC target
{MISENSOR_8BIT, 0x307c, 0x13},	// mirror off, flip off
{MISENSOR_8BIT, 0x3087, 0x02},	// BLC always on

{MISENSOR_8BIT, 0x3300, 0xfc},	// ISP on, gamma on, AWB stat on, AWB gain on, Lenc on, Lenc low light on, ISP format = YUV
{MISENSOR_8BIT, 0x3302, 0x01},	// Scale off, UV average on
{MISENSOR_8BIT, 0x3400, 0x02},	// YUV 422
{MISENSOR_8BIT, 0x3401, 0x00},	// YUV 422
{MISENSOR_8BIT, 0x3606, 0x20},	// DVP on
{MISENSOR_8BIT, 0x3601, 0x30},	// DVP Data order D[9:0] 
{MISENSOR_8BIT, 0x300e, 0x31},  // PLL
{MISENSOR_8BIT, 0x30f3, 0x83},  
{MISENSOR_8BIT, 0x304e, 0x88},  
//MIPI
{MISENSOR_8BIT, 0x363b, 0x01},  
{MISENSOR_8BIT, 0x309e, 0x08},  
{MISENSOR_8BIT, 0x3606, 0x00},	// DVP off
{MISENSOR_8BIT, 0x3630, 0x31},  // Disable short package output
{MISENSOR_8BIT, 0x3086, 0x0f},	//sleep on
{MISENSOR_8BIT, 0x3086, 0x00},	//sleep off

{MISENSOR_8BIT, 0x304e, 0x04},	// [7] DVP_CLK_snr
{MISENSOR_8BIT, 0x363b, 0x01},	// disable cd
{MISENSOR_8BIT, 0x309e, 0x08},	// disable lp_rx
{MISENSOR_8BIT, 0x3606, 0x00},	// disable dvp
{MISENSOR_8BIT, 0x3084, 0x01},	// scale_div_man_en
{MISENSOR_8BIT, 0x3010, 0x82},	// scale_div_man
{MISENSOR_8BIT, 0x3011, 0x03},	// sys_div /4
{MISENSOR_8BIT, 0x3634, 0x26},  //
//michael cong add for effect
//shading
{MISENSOR_8BIT, 0x3350, 0x30},	
{MISENSOR_8BIT, 0x3351, 0x26},
{MISENSOR_8BIT, 0x3352, 0x0c},
{MISENSOR_8BIT, 0x3353, 0x35},
{MISENSOR_8BIT, 0x3354, 0x00},
{MISENSOR_8BIT, 0x3355, 0x85},		
{MISENSOR_8BIT, 0x3356, 0x30},
{MISENSOR_8BIT, 0x3357, 0x26},
{MISENSOR_8BIT, 0x3358, 0x08},
{MISENSOR_8BIT, 0x3359, 0x2c},
{MISENSOR_8BIT, 0x335a, 0x00},
{MISENSOR_8BIT, 0x335b, 0x85},		 
{MISENSOR_8BIT, 0x335c, 0x2f},
{MISENSOR_8BIT, 0x335d, 0x25},
{MISENSOR_8BIT, 0x335e, 0x88},	
{MISENSOR_8BIT, 0x335f, 0x2a},
{MISENSOR_8BIT, 0x3360, 0x00},
{MISENSOR_8BIT, 0x3361, 0x85},
//awb
{MISENSOR_8BIT, 0x3320, 0xfa},
{MISENSOR_8BIT, 0x3321, 0x11},
{MISENSOR_8BIT, 0x3322, 0x92},
{MISENSOR_8BIT, 0x3323, 0x05},
{MISENSOR_8BIT, 0x3324, 0x97},
{MISENSOR_8BIT, 0x3325, 0x02},
{MISENSOR_8BIT, 0x3326, 0xff},
{MISENSOR_8BIT, 0x3327, 0x0f},
{MISENSOR_8BIT, 0x3328, 0x14},
{MISENSOR_8BIT, 0x3329, 0x18},
{MISENSOR_8BIT, 0x332a, 0x61},
{MISENSOR_8BIT, 0x332b, 0x56},
{MISENSOR_8BIT, 0x332c, 0x94},
{MISENSOR_8BIT, 0x332d, 0x9e},
{MISENSOR_8BIT, 0x332e, 0x3d},
{MISENSOR_8BIT, 0x332f, 0x34},
{MISENSOR_8BIT, 0x3330, 0x4f},
{MISENSOR_8BIT, 0x3331, 0x44},
{MISENSOR_8BIT, 0x3332, 0xf8},
{MISENSOR_8BIT, 0x3333, 0x0a},
{MISENSOR_8BIT, 0x3334, 0xf0},
{MISENSOR_8BIT, 0x3335, 0xf0},
{MISENSOR_8BIT, 0x3336, 0xf0},
{MISENSOR_8BIT, 0x3337, 0x40},
{MISENSOR_8BIT, 0x3338, 0x40},
{MISENSOR_8BIT, 0x3339, 0x40},
{MISENSOR_8BIT, 0x333a, 0x00},
{MISENSOR_8BIT, 0x333b, 0x00},
//gamma
{MISENSOR_8BIT, 0x3340, 0x07}, 
{MISENSOR_8BIT, 0x3341, 0x0F},
{MISENSOR_8BIT, 0x3342, 0x1F},
{MISENSOR_8BIT, 0x3343, 0x2E},
{MISENSOR_8BIT, 0x3344, 0x3C},
{MISENSOR_8BIT, 0x3345, 0x49},
{MISENSOR_8BIT, 0x3346, 0x56},
{MISENSOR_8BIT, 0x3347, 0x62},
{MISENSOR_8BIT, 0x3348, 0x6B},
{MISENSOR_8BIT, 0x3349, 0x7E},
{MISENSOR_8BIT, 0x334A, 0x8E},
{MISENSOR_8BIT, 0x334B, 0x9C},
{MISENSOR_8BIT, 0x334C, 0xB6},
{MISENSOR_8BIT, 0x334D, 0xCB},
{MISENSOR_8BIT, 0x334E, 0xDF},
{MISENSOR_8BIT, 0x334F, 0x2C},
//matrix
{MISENSOR_8BIT, 0x3380, 0x24}, 
{MISENSOR_8BIT, 0x3381, 0x55},
{MISENSOR_8BIT, 0x3382, 0x07},
{MISENSOR_8BIT, 0x3383, 0x24},
{MISENSOR_8BIT, 0x3384, 0xC8},
{MISENSOR_8BIT, 0x3385, 0xEC},
{MISENSOR_8BIT, 0x3386, 0xDD},
{MISENSOR_8BIT, 0x3387, 0xE8},
{MISENSOR_8BIT, 0x3388, 0x0B},
{MISENSOR_8BIT, 0x3389, 0x98},
{MISENSOR_8BIT, 0x338A, 0x00},
//end

{MISENSOR_8BIT, 0x3086, 0x0f},	//sleep on
{MISENSOR_8BIT, 0x3086, 0x00},	//sleep off
{MISENSOR_TOK_TERM, 0, 0}
};
#endif
