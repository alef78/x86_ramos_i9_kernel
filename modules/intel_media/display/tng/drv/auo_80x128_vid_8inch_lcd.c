/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * wang junxian: Himax LCD blink sometimes, maybe MIPI clock
 * need to adjust
 * ----------------------------------------------------------------------------
 */
/*
 * Copyright ? 2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 * Xiaojun Wang <wang.xiaojun3@byd.com>
 */

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_esd.h"
#include <asm/intel_scu_pmic.h>
#include <linux/gpio.h>
#include <psb_intel_reg.h>
//#include "displays/auo_inch_vid.h"

/* Brightness related */
#define MINIMUM_BRIGHTNESS_LEVEL_20KHZ	15
/* MSIC PWM duty cycle goes up to 0x63 = 99% */
#define BACKLIGHT_DUTY_FACTOR	0x63
#define PWM0DUTYCYCLE		0x67

#define GPIOPWMCTRL	0x38F
#define PWM0CLKDIV0	0x62 /* low byte */
#define PWM0CLKDIV1	0x61 /* high byte */ 

#define MSICGPIO_H 		0x3B	//0x3D	//0x21	//0x3F
#define MSICGPIO_L 		0x3E	//0x3C	//0x20	// 0x3E

#define AUO_inch_PANEL_WIDTH 150
#define AUO_inch_PANEL_HEIGHT 94

#define LCD_RESET  190
#define LCD_VCC_3P3 189
#define LCD_VCC_1P8 188
static int mipi_reset_gpio;
static int bias_en_gpio;

#define LCD_PANEL_LDO_EN 		0x6E	 	//GPIO0HV2
#define LCD_PANEL_LDO_EN_2	 	0x73		//GPIO1HV1 control VDDI 3.3V
static int lcd_gpio_vcc_en = 175;		//gp_core_079  (GP_CAMERA_SB7) control VCCI 1.8V
static int lcd_gpio_stbyb = 177; 		//gp_core_081(GP_CAMERA_SB9)
static u8 auo_enable_ic_power[]      = {0xC3, 0x40, 0x00, 0x28};
static u8 auo_disable_ic_power[]      = {0xC3, 0x40, 0x00, 0x20};
static u8 auo_password[]      = {0xF0, 0x5A, 0x5A};
#if 1
static u8 auo_init_sequence1[]      = {0xF1, 0x5A, 0x5A};
static u8 auo_init_sequence2[]      = {0xFC, 0xA5, 0xA5};
static u8 auo_init_sequence3[]      = {0xD0, 0x00, 0x10};
static u8 auo_init_sequence4[]      = {0xbc, 0x1, 0x4e,0xa0};
static u8 auo_init_sequence5[]      = {0xE1, 0x3, 0x10,0x1C,0X82,0X07};
static u8 auo_init_sequence6[]      = {0xB1, 0x10};
static u8 auo_init_sequence7[]      = {0xB2, 0x14, 0x22,0x2F,0X04};
static u8 auo_init_sequence8[]      = {0xF2, 0x02, 0xC,0x8,0X88,0X018};
static u8 auo_init_sequence9[]      = {0xB5, 0x1e};
static u8 auo_init_sequence10[]      = {0xB0, 0x04};
static u8 auo_init_sequence11[]      = {0xFD, 0x9};
static u8 auo_init_sequence12[]      = {0xF6, 0x63, 0x21,0x86,0X00,0X00,0};
static u8 auo_init_sequence13[]      = {0xD8, 0x5E, 0x4C,0x10};
static u8 auo_init_sequence14[]      = {0xF3, 0x1, 0xC0,0xE0,0X62,0XD0,0X81,0X35,0XF3,0X30,0X24,0};
static u8 auo_init_sequence15[]      = {0XF4,0X00,0X02,0X03 ,0X26 ,0X03 ,0X02 ,0X09 ,0X00 ,0X07 ,0X16 ,0X16 ,0X03 ,0X00
 ,0X08 ,0X08 ,0X03 ,0X00 ,0X00 ,0X12 ,0X1C ,0X1D ,0X1E ,0X01 ,0X09 ,0X01 ,0X04 ,0X02 ,0X61 ,0X74 ,0X75
 ,0X72 ,0X83 ,0X80 ,0X80,0X00 ,0X00 ,0X01 ,0X01 ,0X28 ,0X04 ,0X03 ,0X28 ,0X01 ,0XD1 ,0X32};
static u8 auo_init_sequence16[]      = {0XF5  ,0X9D  ,0X42  ,0X42  ,0X5F  ,0XAB  ,0X98  ,0X4F  ,0X0F
  ,0X33  ,0X43  ,0X04  ,0X59  ,0X54  ,0X52  ,0X05  ,0X40  ,0X60  ,0X40  ,0X60  ,0X40  ,0X27  ,0X26  ,0X52  ,0X25
  ,0X6D  ,0X18}; 

static u8 auo_init_sequence17[] ={0XEE ,0X3F ,0X3F ,0X3F 
,0X00 ,0X3F ,0X3F ,0X3F
 ,0X00 ,0X11 ,0X22};

static u8 auo_init_sequence18[] ={0XEF ,0X12 ,0X12 ,0X43 
,0X43 ,0X90 ,0X84 ,0X24
 ,0X81 ,0X00 ,0X21 ,0X21
 ,0X03 ,0X03 ,0X40 ,0X80 
,0X82 ,0X00};
static u8 auo_init_sequence19[]= {0XFA ,0X00 ,0X35 ,0X06 ,0X0A ,0X14 ,0X0D ,0X13 ,0X19 
	,0X1C ,0X25 ,0X2B ,0X32 ,0X3B ,0X39 ,0X3D ,0X38 ,0X3D ,0X25 ,0X30 ,0X26 ,0X2A ,0X2B ,0X1E ,0X22 
	,0X23 ,0X22 ,0X28 ,0X2D ,0X33 ,0X3B ,0X38 ,0X2D ,0X2D ,0X2A ,0X0C ,0X35 ,0X10 ,0X14 
	,0X1C ,0X14 ,0X1A ,0X1E ,0X1F ,0X27 ,0X2C ,0X33 ,0X3B ,0X38 ,0X30 ,0X30 ,0X30};

static u8 auo_init_sequence20[] = {0XFA ,0X00 ,0X35 ,0X06 ,0X0A ,0X14 ,0X0D ,0X13 ,0X19 
	,0X1C ,0X25 ,0X2B ,0X32 ,0X3B ,0X39 ,0X3D ,0X38 ,0X3D ,0X25 ,0X30 ,0X26 ,0X2A ,0X2B ,0X1E ,0X22 
	,0X23 ,0X22 ,0X28 ,0X2D ,0X33 ,0X3B ,0X38 ,0X2D ,0X2D ,0X2A ,0X0C ,0X35 ,0X10 ,0X14 
	,0X1C ,0X14 ,0X1A ,0X1E ,0X1F ,0X27 ,0X2C ,0X33 ,0X3B ,0X38 ,0X30 ,0X30 ,0X30};

static u8 auo_init_sequence21[] ={0XF7 ,0X0B ,0X0B ,0X09 
,0X09 ,0X0A ,0X0A ,0X08
 ,0X08 ,0X01 ,0X16 ,0X16
 ,0X17 ,0X17 ,0X07 ,0X01
 ,0X01 ,0X0B ,0X0B ,0X09
 ,0X09 ,0X0A ,0X0A ,0X08 
,0X08 ,0X01 ,0X16 ,0X16
 ,0X17 ,0X17 ,0X07 ,0X01
 ,0X01};

#endif
static
int auo_inch_vid_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	PSB_DEBUG_WARN("\n");
	
	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}	

//	mdfld_dsi_send_mcs_short_hs(sender, auo_inch_set_RTERM[0], auo_inch_set_RTERM[1], 1, 0);
	mdfld_dsi_send_gen_long_hs(sender, auo_password, sizeof(auo_password), 0);
	
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence1, sizeof(auo_init_sequence1), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence2, sizeof(auo_init_sequence2), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence3, sizeof(auo_init_sequence3), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence4, sizeof(auo_init_sequence4), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence5, sizeof(auo_init_sequence5), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence6, sizeof(auo_init_sequence6), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence7, sizeof(auo_init_sequence7), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence8, sizeof(auo_init_sequence8), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence9, sizeof(auo_init_sequence9), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence10, sizeof(auo_init_sequence10), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence11, sizeof(auo_init_sequence11), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence12, sizeof(auo_init_sequence12), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence13, sizeof(auo_init_sequence13), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence14, sizeof(auo_init_sequence14), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence15, sizeof(auo_init_sequence15), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence16, sizeof(auo_init_sequence16), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence17, sizeof(auo_init_sequence17), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence18, sizeof(auo_init_sequence18), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence19, sizeof(auo_init_sequence19), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence20, sizeof(auo_init_sequence20), 0);
		mdfld_dsi_send_gen_long_hs(sender, auo_init_sequence21, sizeof(auo_init_sequence21), 0);
		mdelay(10);
	
#if 0
		mdfld_dsi_send_mcs_short_hs(sender, 0x11, 0, 0,
				MDFLD_DSI_SEND_PACKAGE);
	
		mdelay(10);
		mdfld_dsi_send_gen_long_hs(sender, auo_enable_ic_power, 4, 0);
			msleep(180);
	
		/* Set Display on */
		mdfld_dsi_send_mcs_short_hs(sender, 0x29, 0, 0,
				MDFLD_DSI_SEND_PACKAGE);
#endif

	return 0;
}


static
int auo_inch_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_WARN("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	gpio_direction_output(12,1);

	err = mdfld_dsi_send_mcs_long_hs(sender, auo_password, 3, 0);
	if (err) {
		DRM_ERROR("Faild to send SW reset packet\n");
	}
	/* Sleep Out */
	msleep(10);
	err = mdfld_dsi_send_mcs_short_hs(sender, 0x11, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Exit Sleep Mode\n", __func__, __LINE__);
	}
	/* Wait for 6 frames after exit_sleep_mode. */
	msleep(30);
#if 1
	/* Set Display on */
	err = mdfld_dsi_send_mcs_short_hs(sender, 0x29, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Display On\n", __func__, __LINE__);
	}
	/* Wait for 1 frame after set_display_on. */
	msleep(20);
#endif
	/*send TURN_ON packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender, 
		MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Faild to send turn on packet\n");
	}

	msleep(20);
	err = mdfld_dsi_send_mcs_long_hs(sender, auo_enable_ic_power, 4, 0);
	msleep(200);

#if 0	
	/* Set Display on */
	err = mdfld_dsi_send_mcs_short_hs(sender, 0x29, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Display On\n", __func__, __LINE__);
	}
	
	msleep(100);
#endif
	dsi_config->dsi_hw_context.panel_on = true;

	return 0;
}

static
int auo_inch_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;
	
	PSB_DEBUG_WARN("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	mdfld_dsi_send_mcs_short_hs(sender, 0x28, 0x00, 1, 0);
	msleep(30);
	
	err = mdfld_dsi_send_mcs_long_hs(sender, auo_disable_ic_power, 4, 0);
	if (err) {
		DRM_ERROR("Faild to send SW reset packet\n");
	}
	msleep(15);



	mdfld_dsi_send_mcs_short_hs(sender, 0x10, 0x00, 1, 0);
	msleep(30);	
	
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
			MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Faild to send turn on packet\n");
		return err;
		}

	msleep(20);
	if (err)
		pr_err("%s: LCD_PANEL_LDO_EN reset failed\n", __func__);
	msleep(25);

	gpio_set_value_cansleep(LCD_VCC_3P3, 0);

	dsi_config->dsi_hw_context.panel_on = false;
	return 0;
}

static
int auo_inch_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
	int pipe = dsi_config->pipe;
	u32 power_island = 0;


	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
		power_island = pipe_to_island(pipe);

		if (!power_island_get(power_island)) {
			DRM_ERROR("Failed to turn on power island\n");
			return -EAGAIN;
		}

		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		if ((device_ready_val & DSI_DEVICE_READY) &&
				(dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			DRM_INFO("%s: panel is not detected!\n", __func__);
		}

		status = MDFLD_DSI_PANEL_CONNECTED;

		power_island_put(power_island);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}
	return status;
}

static
int mdfld_dsi_auo_inch_panel_reset(struct mdfld_dsi_config *dsi_config)
{


		gpio_direction_output(LCD_VCC_1P8, 0);
		gpio_set_value_cansleep(LCD_VCC_1P8, 0);
				usleep_range(2000, 2500);
		gpio_set_value_cansleep(LCD_VCC_1P8, 1);
				usleep_range(2000, 2500);
		gpio_direction_output(LCD_VCC_3P3, 0);
		gpio_direction_output(LCD_RESET, 0);
		gpio_set_value_cansleep(LCD_VCC_3P3, 0);

		gpio_set_value_cansleep(LCD_RESET, 0);
		usleep_range(2000, 2500);
		gpio_set_value_cansleep(LCD_VCC_3P3, 1);
		usleep_range(2000, 2500);
		gpio_set_value_cansleep(LCD_RESET, 1);
		usleep_range(3000, 3500);

	return 0;
}

static
struct drm_display_mode *auo_inch_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;
	mode->hdisplay = 800;
	mode->vdisplay = 1280;

	mode->hsync_start = mode->hdisplay + 24;
	mode->hsync_end = mode->hsync_start + 8;
	mode->htotal = mode->hsync_end + 128;

	mode->vsync_start = mode->vdisplay + 12;
	mode->vsync_end = mode->vsync_start + 8;
	mode->vtotal = mode->vsync_end + 12;

	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->htotal * mode->vtotal / 1000;

	PSB_DEBUG_ENTRY("hdisplay is %d\n", mode->hdisplay);
	PSB_DEBUG_ENTRY("vdisplay is %d\n", mode->vdisplay);
	PSB_DEBUG_ENTRY("HSS is %d\n", mode->hsync_start);
	PSB_DEBUG_ENTRY("HSE is %d\n", mode->hsync_end);
	PSB_DEBUG_ENTRY("htotal is %d\n", mode->htotal);
	PSB_DEBUG_ENTRY("VSS is %d\n", mode->vsync_start);
	PSB_DEBUG_ENTRY("VSE is %d\n", mode->vsync_end);
	PSB_DEBUG_ENTRY("vtotal is %d\n", mode->vtotal);
	PSB_DEBUG_ENTRY("clock is %d\n", mode->clock);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static
void auo_inch_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		pi->width_mm = AUO_inch_PANEL_WIDTH;
		pi->height_mm = AUO_inch_PANEL_HEIGHT;
	}
}

static
int auo_inch_vid_set_brightness(struct mdfld_dsi_config *dsi_config, int level)
{
	int duty_val = 0;
	int ret = 0;
	return 0;

}

static
void auo_inch_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	/* Virtual channel number */
	int mipi_vc = 0;
	int mipi_pixel_format = 0x4;
	/*  MODE */
	int mipi_mode = 0x2;
	/* IP_TG_CONFIG */
	int ip_tg_config = 0x4;
	dsi_config->lane_count = 4;
	dsi_config->bpp = 24;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;

	//702 parameters
	/* This is for 400 mhz.  Set it to 0 for 800mhz */
	hw_ctx->cck_div = 0;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xdcf50;
	hw_ctx->lp_rx_timeout = 0xffff;
	hw_ctx->turn_around_timeout = 0x14;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x18;
	hw_ctx->init_count = 0x7d0;
	hw_ctx->eot_disable = 0x03;
	hw_ctx->lp_byteclk = 0x3;
	hw_ctx->clk_lane_switch_time_cnt = 0x18000b;

	hw_ctx->dphy_param = 0x2A0D350C;	// 0x150D350C; // 0x1F0F350A;	// 0x15091F06; //0x160d3610;  //0x3F0d3510; 

	/*set up func_prg*/
	//hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;

	/*setup video mode format*/
	hw_ctx->video_mode_format = mipi_mode | ip_tg_config;

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = ((mipi_pixel_format << 7) | (mipi_vc << 3) |
			dsi_config->lane_count);

	/*setup mipi port configuration*/
	hw_ctx->mipi = MIPI_PORT_EN | PASS_FROM_SPHY_TO_AFE |
		 dsi_config->lane_config;
}

static void mdfld_dsi_auo_inch_brightness_init(void)
{
	int ret;
	u8 pwmctrl;

	printk(KERN_INFO "[DISPLAY] %s: Enter\n", __func__);
	ret=gpio_request(12,"backlight");
		if(ret)
			printk("backlight gpio request failed\n");
	gpio_direction_output(12,1);

}

static int auo_inch_vid_gpio_init(void)
{
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	ret = gpio_request(LCD_RESET, "LCD_RESET");	
	if (ret) {
		printk(" Faild to request panel reset gpio\n");
	}
	ret = gpio_request(LCD_VCC_3P3, "LCD_3P3");	
	if (ret) {
		printk(" Faild to request gpio 3p3\n");
	}
	ret = gpio_request(LCD_VCC_1P8, "LCD_1P8"); 
		if (ret) {
			DRM_ERROR(" Faild to request gpio 1p8\n");
		}
	
		ret=gpio_request(12,"backlight");
			if(ret)
				printk(" gpio backlight request failed\n");


	return 0;
}

void auo_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = auo_inch_vid_get_config_mode;
	p_funcs->get_panel_info = auo_inch_vid_get_panel_info;
	p_funcs->dsi_controller_init = auo_inch_vid_dsi_controller_init;
	p_funcs->detect = auo_inch_vid_detect;
	p_funcs->reset = mdfld_dsi_auo_inch_panel_reset;
	p_funcs->power_on = auo_inch_vid_power_on;
	p_funcs->power_off = auo_inch_vid_power_off;
	p_funcs->set_brightness = auo_inch_vid_set_brightness;
//	p_funcs->drv_ic_init = auo_inch_vid_ic_init;

	auo_inch_vid_gpio_init();
	//mdfld_dsi_auo_inch_brightness_init();
}

#if 0
static int auo_inch_vid_probe(struct platform_device *pdev)
{
	DRM_INFO("%s: cpt 7inch panel detected\n", __func__);
	intel_mid_panel_register(auo_inch_vid_init);

	return 0;
}

static struct platform_driver auo_inch_driver = {
	.probe	= auo_inch_vid_probe,
	.driver	= {
		.name	= "AUO 8 inch VID",
		.owner	= THIS_MODULE,
	},
};

static int __init auo_inch_lcd_init(void)
{
	DRM_INFO("%s\n", __func__);
	
	return platform_driver_register(&auo_inch_driver);
}

module_init(auo_inch_lcd_init);
#endif
