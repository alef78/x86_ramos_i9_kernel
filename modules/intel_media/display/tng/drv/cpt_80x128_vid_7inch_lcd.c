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
#include <linux/input/intel_mid_vibra.h>

#define CPT_7_inch_PANEL_WIDTH 94
#define CPT_7_inch_PANEL_HEIGHT 151

#define LCD_RESET  190
#define LCD_VCC_3P3 189
#define LCD_VCC_1P8 188

static
int cpt_7_inch_vid_ic_init(struct mdfld_dsi_config *dsi_config)
{
	
	return 0;
}

static
int cpt_7_inch_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/*send TURN_ON packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender, 
		MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Faild to send turn on packet\n");
	}

	mdelay(5);
	mdfld_dsi_send_mcs_short_hs(sender, 0x01, 0x00, 0 , 0);
	mdelay(5);
	/* VDD->VCC->DATA->BL*/
	gpio_set_value_cansleep(LCD_VCC_3P3, 1);
	mdelay(2);
	gpio_set_value_cansleep(LCD_VCC_1P8, 1);
	mdelay(50);

	dsi_config->dsi_hw_context.panel_on = true;

	return 0;
}

static
int cpt_7_inch_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;
	
	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	/* BL->DATA->VCC->VDD*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
			MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Faild to send turn on packet\n");
		return err;
	}

	mdelay(5);
	gpio_set_value_cansleep(LCD_VCC_1P8, 0);
	mdelay(5);
	gpio_set_value_cansleep(LCD_VCC_3P3, 0);
	mdelay(500);

	dsi_config->dsi_hw_context.panel_on = false;
	return 0;
}

static
int cpt_7_inch_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	int pipe = dsi_config->pipe;
	u32 dpll_val, device_ready_val;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			return -EAGAIN;
		}

		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		if ((device_ready_val & DSI_DEVICE_READY) &&
		    (dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
			psb_enable_vblank(dev, pipe);
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
            DRM_INFO("%s: panel is not detected!\n", __func__);
			/* there is no reset on P708 baord it should power off panel after fw initial it */
			gpio_set_value_cansleep(LCD_VCC_1P8, 0);
			mdelay(30);
			gpio_set_value_cansleep(LCD_VCC_3P3, 0);
			mdelay(500);
		}

		status = MDFLD_DSI_PANEL_CONNECTED;

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static
int mdfld_dsi_cpt_7_inch_panel_reset(struct mdfld_dsi_config *dsi_config)
{

	PSB_DEBUG_ENTRY("\n");

//	gpio_set_value_cansleep(LCD_VCC_1P8, 1);
//	msleep(2);
//	gpio_set_value_cansleep(LCD_VCC_3P3, 1);
//	msleep(2);

	return 0;
}


static
struct drm_display_mode *cpt_7_inch_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;
	mode->hdisplay = 800;
	mode->vdisplay = 1280;
#if 0
	mode->hsync_start = mode->hdisplay + 24;
	mode->hsync_end = mode->hsync_start + 8;
	mode->htotal = mode->hsync_end + 64;

	mode->vsync_start = mode->vdisplay + 12;
	mode->vsync_end = mode->vsync_start + 8;
	mode->vtotal = mode->vsync_end + 12;
#else
	mode->hsync_start = mode->hdisplay + 12;
	mode->hsync_end = mode->hsync_start + 4;
	mode->htotal = mode->hsync_end + 48;

	mode->vsync_start = mode->vdisplay + 10;
	mode->vsync_end = mode->vsync_start + 2;
	mode->vtotal = mode->vsync_end + 10;
#endif
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
void cpt_7_inch_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		pi->width_mm = CPT_7_inch_PANEL_WIDTH;
		pi->height_mm = CPT_7_inch_PANEL_HEIGHT;
	}
}

static
int cpt_7_inch_vid_set_brightness(struct mdfld_dsi_config *dsi_config, int level)
{
	int duty_val = 0;

	duty_val = (0xFF * level) / 100;
	bl_pwm_configure((0xFF - duty_val), level);
	return 0;

}

static
void cpt_7_inch_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");
	/* Virtual channel number */
	int mipi_vc = 0;
	int mipi_pixel_format = 0x4;
	/* BURST_MODE */
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
	hw_ctx->high_low_switch_count = 0x19;//0x18;
	hw_ctx->init_count = 0x7d0;
	hw_ctx->eot_disable = 0x03;
	hw_ctx->lp_byteclk = 0x3;
	hw_ctx->clk_lane_switch_time_cnt = 0x1d000d;//0x18000b;

	hw_ctx->dphy_param = 0x190d310d;//0x2A0D350C;	// 0x150D350C; // 0x1F0F350A;	// 0x15091F06; //0x160d3610;  //0x3F0d3510; 

	/*set up func_prg*/
	//hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;

	/*setup video mode format*/
	hw_ctx->video_mode_format = mipi_mode | ip_tg_config | (1<<3);

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = ((mipi_pixel_format << 7) | (mipi_vc << 3) |
			dsi_config->lane_count);

	/*setup mipi port configuration*/
	hw_ctx->mipi = MIPI_PORT_EN | PASS_FROM_SPHY_TO_AFE |
		 dsi_config->lane_config;

}

static int cpt_7_inch_vid_gpio_init(void)
{
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

//	ret = gpio_request(LCD_RESET, "LCD_RESET"); 
//	if (ret) {
//		printk(" Faild to request panel reset gpio\n");
//	}
	ret = gpio_request(LCD_VCC_3P3, "LCD_3P3"); 
	if (ret) {
		printk(" Faild to request gpio 3p3\n");
	}
	ret = gpio_request(LCD_VCC_1P8, "LCD_1P8"); 
	if (ret) {
		DRM_ERROR(" Faild to request gpio 1p8\n");
	}

//	gpio_set_value_cansleep(LCD_RESET, 0);
//	msleep(10);
	gpio_direction_output(LCD_VCC_3P3, 0);
	msleep(1);
	gpio_direction_output(LCD_VCC_1P8, 0);
	msleep(120);
	return 0;
}


void cpt_7_inch_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");
    
	p_funcs->get_config_mode = cpt_7_inch_vid_get_config_mode;
	p_funcs->get_panel_info = cpt_7_inch_vid_get_panel_info;
	p_funcs->dsi_controller_init = cpt_7_inch_vid_dsi_controller_init;
	p_funcs->detect = cpt_7_inch_vid_detect;
//	p_funcs->reset = mdfld_dsi_cpt_7_inch_panel_reset;
	p_funcs->power_on = cpt_7_inch_vid_power_on;
	p_funcs->power_off = cpt_7_inch_vid_power_off;
	p_funcs->set_brightness = cpt_7_inch_vid_set_brightness;
//	p_funcs->drv_ic_init = cpt_7_inch_vid_ic_init;

	cpt_7_inch_vid_gpio_init();
}
