/*
 * Copyright © 2010 Intel Corporation
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
 */

#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/sfi.h>
#include "psb_drv.h"
#include <asm/intel_scu_pmic.h>

/**
 * set GPIO_MIPI_PANEL_RESET to 57 for CTP VV platform
 * set GPIO_MIPI_PANEL_RESET to 117 (core_gpio[21] for Hydra)
 */
#define GPIO_MIPI_PANEL_RESET 57

struct drm_display_mode *ltl089cl02_vid_get_config_mode(void);
static void ltl089cl02_vid_get_panel_info(int pipe, struct panel_info *pi);
static int mdfld_dsi_ltl089cl02_panel_reset(struct mdfld_dsi_config *dsi_config);
static void
mdfld_dsi_ltl089cl02_dsi_controller_init(struct mdfld_dsi_config *dsi_config);
static int mdfld_dsi_ltl089cl02_detect(struct mdfld_dsi_config *dsi_config);
static int mdfld_dsi_ltl089cl02_power_on(struct mdfld_dsi_config *dsi_config);
static int mdfld_dsi_ltl089cl02_power_off(struct mdfld_dsi_config *dsi_config);
static int mdfld_dsi_ltl089cl02_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level);

void sp_ltl089cl02_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = ltl089cl02_vid_get_config_mode;
	p_funcs->get_panel_info = ltl089cl02_vid_get_panel_info;
	p_funcs->reset = mdfld_dsi_ltl089cl02_panel_reset;
	p_funcs->drv_ic_init = 0;//mdfld_dsi_r63311_drv_ic_init;
	p_funcs->dsi_controller_init = mdfld_dsi_ltl089cl02_dsi_controller_init;
	p_funcs->detect = mdfld_dsi_ltl089cl02_detect;
	p_funcs->power_on = mdfld_dsi_ltl089cl02_power_on;
	p_funcs->power_off = mdfld_dsi_ltl089cl02_power_off;
	p_funcs->set_brightness = mdfld_dsi_ltl089cl02_set_brightness;
}


struct drm_display_mode *ltl089cl02_vid_get_config_mode(void)
{
pr_info("ltl get_config_mode\n");
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->type |= DRM_MODE_TYPE_PREFERRED;// verified ok addr 0x38
	mode->hdisplay = 1920;// verified ok addr 0x40
	mode->vdisplay = 1200;//verified ok addr 0x54
	mode->hsync_start = 0x7f0;//verified ok addr 0x44
	mode->hsync_end = 0x800;//verified ok addr 0x48
	mode->htotal = 0x820;//verified ok addr 0x4c
	mode->vsync_start = 0x4c1;//verified ok addr 0x58
	mode->vsync_end = 0x4c3;//verified ok 0x5c
	mode->vtotal = 0x4d3;//verified ok addr 0x60
	mode->vrefresh = 0x3a;//ok
	mode->clock = 0x245fe;//verified ok addr 0x3c //mode->vrefresh * mode->vtotal *
		//mode->htotal / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void ltl089cl02_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	if (pipe == 0) {
		pi->width_mm = 0xc0;//verified ok was TMD_PANEL_WIDTH;
		pi->height_mm =0x78;//verified ok was TMD_PANEL_HEIGHT;
	}
}

static void
mdfld_dsi_ltl089cl02_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	dsi_config->lane_count = 4; // ok addr 0x694
	dsi_config->bpp = 0x18;//verified ok addr 0x68c
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;// = 0 verified ok addr 0x698

	/* This is for 400 mhz.  Set it to 0 for 800mhz */
	hw_ctx->cck_div = 1; // ver ok 0x66c
	hw_ctx->pll_bypass_mode = 0;//ver ok 0x668
	hw_ctx->mipi_control = 0x18;//verified ok 0x64c
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;// from asm; was 0xffff;
	hw_ctx->turn_around_timeout = 0x1f;//
	hw_ctx->device_reset_timer = 0xffff ;
	hw_ctx->high_low_switch_count = 0x2c;//from asm; was 0x35;
	hw_ctx->init_count = 0xf0;//from asm; was 0x7d0;
	hw_ctx->eot_disable = 0;//from asm; was 0x30;
	hw_ctx->lp_byteclk = 0x6;
	hw_ctx->clk_lane_switch_time_cnt = 0x2c0015;//from asm; was 0x2B0014;
	hw_ctx->dphy_param = 0x301b741f;//from asm; was 0x2A18681F;
	hw_ctx->video_mode_format = 0xf;//ok

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = 0x184;//from asm; was (0x200 | dsi_config->lane_count);

	/*setup mipi port configuration*/
	hw_ctx->mipi = 0x2010000;//from asm; was PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;
}


static int mdfld_dsi_ltl089cl02_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int ret=mdfld_dsi_send_dpi_spk_pkg_lp(sender, 2);
	
	msleep(0x64);
        gpio_direction_output(0x67,1);
	msleep(5);
	
	return ret;
}

static int mdfld_dsi_ltl089cl02_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
        gpio_direction_output(0x67,0);
	msleep(5);
        gpio_direction_output(0x56,0);
	msleep(0x12c);
	mdfld_dsi_send_dpi_spk_pkg_hs(sender, 1);
        msleep(0x64);        
	return 0;
}


static int mdfld_dsi_ltl089cl02_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
	int pipe = dsi_config->pipe;

	PSB_DEBUG_ENTRY("\n");
pr_info("ltl detect\n");
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
pr_info("ltl detect en vblank\n");
                psb_enable_vblank(dev, 0);
		} //else {
			dsi_config->dsi_hw_context.panel_on = false;
		//	DRM_INFO("%s: panel is not detected!\n", __func__);
		//}

		status = MDFLD_DSI_PANEL_CONNECTED;
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static int mdfld_dsi_ltl089cl02_panel_reset(struct mdfld_dsi_config *dsi_config)
{
        gpio_direction_output(0x56,1);
	msleep(0x104);
        return 0;
}

static int mdfld_dsi_ltl089cl02_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
        if (level < 0) level=0;
        if (level > 100) level=100;
	level *= 60;
	level /= 100;
	intel_scu_ipc_iowrite8(0x67, level);//todo
	return 0;
}

static int sp_ltl089cl02_lcd_vid_probe(struct platform_device *pdev)
{
	DRM_INFO("%s: LTL089CL02 panel detected\n", __func__);

	int ret=gpio_request(0x67, "LTL089CL02 Backlight Enable");
        gpio_export(0x67,0);
	lnw_gpio_set_alt(0x67,0);


	ret=gpio_request(0x56, "LTL089CL02 VDD Enable");
        gpio_export(0x56,0);
	lnw_gpio_set_alt(0x56,0);
        
	u8 val=0;
        ret=intel_scu_ipc_ioread8(0x38, &val);
	if (ret==0) {
	  if (val!=1) {
	    ret=intel_scu_ipc_iowrite8(0x38,1);
	    if (ret) goto err;
	  }
	}
	intel_scu_ipc_iowrite8(0x62,8);
	intel_scu_ipc_iowrite8(0x61,0);
err:
	intel_mid_panel_register(sp_ltl089cl02_vid_init);

	return 0;
}

struct platform_driver sp_ltl089cl02_lcd_driver = {
	.probe	= sp_ltl089cl02_lcd_vid_probe,
	.driver	= {
		.name	= "LTL089CL02",
		.owner	= THIS_MODULE,
	},
};
