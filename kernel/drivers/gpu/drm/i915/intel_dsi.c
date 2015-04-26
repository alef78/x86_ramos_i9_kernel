/*
 * Copyright © 2013 Intel Corporation
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
 * Author: Jani Nikula <jani.nikula@intel.com>
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/i915_drm.h>
#include <linux/slab.h>
#ifdef CONFIG_CRYSTAL_COVE
#include "linux/mfd/intel_mid_pmic.h"
#endif
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"
#include "intel_dsi_pll.h"

/* the sub-encoders aka panel drivers */
static const struct intel_dsi_device intel_dsi_devices[] = {
	{
		.panel_id = MIPI_DSI_GENERIC_PANEL_ID,
		.type = INTEL_DSI_VIDEO_MODE,
		.name = "vbt-generic-dsi-vid-mode-display",
		.dev_ops = &vbt_generic_dsi_display_ops,
	},
	{
		.panel_id = MIPI_DSI_AUO_B101UAN01_PANEL_ID,
		.type = INTEL_DSI_VIDEO_MODE,
		.name = "auo-b101uan01-dsi-vid-mode-display",
		.dev_ops = &auo_b101uan01_dsi_display_ops,
	},
	{
		.panel_id = MIPI_DSI_AUO_B080XAT_PANEL_ID,
		.type = INTEL_DSI_VIDEO_MODE,
		.name = "auo-b080xat-dsi-vid-mode-display",
		.dev_ops = &auo_b080xat_dsi_display_ops,
	},
	{
		.panel_id = MIPI_DSI_PANASONIC_VXX09F006A00_PANEL_ID,
		.type = INTEL_DSI_VIDEO_MODE,
		.name = "auo-panasonic-dsi-vid-mode-display",
		.dev_ops = &panasonic_vvx09f006a00_dsi_display_ops,
	},
	{
		.panel_id = MIPI_DSI_JDI_LPM070W425B_PANEL_ID,
		.type = INTEL_DSI_VIDEO_MODE,
		.name = "jdi-lpm070w425b-dsi-vid-mode-display",
		.dev_ops = &jdi_lpm070w425b_dsi_display_ops,
	},
};

static struct intel_dsi *intel_attached_dsi(struct drm_connector *connector)
{
	return container_of(intel_attached_encoder(connector),
			    struct intel_dsi, base);
}

static inline bool is_vid_mode(struct intel_dsi *intel_dsi)
{
	return intel_dsi->dev.type == INTEL_DSI_VIDEO_MODE;
}

static inline bool is_cmd_mode(struct intel_dsi *intel_dsi)
{
	return intel_dsi->dev.type == INTEL_DSI_COMMAND_MODE;
}

static void intel_dsi_hot_plug(struct intel_encoder *encoder)
{
	DRM_DEBUG_KMS("\n");
}

static bool intel_dsi_compute_config(struct intel_encoder *encoder,
				     struct intel_crtc_config *config)
{
	struct intel_dsi *intel_dsi = container_of(encoder, struct intel_dsi,
						   base);
	struct intel_connector *intel_connector = intel_dsi->attached_connector;
	struct drm_display_mode *fixed_mode = intel_connector->panel.fixed_mode;
	struct drm_display_mode *adjusted_mode = &config->adjusted_mode;
	struct drm_display_mode *mode = &config->requested_mode;

	DRM_DEBUG_KMS("\n");

	if (fixed_mode)
		intel_fixed_panel_mode(fixed_mode, adjusted_mode);

	if (intel_dsi->dev.dev_ops->mode_fixup)
		return intel_dsi->dev.dev_ops->mode_fixup(&intel_dsi->dev,
							  mode, adjusted_mode);

	return true;
}

static void intel_dsi_pre_pll_enable(struct intel_encoder *encoder)
{
	DRM_DEBUG_KMS("\n");

	/* nothing to do here as we do necessary stuff in pre_enable
	 * to comply to the hw team recommended DSI sequence */
}

static void band_gap_reset(struct drm_i915_private *dev_priv)
{
	mutex_lock(&dev_priv->dpio_lock);

	intel_flisdsi_write32(dev_priv, 0x08, 0x0001);
	intel_flisdsi_write32(dev_priv, 0x0F, 0x0005);
	intel_flisdsi_write32(dev_priv, 0x0F, 0x0025);
	udelay(150);
	intel_flisdsi_write32(dev_priv, 0x0F, 0x0000);
	intel_flisdsi_write32(dev_priv, 0x08, 0x0000);

	mutex_unlock(&dev_priv->dpio_lock);
}

void intel_dsi_device_ready(struct intel_encoder *encoder)
{
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);
	int pipe = intel_crtc->pipe;
	u32 tmp;

	DRM_DEBUG_KMS("\n");

	/* program rcomp for compliance
	 * reduce form 50 ohms to 45 ohms */
	intel_flisdsi_write32(dev_priv, 0x04, 0x0004);

	band_gap_reset(dev_priv);

#ifdef CONFIG_CRYSTAL_COVE
	/* Panel Enable */
	if (BYT_CR_CONFIG) {
		/*  cabc disable */
		vlv_gpio_nc_write(dev_priv, GPIO_NC_9_PCONF0, 0x2000CC00);
		vlv_gpio_nc_write(dev_priv, GPIO_NC_9_PAD, 0x00000004);

		/* panel enable */
		vlv_gpio_nc_write(dev_priv, GPIO_NC_11_PCONF0, 0x2000CC00);
		vlv_gpio_nc_write(dev_priv, GPIO_NC_11_PAD, 0x00000005);
		udelay(500);
	} else
		intel_mid_pmic_writeb(PMIC_PANEL_EN, 0x01);
#else
	/* need to code for BYT-CR for example where things have changed */
	DRM_ERROR("PANEL Enable to supported yet\n");
#endif
	msleep(intel_dsi->panel_on_delay);

	if (intel_dsi->dev.dev_ops->panel_reset)
		intel_dsi->dev.dev_ops->panel_reset(&intel_dsi->dev);

	/* Disable DPOunit clock gating, can stall pipe */
	tmp = I915_READ(DPLL(pipe));
	tmp |= DPLL_RESERVED_BIT;
	I915_WRITE(DPLL(pipe), tmp);

	tmp = I915_READ(DSPCLK_GATE_D);
	tmp |= VSUNIT_CLOCK_GATE_DISABLE;
	I915_WRITE(DSPCLK_GATE_D, tmp);

	intel_enable_dsi_pll(intel_dsi);

	I915_WRITE_BITS(MIPI_PORT_CTRL(pipe), LP_OUTPUT_HOLD,
						LP_OUTPUT_HOLD);

	usleep_range(1000, 1500);
	I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), DEVICE_READY |
			ULPS_STATE_EXIT, DEVICE_READY |
			ULPS_STATE_MASK);

	usleep_range(2000, 2500);
	I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), DEVICE_READY,
			DEVICE_READY | ULPS_STATE_MASK);
	usleep_range(2000, 2500);
	I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), 0x00,
			DEVICE_READY | ULPS_STATE_MASK);
	usleep_range(2000, 2500);
	I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), DEVICE_READY,
			DEVICE_READY | ULPS_STATE_MASK);
	usleep_range(2000, 2500);
}

static void intel_dsi_pre_enable(struct intel_encoder *encoder)
{
	DRM_DEBUG_KMS("\n");

	/* put device in ready state */
	intel_dsi_device_ready(encoder);
}

static void intel_dsi_enable(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);
	int pipe = intel_crtc->pipe;
	bool is_dsi;
	u32 temp;

	DRM_DEBUG_KMS("\n");

	is_dsi = intel_pipe_has_type(encoder->base.crtc, INTEL_OUTPUT_DSI);

	intel_enable_dsi_pll(intel_dsi);

	if (is_cmd_mode(intel_dsi)) {
		/* XXX: Implement me */
		I915_WRITE(MIPI_MAX_RETURN_PKT_SIZE(pipe), 8 * 4);
	}
	else {
		intel_dsi->hs = 0;
		dpi_send_cmd(intel_dsi, TURN_ON);
		usleep_range(1000, 1500);

		if (intel_dsi->dev.dev_ops->enable)
			intel_dsi->dev.dev_ops->enable(&intel_dsi->dev);

		temp = I915_READ(MIPI_PORT_CTRL(pipe));
		temp = temp | intel_dsi->port_bits;

		if (is_dsi && intel_crtc->config.dither)
			temp |= DITHERING_ENABLE;
		I915_WRITE(MIPI_PORT_CTRL(pipe), temp | DPI_ENABLE);
		usleep_range(2000, 2500);
	}

	/* Adjust backlight timing for specific panel */
	if (intel_dsi->backlight_on_delay >= 20)
		msleep(intel_dsi->backlight_on_delay);
	else
		usleep_range(intel_dsi->backlight_on_delay * 1000,
			(intel_dsi->backlight_on_delay * 1000) + 500);

	intel_panel_enable_backlight(dev, pipe);
}

static void intel_dsi_disable(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);
	int pipe = intel_crtc->pipe;
	u32 tmp;

	DRM_DEBUG_KMS("\n");

	intel_panel_disable_backlight(dev);
	if (intel_dsi->backlight_off_delay >= 20)
		msleep(intel_dsi->backlight_off_delay);
	else
		usleep_range(intel_dsi->backlight_off_delay * 1000,
			(intel_dsi->backlight_off_delay * 1000) + 500);

	if (is_cmd_mode(intel_dsi)) {
		/* XXX Impementation TBD */
	} else {
		/* Send Shutdown command to the panel in LP mode */
		intel_dsi->hs = 0;
		dpi_send_cmd(intel_dsi, SHUTDOWN);
		usleep_range(1000, 1500);

		I915_WRITE_BITS(MIPI_PORT_CTRL(pipe), 0, DPI_ENABLE);
		POSTING_READ(MIPI_PORT_CTRL(pipe));
		usleep_range(2000, 2500);
	}

	/* Panel commands can be sent when clock is in LP11 */
	I915_WRITE(MIPI_DEVICE_READY(pipe), 0x0);

	tmp = I915_READ(MIPI_CTRL(pipe));
	tmp &= ~ESCAPE_CLOCK_DIVIDER_MASK;
	I915_WRITE(MIPI_CTRL(pipe), tmp |
			intel_dsi->escape_clk_div <<
			ESCAPE_CLOCK_DIVIDER_SHIFT);

	I915_WRITE(MIPI_EOT_DISABLE(pipe), CLOCKSTOP);

	tmp = I915_READ(MIPI_DSI_FUNC_PRG(pipe));
	tmp &= ~VID_MODE_FORMAT_MASK;
	I915_WRITE(MIPI_DSI_FUNC_PRG(pipe), tmp);

	I915_WRITE(MIPI_DEVICE_READY(pipe), 0x1);

	/* if disable packets are sent before sending shutdown packet then in
	 * some next enable sequence send turn on packet error is observed */
	if (intel_dsi->dev.dev_ops->disable)
		intel_dsi->dev.dev_ops->disable(&intel_dsi->dev);
}

void intel_dsi_clear_device_ready(struct intel_encoder *encoder)
{
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);
	int pipe = intel_crtc->pipe;
	u32 val;

	DRM_DEBUG_KMS("\n");

	I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), ULPS_STATE_ENTER,
							ULPS_STATE_MASK);
	usleep_range(2000, 2500);

	I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), ULPS_STATE_EXIT,
							ULPS_STATE_MASK);
	usleep_range(2000, 2500);

	I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), ULPS_STATE_ENTER,
							ULPS_STATE_MASK);
	usleep_range(2000, 2500);

	I915_WRITE_BITS(MIPI_PORT_CTRL(pipe), 0, LP_OUTPUT_HOLD);
	usleep_range(1000, 1500);

	if (wait_for(((I915_READ(MIPI_PORT_CTRL(pipe)) & 0x20000)
					== 0x00000), 30))
		DRM_ERROR("DSI LP not going Low\n");

	I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), 0x00, DEVICE_READY);
	usleep_range(2000, 2500);

	intel_disable_dsi_pll(intel_dsi);

	val = I915_READ(DSPCLK_GATE_D);
	val &= ~VSUNIT_CLOCK_GATE_DISABLE;
	I915_WRITE(DSPCLK_GATE_D, val);

	if (intel_dsi->dev.dev_ops->disable_panel_power)
		intel_dsi->dev.dev_ops->disable_panel_power(&intel_dsi->dev);

#ifdef CONFIG_CRYSTAL_COVE
	if (BYT_CR_CONFIG) {
		/* Disable Panel */
		vlv_gpio_nc_write(dev_priv, GPIO_NC_11_PCONF0, 0x2000CC00);
		vlv_gpio_nc_write(dev_priv, GPIO_NC_11_PAD, 0x00000004);
		udelay(500);
	} else
		intel_mid_pmic_writeb(PMIC_PANEL_EN, 0x00);
#else
	/* need to code for BYT-CR for example where things have changed */
	DRM_ERROR("PANEL Disable to supported yet\n");
#endif
	msleep(intel_dsi->panel_off_delay);
	msleep(intel_dsi->panel_pwr_cycle_delay);
}

static void intel_dsi_post_disable(struct intel_encoder *encoder)
{
	DRM_DEBUG_KMS("\n");

	intel_dsi_clear_device_ready(encoder);
}

static bool intel_dsi_get_hw_state(struct intel_encoder *encoder,
				   enum pipe *pipe)
{
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	u32 port, func;
	enum pipe p;

	DRM_DEBUG_KMS("\n");

	/* XXX: this only works for one DSI output */
	for (p = PIPE_A; p <= PIPE_B; p++) {
		port = I915_READ(MIPI_PORT_CTRL(p));
		func = I915_READ(MIPI_DSI_FUNC_PRG(p));

		if ((port & DPI_ENABLE) || (func & CMD_MODE_DATA_WIDTH_MASK)) {
			if (I915_READ(MIPI_DEVICE_READY(p)) & DEVICE_READY) {
				*pipe = p;
				return true;
			}
		}
	}

	return false;
}

static void intel_dsi_get_config(struct intel_encoder *encoder,
				 struct intel_crtc_config *pipe_config)
{
	DRM_DEBUG_KMS("\n");

	/* XXX: read flags, set to adjusted_mode */
}

static int intel_dsi_mode_valid(struct drm_connector *connector,
				struct drm_display_mode *mode)
{
	struct intel_connector *intel_connector = to_intel_connector(connector);
	struct drm_display_mode *fixed_mode = intel_connector->panel.fixed_mode;
	struct intel_dsi *intel_dsi = intel_attached_dsi(connector);

	DRM_DEBUG_KMS("\n");

	if (mode->flags & DRM_MODE_FLAG_DBLSCAN) {
		DRM_DEBUG_KMS("MODE_NO_DBLESCAN\n");
		return MODE_NO_DBLESCAN;
	}

	if (fixed_mode) {
		if (mode->hdisplay > fixed_mode->hdisplay)
			return MODE_PANEL;
		if (mode->vdisplay > fixed_mode->vdisplay)
			return MODE_PANEL;
	}

	return intel_dsi->dev.dev_ops->mode_valid(&intel_dsi->dev, mode);
}


/* return pixels in terms of txbyteclkhs */
static u32 txbyteclkhs(u32 pixels, int bpp, int lane_count)
{
	return DIV_ROUND_UP(pixels * bpp, 8 * lane_count);
}

static void set_dsi_timings(struct drm_encoder *encoder,
			    const struct drm_display_mode *mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(encoder);
	int pipe = intel_crtc->pipe;
	unsigned int bpp = intel_crtc->config.pipe_bpp;
	unsigned int lane_count = intel_dsi->lane_count;

	u16 hactive, hfp, hsync, hbp, vfp, vsync, vbp;

	hactive = mode->hdisplay;

	hfp = mode->hsync_start - mode->hdisplay;
	hsync = mode->hsync_end - mode->hsync_start;
	hbp = mode->htotal - mode->hsync_end;

	vfp = mode->vsync_start - mode->vdisplay;
	vsync = mode->vsync_end - mode->vsync_start;
	vbp = mode->vtotal - mode->vsync_end;

	/* horizontal values are in terms of high speed byte clock */
	hactive = txbyteclkhs(hactive, bpp, lane_count);
	hfp = txbyteclkhs(hfp, bpp, lane_count);
	hsync = txbyteclkhs(hsync, bpp, lane_count);
	hbp = txbyteclkhs(hbp, bpp, lane_count);

	/* FIXME: Find better way to do this */
	/* For 7x10 panel we need to have BLLP added to active */
	/* Trying to find optimal BLLP Multiplier */
	/*	2.875 - Original multiplier, Works with flicker */
	/*	2.000 - works but still some flicker */
	/*	1.500 - Works, No Flicker */
	/*	1.250 - Works, No Flicker */
	/*	1.100 - Doesn't work */
	/* FIXME: Acer Mango spec requires to run the DSI clock at 500 to
	 * 560Mbps. Recomendation is to run at 513 Mbps. The addition dsi
	 * clock is to be filled with NULL packets. Refer to acer panel
	 * spec for more details.
	 */
	if (dev_priv->mipi_panel_id == MIPI_DSI_AUO_B080XAT_PANEL_ID)
		hactive = (hactive * 10) / 8;

	I915_WRITE(MIPI_HACTIVE_AREA_COUNT(pipe), hactive);
	I915_WRITE(MIPI_HFP_COUNT(pipe), hfp);

	/* meaningful for video mode non-burst sync pulse mode only, can be zero
	 * for non-burst sync events and burst modes */
	I915_WRITE(MIPI_HSYNC_PADDING_COUNT(pipe), hsync);
	I915_WRITE(MIPI_HBP_COUNT(pipe), hbp);

	/* vertical values are in terms of lines */
	I915_WRITE(MIPI_VFP_COUNT(pipe), vfp);
	I915_WRITE(MIPI_VSYNC_PADDING_COUNT(pipe), vsync);
	I915_WRITE(MIPI_VBP_COUNT(pipe), vbp);
}

static void dsi_config(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(encoder);
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	int pipe = intel_crtc->pipe;
	u32 tmp;

	DRM_DEBUG_KMS("\n");

	/* escape clock divider, 20MHz, shared for A and C. device ready must be
	 * off when doing this! txclkesc? */
	tmp = I915_READ(MIPI_CTRL(0));
	tmp &= ~ESCAPE_CLOCK_DIVIDER_MASK;
	I915_WRITE(MIPI_CTRL(0), tmp | ESCAPE_CLOCK_DIVIDER_1);

	/* read request priority is per pipe */
	tmp = I915_READ(MIPI_CTRL(pipe));
	tmp &= ~READ_REQUEST_PRIORITY_MASK;
	I915_WRITE(MIPI_CTRL(pipe), tmp | READ_REQUEST_PRIORITY_HIGH);

	/* XXX: why here, why like this? handling in irq handler?! */
	I915_WRITE(MIPI_INTR_EN(pipe), 0xffffffff);

	/* why here, was elsewhere... also 2a, 0c, 60, 08 for values */
	I915_WRITE(MIPI_DPHY_PARAM(pipe), intel_dsi->dphy_reg);
}

static void intel_dsi_mode_set(struct intel_encoder *intel_encoder)
{
	struct drm_encoder *encoder = &intel_encoder->base;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(encoder);
	int pipe = intel_crtc->pipe;
	unsigned int bpp = intel_crtc->config.pipe_bpp;
	struct drm_display_mode *adjusted_mode = &intel_crtc->config.adjusted_mode;
	u32 val;

	I915_WRITE(MIPI_DEVICE_READY(pipe), 0x0);

	dsi_config(encoder);

	I915_WRITE(MIPI_LP_RX_TIMEOUT(pipe), intel_dsi->lp_rx_timeout);
	I915_WRITE(MIPI_TURN_AROUND_TIMEOUT(pipe),
					intel_dsi->turn_arnd_val);
	I915_WRITE(MIPI_DEVICE_RESET_TIMER(pipe),
					intel_dsi->rst_timer_val);
	/* in terms of low power clock */
	I915_WRITE(MIPI_INIT_COUNT(pipe), intel_dsi->init_count);

	I915_WRITE(MIPI_HIGH_LOW_SWITCH_COUNT(pipe), \
					intel_dsi->hs_to_lp_count);
	I915_WRITE(MIPI_LP_BYTECLK(pipe), intel_dsi->lp_byte_clk);

	I915_WRITE(MIPI_CLK_LANE_SWITCH_TIME_CNT(pipe),
		((u32)intel_dsi->clk_lp_to_hs_count
		<< LP_HS_SSW_CNT_SHIFT) |
		(intel_dsi->clk_hs_to_lp_count << HS_LP_PWR_SW_CNT_SHIFT));

	if (is_vid_mode(intel_dsi)) {
		I915_WRITE(MIPI_DPI_RESOLUTION(pipe),
			(adjusted_mode->vdisplay << VERTICAL_ADDRESS_SHIFT) |
			(adjusted_mode->hdisplay << HORIZONTAL_ADDRESS_SHIFT));

		set_dsi_timings(encoder, adjusted_mode);

		if (intel_dsi->video_mode_type == DSI_VIDEO_BURST) {
			I915_WRITE(MIPI_HS_TX_TIMEOUT(pipe),
				txbyteclkhs(adjusted_mode->htotal, bpp,
				intel_dsi->lane_count) + 1);
		}
		else {
			I915_WRITE(MIPI_HS_TX_TIMEOUT(pipe),
			   txbyteclkhs(adjusted_mode->vtotal *
				   adjusted_mode->htotal,
				   bpp, intel_dsi->lane_count) + 1);
		}
	} else {
		val = intel_dsi->channel << CMD_MODE_CHANNEL_NUMBER_SHIFT |
			intel_dsi->lane_count << DATA_LANES_PRG_REG_SHIFT |
			intel_dsi->data_width;
		I915_WRITE(MIPI_DSI_FUNC_PRG(pipe), val);

		I915_WRITE(MIPI_HS_TX_TIMEOUT(pipe),
			txbyteclkhs(adjusted_mode->hdisplay *
			adjusted_mode->vdisplay,
			bpp, intel_dsi->lane_count) + 1);

		I915_WRITE(MIPI_DBI_BW_CTRL(pipe), intel_dsi->bw_timer);
	}

	I915_WRITE(MIPI_EOT_DISABLE(pipe), CLOCKSTOP);

	val = I915_READ(MIPI_DSI_FUNC_PRG(pipe));
	val &= ~VID_MODE_FORMAT_MASK;
	I915_WRITE(MIPI_DSI_FUNC_PRG(pipe), val);

	I915_WRITE(MIPI_DEVICE_READY(pipe), 0x1);

	if (intel_dsi->dev.dev_ops->send_otp_cmds)
		intel_dsi->dev.dev_ops->send_otp_cmds(&intel_dsi->dev);

	I915_WRITE(MIPI_DEVICE_READY(pipe), 0x0);

	set_dsi_timings(encoder, adjusted_mode);

	/* Some panels might have resolution which is not a multiple of
	 * 64 like 1366 x 768. Enable RANDOM resolution support for such
	 * panels by default */
	I915_WRITE(MIPI_VIDEO_MODE_FORMAT(pipe),
				intel_dsi->video_frmt_cfg_bits |
				intel_dsi->video_mode_type |
				IP_TG_CONFIG |
				RANDOM_DPI_DISPLAY_RESOLUTION);

	val = 0;
	if (intel_dsi->eotp_pkt == 0)
		val |= EOT_DISABLE;

	if (intel_dsi->clock_stop)
		val |= CLOCKSTOP;

	I915_WRITE(MIPI_EOT_DISABLE(pipe), val);

	val = intel_dsi->channel << VID_MODE_CHANNEL_NUMBER_SHIFT |
		intel_dsi->lane_count << DATA_LANES_PRG_REG_SHIFT |
		intel_dsi->pixel_format;
	I915_WRITE(MIPI_DSI_FUNC_PRG(pipe), val);

	I915_WRITE(MIPI_DEVICE_READY(pipe), 0x1);

	I915_WRITE(MIPI_INTR_STAT(pipe), 0xFFFFFFFF);
	/* Max packet return size limits the size of returning
	* packet so that host processor can prevent buffer overflow
	* condition when receiving data from peripheral. DCS read
	* need this to be set.*/
	I915_WRITE(MIPI_MAX_RETURN_PKT_SIZE(pipe), 0xff);

	if ((adjusted_mode->hdisplay < PFIT_SIZE_LIMIT) &&
	(adjusted_mode->vdisplay < PFIT_SIZE_LIMIT)) {
		/* BYT-CR needs panel fitter only with Panasonic panel */
		if (BYT_CR_CONFIG && (i915_mipi_panel_id ==
			MIPI_DSI_PANASONIC_VXX09F006A00_PANEL_ID)) {
			val = PFIT_ENABLE | (intel_crtc->pipe << PFIT_PIPE_SHIFT) |
					PFIT_SCALING_AUTO;
			I915_WRITE(PFIT_CONTROL, val);
			intel_crtc->base.panning_en = true;
		} else if (intel_dsi->pfit) {
			/* Normal scenario, enable panel fitter only if the
			scaling ratio is > 1 and the input src size < 2kx2k */
			if ((adjusted_mode->hdisplay != intel_crtc->base.fb->width) ||
			(adjusted_mode->vdisplay != intel_crtc->base.fb->height)) {
				if (intel_dsi->pfit == AUTOSCALE)
					val = PFIT_ENABLE | (intel_crtc->pipe <<
						PFIT_PIPE_SHIFT) | PFIT_SCALING_AUTO;
				if (intel_dsi->pfit == PILLARBOX)
					val = PFIT_ENABLE | (intel_crtc->pipe <<
						PFIT_PIPE_SHIFT) | PFIT_SCALING_PILLAR;
				else if (intel_dsi->pfit == LETTERBOX)
					val = PFIT_ENABLE | (intel_crtc->pipe <<
						PFIT_PIPE_SHIFT) | PFIT_SCALING_LETTER;
				I915_WRITE(PFIT_CONTROL, val);
				intel_crtc->base.panning_en = true;
				DRM_DEBUG_DRIVER("pfit val = %x", val);
			} else
				DRM_DEBUG_DRIVER("Wrong pfit input src config");
		} else
			intel_crtc->base.panning_en = false;

	} else
		DRM_DEBUG_DRIVER("Wrong pfit input src config");

	intel_crtc->config.gmch_pfit.control = val;
}

static enum drm_connector_status
intel_dsi_detect(struct drm_connector *connector, bool force)
{
	struct intel_dsi *intel_dsi = intel_attached_dsi(connector);
	DRM_DEBUG_KMS("\n");

	return intel_dsi->dev.dev_ops->detect(&intel_dsi->dev);
}

static int intel_dsi_get_modes(struct drm_connector *connector)
{
	struct intel_connector *intel_connector = to_intel_connector(connector);
	struct intel_dsi *intel_dsi = intel_attached_dsi(connector);
	struct drm_display_mode *mode;
	struct drm_display_mode *input_mode = NULL;
	DRM_DEBUG_KMS("\n");

	if (!intel_connector->panel.fixed_mode) {
		DRM_DEBUG_KMS("no fixed mode\n");
		return 0;
	}

	input_mode = intel_connector->panel.fixed_mode;
	mode = drm_mode_duplicate(connector->dev,
				  input_mode);
	if (!mode) {
		DRM_DEBUG_KMS("drm_mode_duplicate failed\n");
		return 0;
	}

	drm_mode_probed_add(connector, mode);
	/*Fill the panel info here*/
	intel_dsi->dev.dev_ops->get_info(0, connector);
	return 1;
}

static void intel_dsi_destroy(struct drm_connector *connector)
{
	struct intel_connector *intel_connector = to_intel_connector(connector);

	DRM_DEBUG_KMS("\n");
	intel_panel_fini(&intel_connector->panel);
	intel_panel_destroy_backlight(connector->dev);
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(connector);
}

static int intel_dsi_set_property(struct drm_connector *connector,
		struct drm_property *property,
		uint64_t value)
{
	struct intel_dsi *intel_dsi = intel_attached_dsi(connector);
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	int ret;

	ret = drm_object_property_set_value(&connector->base, property, value);
	if (ret)
		return ret;

	if (property == dev_priv->force_pfit_property) {
		if (value == intel_dsi->pfit)
			return 0;
		DRM_DEBUG_DRIVER("val = %d", (int)value);
		intel_dsi->pfit = value;
	}

	return 0;
}

/* Encoder dpms must, add functionality later */
void intel_dsi_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(encoder);
	struct intel_encoder *intel_encoder = &intel_dsi->base;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	int pipe = intel_crtc->pipe;
	u32 val;

	DRM_DEBUG_KMS("\n");

	if (dev_priv->mipi_fw) {
		/* IAFW enabled MIPI; we do not need to
		 * do anything for this dpms call
		 *
		 * Next enabling will be done by driver
		 * so clear the mmipi_fw flag
		 */
		dev_priv->mipi_fw = 0;
		DRM_DEBUG_KMS("FW enabled MIPI nothing to do\n");
		return;
	}

	if (mode != DRM_MODE_DPMS_ON)
		mode = DRM_MODE_DPMS_OFF;

	DRM_DEBUG_DRIVER("DPMS mode = %d\n", mode);

	if (mode == DRM_MODE_DPMS_ON) {
		intel_dsi_device_ready(intel_encoder);

		/* need to resend OTP as panel off was done in
		 * DPMS off */

		/* Clock needs to be in LP11 mode before we can send
		 * commands to panel */

		I915_WRITE(MIPI_DEVICE_READY(pipe), 0x0);
		I915_WRITE(MIPI_EOT_DISABLE(pipe), CLOCKSTOP);

		val = I915_READ(MIPI_DSI_FUNC_PRG(pipe));
		val &= ~VID_MODE_FORMAT_MASK;
		I915_WRITE(MIPI_DSI_FUNC_PRG(pipe), val);

		I915_WRITE(MIPI_DEVICE_READY(pipe), 0x1);

		if (intel_dsi->dev.dev_ops->send_otp_cmds)
			intel_dsi->dev.dev_ops->send_otp_cmds(&intel_dsi->dev);

		/* Now we need to restore MIPI_DSI_FUNC_PRG to needed value */
		I915_WRITE(MIPI_DEVICE_READY(pipe), 0x0);

		val = intel_dsi->channel << VID_MODE_CHANNEL_NUMBER_SHIFT |
		intel_dsi->lane_count << DATA_LANES_PRG_REG_SHIFT |
		intel_dsi->pixel_format;
		I915_WRITE(MIPI_DSI_FUNC_PRG(pipe), val);

		I915_WRITE(MIPI_DEVICE_READY(pipe), 0x1);
	}
}

static const struct drm_encoder_funcs intel_dsi_funcs = {
	.destroy = intel_encoder_destroy,
};

static const struct drm_encoder_helper_funcs intel_dsi_helper_funcs = {
	.dpms = intel_dsi_encoder_dpms,
};

static const struct drm_connector_helper_funcs intel_dsi_connector_helper_funcs = {
	.get_modes = intel_dsi_get_modes,
	.mode_valid = intel_dsi_mode_valid,
	.best_encoder = intel_best_encoder,
};

static const struct drm_connector_funcs intel_dsi_connector_funcs = {
	.dpms = intel_connector_dpms,
	.detect = intel_dsi_detect,
	.destroy = intel_dsi_destroy,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.set_property = intel_dsi_set_property,
};

static void
intel_dsi_add_properties(struct intel_dsi *intel_dsi,
				struct drm_connector *connector)
{
	intel_attach_force_pfit_property(connector);
}

bool intel_dsi_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_dsi *intel_dsi;
	struct intel_encoder *intel_encoder;
	struct drm_encoder *encoder;
	struct intel_connector *intel_connector;
	struct drm_connector *connector;
	struct drm_display_mode *fixed_mode = NULL;
	const struct intel_dsi_device *dsi;
	unsigned int i;

	DRM_DEBUG_KMS("\n");

	intel_dsi = kzalloc(sizeof(*intel_dsi), GFP_KERNEL);
	if (!intel_dsi)
		return false;
	intel_dsi->pfit = 0;

	intel_connector = kzalloc(sizeof(*intel_connector), GFP_KERNEL);
	if (!intel_connector) {
		kfree(intel_dsi);
		return false;
	}

	intel_encoder = &intel_dsi->base;
	encoder = &intel_encoder->base;
	intel_dsi->attached_connector = intel_connector;

	connector = &intel_connector->base;

	drm_encoder_init(dev, encoder, &intel_dsi_funcs, DRM_MODE_ENCODER_DSI);

	/* XXX: very likely not all of these are needed */
	intel_encoder->hot_plug = intel_dsi_hot_plug;
	intel_encoder->compute_config = intel_dsi_compute_config;
	intel_encoder->pre_pll_enable = intel_dsi_pre_pll_enable;
	intel_encoder->pre_enable = intel_dsi_pre_enable;
	intel_encoder->enable = intel_dsi_enable;
	intel_encoder->mode_set = intel_dsi_mode_set;
	intel_encoder->disable = intel_dsi_disable;
	intel_encoder->post_disable = intel_dsi_post_disable;
	intel_encoder->get_hw_state = intel_dsi_get_hw_state;
	intel_encoder->get_config = intel_dsi_get_config;

	intel_connector->get_hw_state = intel_connector_get_hw_state;

	/* Initialize panel id based on kernel param.
	 * If no kernel param use panel id from VBT
	 * If no  param and no VBT initialize with
	 * default ASUS panel ID for now */
	if (i915_mipi_panel_id <= 0) {
		/* check if panel id available from VBT */
		if (!dev_priv->vbt.dsi.panel_id) {
			/* default Panasonic panel */
			dev_priv->mipi_panel_id = MIPI_DSI_PANASONIC_VXX09F006A00_PANEL_ID;
		} else
			dev_priv->mipi_panel_id = dev_priv->vbt.dsi.panel_id;
	} else
		dev_priv->mipi_panel_id = i915_mipi_panel_id;

	for (i = 0; i < ARRAY_SIZE(intel_dsi_devices); i++) {
		dsi = &intel_dsi_devices[i];
		if (dsi->panel_id == dev_priv->mipi_panel_id) {
			intel_dsi->dev = *dsi;

			if (dsi->dev_ops->init(&intel_dsi->dev))
				break;
		}
	}

	if (i == ARRAY_SIZE(intel_dsi_devices)) {
		DRM_DEBUG_KMS("no device found\n");
		goto err;
	}

	intel_encoder->type = INTEL_OUTPUT_DSI;
	intel_encoder->crtc_mask = (1 << 0); /* XXX */

	intel_encoder->cloneable = false;
	drm_connector_init(dev, connector, &intel_dsi_connector_funcs,
			   DRM_MODE_CONNECTOR_DSI);

	drm_encoder_helper_add(encoder, &intel_dsi_helper_funcs);
	drm_connector_helper_add(connector, &intel_dsi_connector_helper_funcs);

	connector->display_info.subpixel_order = SubPixelHorizontalRGB; /*XXX*/
	connector->interlace_allowed = false;
	connector->doublescan_allowed = false;

	intel_dsi_add_properties(intel_dsi, connector);
	intel_connector_attach_encoder(intel_connector, intel_encoder);

	drm_sysfs_connector_add(connector);

	fixed_mode = dsi->dev_ops->get_modes(&intel_dsi->dev);
	if (!fixed_mode) {
		DRM_DEBUG_KMS("no fixed mode\n");
		goto err;
	}

	dev_priv->is_mipi = true;
	fixed_mode->type |= DRM_MODE_TYPE_PREFERRED;
	intel_panel_init(&intel_connector->panel, fixed_mode);
	intel_panel_setup_backlight(connector);

	return true;
err:
	drm_encoder_cleanup(&intel_encoder->base);
	kfree(intel_dsi);
	kfree(intel_connector);

	return false;
}
