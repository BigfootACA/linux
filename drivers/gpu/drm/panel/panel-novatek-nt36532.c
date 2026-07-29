// SPDX-License-Identifier: GPL-2.0-only
/*
 * Novatek NT36532 based dual-DSI DSC video mode LCD panels
 *
 * Found on the Xiaomi Pad 8 Pro (codename "piano", panel project "P81"),
 * an 11.16" 3200x2136 TDDI module second-sourced from BOE and CSOT.
 *
 * Copyright (c) 2026 BigfootACA <bigfoot@classfun.cn>
 *
 * Multiple panel handling based on panel-himax-hx83121a.c
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

/*
 * The DDIC groups the supported frame rates into timing groups. Every group has
 * its own horizontal front porch and its own pair of frame rate registers.
 * Rates within a group are reached by stretching the vertical front porch only,
 * so they share the same register values.
 *
 * The uncompressed pixel clocks below differ between groups because the front
 * porches do, but DSC shrinks the active part of every line to the same number
 * of bytes, so all of them end up driving the link at 1.1975 Gbps per lane.
 */
struct nt36532_mode {
	struct drm_display_mode mode;
	u8 fps_reg_b2;
	u8 fps_reg_b3;
};

struct nt36532_desc {
	unsigned int width_mm;
	unsigned int height_mm;

	/*
	 * Both sources share the bulk of the init sequence and only differ in
	 * the ESD threshold, one vendor specific register block and the order
	 * of the two frame rate registers.
	 */
	u8 esd_threshold;
	void (*init_vendor_block)(struct mipi_dsi_multi_context *ctx,
				  struct mipi_dsi_device *dsi0,
				  struct mipi_dsi_device *dsi1);
	bool framerate_regs_swapped;
};

struct nt36532 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	struct drm_connector *connector;
	const struct nt36532_desc *desc;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator *vddi;
};

static inline struct nt36532 *to_nt36532(struct drm_panel *panel)
{
	return container_of(panel, struct nt36532, panel);
}

/*
 * The DDIC is fed by both DSI links in broadcast mode, so every command has
 * to be issued on both of them.
 */
#define nt36532_write_seq(ctx, seq...) \
	mipi_dsi_dual_dcs_write_seq_multi(ctx, dsi0, dsi1, seq)

/* Select the register page to operate on and unlock it for writing. */
static void nt36532_select_page(struct mipi_dsi_multi_context *ctx,
				struct mipi_dsi_device *dsi0,
				struct mipi_dsi_device *dsi1,
				u8 page)
{
	const u8 select[] = { 0xff, page };

	mipi_dsi_dual_dcs_write_buffer_multi(ctx, dsi0, dsi1,
					     select, ARRAY_SIZE(select));
	nt36532_write_seq(ctx, 0xfb, 0x01);
}

/* Content Adaptive Backlight Control gamma tables: UI, still image and video */
static void nt36532_init_cabc(struct mipi_dsi_multi_context *ctx,
			      struct mipi_dsi_device *dsi0,
			      struct mipi_dsi_device *dsi1)
{
	nt36532_select_page(ctx, dsi0, dsi1, 0x23);
	nt36532_write_seq(ctx, 0x00, 0x80);
	nt36532_write_seq(ctx, 0x01, 0x84);
	nt36532_write_seq(ctx, 0x05, 0x2d);
	nt36532_write_seq(ctx, 0x06, 0x00);
	nt36532_write_seq(ctx, 0x11, 0x04);
	nt36532_write_seq(ctx, 0x12, 0x2c);
	nt36532_write_seq(ctx, 0x15, 0x9e);
	nt36532_write_seq(ctx, 0x16, 0x16);

	/* UI mode */
	nt36532_write_seq(ctx, 0x29, 0x0a);
	nt36532_write_seq(ctx, 0x30, 0xff);
	nt36532_write_seq(ctx, 0x31, 0xfe);
	nt36532_write_seq(ctx, 0x32, 0xfd);
	nt36532_write_seq(ctx, 0x33, 0xfb);
	nt36532_write_seq(ctx, 0x34, 0xf8);
	nt36532_write_seq(ctx, 0x35, 0xf5);
	nt36532_write_seq(ctx, 0x36, 0xf3);
	nt36532_write_seq(ctx, 0x37, 0xf2);
	nt36532_write_seq(ctx, 0x38, 0xf2);
	nt36532_write_seq(ctx, 0x39, 0xf2);
	nt36532_write_seq(ctx, 0x3a, 0xef);
	nt36532_write_seq(ctx, 0x3b, 0xec);
	nt36532_write_seq(ctx, 0x3d, 0xe9);
	nt36532_write_seq(ctx, 0x3f, 0xe5);
	nt36532_write_seq(ctx, 0x40, 0xe5);
	nt36532_write_seq(ctx, 0x41, 0xe5);

	/* Still image mode */
	nt36532_write_seq(ctx, 0x2a, 0x13);
	nt36532_write_seq(ctx, 0x45, 0xff);
	nt36532_write_seq(ctx, 0x46, 0xf4);
	nt36532_write_seq(ctx, 0x47, 0xe7);
	nt36532_write_seq(ctx, 0x48, 0xda);
	nt36532_write_seq(ctx, 0x49, 0xcd);
	nt36532_write_seq(ctx, 0x4a, 0xc0);
	nt36532_write_seq(ctx, 0x4b, 0xb3);
	nt36532_write_seq(ctx, 0x4c, 0xb1);
	nt36532_write_seq(ctx, 0x4d, 0xb1);
	nt36532_write_seq(ctx, 0x4e, 0xb1);
	nt36532_write_seq(ctx, 0x4f, 0x95);
	nt36532_write_seq(ctx, 0x50, 0x79);
	nt36532_write_seq(ctx, 0x51, 0x5c);
	nt36532_write_seq(ctx, 0x52, 0x58);
	nt36532_write_seq(ctx, 0x53, 0x58);
	nt36532_write_seq(ctx, 0x54, 0x58);

	/* Moving image mode */
	nt36532_write_seq(ctx, 0x2b, 0x0e);
	nt36532_write_seq(ctx, 0x58, 0xff);
	nt36532_write_seq(ctx, 0x59, 0xfb);
	nt36532_write_seq(ctx, 0x5a, 0xf7);
	nt36532_write_seq(ctx, 0x5b, 0xf3);
	nt36532_write_seq(ctx, 0x5c, 0xef);
	nt36532_write_seq(ctx, 0x5d, 0xe3);
	nt36532_write_seq(ctx, 0x5e, 0xd8);
	nt36532_write_seq(ctx, 0x5f, 0xd6);
	nt36532_write_seq(ctx, 0x60, 0xd6);
	nt36532_write_seq(ctx, 0x61, 0xd6);
	nt36532_write_seq(ctx, 0x62, 0xc8);
	nt36532_write_seq(ctx, 0x63, 0xb7);
	nt36532_write_seq(ctx, 0x64, 0xaa);
	nt36532_write_seq(ctx, 0x65, 0xa8);
	nt36532_write_seq(ctx, 0x66, 0xa8);
	nt36532_write_seq(ctx, 0x67, 0xa8);

	/* Hand the PWM duty over to CABC */
	nt36532_select_page(ctx, dsi0, dsi1, 0x10);
	nt36532_write_seq(ctx, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x0f, 0xff);
	nt36532_write_seq(ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x24);
}

static void boe_p81_vendor_block(struct mipi_dsi_multi_context *ctx,
				 struct mipi_dsi_device *dsi0,
				 struct mipi_dsi_device *dsi1)
{
	nt36532_select_page(ctx, dsi0, dsi1, 0x2a);
	nt36532_write_seq(ctx, 0xc4, 0x82);
	nt36532_select_page(ctx, dsi0, dsi1, 0x26);
	nt36532_write_seq(ctx, 0x3b, 0x06);
	nt36532_write_seq(ctx, 0x4b, 0x06);
	nt36532_select_page(ctx, dsi0, dsi1, 0x22);
	nt36532_write_seq(ctx, 0xc4, 0x06);
}

static void csot_p81_vendor_block(struct mipi_dsi_multi_context *ctx,
				  struct mipi_dsi_device *dsi0,
				  struct mipi_dsi_device *dsi1)
{
	nt36532_select_page(ctx, dsi0, dsi1, 0x2a);
	nt36532_write_seq(ctx, 0xbc, 0x77, 0x07);
}

/*
 * Horizontal timings are given for the complete panel: drm/msm splits them
 * across the two DSI links itself.
 */
#define NT36532_MODE(hfp, vfp, fps) \
	.clock = (3200 + (hfp) + 32 + 32) * (2136 + (vfp) + 2 + 104) * (fps) / 1000, \
	.hdisplay = 3200, \
	.hsync_start = 3200 + (hfp), \
	.hsync_end = 3200 + (hfp) + 32, \
	.htotal = 3200 + (hfp) + 32 + 32, \
	.vdisplay = 2136, \
	.vsync_start = 2136 + (vfp), \
	.vsync_end = 2136 + (vfp) + 2, \
	.vtotal = 2136 + (vfp) + 2 + 104

static const struct nt36532_mode nt36532_p81_modes[] = {
	/* 120/60 Hz timing group */
	{
		.mode = { NT36532_MODE(234, 68, 120) },
		.fps_reg_b2 = 0x91,
		.fps_reg_b3 = 0x40,
	},
	/* 144 Hz timing group */
	{
		.mode = { NT36532_MODE(42, 68, 144) },
		.fps_reg_b2 = 0x00,
		.fps_reg_b3 = 0x00,
	},
	/* 90 Hz timing group */
	{
		.mode = { NT36532_MODE(618, 68, 90) },
		.fps_reg_b2 = 0x00,
		.fps_reg_b3 = 0x80,
	},
	/* Same group as 120 Hz, reached by stretching the vertical front porch */
	{
		.mode = { NT36532_MODE(234, 2378, 60) },
		.fps_reg_b2 = 0x91,
		.fps_reg_b3 = 0x40,
	},
};

static const struct nt36532_mode *nt36532_current_mode(struct nt36532 *ctx)
{
	struct drm_connector *connector = ctx->connector;
	struct drm_crtc_state *crtc_state;
	int i;

	/* Fall back to the preferred mode if nothing has been committed yet */
	if (!connector || !connector->state || !connector->state->crtc)
		return &nt36532_p81_modes[0];

	crtc_state = connector->state->crtc->state;
	if (!crtc_state)
		return &nt36532_p81_modes[0];

	for (i = 0; i < ARRAY_SIZE(nt36532_p81_modes); i++) {
		if (drm_mode_match(&crtc_state->mode, &nt36532_p81_modes[i].mode,
				   DRM_MODE_MATCH_TIMINGS | DRM_MODE_MATCH_CLOCK))
			return &nt36532_p81_modes[i];
	}

	return &nt36532_p81_modes[0];
}

static void nt36532_init_sequence(struct nt36532 *ctx,
				  struct mipi_dsi_multi_context *dsi_ctx)
{
	struct mipi_dsi_device *dsi0 = ctx->dsi[0];
	struct mipi_dsi_device *dsi1 = ctx->dsi[1];
	const struct nt36532_desc *desc = ctx->desc;
	const struct nt36532_mode *mode = nt36532_current_mode(ctx);
	const u8 esd[] = { 0xd1, desc->esd_threshold };
	const u8 b2[] = { 0xb2, mode->fps_reg_b2 };
	const u8 b3[] = { 0xb3, mode->fps_reg_b3 };

	/* ESD detection */
	nt36532_select_page(dsi_ctx, dsi0, dsi1, 0x27);
	nt36532_write_seq(dsi_ctx, 0xd0, 0x31);
	mipi_dsi_dual_dcs_write_buffer_multi(dsi_ctx, dsi0, dsi1,
					    esd, ARRAY_SIZE(esd));
	nt36532_write_seq(dsi_ctx, 0xd2, 0x38);
	nt36532_write_seq(dsi_ctx, 0xde, 0x43);
	nt36532_write_seq(dsi_ctx, 0xdf, 0x02);

	nt36532_init_cabc(dsi_ctx, dsi0, dsi1);

	nt36532_select_page(dsi_ctx, dsi0, dsi1, 0x25);
	nt36532_write_seq(dsi_ctx, 0x0f, 0x20);

	/* Keep the panel quiet when the magnetic cover is attached */
	nt36532_select_page(dsi_ctx, dsi0, dsi1, 0x27);
	nt36532_write_seq(dsi_ctx, 0x13, 0x00);
	nt36532_write_seq(dsi_ctx, 0x14, 0x11);

	desc->init_vendor_block(dsi_ctx, dsi0, dsi1);

	/* Novatek stock init code */
	nt36532_select_page(dsi_ctx, dsi0, dsi1, 0xf0);
	nt36532_write_seq(dsi_ctx, 0xfa, 0x05);
	nt36532_write_seq(dsi_ctx, 0x76, 0x16);

	nt36532_select_page(dsi_ctx, dsi0, dsi1, 0x10);
	nt36532_write_seq(dsi_ctx, 0x3b, 0x03, 0x6a, 0x44, 0x04, 0x04, 0x00);

	/* DDIC side DSC decoder setup */
	nt36532_write_seq(dsi_ctx, 0x90, 0x03);
	nt36532_write_seq(dsi_ctx, 0x91,
			  0xab, 0xa8, 0x00, 0x18, 0xd2, 0x00, 0x00, 0x00,
			  0x02, 0x9f, 0x00, 0x0b, 0x04, 0x86, 0x02, 0xdc);
	nt36532_write_seq(dsi_ctx, 0x92, 0x10, 0xf0);
	nt36532_write_seq(dsi_ctx, 0x9d, 0x01);

	/* Frame rate selection, the two sources want these in opposite order */
	if (desc->framerate_regs_swapped) {
		mipi_dsi_dual_dcs_write_buffer_multi(dsi_ctx, dsi0, dsi1,
						     b2, ARRAY_SIZE(b2));
		mipi_dsi_dual_dcs_write_buffer_multi(dsi_ctx, dsi0, dsi1,
						     b3, ARRAY_SIZE(b3));
	} else {
		mipi_dsi_dual_dcs_write_buffer_multi(dsi_ctx, dsi0, dsi1,
						     b3, ARRAY_SIZE(b3));
		mipi_dsi_dual_dcs_write_buffer_multi(dsi_ctx, dsi0, dsi1,
						     b2, ARRAY_SIZE(b2));
	}
}

static void nt36532_reset(struct nt36532 *ctx)
{
	/* Reset is asserted (driven low) on entry, see GPIOD_OUT_HIGH below */
	msleep(10);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(3000, 4000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(3000, 4000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(15);
}

static int nt36532_prepare(struct drm_panel *panel)
{
	struct nt36532 *ctx = to_nt36532(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi[0] };
	int ret;

	ret = regulator_enable(ctx->vddi);
	if (ret < 0)
		return ret;

	nt36532_reset(ctx);

	nt36532_init_sequence(ctx, &dsi_ctx);

	mipi_dsi_dual(mipi_dsi_dcs_exit_sleep_mode_multi, &dsi_ctx,
		      ctx->dsi[0], ctx->dsi[1]);
	mipi_dsi_msleep(&dsi_ctx, 120);

	if (dsi_ctx.accum_err) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_disable(ctx->vddi);
		return dsi_ctx.accum_err;
	}

	return 0;
}

static int nt36532_enable(struct drm_panel *panel)
{
	struct nt36532 *ctx = to_nt36532(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi[0] };
	struct drm_dsc_picture_parameter_set pps;

	/* Only start scanning once the timing engine is feeding the link */
	mipi_dsi_dual(mipi_dsi_dcs_set_display_on_multi, &dsi_ctx,
		      ctx->dsi[0], ctx->dsi[1]);

	/*
	 * Both links get the same PPS, describing the full 3200 pixel wide
	 * picture, i.e. four slices per line.  The DDIC knows that each of
	 * the two links carries two of them.  Only the DSI wire parameters
	 * are per link.  drm/msm already left pic_width at the full mode
	 * width for us.
	 */
	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	mipi_dsi_dual(mipi_dsi_picture_parameter_set_multi, &dsi_ctx,
		      ctx->dsi[0], ctx->dsi[1], &pps);

	return dsi_ctx.accum_err;
}

static int nt36532_disable(struct drm_panel *panel)
{
	struct nt36532 *ctx = to_nt36532(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi[0] };

	mipi_dsi_dual(mipi_dsi_dcs_set_display_off_multi, &dsi_ctx,
		      ctx->dsi[0], ctx->dsi[1]);
	mipi_dsi_msleep(&dsi_ctx, 20);

	return dsi_ctx.accum_err;
}

static int nt36532_unprepare(struct drm_panel *panel)
{
	struct nt36532 *ctx = to_nt36532(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi[0] };

	mipi_dsi_dual(mipi_dsi_dcs_enter_sleep_mode_multi, &dsi_ctx,
		      ctx->dsi[0], ctx->dsi[1]);
	mipi_dsi_msleep(&dsi_ctx, 100);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->vddi);

	return 0;
}

static int nt36532_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct nt36532 *ctx = to_nt36532(panel);
	int i;

	for (i = 0; i < ARRAY_SIZE(nt36532_p81_modes); i++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev,
					  &nt36532_p81_modes[i].mode);
		if (!mode)
			return -ENOMEM;

		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = ctx->desc->width_mm;
	connector->display_info.height_mm = ctx->desc->height_mm;
	connector->display_info.bpc = 10;

	ctx->connector = connector;

	return ARRAY_SIZE(nt36532_p81_modes);
}

static const struct drm_panel_funcs nt36532_panel_funcs = {
	.prepare = nt36532_prepare,
	.enable = nt36532_enable,
	.disable = nt36532_disable,
	.unprepare = nt36532_unprepare,
	.get_modes = nt36532_get_modes,
};

static const struct drm_dsc_config nt36532_p81_dsc_cfg = {
	.dsc_version_major = 1,
	.dsc_version_minor = 1,
	.slice_height = 24,
	.slice_width = 800,
	/* Per DSI link: 1600 / 800 */
	.slice_count = 2,
	.bits_per_component = 10,
	.bits_per_pixel = 8 << 4,
	.block_pred_enable = true,
	/*
	 * The DDIC decoder is preconfigured over the vendor specific 0x91
	 * register with 12 + 9 * min(34, slice_height - 8) / 100 == 13, i.e.
	 * the DSC 1.2 first_line_bpg_offset formula, even though the panel
	 * is a DSC 1.1 pre-SCR one.  The values it derives from that
	 * (nfl_bpg_offset 1158, scale_increment_interval 671) are readable
	 * in the 0x91 payload below.  Match it here, otherwise the DPU
	 * encoder builds a different rate control model than the decoder.
	 */
	.first_line_bpg_offset = 13,
	/*
	 * Qualcomm's NT36532 specific rate control table uses the DSC C model
	 * reference value of 8 for the first range instead of the 4 that VESA
	 * DSC 1.1 Table E-5 lists.  Everything else is identical to the
	 * generic pre-SCR 8bpp/10bpc table, so only override this one entry.
	 */
	.rc_range_params[0].range_max_qp = 8,
};

static void nt36532_detect_source(struct nt36532 *ctx, struct device *dev);

static int nt36532_probe(struct mipi_dsi_device *dsi)
{
	struct mipi_dsi_device_info info = { "dsi-secondary", 0, NULL };
	struct device *dev = &dsi->dev;
	struct mipi_dsi_host *dsi1_host;
	struct device_node *dsi1;
	struct nt36532 *ctx;
	int ret, i;

	ctx = devm_drm_panel_alloc(dev, struct nt36532, panel,
				   &nt36532_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->desc = of_device_get_match_data(dev);
	if (!ctx->desc)
		return -ENODEV;

	nt36532_detect_source(ctx, dev);

	ctx->dsc = nt36532_p81_dsc_cfg;

	ctx->vddi = devm_regulator_get(dev, "vddi");
	if (IS_ERR(ctx->vddi))
		return dev_err_probe(dev, PTR_ERR(ctx->vddi),
				     "Failed to get vddi regulator\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	dsi1 = of_graph_get_remote_node(dev->of_node, 1, -1);
	if (!dsi1)
		return dev_err_probe(dev, -ENODEV,
				     "Cannot get secondary DSI node\n");

	dsi1_host = of_find_mipi_dsi_host_by_node(dsi1);
	of_node_put(dsi1);
	if (!dsi1_host)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "Cannot get secondary DSI host\n");

	ctx->dsi[1] = devm_mipi_dsi_device_register_full(dev, dsi1_host, &info);
	if (IS_ERR(ctx->dsi[1]))
		return dev_err_probe(dev, PTR_ERR(ctx->dsi[1]),
				     "Cannot get secondary DSI device\n");

	ctx->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);
	mipi_dsi_set_drvdata(ctx->dsi[1], ctx);

	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		ctx->dsi[i]->lanes = 4;
		ctx->dsi[i]->format = MIPI_DSI_FMT_RGB101010;
		ctx->dsi[i]->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM;
		ctx->dsi[i]->dsc = &ctx->dsc;

		ret = devm_mipi_dsi_attach(dev, ctx->dsi[i]);
		if (ret < 0) {
			drm_panel_remove(&ctx->panel);
			return dev_err_probe(dev, ret,
					     "Failed to attach to DSI host %d\n", i);
		}
	}

	return 0;
}

static void nt36532_remove(struct mipi_dsi_device *dsi)
{
	struct nt36532 *ctx = mipi_dsi_get_drvdata(dsi);

	drm_panel_remove(&ctx->panel);
	ctx->connector = NULL;
}

/* BOE module, Xiaomi panel code 42 02 0a */
static const struct nt36532_desc boe_p81_desc = {
	.width_mm = 239,
	.height_mm = 159,
	.esd_threshold = 0x20,
	.init_vendor_block = boe_p81_vendor_block,
};

/* CSOT module, Xiaomi panel code 35 02 0b */
static const struct nt36532_desc csot_p81_desc = {
	.width_mm = 239,
	.height_mm = 159,
	.esd_threshold = 0x00,
	.init_vendor_block = csot_p81_vendor_block,
	.framerate_regs_swapped = true,
};

/*
 * Both module sources share a single device tree, so an optional strapping pin
 * is used to pick the right one at runtime. The device tree compatible stays
 * authoritative when the pin is absent.
 */
static void nt36532_detect_source(struct nt36532 *ctx, struct device *dev)
{
	const struct nt36532_desc *strapped;
	struct gpio_desc *lcd_id;
	int val;

	/*
	 * The touch half of the die needs the same strap to pick its firmware,
	 * so only borrow the line for the read instead of holding on to it.
	 */
	lcd_id = gpiod_get_optional(dev, "lcd-id", GPIOD_IN);
	if (IS_ERR_OR_NULL(lcd_id))
		return;

	val = gpiod_get_value_cansleep(lcd_id);
	gpiod_put(lcd_id);

	if (val < 0)
		return;

	strapped = val ? &boe_p81_desc : &csot_p81_desc;
	if (strapped != ctx->desc) {
		dev_info(dev, "strap selects the %s module\n",
			 val ? "BOE" : "CSOT");
		ctx->desc = strapped;
	}
}

static const struct of_device_id nt36532_of_match[] = {
	{ .compatible = "xiaomi,p81-boe", .data = &boe_p81_desc },
	{ .compatible = "xiaomi,p81-csot", .data = &csot_p81_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nt36532_of_match);

static struct mipi_dsi_driver nt36532_driver = {
	.probe = nt36532_probe,
	.remove = nt36532_remove,
	.driver = {
		.name = "panel-novatek-nt36532",
		.of_match_table = nt36532_of_match,
	},
};
module_mipi_dsi_driver(nt36532_driver);

MODULE_AUTHOR("BigfootACA <bigfoot@classfun.cn>");
MODULE_DESCRIPTION("Novatek NT36532 dual-DSI DSC LCD panel driver");
MODULE_LICENSE("GPL");
