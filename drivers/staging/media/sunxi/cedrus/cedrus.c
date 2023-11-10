// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2016 Florent Revest <florent.revest@free-electrons.com>
 * Copyright 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright 2018-2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/soc/sunxi/sunxi_sram.h>
#include <linux/types.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "cedrus.h"
#include "cedrus_context.h"
#include "cedrus_dec.h"
#include "cedrus_enc.h"
#include "cedrus_engine.h"
#include "cedrus_proc.h"

/* Media */

static int cedrus_media_request_validate(struct media_request *req)
{
	struct media_request_object *obj;
	struct cedrus_context *ctx = NULL;
	struct v4l2_device *v4l2_dev;
	unsigned int count;

	list_for_each_entry(obj, &req->objects, list) {
		if (vb2_request_object_is_buffer(obj)) {
			struct vb2_buffer *buffer =
				container_of(obj, struct vb2_buffer, req_obj);

			ctx = vb2_get_drv_priv(buffer->vb2_queue);
			break;
		}
	}

	if (!ctx)
		return -ENOENT;

	v4l2_dev = &ctx->proc->dev->v4l2.v4l2_dev;

	count = vb2_request_buffer_cnt(req);
	if (!count) {
		v4l2_err(v4l2_dev, "no buffer provided with the request\n");
		return -ENOENT;
	} else if (count > 1) {
		v4l2_err(v4l2_dev,
			  "too many buffers provided with the request\n");
		return -EINVAL;
	}

	return vb2_request_validate(req);
}

static const struct media_device_ops cedrus_media_ops = {
	.req_validate	= cedrus_media_request_validate,
	.req_queue	= v4l2_m2m_request_queue,
};

/* V4L2 */

static void cedrus_v4l2_m2m_device_run(void *private)
{
	cedrus_context_job_run(private);
}

static const struct v4l2_m2m_ops cedrus_v4l2_m2m_ops = {
	.device_run	= cedrus_v4l2_m2m_device_run,
};

static int cedrus_v4l2_setup(struct cedrus_device *cedrus_dev)
{
	struct device *dev = cedrus_dev->dev;
	struct cedrus_v4l2 *v4l2 = &cedrus_dev->v4l2;
	struct v4l2_device *v4l2_dev = &v4l2->v4l2_dev;
	struct media_device *media_dev = &v4l2->media_dev;
	int ret;

	/* Media Device */

	strscpy(media_dev->model, CEDRUS_NAME, sizeof(media_dev->model));
	strscpy(media_dev->bus_info, "platform:" CEDRUS_NAME,
		sizeof(media_dev->bus_info));
	media_dev->ops = &cedrus_media_ops;
	media_dev->dev = dev;

	media_device_init(media_dev);

	ret = media_device_register(media_dev);
	if (ret) {
		dev_err(dev, "failed to register media device\n");
		return ret;
	}

	/* V4L2 Device */

	v4l2_dev->mdev = media_dev;

	ret = v4l2_device_register(dev, v4l2_dev);
	if (ret) {
		dev_err(dev, "failed to register V4L2 device\n");
		goto error_media;
	}

	/* V4L2 M2M */

	v4l2->m2m_dev = v4l2_m2m_init(&cedrus_v4l2_m2m_ops);
	if (IS_ERR(v4l2->m2m_dev)) {
		v4l2_err(v4l2_dev, "failed to initialize V4L2 M2M device\n");
		ret = PTR_ERR(v4l2->m2m_dev);
		goto error_v4l2;
	}

	return 0;

error_v4l2:
	v4l2_device_unregister(v4l2_dev);

error_media:
	media_device_unregister(media_dev);
	media_device_cleanup(media_dev);

	return ret;
}

static void cedrus_v4l2_cleanup(struct cedrus_device *cedrus_dev)
{
	struct cedrus_v4l2 *v4l2 = &cedrus_dev->v4l2;
	struct media_device *media_dev = &v4l2->media_dev;

	v4l2_m2m_release(v4l2->m2m_dev);
	v4l2_device_unregister(&v4l2->v4l2_dev);
	media_device_unregister(media_dev);
	media_device_cleanup(media_dev);
}

/* Platform */

void cedrus_watchdog(struct work_struct *work)
{
	struct cedrus_device *cedrus_dev =
		container_of(to_delayed_work(work), struct cedrus_device,
			     watchdog_work);
	struct v4l2_device *v4l2_dev = &cedrus_dev->v4l2.v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev = cedrus_dev->v4l2.m2m_dev;
	struct cedrus_context *ctx = v4l2_m2m_get_curr_priv(m2m_dev);

	if (!ctx)
		return;

	v4l2_err(v4l2_dev, "frame processing timed out!\n");
	reset_control_reset(cedrus_dev->reset);

	cedrus_context_job_finish(ctx, VB2_BUF_STATE_ERROR);
}

static void cedrus_irq_disable_clear(struct cedrus_context *ctx)
{
	/* Disable and clear IRQ on the current engine. */
	cedrus_engine_irq_disable(ctx);
	cedrus_engine_irq_clear(ctx);
}

static void cedrus_irq_spurious(struct cedrus_device *dev)
{
	struct cedrus_proc *proc;

	/* Disable/clear IRQ on the decoder. */

	proc = &dev->dec;
	spin_lock(&proc->ctx_active_lock);

	if (proc->ctx_active)
		cedrus_irq_disable_clear(proc->ctx_active);

	spin_unlock(&proc->ctx_active_lock);

	/* Disable/clear IRQ on the encoder. */

	proc = &dev->enc;
	spin_lock(&proc->ctx_active_lock);

	if (proc->ctx_active)
		cedrus_irq_disable_clear(proc->ctx_active);

	spin_unlock(&proc->ctx_active_lock);
}

static irqreturn_t cedrus_irq(int irq, void *private)
{
	struct cedrus_device *cedrus_dev = private;
	struct v4l2_m2m_dev *m2m_dev = cedrus_dev->v4l2.m2m_dev;
	struct cedrus_context *ctx = v4l2_m2m_get_curr_priv(m2m_dev);
	int status;
	int state;

	/*
	 * If cancel_delayed_work returns false it means watchdog already
	 * executed and finished the job.
	 */
	if (!cancel_delayed_work(&cedrus_dev->watchdog_work)) {
		cedrus_irq_spurious(cedrus_dev);
		return IRQ_HANDLED;
	}

	/*
	 * V4L2 M2M will always wait for the current job to finish so we should
	 * never catch an unexpected interrupt.
	 */
	if (WARN_ON(!ctx)) {
		cedrus_irq_spurious(cedrus_dev);
		return IRQ_NONE;
	}

	status = cedrus_engine_irq_status(ctx);
	if (status == CEDRUS_IRQ_NONE)
		return IRQ_NONE;

	cedrus_irq_disable_clear(ctx);

	if (status == CEDRUS_IRQ_ERROR)
		state = VB2_BUF_STATE_ERROR;
	else
		state = VB2_BUF_STATE_DONE;

	cedrus_context_job_finish(ctx, state);

	return IRQ_HANDLED;
}

static int cedrus_suspend(struct device *dev)
{
	struct cedrus_device *cedrus_dev = dev_get_drvdata(dev);

	clk_disable_unprepare(cedrus_dev->clock_ram);
	clk_disable_unprepare(cedrus_dev->clock_mod);
	clk_disable_unprepare(cedrus_dev->clock_ahb);

	reset_control_assert(cedrus_dev->reset);

	return 0;
}

static int cedrus_resume(struct device *dev)
{
	struct cedrus_device *cedrus_dev = dev_get_drvdata(dev);
	int ret;

	ret = reset_control_reset(cedrus_dev->reset);
	if (ret) {
		dev_err(dev, "failed to reset\n");
		return ret;
	}

	ret = clk_prepare_enable(cedrus_dev->clock_ahb);
	if (ret) {
		dev_err(dev, "failed to enable ahb clock\n");
		goto error_reset;
	}

	ret = clk_prepare_enable(cedrus_dev->clock_mod);
	if (ret) {
		dev_err(dev, "failed to enable module clock\n");
		goto error_clock_ahb;
	}

	ret = clk_prepare_enable(cedrus_dev->clock_ram);
	if (ret) {
		dev_err(dev, "failed to enable ram clock\n");
		goto error_clock_mod;
	}

	return 0;

error_clock_mod:
	clk_disable_unprepare(cedrus_dev->clock_mod);

error_clock_ahb:
	clk_disable_unprepare(cedrus_dev->clock_ahb);

error_reset:
	reset_control_assert(cedrus_dev->reset);

	return ret;
}

static const struct dev_pm_ops cedrus_pm_ops = {
	.runtime_suspend	= cedrus_suspend,
	.runtime_resume		= cedrus_resume,
};

static int cedrus_resources_setup(struct cedrus_device *cedrus_dev,
				  struct platform_device *platform_dev)
{
	const struct cedrus_variant *variant;
	struct device *dev = cedrus_dev->dev;
	int irq;
	int ret;

	/* Variant */

	variant = of_device_get_match_data(dev);
	if (!variant)
		return -EINVAL;

	cedrus_dev->capabilities = variant->capabilities;

	/* Registers */

	cedrus_dev->io_base = devm_platform_ioremap_resource(platform_dev, 0);
	if (IS_ERR(cedrus_dev->io_base)) {
		dev_err(dev, "failed to map registers\n");
		return PTR_ERR(cedrus_dev->io_base);
	}

	/* Clocks */

	cedrus_dev->clock_ahb = devm_clk_get(dev, "ahb");
	if (IS_ERR(cedrus_dev->clock_ahb)) {
		dev_err(dev, "failed to get ahb clock\n");
		return PTR_ERR(cedrus_dev->clock_ahb);
	}

	cedrus_dev->clock_mod = devm_clk_get(dev, "mod");
	if (IS_ERR(cedrus_dev->clock_mod)) {
		dev_err(dev, "failed to get module clock\n");
		return PTR_ERR(cedrus_dev->clock_mod);
	}

	cedrus_dev->clock_ram = devm_clk_get(dev, "ram");
	if (IS_ERR(cedrus_dev->clock_ram)) {
		dev_err(dev, "failed to get ram clock\n");
		return PTR_ERR(cedrus_dev->clock_ram);
	}

	ret = clk_set_rate(cedrus_dev->clock_mod, variant->clock_mod_rate);
	if (ret) {
		dev_err(dev, "failed to set module clock rate\n");
		return ret;
	}

	/* Reset */

	cedrus_dev->reset = devm_reset_control_get(dev, NULL);
	if (IS_ERR(cedrus_dev->reset)) {
		dev_err(dev, "failed to get reset\n");
		return PTR_ERR(cedrus_dev->reset);
	}

	/* IRQ */

	irq = platform_get_irq(platform_dev, 0);
	if (irq < 0) {
		dev_err(dev, "failed to get irq\n");
		return -ENXIO;
	}

	ret = devm_request_irq(dev, irq, cedrus_irq, 0, CEDRUS_NAME,
			       cedrus_dev);
	if (ret) {
		dev_err(dev, "failed to request irq\n");
		return ret;
	}

	/* Memory */

	ret = of_reserved_mem_device_init(dev);
	if (ret && ret != -ENODEV) {
		dev_err(dev, "failed to reserve memory\n");
		return ret;
	}

	/* SRAM */

	ret = sunxi_sram_claim(dev);
	if (ret) {
		dev_err(dev, "failed to claim SRAM\n");
		goto error_memory;
	}

	/* Runtime PM */

	pm_runtime_enable(dev);

	return 0;

error_memory:
	of_reserved_mem_device_release(dev);

	return ret;
}

static void cedrus_resources_cleanup(struct cedrus_device *cedrus_dev)
{
	struct device *dev = cedrus_dev->dev;

	pm_runtime_disable(dev);
	sunxi_sram_release(dev);
	of_reserved_mem_device_release(dev);
}

static int cedrus_probe(struct platform_device *platform_dev)
{
	struct cedrus_device *cedrus_dev;
	struct device *dev = &platform_dev->dev;
	int ret;

	cedrus_dev = devm_kzalloc(dev, sizeof(*cedrus_dev), GFP_KERNEL);
	if (!cedrus_dev)
		return -ENOMEM;

	cedrus_dev->dev = dev;
	platform_set_drvdata(platform_dev, cedrus_dev);

	INIT_DELAYED_WORK(&cedrus_dev->watchdog_work, cedrus_watchdog);

	ret = cedrus_resources_setup(cedrus_dev, platform_dev);
	if (ret)
		return ret;

	ret = cedrus_v4l2_setup(cedrus_dev);
	if (ret)
		goto error_resources;

	ret = cedrus_dec_setup(cedrus_dev);
	if (ret)
		goto error_v4l2;

	ret = cedrus_enc_setup(cedrus_dev);
	if (ret)
		goto error_dec;

	return 0;

error_dec:
	cedrus_dec_cleanup(cedrus_dev);

error_v4l2:
	cedrus_v4l2_cleanup(cedrus_dev);

error_resources:
	cedrus_resources_cleanup(cedrus_dev);

	return ret;
}

static void cedrus_remove(struct platform_device *platform_dev)
{
	struct cedrus_device *cedrus_dev = platform_get_drvdata(platform_dev);

	cancel_delayed_work_sync(&cedrus_dev->watchdog_work);

	cedrus_enc_cleanup(cedrus_dev);
	cedrus_dec_cleanup(cedrus_dev);
	cedrus_v4l2_cleanup(cedrus_dev);
	cedrus_resources_cleanup(cedrus_dev);
}

static const struct cedrus_variant cedrus_variant_sun4i_a10 = {
	.capabilities	= CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.clock_mod_rate	= 320000000,
};

static const struct cedrus_variant cedrus_variant_sun5i_a13 = {
	.capabilities	= CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.clock_mod_rate	= 320000000,
};

static const struct cedrus_variant cedrus_variant_sun7i_a20 = {
	.capabilities	= CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.clock_mod_rate	= 320000000,
};

static const struct cedrus_variant cedrus_variant_sun8i_a33 = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.clock_mod_rate	= 320000000,
};

static const struct cedrus_variant cedrus_variant_sun8i_h3 = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_H265_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.clock_mod_rate	= 402000000,
};

static const struct cedrus_variant cedrus_variant_sun8i_v3s = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_H264_DEC,
	.clock_mod_rate	= 402000000,
};

static const struct cedrus_variant cedrus_variant_sun8i_r40 = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.clock_mod_rate	= 297000000,
};

static const struct cedrus_variant cedrus_variant_sun20i_d1 = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_H265_DEC,
	.clock_mod_rate	= 432000000,
};

static const struct cedrus_variant cedrus_variant_sun50i_a64 = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_H265_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.clock_mod_rate	= 402000000,
};

static const struct cedrus_variant cedrus_variant_sun50i_h5 = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_H265_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.clock_mod_rate	= 402000000,
};

static const struct cedrus_variant cedrus_variant_sun50i_h6 = {
	.capabilities	= CEDRUS_CAPABILITY_UNTILED |
			  CEDRUS_CAPABILITY_MPEG2_DEC |
			  CEDRUS_CAPABILITY_H264_DEC |
			  CEDRUS_CAPABILITY_H265_DEC |
			  CEDRUS_CAPABILITY_H265_10_DEC |
			  CEDRUS_CAPABILITY_VP8_DEC,
	.clock_mod_rate	= 600000000,
};

static const struct of_device_id cedrus_of_match[] = {
	{
		.compatible	= "allwinner,sun4i-a10-video-engine",
		.data		= &cedrus_variant_sun4i_a10,
	},
	{
		.compatible	= "allwinner,sun5i-a13-video-engine",
		.data		= &cedrus_variant_sun5i_a13,
	},
	{
		.compatible	= "allwinner,sun7i-a20-video-engine",
		.data		= &cedrus_variant_sun7i_a20,
	},
	{
		.compatible	= "allwinner,sun8i-a33-video-engine",
		.data		= &cedrus_variant_sun8i_a33,
	},
	{
		.compatible	= "allwinner,sun8i-h3-video-engine",
		.data		= &cedrus_variant_sun8i_h3,
	},
	{
		.compatible	= "allwinner,sun8i-v3s-video-engine",
		.data		= &cedrus_variant_sun8i_v3s,
	},
	{
		.compatible	= "allwinner,sun8i-r40-video-engine",
		.data		= &cedrus_variant_sun8i_r40,
	},
	{
		.compatible	= "allwinner,sun20i-d1-video-engine",
		.data		= &cedrus_variant_sun20i_d1,
	},
	{
		.compatible	= "allwinner,sun50i-a64-video-engine",
		.data		= &cedrus_variant_sun50i_a64,
	},
	{
		.compatible	= "allwinner,sun50i-h5-video-engine",
		.data		= &cedrus_variant_sun50i_h5,
	},
	{
		.compatible	= "allwinner,sun50i-h6-video-engine",
		.data		= &cedrus_variant_sun50i_h6,
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, cedrus_of_match);

static struct platform_driver cedrus_driver = {
	.probe		= cedrus_probe,
	.remove_new	= cedrus_remove,
	.driver		= {
		.name		= CEDRUS_NAME,
		.of_match_table	= cedrus_of_match,
		.pm		= &cedrus_pm_ops,
	},
};

module_platform_driver(cedrus_driver);

MODULE_DESCRIPTION("Allwinner Cedrus Video Engine Driver");
MODULE_AUTHOR("Florent Revest <florent.revest@free-electrons.com>");
MODULE_AUTHOR("Paul Kocialkowski <paul.kocialkowski@bootlin.com>");
MODULE_AUTHOR("Maxime Ripard <maxime.ripard@bootlin.com>");
MODULE_LICENSE("GPL v2");
