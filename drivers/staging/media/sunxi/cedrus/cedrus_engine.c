// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>

#include "cedrus.h"
#include "cedrus_context.h"
#include "cedrus_engine.h"

/* Ctrl */

int cedrus_engine_ctrl_validate(struct cedrus_context *ctx,
				struct v4l2_ctrl *ctrl)
{
	const struct cedrus_engine *engine = ctx->engine;

	if (WARN_ON(!engine || !engine->ops))
		return -ENODEV;

	if (!engine->ops->ctrl_validate)
		return 0;

	return engine->ops->ctrl_validate(ctx, ctrl);
}

int cedrus_engine_ctrl_prepare(struct cedrus_context *ctx,
			       struct v4l2_ctrl *ctrl)
{
	const struct cedrus_engine *engine = ctx->engine;

	if (WARN_ON(!engine || !engine->ops))
		return -ENODEV;

	if (!engine->ops->ctrl_prepare)
		return 0;

	return engine->ops->ctrl_prepare(ctx, ctrl);
}

/* Format */

int cedrus_engine_format_prepare(struct cedrus_context *ctx,
				 struct v4l2_format *format)
{
	const struct cedrus_engine *engine = ctx->engine;

	if (WARN_ON(!engine || !engine->ops || !engine->ops->format_prepare))
		return -ENODEV;

	return engine->ops->format_prepare(ctx, format);
}

int cedrus_engine_format_configure(struct cedrus_context *ctx)
{
	const struct cedrus_engine *engine = ctx->engine;

	if (WARN_ON(!engine || !engine->ops || !engine->ops->format_configure))
		return -ENODEV;

	return engine->ops->format_configure(ctx);
}

/* Context */

int cedrus_engine_setup(struct cedrus_context *ctx)
{
	const struct cedrus_engine *engine = ctx->engine;

	if (WARN_ON(!engine || !engine->ops))
		return -ENODEV;

	if (!engine->ops->setup)
		return 0;

	return engine->ops->setup(ctx);
}

void cedrus_engine_cleanup(struct cedrus_context *ctx)
{
	const struct cedrus_engine *engine = ctx->engine;

	if (WARN_ON(!engine || !engine->ops) || !engine->ops->cleanup)
		return;

	engine->ops->cleanup(ctx);
}

/* Buffer */

int cedrus_engine_buffer_setup(struct cedrus_context *ctx,
			       struct cedrus_buffer *buffer)
{
	const struct cedrus_engine *engine = ctx->engine;

	if (WARN_ON(!engine || !engine->ops))
		return -ENODEV;

	if (!engine->ops->buffer_setup)
		return 0;

	return engine->ops->buffer_setup(ctx, buffer);
}

void cedrus_engine_buffer_cleanup(struct cedrus_context *ctx,
				  struct cedrus_buffer *buffer)
{
	const struct cedrus_engine *engine = ctx->engine;

	if (WARN_ON(!engine || !engine->ops) || !engine->ops->buffer_cleanup)
		return;

	engine->ops->buffer_cleanup(ctx, buffer);
}

/* Job */

int cedrus_engine_job_prepare(struct cedrus_context *ctx)
{
	const struct cedrus_engine *engine = ctx->engine;

	if (WARN_ON(!engine || !engine->ops))
		return -ENODEV;

	if (!engine->ops->job_prepare)
		return 0;

	return engine->ops->job_prepare(ctx);
}

int cedrus_engine_job_configure(struct cedrus_context *ctx)
{
	const struct cedrus_engine *engine = ctx->engine;

	if (WARN_ON(!engine || !engine->ops))
		return -ENODEV;

	if (!engine->ops->job_configure)
		return 0;

	return engine->ops->job_configure(ctx);
}

void cedrus_engine_job_trigger(struct cedrus_context *ctx)
{
	const struct cedrus_engine *engine = ctx->engine;

	if (WARN_ON(!engine || !engine->ops || !engine->ops->job_trigger))
		return;

	engine->ops->job_trigger(ctx);
}

void cedrus_engine_job_finish(struct cedrus_context *ctx, int state)
{
	const struct cedrus_engine *engine = ctx->engine;

	if (WARN_ON(!engine || !engine->ops) || !engine->ops->job_finish)
		return;

	engine->ops->job_finish(ctx, state);
}

/* IRQ */

irqreturn_t cedrus_engine_irq_status(struct cedrus_context *ctx)
{
	const struct cedrus_engine *engine = ctx->engine;

	if (WARN_ON(!engine || !engine->ops) || !engine->ops->irq_status)
		return IRQ_NONE;

	return engine->ops->irq_status(ctx);
}

void cedrus_engine_irq_clear(struct cedrus_context *ctx)
{
	const struct cedrus_engine *engine = ctx->engine;

	if (WARN_ON(!engine || !engine->ops) || !engine->ops->irq_clear)
		return;

	engine->ops->irq_clear(ctx);
}

void cedrus_engine_irq_disable(struct cedrus_context *ctx)
{
	const struct cedrus_engine *engine = ctx->engine;

	if (WARN_ON(!engine || !engine->ops) || !engine->ops->irq_disable)
		return;

	engine->ops->irq_disable(ctx);
}
