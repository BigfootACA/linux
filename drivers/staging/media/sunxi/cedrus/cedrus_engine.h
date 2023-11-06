/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _CEDRUS_ENGINE_H_
#define _CEDRUS_ENGINE_H_

#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>

struct cedrus_context;
struct cedrus_buffer;

struct cedrus_engine_ops {
	int (*ctrl_validate)(struct cedrus_context *ctx,
			     struct v4l2_ctrl *ctrl);
	int (*ctrl_prepare)(struct cedrus_context *ctx, struct v4l2_ctrl *ctrl);

	int (*format_prepare)(struct cedrus_context *ctx,
			      struct v4l2_format *format);
	int (*format_configure)(struct cedrus_context *ctx);

	int (*setup)(struct cedrus_context *ctx);
	void (*cleanup)(struct cedrus_context *ctx);

	int (*buffer_setup)(struct cedrus_context *ctx,
			    struct cedrus_buffer *buffer);
	void (*buffer_cleanup)(struct cedrus_context *ctx,
			       struct cedrus_buffer *buffer);

	int (*job_prepare)(struct cedrus_context *ctx);
	int (*job_configure)(struct cedrus_context *ctx);
	void (*job_trigger)(struct cedrus_context *ctx);
	void (*job_finish)(struct cedrus_context *ctx, int state);

	int (*irq_status)(struct cedrus_context *ctx);
	void (*irq_clear)(struct cedrus_context *ctx);
	void (*irq_disable)(struct cedrus_context *ctx);
};

struct cedrus_engine {
	int					codec;
	int					role;

	unsigned int				capabilities;

	const struct cedrus_engine_ops		*ops;

	u32					pixelformat;
	bool					slice_based;

	const struct v4l2_ctrl_config		*ctrl_configs;
	unsigned int				ctrl_configs_count;

	const struct v4l2_frmsize_stepwise	*frmsize;

	unsigned int				ctx_size;
	unsigned int				job_size;
	unsigned int				buffer_size;
};

/* Ctrl */

int cedrus_engine_ctrl_validate(struct cedrus_context *ctx,
				struct v4l2_ctrl *ctrl);
int cedrus_engine_ctrl_prepare(struct cedrus_context *ctx,
			       struct v4l2_ctrl *ctrl);

/* Format */

int cedrus_engine_format_prepare(struct cedrus_context *ctx,
				 struct v4l2_format *format);
int cedrus_engine_format_configure(struct cedrus_context *ctx);

/* Context */

int cedrus_engine_setup(struct cedrus_context *ctx);
void cedrus_engine_cleanup(struct cedrus_context *ctx);

/* Buffer */

int cedrus_engine_buffer_setup(struct cedrus_context *ctx,
			       struct cedrus_buffer *buffer);
void cedrus_engine_buffer_cleanup(struct cedrus_context *ctx,
				  struct cedrus_buffer *buffer);

/* Job */

int cedrus_engine_job_prepare(struct cedrus_context *ctx);
int cedrus_engine_job_configure(struct cedrus_context *ctx);
void cedrus_engine_job_trigger(struct cedrus_context *ctx);
void cedrus_engine_job_finish(struct cedrus_context *ctx, int state);

/* IRQ */

irqreturn_t cedrus_engine_irq_status(struct cedrus_context *ctx);
void cedrus_engine_irq_clear(struct cedrus_context *ctx);
void cedrus_engine_irq_disable(struct cedrus_context *ctx);

#endif
