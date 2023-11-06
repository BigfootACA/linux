/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _CEDRUS_CONTEXT_H_
#define _CEDRUS_CONTEXT_H_

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>

#include "cedrus.h"

struct cedrus_engine;
struct cedrus_proc;

struct cedrus_job {
	struct vb2_queue	*queue_coded;
	struct vb2_queue	*queue_picture;

	struct vb2_v4l2_buffer	*buffer_coded;
	struct vb2_v4l2_buffer	*buffer_picture;
};

struct cedrus_buffer {
	struct v4l2_m2m_buffer	m2m_buffer;
	void			*engine_buffer;
};

struct cedrus_context_v4l2 {
	struct v4l2_fh			fh;

	struct v4l2_ctrl_handler	ctrl_handler;
	struct v4l2_ctrl		**ctrls;

	struct v4l2_format		format_coded;
	struct v4l2_format		format_picture;

	struct v4l2_fract		timeperframe_coded;
	struct v4l2_fract		timeperframe_picture;

	struct v4l2_rect		selection_picture;
};

struct cedrus_context {
	struct cedrus_proc		*proc;
	const struct cedrus_engine	*engine;
	void				*engine_ctx;
	void				*engine_job;

	struct cedrus_context_v4l2	v4l2;
	struct cedrus_job		job;

	unsigned int			bit_depth_coded;
};

static inline void cedrus_buffer_picture_dma(struct cedrus_context *ctx,
					     struct cedrus_buffer *cedrus_buffer,
					     dma_addr_t *luma_addr,
					     dma_addr_t *chroma_addr);
static inline void cedrus_buffer_coded_dma(struct cedrus_context *ctx,
					   struct cedrus_buffer *cedrus_buffer,
					   dma_addr_t *addr, unsigned int *size);

/* Job */

static inline struct cedrus_buffer *
cedrus_job_buffer_coded(struct cedrus_context *ctx)
{
	struct vb2_v4l2_buffer *buffer = ctx->job.buffer_coded;

	return container_of(buffer, struct cedrus_buffer, m2m_buffer.vb);
}

static inline struct cedrus_buffer *
cedrus_job_buffer_picture(struct cedrus_context *ctx)
{
	struct vb2_v4l2_buffer *buffer = ctx->job.buffer_picture;

	return container_of(buffer, struct cedrus_buffer, m2m_buffer.vb);
}

static inline void cedrus_job_buffer_coded_dma(struct cedrus_context *ctx,
					       dma_addr_t *addr,
					       unsigned int *size)
{
	struct cedrus_buffer *buffer = cedrus_job_buffer_coded(ctx);

	cedrus_buffer_coded_dma(ctx, buffer, addr, size);
}

static inline void cedrus_job_buffer_picture_dma(struct cedrus_context *ctx,
						 dma_addr_t *luma_addr,
						 dma_addr_t *chroma_addr)
{
	struct cedrus_buffer *buffer = cedrus_job_buffer_picture(ctx);

	cedrus_buffer_picture_dma(ctx, buffer, luma_addr, chroma_addr);
}

static inline void cedrus_job_buffer_picture_ref_dma(struct cedrus_context *ctx,
						     u64 timestamp,
						     dma_addr_t *luma_addr,
						     dma_addr_t *chroma_addr)
{
	struct vb2_queue *queue = ctx->job.queue_picture;
	struct vb2_buffer *vb2_buffer;
	struct cedrus_buffer *cedrus_buffer;

	vb2_buffer = vb2_find_buffer(queue, timestamp);
	if (!vb2_buffer) {
		*luma_addr = 0;
		*chroma_addr = 0;
		return;
	}

	cedrus_buffer = container_of(vb2_buffer, struct cedrus_buffer,
				     m2m_buffer.vb.vb2_buf);

	cedrus_buffer_picture_dma(ctx, cedrus_buffer, luma_addr, chroma_addr);
}

static inline void *cedrus_job_engine_buffer(struct cedrus_context *ctx)
{
	/* Engine buffer is attached to picture buffer. */
	struct vb2_v4l2_buffer *v4l2_buffer = ctx->job.buffer_picture;
	struct cedrus_buffer *cedrus_buffer =
		container_of(v4l2_buffer, struct cedrus_buffer, m2m_buffer.vb);

	return cedrus_buffer->engine_buffer;
}

/* Ctrl */

extern const struct v4l2_ctrl_ops cedrus_context_ctrl_ops;
struct v4l2_ctrl *cedrus_context_ctrl_find(struct cedrus_context *ctx, u32 id);
void *cedrus_context_ctrl_data(struct cedrus_context *ctx, u32 id);
int cedrus_context_ctrl_value(struct cedrus_context *ctx, u32 id);
int cedrus_context_ctrl_array_count(struct cedrus_context *ctx, u32 id);

/* Engine */

int cedrus_context_engine_update(struct cedrus_context *ctx);

/* Selection */

int cedrus_context_selection_picture_reset(struct cedrus_context *ctx);

/* Job */

void cedrus_context_job_finish(struct cedrus_context *ctx, int state);
int cedrus_context_job_run(struct cedrus_context *ctx);

/* Queue */

bool cedrus_context_queue_busy_check(struct cedrus_context *ctx,
				     unsigned int buffer_type);
bool cedrus_context_queue_streaming_check(struct cedrus_context *ctx,
					  unsigned int buffer_type);

/* Context */

int cedrus_context_setup(struct cedrus_proc *proc, struct cedrus_context *ctx);
void cedrus_context_cleanup(struct cedrus_context *ctx);

#endif
