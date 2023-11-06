// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2016 Florent Revest <florent.revest@free-electrons.com>
 * Copyright 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright 2018-2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#include <linux/pm_runtime.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#include "cedrus.h"
#include "cedrus_context.h"
#include "cedrus_engine.h"
#include "cedrus_proc.h"

/* Ctrl */

struct v4l2_ctrl *cedrus_context_ctrl_find(struct cedrus_context *ctx, u32 id)
{
	struct v4l2_ctrl **ctrls = ctx->v4l2.ctrls;
	unsigned int index = 0;

	while (ctrls[index]) {
		struct v4l2_ctrl *ctrl = ctrls[index];

		if (ctrl->id == id)
			return ctrl;

		index++;
	}

	return NULL;
}

void *cedrus_context_ctrl_data(struct cedrus_context *ctx, u32 id)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&ctx->v4l2.ctrl_handler, id);

	if (WARN_ON(!ctrl))
		return NULL;

	return ctrl->p_cur.p;
}

int cedrus_context_ctrl_value(struct cedrus_context *ctx, u32 id)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&ctx->v4l2.ctrl_handler, id);

	if (WARN_ON(!ctrl))
		return 0;

	return ctrl->cur.val;
}

int cedrus_context_ctrl_array_count(struct cedrus_context *ctx, u32 id)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&ctx->v4l2.ctrl_handler, id);

	if (WARN_ON(!ctrl))
		return 0;

	return ctrl->elems;
}

static int cedrus_context_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct cedrus_context *ctx = ctrl->priv;

	/* XXX: monitor this when using with request, plan is to not use it
	 * during streaming, maybe needs a check here. */

	return cedrus_engine_ctrl_prepare(ctx, ctrl);
}

static int cedrus_context_try_ctrl(struct v4l2_ctrl *ctrl)
{
	struct cedrus_context *ctx = ctrl->priv;

	return cedrus_engine_ctrl_validate(ctx, ctrl);
}

const struct v4l2_ctrl_ops cedrus_context_ctrl_ops = {
	.s_ctrl		= cedrus_context_s_ctrl,
	.try_ctrl	= cedrus_context_try_ctrl,
};

static int cedrus_context_ctrls_setup(struct cedrus_context *ctx)
{
	struct cedrus_proc *proc = ctx->proc;
	struct cedrus_context_v4l2 *v4l2 = &ctx->v4l2;
	struct v4l2_device *v4l2_dev = &proc->dev->v4l2.v4l2_dev;
	struct v4l2_ctrl_handler *handler = &v4l2->ctrl_handler;
	unsigned int count = 0;
	unsigned int index = 0;
	unsigned int size;
	unsigned int i, j;
	int ret;

	/* TODO: Also get ctrl_configs from proc for shared controls. */

	for (i = 0; i < proc->engines_count; i++)
		count += proc->engines[i]->ctrl_configs_count;

	if (WARN_ON(!count))
		return -ENODEV;

	/* Last entry is a zero sentinel. */
	size = sizeof(*v4l2->ctrls) * (count + 1);

	v4l2->ctrls = kzalloc(size, GFP_KERNEL);
	if (!v4l2->ctrls)
		return -ENOMEM;

	ret = v4l2_ctrl_handler_init(handler, count);
	if (ret) {
		v4l2_err(v4l2_dev, "failed to initialize control handler\n");
		goto error_ctrls;
	}

	for (i = 0; i < proc->engines_count; i++) {
		const struct cedrus_engine *engine = proc->engines[i];

		for (j = 0; j < engine->ctrl_configs_count; j++) {
			const struct v4l2_ctrl_config *ctrl_config =
				&engine->ctrl_configs[j];
			struct v4l2_ctrl *ctrl;

			ctrl = v4l2_ctrl_new_custom(handler, ctrl_config, ctx);
			if (handler->error) {
				v4l2_err(v4l2_dev,
					 "failed to create %s control (%d)\n",
					 v4l2_ctrl_get_name(ctrl_config->id),
					 handler->error);
				ret = handler->error;
				goto error_handler;
			}

			v4l2->ctrls[index] = ctrl;
			index++;
		}
	}

	ctx->v4l2.fh.ctrl_handler = handler;

	ret = v4l2_ctrl_handler_setup(handler);
	if (ret)
		goto error_handler;

	return 0;

error_handler:
	v4l2_ctrl_handler_free(handler);

error_ctrls:
	kfree(v4l2->ctrls);

	return ret;
}

static void cedrus_context_ctrls_cleanup(struct cedrus_context *ctx)
{
	v4l2_ctrl_handler_free(&ctx->v4l2.ctrl_handler);
	kfree(ctx->v4l2.ctrls);
}

/* Engine */

int cedrus_context_engine_update(struct cedrus_context *ctx)
{
	unsigned int pixelformat = ctx->v4l2.format_coded.fmt.pix.pixelformat;
	struct vb2_queue *queue = v4l2_m2m_get_src_vq(ctx->v4l2.fh.m2m_ctx);
	const struct cedrus_engine *engine;

	engine = cedrus_proc_engine_find_format(ctx->proc, pixelformat);
	if (WARN_ON(!engine))
		return -ENODEV;

	ctx->engine = engine;

	if (engine->slice_based)
		queue->subsystem_flags |=
			VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF;
	else
		queue->subsystem_flags &=
			~VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF;

	return 0;
}

/* Selection */

int cedrus_context_selection_picture_reset(struct cedrus_context *ctx)
{
	struct v4l2_pix_format *pix_format = &ctx->v4l2.format_picture.fmt.pix;
	struct v4l2_rect *selection = &ctx->v4l2.selection_picture;

	selection->left = 0;
	selection->top = 0;
	selection->width = pix_format->width;
	selection->height = pix_format->height;

	return 0;
}

/* Job */

void cedrus_context_job_finish(struct cedrus_context *ctx, int state)
{
	struct cedrus_proc *proc = ctx->proc;
	struct v4l2_m2m_dev *m2m_dev = proc->dev->v4l2.m2m_dev;
	struct v4l2_m2m_ctx *m2m_ctx = ctx->v4l2.fh.m2m_ctx;

	cedrus_engine_job_finish(ctx, state);
	memset(&ctx->job, 0, sizeof(ctx->job));

	v4l2_m2m_buf_done_and_job_finish(m2m_dev, m2m_ctx, state);
}

int cedrus_context_job_run(struct cedrus_context *ctx)
{
	struct cedrus_proc *proc = ctx->proc;
	struct cedrus_device *cedrus_dev = proc->dev;
	struct cedrus_job *job = &ctx->job;
	struct v4l2_m2m_ctx *m2m_ctx = ctx->v4l2.fh.m2m_ctx;
	struct v4l2_device *v4l2_dev = &cedrus_dev->v4l2.v4l2_dev;
	struct v4l2_ctrl_handler *ctrl_handler = &ctx->v4l2.ctrl_handler;
	struct vb2_v4l2_buffer *buffer_src, *buffer_dst;
	struct vb2_queue *queue_src, *queue_dst;
	struct media_request *req = NULL;
	int ret;

	/* Clear job data. */

	memset(job, 0, sizeof(*job));

	if (ctx->engine_job)
		memset(ctx->engine_job, 0, ctx->engine->job_size);

	/* Prepare job pointers. */

	queue_src = v4l2_m2m_get_src_vq(m2m_ctx);
	queue_dst = v4l2_m2m_get_dst_vq(m2m_ctx);
	buffer_src = v4l2_m2m_next_src_buf(m2m_ctx);
	buffer_dst = v4l2_m2m_next_dst_buf(m2m_ctx);

	job->queue_coded = queue_src;
	job->queue_picture = queue_dst;
	job->buffer_coded = buffer_src;
	job->buffer_picture = buffer_dst;

	/* Setup request controls. */

	req = ctx->job.buffer_coded->vb2_buf.req_obj.req;
	if (req)
		v4l2_ctrl_request_setup(req, ctrl_handler);

	/* Copy buffer metadata (timestamp). */

	v4l2_m2m_buf_copy_metadata(buffer_src, buffer_dst, true);

	/* Prepare engine job. */

	ret = cedrus_engine_job_prepare(ctx);
	if (ret) {
		v4l2_err(v4l2_dev, "failed to prepare engine job: %d\n", ret);
		goto error_ctrl;
	}

	/* Configure coded and picture formats. */

	ret = cedrus_engine_format_configure(ctx);
	if (ret) {
		v4l2_err(v4l2_dev, "failed to configure coded format: %d\n",
			 ret);
		goto error_ctrl;
	}

	ret = cedrus_proc_format_picture_configure(ctx);
	if (ret) {
		v4l2_err(v4l2_dev, "failed to configure picture format: %d\n",
			 ret);
		goto error_ctrl;
	}

	/* Configure engine job. */

	ret = cedrus_engine_job_configure(ctx);
	if (ret) {
		v4l2_err(v4l2_dev, "failed to configure engine job: %d\n", ret);
		goto error_ctrl;
	}

	/* Complete request controls. */

	if (req)
		v4l2_ctrl_request_complete(req, ctrl_handler);

	/* Keep track of the active context (in case of spurious IRQs). */

	cedrus_proc_context_active_update(ctx->proc, ctx);

	/* Schedule the global watchdog. */

	schedule_delayed_work(&cedrus_dev->watchdog_work,
			      msecs_to_jiffies(2000));

	/* Trigger engine job. */

	cedrus_engine_job_trigger(ctx);

	return 0;

error_ctrl:
	if (req)
		v4l2_ctrl_request_complete(req, ctrl_handler);

	cedrus_context_job_finish(ctx, VB2_BUF_STATE_ERROR);

	return ret;
}

/* Queue */

bool cedrus_context_queue_busy_check(struct cedrus_context *ctx,
				     unsigned int buffer_type)
{
	struct vb2_queue *queue;

	queue = v4l2_m2m_get_vq(ctx->v4l2.fh.m2m_ctx, buffer_type);
	if (WARN_ON(!queue))
		return true;

	return vb2_is_busy(queue);
}

bool cedrus_context_queue_streaming_check(struct cedrus_context *ctx,
					  unsigned int buffer_type)
{
	struct vb2_queue *queue;

	queue = v4l2_m2m_get_vq(ctx->v4l2.fh.m2m_ctx, buffer_type);
	if (WARN_ON(!queue))
		return true;

	return vb2_is_streaming(queue);
}

static int cedrus_context_queue_setup(struct vb2_queue *queue,
				      unsigned int *buffers_count,
				      unsigned int *planes_count,
				      unsigned int sizes[],
				      struct device *alloc_devs[])
{
	struct cedrus_context *ctx = vb2_get_drv_priv(queue);
	unsigned int format_type =
		cedrus_proc_format_type(ctx->proc, queue->type);
	struct v4l2_format *format;
	struct v4l2_pix_format *pix_format;

	if (format_type == CEDRUS_FORMAT_TYPE_CODED)
		format = &ctx->v4l2.format_coded;
	else
		format = &ctx->v4l2.format_picture;

	pix_format = &format->fmt.pix;

	if (*planes_count) {
		if (sizes[0] < pix_format->sizeimage)
			return -EINVAL;
	} else {
		sizes[0] = pix_format->sizeimage;
		*planes_count = 1;
	}

	return 0;
}

static void cedrus_context_queue_cleanup(struct vb2_queue *queue, bool error)
{
	struct cedrus_context *ctx = vb2_get_drv_priv(queue);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->v4l2.fh.m2m_ctx;
	struct vb2_v4l2_buffer *v4l2_buffer;
	struct media_request *req;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(queue->type))
			v4l2_buffer = v4l2_m2m_src_buf_remove(m2m_ctx);
		else
			v4l2_buffer = v4l2_m2m_dst_buf_remove(m2m_ctx);

		if (!v4l2_buffer)
			return;

		req = v4l2_buffer->vb2_buf.req_obj.req;
		if (req)
			v4l2_ctrl_request_complete(req,
						   &ctx->v4l2.ctrl_handler);

		v4l2_m2m_buf_done(v4l2_buffer, error ? VB2_BUF_STATE_ERROR :
				  VB2_BUF_STATE_QUEUED);
	}
}

static int cedrus_context_buffer_init(struct vb2_buffer *vb2_buffer)
{
	struct cedrus_context *ctx = vb2_get_drv_priv(vb2_buffer->vb2_queue);
	struct cedrus_buffer *cedrus_buffer =
		cedrus_buffer_from_vb2(vb2_buffer);
	const struct cedrus_engine *engine = ctx->engine;
	unsigned int format_type =
		cedrus_proc_format_type(ctx->proc, vb2_buffer->type);
	int ret;

	if (!engine->buffer_size)
		return 0;

	/* Allocate engine-specific buffer for picture buffers only. */
	if (format_type == CEDRUS_FORMAT_TYPE_PICTURE) {
		cedrus_buffer->engine_buffer = kzalloc(engine->buffer_size,
						       GFP_KERNEL);
		if (!cedrus_buffer->engine_buffer)
			return -ENOMEM;

		ret = cedrus_engine_buffer_setup(ctx, cedrus_buffer);
		if (ret)
			goto error_buffer;
	}

	return 0;

error_buffer:
	kfree(cedrus_buffer->engine_buffer);
	cedrus_buffer->engine_buffer = NULL;

	return ret;
}

static void cedrus_context_buffer_cleanup(struct vb2_buffer *vb2_buffer)
{
	struct cedrus_context *ctx = vb2_get_drv_priv(vb2_buffer->vb2_queue);
	struct cedrus_buffer *cedrus_buffer =
		cedrus_buffer_from_vb2(vb2_buffer);
	unsigned int format_type =
		cedrus_proc_format_type(ctx->proc, vb2_buffer->type);

	if (format_type == CEDRUS_FORMAT_TYPE_PICTURE &&
	    cedrus_buffer->engine_buffer) {
		cedrus_engine_buffer_cleanup(ctx, cedrus_buffer);
		kfree(cedrus_buffer->engine_buffer);
		cedrus_buffer->engine_buffer = NULL;
	}
}

static int cedrus_context_buffer_prepare(struct vb2_buffer *vb2_buffer)
{
	struct vb2_queue *queue = vb2_buffer->vb2_queue;
	struct cedrus_context *ctx = vb2_get_drv_priv(queue);
	unsigned int format_type =
		cedrus_proc_format_type(ctx->proc, queue->type);
	struct v4l2_format *format;
	struct v4l2_pix_format *pix_format;

	if (format_type == CEDRUS_FORMAT_TYPE_CODED)
		format = &ctx->v4l2.format_coded;
	else
		format = &ctx->v4l2.format_picture;

	pix_format = &format->fmt.pix;

	if (vb2_plane_size(vb2_buffer, 0) < pix_format->sizeimage)
		return -EINVAL;

	/* The picture buffer bytesused is always from the driver side. */
	if (format_type == CEDRUS_FORMAT_TYPE_PICTURE)
		vb2_set_plane_payload(vb2_buffer, 0, pix_format->sizeimage);

	return 0;
}

static void cedrus_context_buffer_queue(struct vb2_buffer *vb2_buffer)
{
	struct cedrus_context *ctx = vb2_get_drv_priv(vb2_buffer->vb2_queue);
	struct vb2_v4l2_buffer *v4l2_buffer = to_vb2_v4l2_buffer(vb2_buffer);

	v4l2_m2m_buf_queue(ctx->v4l2.fh.m2m_ctx, v4l2_buffer);
}

static int cedrus_context_buffer_validate(struct vb2_buffer *vb2_buffer)
{
	struct vb2_v4l2_buffer *v4l2_buffer = to_vb2_v4l2_buffer(vb2_buffer);

	v4l2_buffer->field = V4L2_FIELD_NONE;

	return 0;
}

static void cedrus_context_buffer_complete(struct vb2_buffer *vb2_buffer)
{
	struct cedrus_context *ctx = vb2_get_drv_priv(vb2_buffer->vb2_queue);

	v4l2_ctrl_request_complete(vb2_buffer->req_obj.req,
				   &ctx->v4l2.ctrl_handler);
}

static int cedrus_context_start_streaming(struct vb2_queue *queue,
					  unsigned int count)
{
	struct cedrus_context *ctx = vb2_get_drv_priv(queue);
	const struct cedrus_engine *engine = ctx->engine;
	struct device *dev = ctx->proc->dev->dev;
	unsigned int format_type =
		cedrus_proc_format_type(ctx->proc, queue->type);
	int ret;

	if (WARN_ON(!engine))
		return -ENODEV;

	/* Only start the engine from the coded queue. */
	if (format_type != CEDRUS_FORMAT_TYPE_CODED)
		return 0;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		goto error_queue;

	if (engine->ctx_size > 0) {
		ctx->engine_ctx = kzalloc(engine->ctx_size, GFP_KERNEL);
		if (!ctx->engine_ctx) {
			ret = -ENOMEM;
			goto error_pm;
		}
	}

	if (engine->job_size > 0) {
		ctx->engine_job = kzalloc(engine->job_size, GFP_KERNEL);
		if (!ctx->engine_job) {
			ret = -ENOMEM;
			goto error_alloc_engine;
		}
	}

	ret = cedrus_engine_setup(ctx);
	if (ret)
		goto error_alloc_job;

	return 0;

error_alloc_job:
	if (ctx->engine_job) {
		kfree(ctx->engine_job);
		ctx->engine_job = NULL;
	}

error_alloc_engine:
	if (ctx->engine_ctx) {
		kfree(ctx->engine_ctx);
		ctx->engine_ctx = NULL;
	}

error_pm:
	pm_runtime_put(dev);

error_queue:
	cedrus_context_queue_cleanup(queue, false);

	return ret;
}

static void cedrus_context_stop_streaming(struct vb2_queue *queue)
{
	struct cedrus_context *ctx = vb2_get_drv_priv(queue);
	const struct cedrus_engine *engine = ctx->engine;
	struct device *dev = ctx->proc->dev->dev;
	unsigned int format_type =
		cedrus_proc_format_type(ctx->proc, queue->type);

	if (WARN_ON(!engine))
		return;

	/* Only stop the engine from the coded queue. */
	if (format_type != CEDRUS_FORMAT_TYPE_CODED)
		return;

	cedrus_engine_cleanup(ctx);

	if (ctx->engine_job) {
		kfree(ctx->engine_job);
		ctx->engine_job = NULL;
	}

	if (ctx->engine_ctx) {
		kfree(ctx->engine_ctx);
		ctx->engine_ctx = NULL;
	}

	cedrus_context_queue_cleanup(queue, true);

	pm_runtime_put(dev);
}

static const struct vb2_ops cedrus_context_queue_ops = {
	.queue_setup		= cedrus_context_queue_setup,
	.buf_init		= cedrus_context_buffer_init,
	.buf_cleanup		= cedrus_context_buffer_cleanup,
	.buf_prepare		= cedrus_context_buffer_prepare,
	.buf_queue		= cedrus_context_buffer_queue,
	.buf_out_validate	= cedrus_context_buffer_validate,
	.buf_request_complete	= cedrus_context_buffer_complete,
	.start_streaming	= cedrus_context_start_streaming,
	.stop_streaming		= cedrus_context_stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static int cedrus_context_queue_init(void *private, struct vb2_queue *src_queue,
				     struct vb2_queue *dst_queue)
{
	struct cedrus_context *ctx = private;
	struct cedrus_proc *proc = ctx->proc;
	int ret;

	/* Source (output) */

	src_queue->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_queue->io_modes = VB2_MMAP | VB2_DMABUF;
	src_queue->buf_struct_size = sizeof(struct cedrus_buffer);
	src_queue->ops = &cedrus_context_queue_ops;
	src_queue->mem_ops = &vb2_dma_contig_memops;
	src_queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_queue->supports_requests = true;
	src_queue->requires_requests = true;
	src_queue->lock = &proc->v4l2.lock;
	src_queue->dev = proc->dev->dev;
	src_queue->drv_priv = ctx;

	ret = vb2_queue_init(src_queue);
	if (ret)
		return ret;

	/* Destination (capture) */

	dst_queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_queue->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_queue->buf_struct_size = sizeof(struct cedrus_buffer);
	dst_queue->ops = &cedrus_context_queue_ops;
	dst_queue->mem_ops = &vb2_dma_contig_memops;
	dst_queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_queue->lock = &proc->v4l2.lock;
	dst_queue->dev = proc->dev->dev;
	dst_queue->drv_priv = ctx;

	return vb2_queue_init(dst_queue);
}

/* Context */

int cedrus_context_setup(struct cedrus_proc *proc, struct cedrus_context *ctx)
{
	struct v4l2_device *v4l2_dev = &proc->dev->v4l2.v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev = proc->dev->v4l2.m2m_dev;
	struct video_device *video_dev = &proc->v4l2.video_dev;
	struct v4l2_fh *fh = &ctx->v4l2.fh;
	int ret;

	ctx->proc = proc;
	ctx->engine = proc->engines[0];

	/* V4L2 File Handler */

	v4l2_fh_init(fh, video_dev);

	/* V4L2 M2M */

	fh->m2m_ctx = v4l2_m2m_ctx_init(m2m_dev, ctx,
					cedrus_context_queue_init);
	if (IS_ERR(fh->m2m_ctx)) {
		v4l2_err(v4l2_dev, "failed to initialize V4L2 M2M context\n");
		return PTR_ERR(fh->m2m_ctx);
	}

	/* Ctrls */

	ret = cedrus_context_ctrls_setup(ctx);
	if (ret)
		goto error_v4l2_m2m;

	/* Format */

	ret = cedrus_proc_format_setup(ctx);
	if (ret)
		goto error_ctrls;

	/* V4L2 File Handler */

	v4l2_fh_add(fh);

	return 0;

error_ctrls:
	cedrus_context_ctrls_cleanup(ctx);

error_v4l2_m2m:
	v4l2_m2m_ctx_release(fh->m2m_ctx);

	return ret;
}

void cedrus_context_cleanup(struct cedrus_context *ctx)
{
	struct v4l2_fh *fh = &ctx->v4l2.fh;

	cedrus_proc_context_active_clear(ctx->proc, ctx);

	v4l2_fh_del(fh);
	cedrus_context_ctrls_cleanup(ctx);
	v4l2_m2m_ctx_release(fh->m2m_ctx);
	v4l2_fh_exit(fh);
}
