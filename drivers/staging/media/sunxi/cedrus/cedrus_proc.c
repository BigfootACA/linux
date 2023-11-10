// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#include <linux/types.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>

#include "cedrus.h"
#include "cedrus_context.h"
#include "cedrus_engine.h"
#include "cedrus_proc.h"

/* Context */

void cedrus_proc_context_active_update(struct cedrus_proc *proc,
				       struct cedrus_context *ctx)
{
	unsigned long flags;

	spin_lock_irqsave(&proc->ctx_active_lock, flags);

	proc->ctx_active = ctx;

	spin_unlock_irqrestore(&proc->ctx_active_lock, flags);
}

void cedrus_proc_context_active_clear(struct cedrus_proc *proc,
				      struct cedrus_context *ctx)
{
	unsigned long flags;

	spin_lock_irqsave(&proc->ctx_active_lock, flags);

	if (proc->ctx_active == ctx)
		proc->ctx_active = NULL;

	spin_unlock_irqrestore(&proc->ctx_active_lock, flags);
}

/* Format */

unsigned int cedrus_proc_format_find_first(struct cedrus_proc *proc,
					   unsigned int format_type)
{
	unsigned int i;

	for (i = 0; i < proc->formats_count; i++) {
		struct cedrus_format *format = &proc->formats[i];

		if (format->type == format_type)
			return format->pixelformat;
	}

	return 0;
}

static bool cedrus_proc_format_check(struct cedrus_proc *proc,
				     unsigned int pixelformat,
				     unsigned int format_type)
{
	unsigned int i;

	for (i = 0; i < proc->formats_count; i++) {
		struct cedrus_format *format = &proc->formats[i];

		if (format->pixelformat == pixelformat &&
		    format->type == format_type)
			return true;
	}

	return false;
}

int cedrus_proc_format_coded_prepare(struct cedrus_context *ctx,
				     struct v4l2_format *format)
{
	struct cedrus_proc *proc = ctx->proc;
	struct v4l2_pix_format *pix_format = &format->fmt.pix;

	/* Select the first coded format in case of invalid format. */
	if (!cedrus_proc_format_check(proc, pix_format->pixelformat,
				      CEDRUS_FORMAT_TYPE_CODED))
		pix_format->pixelformat =
			cedrus_proc_format_find_first(proc,
						      CEDRUS_FORMAT_TYPE_CODED);

	return cedrus_engine_format_prepare(ctx, format);
}

int cedrus_proc_format_picture_prepare(struct cedrus_context *ctx,
				       struct v4l2_format *format)
{
	struct cedrus_proc *proc = ctx->proc;
	struct v4l2_pix_format *pix_format = &format->fmt.pix;

	if (WARN_ON(!proc->ops || !proc->ops->format_picture_prepare))
		return -ENODEV;

	/* Select the first picture format in case of invalid format. */
	if (!cedrus_proc_format_check(proc, pix_format->pixelformat,
				      CEDRUS_FORMAT_TYPE_PICTURE))
		pix_format->pixelformat =
			cedrus_proc_format_find_first(proc,
						      CEDRUS_FORMAT_TYPE_PICTURE);

	return proc->ops->format_picture_prepare(ctx, format);
}


int cedrus_proc_format_picture_configure(struct cedrus_context *ctx)
{
	struct cedrus_proc *proc = ctx->proc;

	if (WARN_ON(!proc->ops || !proc->ops->format_picture_configure))
		return -ENODEV;

	return proc->ops->format_picture_configure(ctx);
}

int cedrus_proc_format_setup(struct cedrus_context *ctx)
{
	struct cedrus_proc *proc = ctx->proc;

	if (WARN_ON(!proc->ops || !proc->ops->format_setup))
		return -ENODEV;

	return proc->ops->format_setup(ctx);
}

int cedrus_proc_format_propagate(struct cedrus_context *ctx,
				 unsigned int format_type)
{
	struct cedrus_proc *proc = ctx->proc;

	if (WARN_ON(!proc->ops || !proc->ops->format_propagate))
		return -ENODEV;

	return proc->ops->format_propagate(ctx, format_type);
}

static bool cedrus_proc_format_dynamic_check(struct cedrus_context *ctx,
					     struct v4l2_format *format)
{
	struct cedrus_proc *proc = ctx->proc;

	if (WARN_ON(!proc->ops || !proc->ops->format_dynamic_check))
		return false;

	return proc->ops->format_dynamic_check(ctx, format);
}

static int cedrus_proc_formats_setup(struct cedrus_proc *proc,
				     const struct cedrus_proc_config *config)
{
	struct cedrus_device *cedrus_dev = proc->dev;
	struct device *dev = cedrus_dev->dev;
	unsigned int index = 0;
	unsigned int count, size;
	unsigned int i;

	/* Each engine has its own coded format. */
	count = proc->engines_count;

	for (i = 0; i < config->formats_count; i++) {
		const struct cedrus_format *format_config = &config->formats[i];
		bool check =
			cedrus_capabilities_check(cedrus_dev,
						  format_config->capabilities);

		if (!check || WARN_ON(!format_config->pixelformat))
			continue;

		count++;
	}

	if (!count)
		return -ENODEV;

	size = count * sizeof(*proc->formats);

	proc->formats = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!proc->formats)
		return -ENOMEM;

	proc->formats_count = count;

	for (i = 0; i < proc->engines_count; i++) {
		const struct cedrus_engine *engine = proc->engines[i];
		struct cedrus_format *format = &proc->formats[index];

		format->pixelformat = engine->pixelformat;
		format->type = CEDRUS_FORMAT_TYPE_CODED;

		index++;
	}

	for (i = 0; i < config->formats_count; i++) {
		const struct cedrus_format *format_config = &config->formats[i];
		struct cedrus_format *format = &proc->formats[index];
		bool check =
			cedrus_capabilities_check(cedrus_dev,
						  format_config->capabilities);

		if (!check || !format_config->pixelformat)
			continue;

		*format = *format_config;

		index++;
	}

	return 0;
}

/* Size */

static int cedrus_proc_size_picture_enum(struct cedrus_context *ctx,
					 struct v4l2_frmsizeenum *frmsizeenum)
{
	struct cedrus_proc *proc = ctx->proc;

	if (WARN_ON(!proc->ops || !proc->ops->size_picture_enum))
		return -ENODEV;

	return proc->ops->size_picture_enum(ctx, frmsizeenum);
}

/* Engine */

const struct cedrus_engine *
cedrus_proc_engine_find_format(struct cedrus_proc *proc,
			       unsigned int pixelformat)
{
	unsigned int i;

	for (i = 0; i < proc->engines_count; i++) {
		const struct cedrus_engine *engine = proc->engines[i];

		if (engine->pixelformat == pixelformat)
			return engine;
	}

	return NULL;
}

static int cedrus_proc_engines_setup(struct cedrus_proc *proc,
				     const struct cedrus_proc_config *config)
{
	struct cedrus_device *cedrus_dev = proc->dev;
	struct device *dev = cedrus_dev->dev;
	unsigned int count = 0;
	unsigned int index = 0;
	unsigned int size;
	unsigned int i;

	for (i = 0; i < config->engines_count; i++) {
		const struct cedrus_engine *engine = config->engines[i];
		bool check = cedrus_capabilities_check(cedrus_dev,
						       engine->capabilities);

		if (!check || WARN_ON(!engine->pixelformat) ||
		    WARN_ON(!engine->frmsize))
			continue;

		count++;
	}

	if (!count)
		return -ENODEV;

	size = count * sizeof(*proc->engines);
	proc->engines = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!proc->engines)
		return -ENOMEM;

	proc->engines_count = count;

	for (i = 0; i < config->engines_count; i++) {
		const struct cedrus_engine *engine = config->engines[i];
		bool check = cedrus_capabilities_check(cedrus_dev,
						       engine->capabilities);

		if (!check || !engine->pixelformat || !engine->frmsize)
			continue;

		proc->engines[index] = engine;
		index++;
	}

	return 0;
}

/* Video Device */

static int cedrus_proc_querycap(struct file *file, void *private,
				struct v4l2_capability *capability)
{
	struct cedrus_proc *proc = video_drvdata(file);
	struct video_device *video_dev = &proc->v4l2.video_dev;

	strscpy(capability->driver, CEDRUS_NAME, sizeof(capability->driver));
	strscpy(capability->card, video_dev->name, sizeof(capability->card));
	snprintf(capability->bus_info, sizeof(capability->bus_info),
		 "platform:%s", CEDRUS_NAME);

	return 0;
}

static int cedrus_proc_enum_fmt(struct file *file, void *private,
				struct v4l2_fmtdesc *fmtdesc)
{
	struct cedrus_context *ctx =
		container_of(file->private_data, struct cedrus_context,
			     v4l2.fh);
	struct cedrus_proc *proc = ctx->proc;
	unsigned int format_type = cedrus_proc_format_type(proc, fmtdesc->type);
	unsigned int index = 0;
	unsigned int i;

	for (i = 0; i < proc->formats_count; i++) {
		struct cedrus_format *format = &proc->formats[i];

		if (format->type != format_type)
			continue;

		if (fmtdesc->index == index) {
			fmtdesc->pixelformat = format->pixelformat;

			if (format_type == CEDRUS_FORMAT_TYPE_CODED)
				fmtdesc->flags |= V4L2_FMT_FLAG_COMPRESSED;

			if (proc->role == CEDRUS_ROLE_ENCODER &&
			    format_type == CEDRUS_FORMAT_TYPE_CODED)
				fmtdesc->flags |=
					V4L2_FMT_FLAG_ENC_CAP_FRAME_INTERVAL;
			return 0;
		} else if (fmtdesc->index < index) {
			break;
		}

		index++;
	}

	return -EINVAL;
}

static int cedrus_proc_g_fmt(struct file *file, void *private,
			     struct v4l2_format *format)
{
	struct cedrus_context *ctx =
		container_of(file->private_data, struct cedrus_context,
			     v4l2.fh);
	unsigned int format_type =
		cedrus_proc_format_type(ctx->proc, format->type);

	if (format_type == CEDRUS_FORMAT_TYPE_CODED)
		*format = ctx->v4l2.format_coded;
	else
		*format = ctx->v4l2.format_picture;

	return 0;
}

static int cedrus_proc_s_fmt(struct file *file, void *private,
			     struct v4l2_format *format)
{
	struct cedrus_context *ctx =
		container_of(file->private_data, struct cedrus_context,
			     v4l2.fh);
	unsigned int format_type =
		cedrus_proc_format_type(ctx->proc, format->type);
	bool dynamic = cedrus_proc_format_dynamic_check(ctx, format);
	bool busy = cedrus_context_queue_busy_check(ctx, format->type);
	int ret;

	if (!dynamic && busy)
		return -EINVAL;

	/* Prepare format. */
	if (format_type == CEDRUS_FORMAT_TYPE_CODED)
		ret = cedrus_proc_format_coded_prepare(ctx, format);
	else
		ret = cedrus_proc_format_picture_prepare(ctx, format);

	if (ret)
		return ret;

	/* Update prepared format. */
	if (format_type == CEDRUS_FORMAT_TYPE_CODED)
		ctx->v4l2.format_coded = *format;
	else
		ctx->v4l2.format_picture = *format;

	/* Propagate format. */
	ret = cedrus_proc_format_propagate(ctx, format_type);
	if (ret)
		return ret;

	/* Update the current engine from the coded format. */
	if (format_type == CEDRUS_FORMAT_TYPE_CODED) {
		ret = cedrus_context_engine_update(ctx);
		if (ret)
			return ret;
	}

	return 0;
}

static int cedrus_proc_try_fmt(struct file *file, void *private,
			       struct v4l2_format *format)
{
	struct cedrus_context *ctx =
		container_of(file->private_data, struct cedrus_context,
			     v4l2.fh);
	unsigned int format_type =
		cedrus_proc_format_type(ctx->proc, format->type);

	if (format_type == CEDRUS_FORMAT_TYPE_CODED)
		return cedrus_proc_format_coded_prepare(ctx, format);
	else
		return cedrus_proc_format_picture_prepare(ctx, format);
}

static int cedrus_proc_enum_framesizes(struct file *file, void *private,
				       struct v4l2_frmsizeenum *frmsizeenum)
{
	struct cedrus_context *ctx =
		container_of(file->private_data, struct cedrus_context,
			     v4l2.fh);
	struct cedrus_proc *proc = ctx->proc;
	u32 pixelformat = frmsizeenum->pixel_format;
	const struct cedrus_engine *engine;
	bool check;

	if (frmsizeenum->index > 0)
		return -EINVAL;

	frmsizeenum->type = V4L2_FRMSIZE_TYPE_STEPWISE;

	/* Coded frame sizes come statically from the engine. */
	engine = cedrus_proc_engine_find_format(proc, pixelformat);
	if (engine) {
		frmsizeenum->stepwise = *engine->frmsize;
		return 0;
	}

	/* Picture frame sizes come dynamically from the proc. */
	check = cedrus_proc_format_check(proc, pixelformat,
					 CEDRUS_FORMAT_TYPE_PICTURE);
	if (check)
		return cedrus_proc_size_picture_enum(ctx, frmsizeenum);

	return -EINVAL;
}

static int cedrus_proc_enum_frameintervals(struct file *file, void *private,
					   struct v4l2_frmivalenum *frmivalenum)
{
	struct v4l2_frmsizeenum frmsizeenum = { 0 };
	unsigned int width = frmivalenum->width;
	unsigned int height = frmivalenum->height;
	int ret;

	if (frmivalenum->index > 0)
		return -EINVAL;

	/* First check that the provided format and dimensions are valid. */
	frmsizeenum.pixel_format = frmivalenum->pixel_format;
	ret = cedrus_proc_enum_framesizes(file, private, &frmsizeenum);
	if (ret)
		return ret;

	if (width < frmsizeenum.stepwise.min_width ||
	    width > frmsizeenum.stepwise.max_width ||
	    height < frmsizeenum.stepwise.min_height ||
	    height > frmsizeenum.stepwise.max_height)
		return -EINVAL;

	/* Any possible frame interval is acceptable. */
	frmivalenum->type = V4L2_FRMIVAL_TYPE_CONTINUOUS;
	frmivalenum->stepwise.min.numerator = 1;
	frmivalenum->stepwise.min.denominator = USHRT_MAX;
	frmivalenum->stepwise.max.numerator = USHRT_MAX;
	frmivalenum->stepwise.max.denominator = 1;
	frmivalenum->stepwise.step.numerator = 1;
	frmivalenum->stepwise.step.denominator = 1;

	return 0;
}

static int cedrus_proc_g_selection(struct file *file, void *private,
				   struct v4l2_selection *selection)
{
	struct cedrus_context *ctx =
		container_of(file->private_data, struct cedrus_context,
			     v4l2.fh);
	unsigned int format_type =
		cedrus_proc_format_type(ctx->proc, selection->type);
	struct v4l2_pix_format *pix_format =
		&ctx->v4l2.format_picture.fmt.pix;

	if (format_type != CEDRUS_FORMAT_TYPE_PICTURE)
		return -EINVAL;

	switch (selection->target) {
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		selection->r.top = 0;
		selection->r.left = 0;
		selection->r.width = pix_format->width;
		selection->r.height = pix_format->height;
		return 0;
	case V4L2_SEL_TGT_CROP:
		selection->r = ctx->v4l2.selection_picture;
		return 0;
	}

	return -EINVAL;
}

static int cedrus_proc_s_selection(struct file *file, void *private,
				   struct v4l2_selection *selection)
{
	struct cedrus_context *ctx =
		container_of(file->private_data, struct cedrus_context,
			     v4l2.fh);
	unsigned int format_type =
		cedrus_proc_format_type(ctx->proc, selection->type);
	struct v4l2_pix_format *pix_format =
		&ctx->v4l2.format_picture.fmt.pix;
	unsigned int width_max, height_max;

	if (format_type != CEDRUS_FORMAT_TYPE_PICTURE)
		return -EINVAL;

	switch (selection->target) {
	case V4L2_SEL_TGT_CROP:
		/* Even dimensions are expected by most codecs. */
		selection->r.left = round_up(selection->r.left, 2);
		selection->r.top = round_up(selection->r.top, 2);

		width_max = pix_format->width - selection->r.left;
		height_max = pix_format->height - selection->r.top;

		selection->r.width = clamp(selection->r.width, 2U, width_max);
		selection->r.height = clamp(selection->r.height, 2U,
					    height_max);

		ctx->v4l2.selection_picture = selection->r;
		return 0;
	}

	return -EINVAL;
}

static int cedrus_proc_g_parm(struct file *file, void *private,
			      struct v4l2_streamparm *streamparm)
{
	struct cedrus_context *ctx =
		container_of(file->private_data, struct cedrus_context,
			     v4l2.fh);
	unsigned int format_type =
		cedrus_proc_format_type(ctx->proc, streamparm->type);
	struct v4l2_fract *timeperframe;

	if (format_type == CEDRUS_FORMAT_TYPE_CODED)
		timeperframe = &ctx->v4l2.timeperframe_coded;
	else
		timeperframe = &ctx->v4l2.timeperframe_picture;

	if (V4L2_TYPE_IS_OUTPUT(streamparm->type)) {
		streamparm->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
		streamparm->parm.output.timeperframe = *timeperframe;
	} else {
		streamparm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		streamparm->parm.capture.timeperframe = *timeperframe;
	}

	return 0;
}

static int cedrus_proc_s_parm(struct file *file, void *private,
			      struct v4l2_streamparm *streamparm)
{
	struct cedrus_context *ctx =
		container_of(file->private_data, struct cedrus_context,
			     v4l2.fh);
	unsigned int format_type =
		cedrus_proc_format_type(ctx->proc, streamparm->type);
	struct v4l2_fract *timeperframe_propagate;
	struct v4l2_fract *timeperframe_format;
	struct v4l2_fract *timeperframe;

	if (V4L2_TYPE_IS_OUTPUT(streamparm->type)) {
		streamparm->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
		timeperframe = &streamparm->parm.output.timeperframe;
	} else {
		streamparm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		timeperframe = &streamparm->parm.capture.timeperframe;
	}

	/* Find out where to assign the provided values. */
	if (format_type == CEDRUS_FORMAT_TYPE_CODED) {
		timeperframe_format = &ctx->v4l2.timeperframe_coded;
		timeperframe_propagate = NULL;
	} else {
		timeperframe_format = &ctx->v4l2.timeperframe_picture;
		timeperframe_propagate = &ctx->v4l2.timeperframe_coded;
	}

	/* Return the current timeperframe in case of invalid values. */
	if (!timeperframe->numerator || !timeperframe->denominator) {
		*timeperframe = *timeperframe_format;
		return 0;
	}

	*timeperframe_format = *timeperframe;

	/* Propagate picture timeperframe to coded. */
	if (timeperframe_propagate)
		*timeperframe_propagate = *timeperframe;

	return 0;
}

static const struct v4l2_ioctl_ops cedrus_proc_ioctl_ops = {
	.vidioc_querycap		= cedrus_proc_querycap,

	.vidioc_enum_fmt_vid_out	= cedrus_proc_enum_fmt,
	.vidioc_g_fmt_vid_out		= cedrus_proc_g_fmt,
	.vidioc_s_fmt_vid_out		= cedrus_proc_s_fmt,
	.vidioc_try_fmt_vid_out		= cedrus_proc_try_fmt,

	.vidioc_enum_fmt_vid_cap	= cedrus_proc_enum_fmt,
	.vidioc_g_fmt_vid_cap		= cedrus_proc_g_fmt,
	.vidioc_s_fmt_vid_cap		= cedrus_proc_s_fmt,
	.vidioc_try_fmt_vid_cap		= cedrus_proc_try_fmt,

	.vidioc_enum_framesizes		= cedrus_proc_enum_framesizes,
	.vidioc_enum_frameintervals	= cedrus_proc_enum_frameintervals,

	.vidioc_g_selection		= cedrus_proc_g_selection,
	.vidioc_s_selection		= cedrus_proc_s_selection,

	.vidioc_g_parm			= cedrus_proc_g_parm,
	.vidioc_s_parm			= cedrus_proc_s_parm,

	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,

	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,

	.vidioc_decoder_cmd		= v4l2_m2m_ioctl_stateless_decoder_cmd,
	.vidioc_try_decoder_cmd		= v4l2_m2m_ioctl_stateless_try_decoder_cmd,

	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

static int cedrus_proc_open(struct file *file)
{
	struct cedrus_proc *proc = video_drvdata(file);
	struct cedrus_context *ctx;
	struct mutex *lock = &proc->v4l2.lock;
	int ret;

	if (mutex_lock_interruptible(lock))
		return -ERESTARTSYS;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto complete;
	}

	file->private_data = &ctx->v4l2.fh;

	ret = cedrus_context_setup(proc, ctx);

complete:
	mutex_unlock(lock);

	return ret;
}

static int cedrus_proc_release(struct file *file)
{
	struct cedrus_proc *proc = video_drvdata(file);
	struct cedrus_context *ctx =
		container_of(file->private_data, struct cedrus_context,
			     v4l2.fh);
	struct mutex *lock = &proc->v4l2.lock;

	mutex_lock(lock);

	cedrus_context_cleanup(ctx);
	kfree(ctx);

	mutex_unlock(lock);

	return 0;
}

static const struct v4l2_file_operations cedrus_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= cedrus_proc_open,
	.release	= cedrus_proc_release,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
	.poll		= v4l2_m2m_fop_poll,
};

/* V4L2 */

static int cedrus_proc_v4l2_setup_entity(struct cedrus_proc *proc,
					 struct media_entity *entity,
					 const char *suffix,
					 struct media_pad *pads,
					 unsigned int pads_count,
					 unsigned int function)
{
	struct device *dev = proc->dev->dev;
	struct media_device *media_dev = &proc->dev->v4l2.media_dev;
	struct cedrus_proc_v4l2 *v4l2 = &proc->v4l2;
	struct video_device *video_dev = &v4l2->video_dev;
	char *name;
	int ret;

	entity->obj_type = MEDIA_ENTITY_TYPE_BASE;
	if (function == MEDIA_ENT_F_IO_V4L) {
		entity->info.dev.major = VIDEO_MAJOR;
		entity->info.dev.minor = video_dev->minor;
	}

	name = devm_kasprintf(dev, GFP_KERNEL, "%s-%s", video_dev->name,
			      suffix);
	if (!name)
		return -ENOMEM;

	entity->name = name;
	entity->function = function;

	ret = media_entity_pads_init(entity, pads_count, pads);
	if (ret)
		return ret;

	ret = media_device_register_entity(media_dev, entity);
	if (ret)
		return ret;

	return 0;
}

static int cedrus_proc_v4l2_setup(struct cedrus_proc *proc)
{
	struct cedrus_proc_v4l2 *v4l2 = &proc->v4l2;
	struct v4l2_device *v4l2_dev = &proc->dev->v4l2.v4l2_dev;
	struct media_device *media_dev = &proc->dev->v4l2.media_dev;
	struct video_device *video_dev = &v4l2->video_dev;
	struct media_link *link;
	unsigned int function;
	const char *suffix;
	int ret;

	mutex_init(&v4l2->lock);

	/* Video Device */

	if (proc->role == CEDRUS_ROLE_DECODER)
		suffix = "dec";
	else
		suffix = "enc";

	snprintf(video_dev->name, sizeof(video_dev->name), CEDRUS_NAME "-%s",
		 suffix);
	video_dev->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_dev->vfl_dir = VFL_DIR_M2M;
	video_dev->release = video_device_release_empty;
	video_dev->fops = &cedrus_proc_fops;
	video_dev->ioctl_ops = &cedrus_proc_ioctl_ops;
	video_dev->v4l2_dev = v4l2_dev;
	video_dev->lock = &v4l2->lock;

	video_set_drvdata(video_dev, proc);

	if (proc->role == CEDRUS_ROLE_DECODER) {
		v4l2_disable_ioctl(video_dev, VIDIOC_ENUM_FRAMEINTERVALS);
		v4l2_disable_ioctl(video_dev, VIDIOC_G_SELECTION);
		v4l2_disable_ioctl(video_dev, VIDIOC_S_SELECTION);
		v4l2_disable_ioctl(video_dev, VIDIOC_G_PARM);
		v4l2_disable_ioctl(video_dev, VIDIOC_S_PARM);
	} else {
		v4l2_disable_ioctl(video_dev, VIDIOC_DECODER_CMD);
		v4l2_disable_ioctl(video_dev, VIDIOC_TRY_DECODER_CMD);
	}

	ret = video_register_device(video_dev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_err(v4l2_dev, "failed to register video device\n");
		return ret;
	}

	/* Media Entities: Source */

	v4l2->source_pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = cedrus_proc_v4l2_setup_entity(proc, &video_dev->entity, "source",
					    &v4l2->source_pad, 1,
					    MEDIA_ENT_F_IO_V4L);
	if (ret)
		goto error_video;

	/* Media Entities: Proc */

	v4l2->proc_pads[0].flags = MEDIA_PAD_FL_SINK;
	v4l2->proc_pads[1].flags = MEDIA_PAD_FL_SOURCE;

	if (proc->role == CEDRUS_ROLE_DECODER)
		function = MEDIA_ENT_F_PROC_VIDEO_DECODER;
	else
		function = MEDIA_ENT_F_PROC_VIDEO_ENCODER;

	ret = cedrus_proc_v4l2_setup_entity(proc, &v4l2->proc, "proc",
					    v4l2->proc_pads, 2, function);
	if (ret)
		goto error_media_source;

	/* Media Entities: Sink */

	v4l2->sink_pad.flags = MEDIA_PAD_FL_SINK;

	ret = cedrus_proc_v4l2_setup_entity(proc, &v4l2->sink, "sink",
					    &v4l2->sink_pad, 1,
					    MEDIA_ENT_F_IO_V4L);
	if (ret)
		goto error_media_proc;

	/* Media Devnode */

	/* XXX: already created by video device (vdev->intf_devnode) */
	v4l2->devnode = media_devnode_create(media_dev, MEDIA_INTF_T_V4L_VIDEO,
					     0, VIDEO_MAJOR, video_dev->minor);
	if (!v4l2->devnode) {
		ret = -ENOMEM;
		goto error_media_sink;
	}

	/* Media Pad Links */

	ret = media_create_pad_link(&video_dev->entity, 0, &v4l2->proc, 0,
				    MEDIA_LNK_FL_IMMUTABLE |
				    MEDIA_LNK_FL_ENABLED);
	if (ret)
		goto error_media_devnode;

	ret = media_create_pad_link(&v4l2->proc, 1, &v4l2->sink, 0,
				    MEDIA_LNK_FL_IMMUTABLE |
				    MEDIA_LNK_FL_ENABLED);
	if (ret)
		goto error_media_source_link;

	/* Media Interface Links */

	link = media_create_intf_link(&video_dev->entity, &v4l2->devnode->intf,
				      MEDIA_LNK_FL_IMMUTABLE |
				      MEDIA_LNK_FL_ENABLED);
	if (!link) {
		ret = -ENOMEM;
		goto error_media_sink_link;
	}

	link = media_create_intf_link(&v4l2->sink, &v4l2->devnode->intf,
				      MEDIA_LNK_FL_IMMUTABLE |
				      MEDIA_LNK_FL_ENABLED);
	if (!link) {
		ret = -ENOMEM;
		goto error_media_sink_link;
	}

	return 0;

error_media_sink_link:
	media_entity_remove_links(&v4l2->sink);

error_media_source_link:
	media_entity_remove_links(&video_dev->entity);
	media_entity_remove_links(&v4l2->proc);

error_media_devnode:
	media_devnode_remove(v4l2->devnode);

error_media_sink:
	media_device_unregister_entity(&v4l2->sink);

error_media_proc:
	media_device_unregister_entity(&v4l2->proc);

error_media_source:
	media_device_unregister_entity(&video_dev->entity);

error_video:
	video_unregister_device(video_dev);

	return ret;
}

void cedrus_proc_v4l2_cleanup(struct cedrus_proc *proc)
{
	struct cedrus_proc_v4l2 *v4l2 = &proc->v4l2;
	struct video_device *video_dev = &v4l2->video_dev;

	media_entity_remove_links(&v4l2->sink);
	media_entity_remove_links(&video_dev->entity);
	media_entity_remove_links(&v4l2->proc);

	media_device_unregister_entity(&v4l2->sink);
	media_device_unregister_entity(&video_dev->entity);
	media_device_unregister_entity(&v4l2->proc);

	video_unregister_device(video_dev);
}

/* Proc */

int cedrus_proc_setup(struct cedrus_device *dev, struct cedrus_proc *proc,
		      const struct cedrus_proc_ops *ops,
		      const struct cedrus_proc_config *config)
{
	int ret;

	proc->dev = dev;
	proc->ops = ops;
	proc->role = config->role;

	spin_lock_init(&proc->ctx_active_lock);

	ret = cedrus_proc_engines_setup(proc, config);
	if (ret == -ENODEV)
		return 0;
	else if (ret)
		return ret;

	ret = cedrus_proc_formats_setup(proc, config);
	if (ret)
		return ret;

	ret = cedrus_proc_v4l2_setup(proc);
	if (ret)
		return ret;

	return 0;
}

void cedrus_proc_cleanup(struct cedrus_proc *proc)
{
	if (!proc->engines)
		return;

	cedrus_proc_v4l2_cleanup(proc);
}
