/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _CEDRUS_PROC_H_
#define _CEDRUS_PROC_H_

#include <linux/videodev2.h>
#include <media/media-device.h>
#include <media/media-entity.h>

#include "cedrus.h"

struct cedrus_format;
struct cedrus_device;
struct cedrus_proc;
struct cedrus_context;

enum cedrus_role {
	CEDRUS_ROLE_DECODER,
	CEDRUS_ROLE_ENCODER
};

enum cedrus_format_type {
	CEDRUS_FORMAT_TYPE_CODED,
	CEDRUS_FORMAT_TYPE_PICTURE
};

struct cedrus_proc_config {
	int				role;

	const struct cedrus_engine	**engines;
	unsigned int			engines_count;

	const struct cedrus_format	*formats;
	unsigned int			formats_count;
};

struct cedrus_proc_ops {
	int (*format_picture_prepare)(struct cedrus_context *ctx,
				      struct v4l2_format *format);
	int (*format_picture_configure)(struct cedrus_context *ctx);

	int (*format_setup)(struct cedrus_context *ctx);
	int (*format_propagate)(struct cedrus_context *ctx,
				unsigned int format_type);
	bool (*format_dynamic_check)(struct cedrus_context *ctx,
				     struct v4l2_format *format);

	int (*size_picture_enum)(struct cedrus_context *ctx,
				 struct v4l2_frmsizeenum *frmsizeenum);
};

struct cedrus_proc_v4l2 {
	struct video_device		video_dev;
	struct media_pad		source_pad;

	struct media_entity		proc;
	struct media_pad		proc_pads[2];

	struct media_entity		sink;
	struct media_pad		sink_pad;

	struct media_intf_devnode	*devnode;

	struct mutex			lock;
};

struct cedrus_proc {
	struct cedrus_device		*dev;
	int				role;

	struct cedrus_proc_v4l2		v4l2;

	const struct cedrus_proc_ops	*ops;

	const struct cedrus_engine	**engines;
	unsigned int			engines_count;

	struct cedrus_format		*formats;
	unsigned int			formats_count;

	struct cedrus_context		*ctx_active;
	spinlock_t			ctx_active_lock;
};

/* Format */

static inline unsigned int cedrus_proc_format_type(struct cedrus_proc *proc,
						   unsigned int buffer_type)
{
	if (proc->role == CEDRUS_ROLE_DECODER) {
		if (V4L2_TYPE_IS_OUTPUT(buffer_type))
			return CEDRUS_FORMAT_TYPE_CODED;
		else
			return CEDRUS_FORMAT_TYPE_PICTURE;
	} else {
		if (V4L2_TYPE_IS_OUTPUT(buffer_type))
			return CEDRUS_FORMAT_TYPE_PICTURE;
		else
			return CEDRUS_FORMAT_TYPE_CODED;
	}
}

/* Buffer */

static inline int cedrus_proc_buffer_type(struct cedrus_proc *proc,
					  unsigned int format_type)
{
	if (proc->role == CEDRUS_ROLE_DECODER) {
		if (format_type == CEDRUS_FORMAT_TYPE_CODED)
			return V4L2_BUF_TYPE_VIDEO_OUTPUT;
		else
			return V4L2_BUF_TYPE_VIDEO_CAPTURE;
	} else {
		if (format_type == CEDRUS_FORMAT_TYPE_CODED)
			return V4L2_BUF_TYPE_VIDEO_CAPTURE;
		else
			return V4L2_BUF_TYPE_VIDEO_OUTPUT;
	}
}

/* Context */

void cedrus_proc_context_active_update(struct cedrus_proc *proc,
				       struct cedrus_context *ctx);
void cedrus_proc_context_active_clear(struct cedrus_proc *proc,
				      struct cedrus_context *ctx);

/* Format */

unsigned int cedrus_proc_format_find_first(struct cedrus_proc *proc,
					   unsigned int format_type);
int cedrus_proc_format_coded_prepare(struct cedrus_context *ctx,
				     struct v4l2_format *format);
int cedrus_proc_format_picture_prepare(struct cedrus_context *ctx,
				       struct v4l2_format *format);
int cedrus_proc_format_picture_configure(struct cedrus_context *ctx);
int cedrus_proc_format_setup(struct cedrus_context *ctx);
int cedrus_proc_format_propagate(struct cedrus_context *ctx,
				 unsigned int format_type);

/* Engine */

const struct cedrus_engine *
cedrus_proc_engine_find_format(struct cedrus_proc *proc,
			       unsigned int pixelformat);

/* Proc */

int cedrus_proc_setup(struct cedrus_device *dev, struct cedrus_proc *proc,
		      const struct cedrus_proc_ops *ops,
		      const struct cedrus_proc_config *config);
void cedrus_proc_cleanup(struct cedrus_proc *proc);

#endif
