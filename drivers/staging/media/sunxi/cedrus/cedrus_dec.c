// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2016 Florent Revest <florent.revest@free-electrons.com>
 * Copyright 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright 2018-2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#include <linux/types.h>
#include <linux/videodev2.h>

#include "cedrus.h"
#include "cedrus_context.h"
#include "cedrus_dec.h"
#include "cedrus_dec_mpeg2.h"
#include "cedrus_dec_h264.h"
#include "cedrus_dec_h265.h"
#include "cedrus_dec_vp8.h"
#include "cedrus_engine.h"
#include "cedrus_proc.h"
#include "cedrus_regs.h"

/* Format */

static const struct cedrus_format cedrus_dec_formats[] = {
	{
		.pixelformat	= V4L2_PIX_FMT_NV12,
		.capabilities	= CEDRUS_CAPABILITY_UNTILED,
		.type		= CEDRUS_FORMAT_TYPE_PICTURE,
	},
	{
		.pixelformat	= V4L2_PIX_FMT_NV12_32L32,
		.type		= CEDRUS_FORMAT_TYPE_PICTURE,
	}
};

int cedrus_dec_format_coded_prepare(struct cedrus_context *ctx,
				    struct v4l2_format *format)
{
	struct v4l2_pix_format *pix_format = &format->fmt.pix;
	/* Apply dimension and alignment constraints. */
	v4l2_apply_frmsize_constraints(&pix_format->width, &pix_format->height,
				       ctx->engine->frmsize);

	/* Zero bytes per line for encoded source. */
	pix_format->bytesperline = 0;

	/* Choose some minimum size since this can't be 0 */
	pix_format->sizeimage = max_t(u32, SZ_1K, pix_format->sizeimage);

	pix_format->field = V4L2_FIELD_NONE;

	return 0;
}

int cedrus_dec_format_coded_configure(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	struct v4l2_pix_format *pix_format = &ctx->v4l2.format_coded.fmt.pix;
	struct v4l2_pix_format *pix_format_picture =
		&ctx->v4l2.format_picture.fmt.pix;
	unsigned int width_picture = pix_format_picture->width;
	u32 value = 0;

	/*
	 * FIXME: This is only valid on 32-bits DDR's, we should test
	 * it on the A13/A33.
	 */
	value |= VE_MODE_REC_WR_MODE_2MB;
	value |= VE_MODE_DDR_MODE_BW_128;

	switch (pix_format->pixelformat) {
	case V4L2_PIX_FMT_MPEG2_SLICE:
		value |= VE_MODE_DEC_MPEG;
		break;
	case V4L2_PIX_FMT_H264_SLICE:
	case V4L2_PIX_FMT_VP8_FRAME:
		/* H.264 and VP8 both use the same decoding mode bit. */
		value |= VE_MODE_DEC_H264;
		break;
	case V4L2_PIX_FMT_HEVC_SLICE:
		value |= VE_MODE_DEC_H265;
		break;
	default:
		return -EINVAL;
	}

	if (width_picture == 4096)
		value |= VE_MODE_PIC_WIDTH_IS_4096;
	if (width_picture > 2048)
		value |= VE_MODE_PIC_WIDTH_MORE_2048;

	cedrus_write(dev, VE_MODE_REG, value);

	return 0;
}

static int cedrus_dec_format_picture_prepare(struct cedrus_context *ctx,
					     struct v4l2_format *format)
{
	struct v4l2_pix_format *pix_format = &format->fmt.pix;
	struct v4l2_pix_format *pix_format_coded =
		&ctx->v4l2.format_coded.fmt.pix;
	unsigned int width, height;
	unsigned int sizeimage;
	unsigned int bytesperline = pix_format->bytesperline;

	/* Picture format dimensions are copied from coded format. */
	width = pix_format_coded->width;
	height = pix_format_coded->height;

	/* Check minimum allowed bytesperline, maximum is to avoid overflow. */
	if (bytesperline < width || bytesperline > (32 * width))
		bytesperline = width;

	/* Macroblock-aligned stride. */
	bytesperline = ALIGN(bytesperline, 16);

	switch (pix_format->pixelformat) {
	case V4L2_PIX_FMT_NV12:
		/* Luma plane size. */
		sizeimage = bytesperline * height;

		/* Chroma plane size. */
		sizeimage += bytesperline * height / 2;
		break;
	case V4L2_PIX_FMT_NV12_32L32:
		/* 32-aligned stride. */
		width = ALIGN(width, 32);

		/* 32-aligned height. */
		height = ALIGN(height, 32);

		/* 32-aligned stride, forced to match width exactly. */
		bytesperline = ALIGN(width, 32);

		/* Luma plane size. */
		sizeimage = bytesperline * height;

		/* Chroma plane size. */
		sizeimage += bytesperline * ALIGN(height, 64) / 2;
		break;
	default:
		return -EINVAL;
	}

	pix_format->width = width;
	pix_format->height = height;
	pix_format->bytesperline = bytesperline;
	pix_format->sizeimage = sizeimage;
	pix_format->field = V4L2_FIELD_NONE;

	/* Picture format information is copied from coded format. */
	pix_format->colorspace = pix_format_coded->colorspace;
	pix_format->xfer_func = pix_format_coded->xfer_func;
	pix_format->ycbcr_enc = pix_format_coded->ycbcr_enc;
	pix_format->quantization = pix_format_coded->quantization;

	return 0;
}

int cedrus_dec_format_picture_configure(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	struct v4l2_pix_format *pix_format = &ctx->v4l2.format_picture.fmt.pix;
	u32 luma_stride, chroma_stride;
	u32 chroma_size;
	u32 value;

	switch (pix_format->pixelformat) {
	case V4L2_PIX_FMT_NV12:
		cedrus_write(dev, VE_PRIMARY_OUT_FMT, VE_PRIMARY_OUT_FMT_NV12);

		chroma_size = ALIGN(pix_format->width, 16) *
			      ALIGN(pix_format->height, 16) / 2;
		cedrus_write(dev, VE_PRIMARY_CHROMA_BUF_LEN, chroma_size / 2);

		luma_stride = ALIGN(pix_format->width, 16);
		chroma_stride = luma_stride / 2;

		value = VE_PRIMARY_FB_LINE_STRIDE_LUMA(luma_stride) |
			VE_PRIMARY_FB_LINE_STRIDE_CHROMA(chroma_stride);
		cedrus_write(dev, VE_PRIMARY_FB_LINE_STRIDE, value);
		break;
	case V4L2_PIX_FMT_NV12_32L32:
		cedrus_write(dev, VE_PRIMARY_OUT_FMT,
			     VE_PRIMARY_OUT_FMT_TILED_32_NV12);

		cedrus_write(dev, VE_CHROMA_BUF_LEN,
			     VE_SECONDARY_OUT_FMT_TILED_32_NV12);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int cedrus_dec_format_setup(struct cedrus_context *ctx)
{
	struct cedrus_proc *proc = ctx->proc;
	struct v4l2_format *format = &ctx->v4l2.format_coded;
	struct v4l2_pix_format *pix_format = &format->fmt.pix;
	int ret;

	format->type = cedrus_proc_buffer_type(proc, CEDRUS_FORMAT_TYPE_CODED);

	pix_format->pixelformat = ctx->engine->pixelformat;
	pix_format->width = 1280;
	pix_format->height = 720;

	ret = cedrus_proc_format_coded_prepare(ctx, format);
	if (ret)
		return ret;

	format = &ctx->v4l2.format_picture;
	format->type =
		cedrus_proc_buffer_type(proc, CEDRUS_FORMAT_TYPE_PICTURE);

	ret = cedrus_proc_format_propagate(ctx, CEDRUS_FORMAT_TYPE_CODED);
	if (ret)
		return ret;

	return 0;
}

static int cedrus_dec_format_propagate(struct cedrus_context *ctx,
				       unsigned int format_type)
{
	/* Format is propagated from coded to picture. */
	if (format_type != CEDRUS_FORMAT_TYPE_CODED)
		return 0;

	return cedrus_proc_format_picture_prepare(ctx,
						  &ctx->v4l2.format_picture);
}

static bool cedrus_dec_format_dynamic_check(struct cedrus_context *ctx,
					    struct v4l2_format *format)
{
	struct v4l2_pix_format *pix_format = &format->fmt.pix;
	struct v4l2_pix_format *pix_format_coded =
		&ctx->v4l2.format_coded.fmt.pix;
	unsigned int buffer_type;
	bool streaming;
	bool busy;

	/* Dynamic format change starts on the coded (output) queue. */
	if (!V4L2_TYPE_IS_OUTPUT(format->type))
		return false;

	/* With no buffer allocated, this is just a regular format change. */
	busy = cedrus_context_queue_busy_check(ctx, format->type);
	if (!busy)
		return false;

	/*
	 * The coded queue will be reconfigured, thus it must not be streaming.
	 * However we can keep using the same buffers since there is not direct
	 * relationship between the buffer size and the format.
	 */
	streaming = cedrus_context_queue_streaming_check(ctx, format->type);
	if (streaming)
		return false;

	/*
	 * The picture queue will be reconfigured, thus it must not have any
	 * buffers allocated.
	 */
	buffer_type = cedrus_proc_buffer_type(ctx->proc,
					      CEDRUS_FORMAT_TYPE_PICTURE);
	busy = cedrus_context_queue_busy_check(ctx, buffer_type);
	if (busy)
		return false;

	/* Coded format must remain the same. */
	if (pix_format->pixelformat != pix_format_coded->pixelformat)
		return false;

	return true;
}

/* Size */

static int cedrus_dec_size_picture_enum(struct cedrus_context *ctx,
					struct v4l2_frmsizeenum *frmsizeenum)
{
	/* Picture frame sizes are constrained by coded frame sizes. */
	frmsizeenum->stepwise = *ctx->engine->frmsize;

	switch (frmsizeenum->pixel_format) {
	case V4L2_PIX_FMT_NV12_32L32:
		frmsizeenum->stepwise.min_width = 32;
		frmsizeenum->stepwise.min_height = 32;
		frmsizeenum->stepwise.step_width = 32;
		frmsizeenum->stepwise.step_height = 32;
		break;
	}

	return 0;
}

/* Engines */

static const struct cedrus_engine *cedrus_dec_engines[] = {
	&cedrus_dec_mpeg2,
	&cedrus_dec_h264,
	&cedrus_dec_h265,
	&cedrus_dec_vp8,
};

/* Decoder */

static const struct cedrus_proc_config cedrus_dec_config = {
	.role			= CEDRUS_ROLE_DECODER,

	.engines		= cedrus_dec_engines,
	.engines_count		= ARRAY_SIZE(cedrus_dec_engines),

	.formats		= cedrus_dec_formats,
	.formats_count		= ARRAY_SIZE(cedrus_dec_formats),
};

static const struct cedrus_proc_ops cedrus_dec_ops = {
	.format_picture_prepare		= cedrus_dec_format_picture_prepare,
	.format_picture_configure	= cedrus_dec_format_picture_configure,

	.format_setup			= cedrus_dec_format_setup,
	.format_propagate		= cedrus_dec_format_propagate,
	.format_dynamic_check		= cedrus_dec_format_dynamic_check,

	.size_picture_enum		= cedrus_dec_size_picture_enum,
};

int cedrus_dec_setup(struct cedrus_device *dev)
{
	return cedrus_proc_setup(dev, &dev->dec, &cedrus_dec_ops,
				 &cedrus_dec_config);
}

void cedrus_dec_cleanup(struct cedrus_device *dev)
{
	cedrus_proc_cleanup(&dev->dec);
}
