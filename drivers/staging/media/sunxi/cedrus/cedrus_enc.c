// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#include <linux/types.h>
#include <linux/videodev2.h>

#include "cedrus.h"
#include "cedrus_context.h"
#include "cedrus_enc.h"
#include "cedrus_engine.h"
#include "cedrus_proc.h"
#include "cedrus_regs.h"

/* Format */

static const struct cedrus_format cedrus_enc_formats[] = {
	{
		.pixelformat	= V4L2_PIX_FMT_NV12,
		.type		= CEDRUS_FORMAT_TYPE_PICTURE,
	},
};

int cedrus_enc_format_coded_prepare(struct cedrus_context *ctx,
				    struct v4l2_format *format)
{
	struct v4l2_pix_format *pix_format = &format->fmt.pix;
	struct v4l2_pix_format *pix_format_picture =
		&ctx->v4l2.format_picture.fmt.pix;

	/* Coded format dimensions are copied from picture format. */
	pix_format->width = pix_format_picture->width;
	pix_format->height = pix_format_picture->height;

	/* Zero bytes per line for encoded source. */
	pix_format->bytesperline = 0;

	/* Choose some minimum size since this can't be 0 */
	pix_format->sizeimage = max_t(u32, SZ_1K, pix_format->sizeimage);

	pix_format->field = V4L2_FIELD_NONE;

	/* Coded format information is copied from picture format. */
	pix_format->colorspace = pix_format_picture->colorspace;
	pix_format->xfer_func = pix_format_picture->xfer_func;
	pix_format->ycbcr_enc = pix_format_picture->ycbcr_enc;
	pix_format->quantization = pix_format_picture->quantization;

	return 0;
}

int cedrus_enc_format_coded_configure(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	u32 value;

	/* Disable encoder. */

	value = cedrus_read(dev, VE_MODE_REG);
	value &= ~(VE_MODE_ENC_ENABLE |
		   VE_MODE_ENC_ISP_ENABLE);
	value |= VE_MODE_DEC_DISABLED;
	cedrus_write(dev, VE_MODE_REG, value);

	/* Reset encoder. */

	value = cedrus_read(dev, VE_RESET_REG);
	value |= VE_RESET_ENCODER_RESET;
	cedrus_write(dev, VE_RESET_REG, value);

	value = cedrus_read(dev, VE_RESET_REG);
	value &= ~VE_RESET_ENCODER_RESET;
	cedrus_write(dev, VE_RESET_REG, value);

	/* Enable encoder. */

	value = cedrus_read(dev, VE_MODE_REG);
	value |= VE_MODE_ENC_ENABLE |
		 VE_MODE_ENC_ISP_ENABLE |
		 VE_MODE_DEC_DISABLED;
	cedrus_write(dev, VE_MODE_REG, value);

	return 0;
}

static int cedrus_enc_format_picture_prepare(struct cedrus_context *ctx,
					     struct v4l2_format *format)
{
	struct v4l2_pix_format *pix_format = &format->fmt.pix;
	unsigned int width = pix_format->width;
	unsigned int height = pix_format->height;
	unsigned int bytesperline = pix_format->bytesperline;
	unsigned int sizeimage = 0;

	/* Apply dimension and alignment constraints. */
	v4l2_apply_frmsize_constraints(&width, &height, ctx->engine->frmsize);

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
	default:
		return -EINVAL;
	}

	pix_format->width = width;
	pix_format->height = height;
	pix_format->bytesperline = bytesperline;
	pix_format->sizeimage = sizeimage;
	pix_format->field = V4L2_FIELD_NONE;

	return 0;
}

int cedrus_enc_format_picture_configure(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	struct v4l2_pix_format *pix_format = &ctx->v4l2.format_picture.fmt.pix;
	dma_addr_t luma_addr, chroma_addr;
	unsigned int width_mbs, height_mbs;
	unsigned int stride_mbs;

	/* Dimensions */

	width_mbs = DIV_ROUND_UP(pix_format->width, 16);
	height_mbs = DIV_ROUND_UP(pix_format->height, 16);

	cedrus_write(dev, VE_ISP_PIC_INFO_REG,
		     VE_ISP_PIC_INFO_WIDTH_MBS(width_mbs) |
		     VE_ISP_PIC_INFO_HEIGHT_MBS(height_mbs));

	cedrus_write(dev, VE_ISP_SCALER_SIZE_REG,
		     VE_ISP_SCALER_SIZE_HEIGHT_MBS(height_mbs) |
		     VE_ISP_SCALER_SIZE_WIDTH_MBS(width_mbs));

	/* Stride */

	if (WARN_ON(pix_format->bytesperline % 16))
		return -EINVAL;

	stride_mbs = pix_format->bytesperline / 16;

	/* XXX: cedar rounds down, not up here. */
	cedrus_write(dev, VE_ISP_PIC_STRIDE0_REG,
		     VE_ISP_PIC_STRIDE0_INPUT_STRIDE_MBS(stride_mbs));

	/* Format */

	cedrus_write(dev, VE_ISP_CTRL_REG,
		     VE_ISP_CTRL_FORMAT_YUV420SP |
		     VE_ISP_CTRL_ROTATION_0 |
		     VE_ISP_CTRL_COLORSPACE_BT601);

	/* Address */

	cedrus_job_buffer_picture_dma(ctx, &luma_addr, &chroma_addr);

	cedrus_write(dev, VE_ISP_INPUT_LUMA_ADDR_REG, luma_addr);
	cedrus_write(dev, VE_ISP_INPUT_CHROMA0_ADDR_REG, chroma_addr);

	return 0;
}

static int cedrus_enc_format_setup(struct cedrus_context *ctx)
{
	struct cedrus_proc *proc = ctx->proc;
	struct v4l2_format *format = &ctx->v4l2.format_picture;
	struct v4l2_pix_format *pix_format = &format->fmt.pix;
	struct v4l2_fract *timeperframe = &ctx->v4l2.timeperframe_picture;
	struct v4l2_fract *timeperframe_propagate =
		&ctx->v4l2.timeperframe_coded;
	int ret;

	format->type =
		cedrus_proc_buffer_type(proc, CEDRUS_FORMAT_TYPE_PICTURE);

	pix_format->pixelformat =
		cedrus_proc_format_find_first(proc, CEDRUS_FORMAT_TYPE_PICTURE);
	pix_format->width = 1280;
	pix_format->height = 720;

	timeperframe->numerator = 1;
	timeperframe->denominator = 25;

	ret = cedrus_proc_format_picture_prepare(ctx, format);
	if (ret)
		return ret;

	format = &ctx->v4l2.format_coded;
	format->type = cedrus_proc_buffer_type(proc, CEDRUS_FORMAT_TYPE_CODED);

	ret = cedrus_proc_format_propagate(ctx, CEDRUS_FORMAT_TYPE_PICTURE);
	if (ret)
		return ret;

	*timeperframe_propagate = *timeperframe;

	return 0;
}

static int cedrus_enc_format_propagate(struct cedrus_context *ctx,
				       unsigned int format_type)
{
	int ret;

	/* Format is propagated from picture to coded. */
	if (format_type != CEDRUS_FORMAT_TYPE_PICTURE)
		return 0;

	/* Reset selection from picture format. */
	ret = cedrus_context_selection_picture_reset(ctx);
	if (ret)
		return ret;

	return cedrus_proc_format_coded_prepare(ctx, &ctx->v4l2.format_coded);
}

static bool cedrus_enc_format_dynamic_check(struct cedrus_context *ctx,
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
	buffer_type =
		cedrus_proc_buffer_type(ctx->proc, CEDRUS_FORMAT_TYPE_PICTURE);
	busy = cedrus_context_queue_busy_check(ctx, buffer_type);
	if (busy)
		return false;

	/* Coded format must remain the same. */
	if (pix_format->pixelformat != pix_format_coded->pixelformat)
		return false;

	return true;
}

/* Size */

static int cedrus_enc_size_picture_enum(struct cedrus_context *ctx,
					struct v4l2_frmsizeenum *frmsizeenum)
{
	/* Picture frame sizes are constrained by coded frame sizes. */
	frmsizeenum->stepwise = *ctx->engine->frmsize;

	return 0;
}

/* Engines */

static const struct cedrus_engine *cedrus_enc_engines[] = {
};

/* Encoder */

static const struct cedrus_proc_config cedrus_enc_config = {
	.role			= CEDRUS_ROLE_ENCODER,

	.engines		= cedrus_enc_engines,
	.engines_count		= ARRAY_SIZE(cedrus_enc_engines),

	.formats		= cedrus_enc_formats,
	.formats_count		= ARRAY_SIZE(cedrus_enc_formats),
};

static const struct cedrus_proc_ops cedrus_enc_ops = {
	.format_picture_prepare		= cedrus_enc_format_picture_prepare,
	.format_picture_configure	= cedrus_enc_format_picture_configure,

	.format_setup			= cedrus_enc_format_setup,
	.format_propagate		= cedrus_enc_format_propagate,
	.format_dynamic_check		= cedrus_enc_format_dynamic_check,

	.size_picture_enum		= cedrus_enc_size_picture_enum,
};

int cedrus_enc_setup(struct cedrus_device *dev)
{
	return cedrus_proc_setup(dev, &dev->enc, &cedrus_enc_ops,
				 &cedrus_enc_config);
}

void cedrus_enc_cleanup(struct cedrus_device *dev)
{
	cedrus_proc_cleanup(&dev->enc);
}
