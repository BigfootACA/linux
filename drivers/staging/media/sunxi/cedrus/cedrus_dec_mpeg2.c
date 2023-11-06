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
#include <media/v4l2-ctrls.h>

#include "cedrus.h"
#include "cedrus_context.h"
#include "cedrus_dec.h"
#include "cedrus_dec_mpeg2.h"
#include "cedrus_engine.h"
#include "cedrus_proc.h"
#include "cedrus_regs.h"

/* Job */

static int cedrus_dec_mpeg2_job_prepare(struct cedrus_context *ctx)
{
	struct cedrus_dec_mpeg2_job *job = ctx->engine_job;
	u32 id;

	id = V4L2_CID_STATELESS_MPEG2_SEQUENCE;
	job->sequence = cedrus_context_ctrl_data(ctx, id);

	id = V4L2_CID_STATELESS_MPEG2_PICTURE;
	job->picture = cedrus_context_ctrl_data(ctx, id);

	id = V4L2_CID_STATELESS_MPEG2_QUANTISATION;
	job->quantisation = cedrus_context_ctrl_data(ctx, id);

	return 0;
}

static int cedrus_dec_mpeg2_job_configure(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	struct cedrus_dec_mpeg2_job *job = ctx->engine_job;
	const struct v4l2_ctrl_mpeg2_sequence *seq = job->sequence;
	const struct v4l2_ctrl_mpeg2_picture *pic = job->picture;
	const struct v4l2_ctrl_mpeg2_quantisation *quant = job->quantisation;
	struct v4l2_pix_format *pix_format = &ctx->v4l2.format_coded.fmt.pix;
	dma_addr_t picture_luma_addr, picture_chroma_addr, coded_addr;
	unsigned int coded_size;
	const u8 *matrix;
	unsigned int i;
	bool check;
	u32 value;

	/* Set intra quantisation matrix. */

	matrix = quant->intra_quantiser_matrix;
	for (i = 0; i < 64; i++)
		cedrus_write(dev, VE_DEC_MPEG_IQMINPUT,
			     VE_DEC_MPEG_IQMINPUT_WEIGHT(i, matrix[i]) |
			     VE_DEC_MPEG_IQMINPUT_FLAG_INTRA);

	/* Set non-intra quantisation matrix. */

	matrix = quant->non_intra_quantiser_matrix;
	for (i = 0; i < 64; i++)
		cedrus_write(dev, VE_DEC_MPEG_IQMINPUT,
			     VE_DEC_MPEG_IQMINPUT_WEIGHT(i, matrix[i]) |
			     VE_DEC_MPEG_IQMINPUT_FLAG_NON_INTRA);

	/* Set MPEG picture header. */

	value = VE_DEC_MPEG_MP12HDR_SLICE_TYPE(pic->picture_coding_type) |
		VE_DEC_MPEG_MP12HDR_F_CODE(0, 0, pic->f_code[0][0]) |
		VE_DEC_MPEG_MP12HDR_F_CODE(0, 1, pic->f_code[0][1]) |
		VE_DEC_MPEG_MP12HDR_F_CODE(1, 0, pic->f_code[1][0]) |
		VE_DEC_MPEG_MP12HDR_F_CODE(1, 1, pic->f_code[1][1]) |
		VE_DEC_MPEG_MP12HDR_INTRA_DC_PRECISION(pic->intra_dc_precision) |
		VE_DEC_MPEG_MP12HDR_INTRA_PICTURE_STRUCTURE(pic->picture_structure);

	check = pic->flags & V4L2_MPEG2_PIC_FLAG_TOP_FIELD_FIRST;
	value |= VE_DEC_MPEG_MP12HDR_TOP_FIELD_FIRST(check);
	check = pic->flags & V4L2_MPEG2_PIC_FLAG_FRAME_PRED_DCT;
	value |= VE_DEC_MPEG_MP12HDR_FRAME_PRED_FRAME_DCT(check);
	check = pic->flags & V4L2_MPEG2_PIC_FLAG_CONCEALMENT_MV;
	value |= VE_DEC_MPEG_MP12HDR_CONCEALMENT_MOTION_VECTORS(check);
	check = pic->flags & V4L2_MPEG2_PIC_FLAG_Q_SCALE_TYPE;
	value |= VE_DEC_MPEG_MP12HDR_Q_SCALE_TYPE(check);
	check = pic->flags & V4L2_MPEG2_PIC_FLAG_INTRA_VLC;
	value |= VE_DEC_MPEG_MP12HDR_INTRA_VLC_FORMAT(check);
	check = pic->flags & V4L2_MPEG2_PIC_FLAG_ALT_SCAN;
	value |= VE_DEC_MPEG_MP12HDR_ALTERNATE_SCAN(check);

	value |= VE_DEC_MPEG_MP12HDR_FULL_PEL_FORWARD_VECTOR(0);
	value |= VE_DEC_MPEG_MP12HDR_FULL_PEL_BACKWARD_VECTOR(0);

	cedrus_write(dev, VE_DEC_MPEG_MP12HDR, value);

	/* Set frame dimensions. */

	value = VE_DEC_MPEG_PICCODEDSIZE_WIDTH(seq->horizontal_size) |
		VE_DEC_MPEG_PICCODEDSIZE_HEIGHT(seq->vertical_size);

	cedrus_write(dev, VE_DEC_MPEG_PICCODEDSIZE, value);

	value = VE_DEC_MPEG_PICBOUNDSIZE_WIDTH(pix_format->width) |
		VE_DEC_MPEG_PICBOUNDSIZE_HEIGHT(pix_format->height);

	cedrus_write(dev, VE_DEC_MPEG_PICBOUNDSIZE, value);

	/* Forward and backward prediction reference buffers. */

	cedrus_job_buffer_picture_ref_dma(ctx, pic->forward_ref_ts,
					  &picture_luma_addr,
					  &picture_chroma_addr);

	cedrus_write(dev, VE_DEC_MPEG_FWD_REF_LUMA_ADDR, picture_luma_addr);
	cedrus_write(dev, VE_DEC_MPEG_FWD_REF_CHROMA_ADDR, picture_chroma_addr);

	cedrus_job_buffer_picture_ref_dma(ctx, pic->backward_ref_ts,
					  &picture_luma_addr,
					  &picture_chroma_addr);

	cedrus_write(dev, VE_DEC_MPEG_BWD_REF_LUMA_ADDR, picture_luma_addr);
	cedrus_write(dev, VE_DEC_MPEG_BWD_REF_CHROMA_ADDR, picture_chroma_addr);

	/* Destination luma and chroma buffers. */

	cedrus_job_buffer_picture_dma(ctx, &picture_luma_addr,
				      &picture_chroma_addr);

	cedrus_write(dev, VE_DEC_MPEG_REC_LUMA, picture_luma_addr);
	cedrus_write(dev, VE_DEC_MPEG_REC_CHROMA, picture_chroma_addr);

	/* Coded buffer. */

	cedrus_job_buffer_coded_dma(ctx, &coded_addr, &coded_size);

	cedrus_write(dev, VE_DEC_MPEG_VLD_LEN, coded_size * 8);
	cedrus_write(dev, VE_DEC_MPEG_VLD_OFFSET, 0);

	value = VE_DEC_MPEG_VLD_ADDR_BASE(coded_addr) |
		VE_DEC_MPEG_VLD_ADDR_VALID_PIC_DATA |
		VE_DEC_MPEG_VLD_ADDR_LAST_PIC_DATA |
		VE_DEC_MPEG_VLD_ADDR_FIRST_PIC_DATA;

	cedrus_write(dev, VE_DEC_MPEG_VLD_ADDR, value);

	cedrus_write(dev, VE_DEC_MPEG_VLD_END_ADDR, coded_addr + coded_size);

	/* Macroblock address: start at the beginning. */

	cedrus_write(dev, VE_DEC_MPEG_MBADDR,
		     VE_DEC_MPEG_MBADDR_Y(0) |
		     VE_DEC_MPEG_MBADDR_X(0));

	/* Clear previous errors. */

	cedrus_write(dev, VE_DEC_MPEG_ERROR, 0);

	/* Clear correct macroblocks register. */

	cedrus_write(dev, VE_DEC_MPEG_CRTMBADDR, 0);

	/* Enable appropriate interruptions and components. */

	cedrus_write(dev, VE_DEC_MPEG_CTRL,
		     VE_DEC_MPEG_CTRL_IRQ_MASK |
		     VE_DEC_MPEG_CTRL_MC_NO_WRITEBACK |
		     VE_DEC_MPEG_CTRL_MC_CACHE_EN);

	return 0;
}

static void cedrus_dec_mpeg2_job_trigger(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;

	cedrus_write(dev, VE_DEC_MPEG_TRIGGER,
		     VE_DEC_MPEG_TRIGGER_HW_MPEG_VLD |
		     VE_DEC_MPEG_TRIGGER_MPEG2 |
		     VE_DEC_MPEG_TRIGGER_MB_BOUNDARY);
}

/* IRQ */

static int cedrus_dec_mpeg2_irq_status(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	u32 status;

	status = cedrus_read(dev, VE_DEC_MPEG_STATUS);
	status &= VE_DEC_MPEG_STATUS_CHECK_MASK;

	if (!status)
		return CEDRUS_IRQ_NONE;

	if (!(status & VE_DEC_MPEG_STATUS_SUCCESS) ||
	    status & VE_DEC_MPEG_STATUS_CHECK_ERROR)
		return CEDRUS_IRQ_ERROR;

	return CEDRUS_IRQ_SUCCESS;
}

static void cedrus_dec_mpeg2_irq_clear(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;

	cedrus_write(dev, VE_DEC_MPEG_STATUS, VE_DEC_MPEG_STATUS_CHECK_MASK);
}

static void cedrus_dec_mpeg2_irq_disable(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	u32 value;

	value = cedrus_read(dev, VE_DEC_MPEG_CTRL);
	value &= ~VE_DEC_MPEG_CTRL_IRQ_MASK;

	cedrus_write(dev, VE_DEC_MPEG_CTRL, value);
}

/* Engine */

static const struct cedrus_engine_ops cedrus_dec_mpeg2_ops = {
	.format_prepare		= cedrus_dec_format_coded_prepare,
	.format_configure	= cedrus_dec_format_coded_configure,

	.job_prepare		= cedrus_dec_mpeg2_job_prepare,
	.job_configure		= cedrus_dec_mpeg2_job_configure,
	.job_trigger		= cedrus_dec_mpeg2_job_trigger,

	.irq_status		= cedrus_dec_mpeg2_irq_status,
	.irq_clear		= cedrus_dec_mpeg2_irq_clear,
	.irq_disable		= cedrus_dec_mpeg2_irq_disable,
};

static const struct v4l2_ctrl_config cedrus_dec_mpeg2_ctrl_configs[] = {
	{
		.id	= V4L2_CID_STATELESS_MPEG2_SEQUENCE,
	},
	{
		.id	= V4L2_CID_STATELESS_MPEG2_PICTURE,
	},
	{
		.id	= V4L2_CID_STATELESS_MPEG2_QUANTISATION,
	},
};

static const struct v4l2_frmsize_stepwise cedrus_dec_mpeg2_frmsize = {
	.min_width	= 16,
	.max_width	= 3840,
	.step_width	= 16,

	.min_height	= 16,
	.max_height	= 3840,
	.step_height	= 16,
};

const struct cedrus_engine cedrus_dec_mpeg2 = {
	.codec			= CEDRUS_CODEC_MPEG2,
	.role			= CEDRUS_ROLE_DECODER,
	.capabilities		= CEDRUS_CAPABILITY_MPEG2_DEC,

	.ops			= &cedrus_dec_mpeg2_ops,

	.pixelformat		= V4L2_PIX_FMT_MPEG2_SLICE,
	.ctrl_configs		= cedrus_dec_mpeg2_ctrl_configs,
	.ctrl_configs_count	= ARRAY_SIZE(cedrus_dec_mpeg2_ctrl_configs),
	.frmsize		= &cedrus_dec_mpeg2_frmsize,

	.ctx_size		= 0,
	.job_size		= sizeof(struct cedrus_dec_mpeg2_job),
	.buffer_size		= 0,
};
