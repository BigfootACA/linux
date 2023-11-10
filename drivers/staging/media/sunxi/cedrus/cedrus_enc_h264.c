// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#include <linux/align.h>
#include <linux/sizes.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>

#include "cedrus.h"
#include "cedrus_context.h"
#include "cedrus_enc.h"
#include "cedrus_enc_h264.h"
#include "cedrus_engine.h"
#include "cedrus_proc.h"
#include "cedrus_regs.h"

/* Helpers */

static u8 cedrus_enc_h264_profile_idc(int profile)
{
	switch (profile) {
	case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
	case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE:
		return 66;
	case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
		return 77;
	case V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED:
		return 88;
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
	case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH:
		return 100;
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10:
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10_INTRA:
		return 110;
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422:
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422_INTRA:
		return 122;
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE:
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_INTRA:
		return 244;
	case V4L2_MPEG_VIDEO_H264_PROFILE_CAVLC_444_INTRA:
		return 44;
	case V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_BASELINE:
		return 83;
	case V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_HIGH:
	case V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_HIGH_INTRA:
		return 86;
	case V4L2_MPEG_VIDEO_H264_PROFILE_STEREO_HIGH:
		return 128;
	case V4L2_MPEG_VIDEO_H264_PROFILE_MULTIVIEW_HIGH:
		return 118;
	default:
		return 0;
	}
}

static bool cedrus_enc_h264_profile_cabac_check(int profile)
{
	switch (profile) {
	case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
	case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE:
	case V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED:
	case V4L2_MPEG_VIDEO_H264_PROFILE_CAVLC_444_INTRA:
		return false;

	default:
		return true;
	}
}

static u8 cedrus_enc_h264_level_idc(int level)
{
	switch (level) {
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_0:
		return 10;
	case V4L2_MPEG_VIDEO_H264_LEVEL_1B:
		return 9;
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_1:
		return 11;
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_2:
		return 12;
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_3:
		return 13;
	case V4L2_MPEG_VIDEO_H264_LEVEL_2_0:
		return 20;
	case V4L2_MPEG_VIDEO_H264_LEVEL_2_1:
		return 21;
	case V4L2_MPEG_VIDEO_H264_LEVEL_2_2:
		return 22;
	case V4L2_MPEG_VIDEO_H264_LEVEL_3_0:
		return 30;
	case V4L2_MPEG_VIDEO_H264_LEVEL_3_1:
		return 31;
	case V4L2_MPEG_VIDEO_H264_LEVEL_3_2:
		return 32;
	case V4L2_MPEG_VIDEO_H264_LEVEL_4_0:
		return 40;
	case V4L2_MPEG_VIDEO_H264_LEVEL_4_1:
		return 41;
	case V4L2_MPEG_VIDEO_H264_LEVEL_4_2:
		return 42;
	case V4L2_MPEG_VIDEO_H264_LEVEL_5_0:
		return 50;
	case V4L2_MPEG_VIDEO_H264_LEVEL_5_1:
		return 51;
	case V4L2_MPEG_VIDEO_H264_LEVEL_5_2:
		return 52;
	case V4L2_MPEG_VIDEO_H264_LEVEL_6_0:
		return 60;
	case V4L2_MPEG_VIDEO_H264_LEVEL_6_1:
		return 61;
	case V4L2_MPEG_VIDEO_H264_LEVEL_6_2:
		return 62;
	default:
		return 0;
	}
}

static u8 cedrus_enc_h264_constraint_set_flags(int profile)
{
	switch (profile) {
	case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
		return CEDRUS_ENC_H264_CONSTRAINT_SET0_FLAG;
	case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE:
		return CEDRUS_ENC_H264_CONSTRAINT_SET0_FLAG |
		       CEDRUS_ENC_H264_CONSTRAINT_SET1_FLAG;
	case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
		return CEDRUS_ENC_H264_CONSTRAINT_SET1_FLAG;
	case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH:
		return CEDRUS_ENC_H264_CONSTRAINT_SET4_FLAG |
		       CEDRUS_ENC_H264_CONSTRAINT_SET5_FLAG;
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10_INTRA:
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422_INTRA:
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_INTRA:
	case V4L2_MPEG_VIDEO_H264_PROFILE_CAVLC_444_INTRA:
	case V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_HIGH_INTRA:
		return CEDRUS_ENC_H264_CONSTRAINT_SET3_FLAG;
	default:
		return 0;
	}
}

static u8 cedrus_enc_h264_vui_sar_idc(int value)
{
	switch (value) {

	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_1x1:
		return 1;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_12x11:
		return 2;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_10x11:
		return 3;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_16x11:
		return 4;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_40x33:
		return 5;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_24x11:
		return 6;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_20x11:
		return 7;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_32x11:
		return 8;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_80x33:
		return 9;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_18x11:
		return 10;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_15x11:
		return 11;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_64x33:
		return 12;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_160x99:
		return 13;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_4x3:
		return 14;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_3x2:
		return 15;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_2x1:
		return 16;
	case V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED:
		return 255;
	default:
		return 0;
	}
}

static u8 cedrus_enc_h264_disable_deblocking_filter_idc(int value)
{
	switch (value) {
	case V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED:
		return 0;
	case V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED:
		return 1;
	case V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY:
		return 2;
	default:
		return 0;
	}
}

static void cedrus_enc_h264_coded_append(struct cedrus_device *dev,
					 unsigned int value, unsigned int count)
{
	cedrus_poll(dev, VE_ENC_AVC_STATUS_REG,
		    VE_ENC_AVC_STATUS_PUT_BITS_READY);

	cedrus_write(dev, VE_ENC_AVC_PUTBITSDATA_REG, value);

	cedrus_write(dev, VE_ENC_AVC_STARTTRIG_REG,
		     VE_ENC_AVC_STARTTRIG_NUM_BITS(count) |
		     VE_ENC_AVC_STARTTRIG_TYPE_PUT_BITS);
}

static void cedrus_enc_h264_coded_ue(struct cedrus_device *dev,
				     unsigned int value)
{
	unsigned int bits_count;

	/*
	 * Exponential-Golomb coding of x stores the value of x + 1.
	 * This takes fls(x + 1) + 1 bits and fls(x + 1) heading zero bits
	 * are added.
	 */
	bits_count = 2 * __fls(value + 1) + 1;

	cedrus_enc_h264_coded_append(dev, value + 1, bits_count);
}

static void cedrus_enc_h264_coded_se(struct cedrus_device *dev, int value)
{
	unsigned int value_ue;

	/*
	 * The signed extension repersents numbers in Exponential-Golomb
	 * with each positive value followed by its corresponding negative
	 * value in sequence order.
	 */

	if (value > 0)
		value_ue = 2 * value - 1;
	else
		value_ue = -2 * value;

	return cedrus_enc_h264_coded_ue(dev, value_ue);
}

static void cedrus_enc_h264_coded_bytes(struct cedrus_device *dev, u32 value,
					unsigned int bytes_count)
{
	unsigned int bits = 8 * bytes_count;

	while (bits > 0) {
		u8 value_slice = (value >> (bits - 8)) & GENMASK(7, 0);

		cedrus_enc_h264_coded_append(dev, value_slice, 8);
		bits -= 8;
	}
}

static void cedrus_enc_h264_coded_u32(struct cedrus_device *dev, u32 value)
{
	cedrus_enc_h264_coded_bytes(dev, value, 4);
}

static void cedrus_enc_h264_coded_u16(struct cedrus_device *dev, u16 value)
{
	cedrus_enc_h264_coded_append(dev, value, 16);
}

static void cedrus_enc_h264_coded_u8(struct cedrus_device *dev, u8 value)
{
	cedrus_enc_h264_coded_append(dev, value, 8);
}

static void cedrus_enc_h264_coded_bit(struct cedrus_device *dev,
				      unsigned int value)
{
	cedrus_enc_h264_coded_append(dev, value, 1);
}


static void cedrus_enc_h264_coded_align(struct cedrus_device *dev)
{
	unsigned int bits_count;
	u32 value;

	value = cedrus_read(dev, VE_ENC_AVC_STM_BIT_LEN_REG);

	bits_count = value % 8;
	if (!bits_count)
		return;

	cedrus_enc_h264_coded_append(dev, 0, 8 - bits_count);
}

static void cedrus_enc_h264_coded_eptb(struct cedrus_device *dev, bool enable)
{
	u32 value;

	value = cedrus_read(dev, VE_ENC_AVC_PARA0_REG);

	if (enable)
		value &= ~VE_ENC_AVC_PARA0_EPTB_DIS;
	else
		value |= VE_ENC_AVC_PARA0_EPTB_DIS;

	cedrus_write(dev, VE_ENC_AVC_PARA0_REG, value);
}

/* Ctrl */

static int cedrus_enc_h264_ctrl_validate(struct cedrus_context *ctx,
					 struct v4l2_ctrl *ctrl)
{
	struct v4l2_device *v4l2_dev = &ctx->proc->dev->v4l2.v4l2_dev;
	int profile;

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE:
		struct v4l2_ctrl *ctrl_profile;
		unsigned int id;

		/* CABAC entropy coding availability depends on profile. */
		if (ctrl->val != V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC)
			return 0;

		id = V4L2_CID_MPEG_VIDEO_H264_PROFILE;
		ctrl_profile = cedrus_context_ctrl_find(ctx, id);
		if (WARN_ON(!ctrl_profile))
			return -ENODEV;

		profile = ctrl_profile->cur.val;

		if (!cedrus_enc_h264_profile_cabac_check(profile)) {
			v4l2_err(v4l2_dev,
				 "CABAC entropy coding is not supported with the profile currently set.\n");
			return -EINVAL;
		}
		break;
	}

	return 0;
}

static int cedrus_enc_h264_ctrl_prepare(struct cedrus_context *cedrus_ctx,
					struct v4l2_ctrl *ctrl)
{
	struct cedrus_enc_h264_context *h264_ctx = cedrus_ctx->engine_ctx;
	struct cedrus_enc_h264_state *state = &h264_ctx->state;

	/*
	 * This might (and will) be called before we have a codec context.
	 * Ignore and call v4l2_ctrl_handler_setup explicitly when the codec
	 * context is created (streaming start).
	 */
	if (!h264_ctx)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR:
		h264_ctx->prepend_sps_pps_idr = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_ENABLE:
		h264_ctx->vui_sar_enable = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_IDC:
		h264_ctx->vui_sar_idc = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_WIDTH:
		h264_ctx->vui_ext_sar_width = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_HEIGHT:
		h264_ctx->vui_ext_sar_height = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		h264_ctx->profile = ctrl->cur.val;

		if (!cedrus_enc_h264_profile_cabac_check(h264_ctx->profile)) {
			unsigned int id = V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE;
			int value = V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC;
			struct v4l2_ctrl *ctrl_entropy =
				cedrus_context_ctrl_find(cedrus_ctx, id);

			__v4l2_ctrl_s_ctrl(ctrl_entropy, value);
		}

		if (state->step > CEDRUS_ENC_H264_STEP_SPS)
			state->step = CEDRUS_ENC_H264_STEP_SPS;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		h264_ctx->level = ctrl->cur.val;

		if (state->step > CEDRUS_ENC_H264_STEP_SPS)
			state->step = CEDRUS_ENC_H264_STEP_SPS;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE:
		h264_ctx->entropy_mode = ctrl->cur.val;

		if (state->step > CEDRUS_ENC_H264_STEP_PPS)
			state->step = CEDRUS_ENC_H264_STEP_PPS;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_CHROMA_QP_INDEX_OFFSET:
		h264_ctx->chroma_qp_index_offset = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE:
		h264_ctx->loop_filter_mode = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA:
		h264_ctx->loop_filter_alpha = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA:
		h264_ctx->loop_filter_beta = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
		h264_ctx->qp_min = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
		h264_ctx->qp_max = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
		h264_ctx->qp_i = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
		h264_ctx->qp_p = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_CLOSURE:
		h264_ctx->gop_closure = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		h264_ctx->gop_size = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_PERIOD:
		h264_ctx->gop_open_i_period = ctrl->cur.val;
		break;
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		h264_ctx->force_key_frame = true;
		break;
	}

	return 0;
}

/* Context */

static int cedrus_enc_h264_setup(struct cedrus_context *cedrus_ctx)
{
	struct device *dev = cedrus_ctx->proc->dev->dev;
	struct v4l2_ctrl_handler *ctrl_handler = &cedrus_ctx->v4l2.ctrl_handler;
	struct cedrus_enc_h264_context *h264_ctx = cedrus_ctx->engine_ctx;
	struct cedrus_enc_h264_state *state = &h264_ctx->state;
	struct v4l2_pix_format *pix_format =
		&cedrus_ctx->v4l2.format_picture.fmt.pix;
	unsigned int id;
	int ret;

	h264_ctx->width_mbs = DIV_ROUND_UP(pix_format->width, 16);
	h264_ctx->height_mbs = DIV_ROUND_UP(pix_format->height, 16);

	/* Macroblock Information Buffer */

	h264_ctx->mb_info_size = DIV_ROUND_UP(h264_ctx->width_mbs, 32) * SZ_4K;
	h264_ctx->mb_info = dma_alloc_attrs(dev, h264_ctx->mb_info_size,
					    &h264_ctx->mb_info_dma, GFP_KERNEL,
					    DMA_ATTR_NO_KERNEL_MAPPING);
	if (!h264_ctx->mb_info)
		return -ENOMEM;

	/* State */

	state->step = CEDRUS_ENC_H264_STEP_START;
	state->gop_index = 0;
	state->frame_num = 0;
	state->pic_order_cnt_lsb = 0;

	h264_ctx->subpix_last_dma = 0;

	/* Bitstream Parameters */

	h264_ctx->log2_max_frame_num = 8;
	h264_ctx->pic_order_cnt_type = 0;
	h264_ctx->log2_max_pic_order_cnt_lsb = 8;

	/* Grab entropy mode control for later use. */

	id = V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE;
	h264_ctx->entropy_mode_ctrl = v4l2_ctrl_find(ctrl_handler, id);
	if (!h264_ctx->entropy_mode_ctrl) {
		ret = -ENODEV;
		goto error_dma;
	}

	/* Apply initial control values. */

	ret = v4l2_ctrl_handler_setup(ctrl_handler);
	if (ret)
		goto error_dma;

	return 0;

error_dma:
	dma_free_attrs(dev, h264_ctx->mb_info_size, h264_ctx->mb_info,
		       h264_ctx->mb_info_dma, DMA_ATTR_NO_KERNEL_MAPPING);

	return ret;
}

static void cedrus_enc_h264_cleanup(struct cedrus_context *cedrus_ctx)
{
	struct device *dev = cedrus_ctx->proc->dev->dev;
	struct cedrus_enc_h264_context *h264_ctx = cedrus_ctx->engine_ctx;

	dma_free_attrs(dev, h264_ctx->mb_info_size, h264_ctx->mb_info,
		       h264_ctx->mb_info_dma, DMA_ATTR_NO_KERNEL_MAPPING);
}

/* Buffer */

static int cedrus_enc_h264_buffer_setup(struct cedrus_context *cedrus_ctx,
					struct cedrus_buffer *cedrus_buffer)
{
	struct device *dev = cedrus_ctx->proc->dev->dev;
	struct cedrus_enc_h264_buffer *h264_buffer =
		cedrus_buffer->engine_buffer;
	struct v4l2_pix_format *pix_format =
		&cedrus_ctx->v4l2.format_picture.fmt.pix;
	unsigned int width_mbs, height_mbs;
	unsigned int subpix_size_width, subpix_size_height;
	int ret;

	width_mbs = DIV_ROUND_UP(pix_format->width, 16);
	height_mbs = DIV_ROUND_UP(pix_format->height, 16);

	/* Sub-pixel Buffer */

	subpix_size_width = ALIGN_DOWN((width_mbs + 47) * 2 / 3, 32) +
			    ALIGN(width_mbs, 32) * 2;
	subpix_size_height = (height_mbs * 16 + 72) / 8;

	h264_buffer->subpix_size = subpix_size_width * subpix_size_height;

	h264_buffer->subpix = dma_alloc_attrs(dev, h264_buffer->subpix_size,
					      &h264_buffer->subpix_dma,
					      GFP_KERNEL,
					      DMA_ATTR_NO_KERNEL_MAPPING);
	if (!h264_buffer->subpix)
		return -ENOMEM;

	/* Reconstruction Buffer */

	h264_buffer->rec_luma_size = ALIGN(width_mbs, 2) * 16 *
				     ALIGN(height_mbs + 1, 4) * 16;
	h264_buffer->rec_chroma_size = ALIGN(width_mbs, 2) * 16 *
				       ALIGN(DIV_ROUND_UP(height_mbs, 2), 4) *
				       16;

	h264_buffer->rec_size = ALIGN(h264_buffer->rec_luma_size +
				      h264_buffer->rec_chroma_size, SZ_4K);

	h264_buffer->rec = dma_alloc_attrs(dev, h264_buffer->rec_size,
					   &h264_buffer->rec_dma, GFP_KERNEL,
					   DMA_ATTR_NO_KERNEL_MAPPING);
	if (!h264_buffer->rec) {
		ret = -ENOMEM;
		goto error_subpix;
	}

	return 0;

error_subpix:
	dma_free_attrs(dev, h264_buffer->subpix_size, h264_buffer->subpix,
		       h264_buffer->subpix_dma, DMA_ATTR_NO_KERNEL_MAPPING);

	return ret;
}

static void cedrus_enc_h264_buffer_cleanup(struct cedrus_context *ctx,
					   struct cedrus_buffer *cedrus_buffer)
{
	struct device *dev = ctx->proc->dev->dev;
	struct cedrus_enc_h264_buffer *h264_buffer =
		cedrus_buffer->engine_buffer;

	dma_free_attrs(dev, h264_buffer->rec_size, h264_buffer->rec,
		       h264_buffer->rec_dma, DMA_ATTR_NO_KERNEL_MAPPING);

	dma_free_attrs(dev, h264_buffer->subpix_size, h264_buffer->subpix,
		       h264_buffer->subpix_dma, DMA_ATTR_NO_KERNEL_MAPPING);
}

/* Job */

static int cedrus_enc_h264_job_prepare(struct cedrus_context *cedrus_ctx)
{
	struct cedrus_enc_h264_context *h264_ctx = cedrus_ctx->engine_ctx;
	struct cedrus_enc_h264_state *state = &h264_ctx->state;
	struct cedrus_enc_h264_job *job = cedrus_ctx->engine_job;
	struct v4l2_ctrl_handler *ctrl_handler = &cedrus_ctx->v4l2.ctrl_handler;

	/* Sample a coherent state of the controls. */
	mutex_lock(ctrl_handler->lock);

	/* Use a single slot for each parameter. */
	job->seq_parameter_set_id = 0;
	job->pic_parameter_set_id = 0;

	/* Mark every frame as reference. */
	job->nal_ref_idc = 2;

	/* GOP */

	if (h264_ctx->gop_closure) {
		if (state->gop_index == 0)
			job->frame_type = CEDRUS_ENC_H264_FRAME_TYPE_IDR;
		else
			job->frame_type = CEDRUS_ENC_H264_FRAME_TYPE_P;
	} else {
		if (state->gop_index == 0)
			job->frame_type = CEDRUS_ENC_H264_FRAME_TYPE_IDR;
		else if (h264_ctx->gop_open_i_period > 0 &&
			 (state->gop_index % h264_ctx->gop_open_i_period) == 0)
			job->frame_type = CEDRUS_ENC_H264_FRAME_TYPE_I;
		else
			job->frame_type = CEDRUS_ENC_H264_FRAME_TYPE_P;
	}

	if (h264_ctx->force_key_frame) {
		job->frame_type = CEDRUS_ENC_H264_FRAME_TYPE_IDR;
		h264_ctx->force_key_frame = false;
	}

	state->gop_index++;

	if (h264_ctx->gop_closure)
		state->gop_index %= h264_ctx->gop_size;

	/* Identification */

	if (job->frame_type == CEDRUS_ENC_H264_FRAME_TYPE_IDR) {
		job->idr_pic_id = 0;
		state->frame_num = 0;
		state->pic_order_cnt_lsb = 0;

		if (h264_ctx->prepend_sps_pps_idr)
			state->step = CEDRUS_ENC_H264_STEP_SPS;
	}

	job->frame_num = state->frame_num;

	state->frame_num++;
	state->frame_num %= BIT(h264_ctx->log2_max_frame_num);

	job->pic_order_cnt_lsb = state->pic_order_cnt_lsb;

	state->pic_order_cnt_lsb += 2;
	state->pic_order_cnt_lsb %= BIT(h264_ctx->log2_max_pic_order_cnt_lsb);

	/* Profile/Level */

	job->profile_idc = cedrus_enc_h264_profile_idc(h264_ctx->profile);
	job->level_idc = cedrus_enc_h264_level_idc(h264_ctx->level);
	job->constraint_set_flags =
		cedrus_enc_h264_constraint_set_flags(h264_ctx->profile);

	/* Features */

	if (h264_ctx->entropy_mode == V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC) {
		job->entropy_coding_mode_flag = 1;

		if (job->frame_type == CEDRUS_ENC_H264_FRAME_TYPE_IDR ||
		    job->frame_type == CEDRUS_ENC_H264_FRAME_TYPE_I)
			job->cabac_init_idc = 0;
		else
			job->cabac_init_idc = 1;
	} else {
		job->entropy_coding_mode_flag = 0;
		job->cabac_init_idc = 0;
	}

	job->chroma_qp_index_offset = h264_ctx->chroma_qp_index_offset;

	job->disable_deblocking_filter_idc =
		cedrus_enc_h264_disable_deblocking_filter_idc(h264_ctx->loop_filter_mode);

	if (job->disable_deblocking_filter_idc != 1) {
		job->slice_alpha_c0_offset_div2 = h264_ctx->loop_filter_alpha;
		job->slice_beta_offset_div2 = h264_ctx->loop_filter_beta;
	}

	/* QP */

	switch (job->frame_type) {
	case CEDRUS_ENC_H264_FRAME_TYPE_IDR:
	case CEDRUS_ENC_H264_FRAME_TYPE_I:
		job->qp = h264_ctx->qp_i;
		break;
	case CEDRUS_ENC_H264_FRAME_TYPE_P:
		job->qp = h264_ctx->qp_p;
		break;
	}

	if (job->qp > h264_ctx->qp_max)
		job->qp = h264_ctx->qp_max;
	else if (job->qp < h264_ctx->qp_min)
		job->qp = h264_ctx->qp_min;

	/* Set initial QP to current QP with each new PPS. */
	if (state->step < CEDRUS_ENC_H264_STEP_SLICE)
		state->qp_init = job->qp;

	mutex_unlock(ctrl_handler->lock);

	return 0;
}

static void cedrus_enc_h264_job_configure_sps(struct cedrus_context *cedrus_ctx)
{
	struct cedrus_device *dev = cedrus_ctx->proc->dev;
	struct cedrus_enc_h264_job *job = cedrus_ctx->engine_job;
	struct cedrus_enc_h264_context *h264_ctx = cedrus_ctx->engine_ctx;
	struct v4l2_pix_format *pix_format =
		&cedrus_ctx->v4l2.format_picture.fmt.pix;
	struct v4l2_fract *timeperframe = &cedrus_ctx->v4l2.timeperframe_coded;
	struct v4l2_rect *selection = &cedrus_ctx->v4l2.selection_picture;
	u32 crop_left, crop_right, crop_top, crop_bottom;
	u8 profile_idc = job->profile_idc;
	u8 header;

	/* Syntax element: Annex-B start code. */
	cedrus_enc_h264_coded_u32(dev, 0x1);

	header = cedrus_enc_h264_nalu_header(CENDRUS_ENC_H264_NALU_TYPE_SPS, 3);

	/* Syntax element: NALU header. */
	cedrus_enc_h264_coded_u8(dev, header);

	/* Syntax element: profile_idc. */
	cedrus_enc_h264_coded_u8(dev, job->profile_idc);

	/* Syntax elements: constraint_set*_flag, reserved_zero_2bits. */
	cedrus_enc_h264_coded_u8(dev, job->constraint_set_flags);

	/* Syntax element: level_idc. */
	cedrus_enc_h264_coded_u8(dev, job->level_idc);

	/* Syntax element: seq_parameter_set_id. */
	cedrus_enc_h264_coded_ue(dev, job->seq_parameter_set_id);

	if (profile_idc == 100 || profile_idc == 110 || profile_idc == 122 ||
	    profile_idc == 244 || profile_idc == 44 || profile_idc == 83 ||
	    profile_idc == 86 || profile_idc == 118 || profile_idc == 128 ||
	    profile_idc == 138 || profile_idc == 139 || profile_idc == 134 ||
	    profile_idc == 135) {
		/* Syntax element: chroma_format_idc, always YUV 4:2:0 (1). */
		cedrus_enc_h264_coded_ue(dev, 1);

		/* Syntax element: bit_depth_luma_minus8. */
		cedrus_enc_h264_coded_ue(dev, 0);

		/* Syntax element: bit_depth_chroma_minus8. */
		cedrus_enc_h264_coded_ue(dev, 0);

		/* Syntax element: qpprime_y_zero_transform_bypass_flag. */
		cedrus_enc_h264_coded_bit(dev, 0);

		/* Syntax element: seq_scaling_matrix_present_flag. */
		cedrus_enc_h264_coded_bit(dev, 0);
	}

	/* Syntax element: log2_max_frame_num_minus4. */
	cedrus_enc_h264_coded_ue(dev, h264_ctx->log2_max_frame_num - 4);

	/* Syntax element: pic_order_cnt_type. */
	cedrus_enc_h264_coded_ue(dev, 0);

	/* Syntax element: log2_max_pic_order_cnt_lsb_minus4. */
	cedrus_enc_h264_coded_ue(dev, h264_ctx->log2_max_pic_order_cnt_lsb - 4);

	/* Syntax element: max_num_ref_frames. */
	cedrus_enc_h264_coded_ue(dev, 1);

	/* Syntax element: gaps_in_frame_num_value_allowed_flag. */
	cedrus_enc_h264_coded_bit(dev, 0);

	/* Syntax element: pic_width_in_mbs_minus1. */
	cedrus_enc_h264_coded_ue(dev, h264_ctx->width_mbs - 1);

	/* Syntax element: pic_height_in_map_units_minus1. */
	cedrus_enc_h264_coded_ue(dev, h264_ctx->height_mbs - 1);

	/* Syntax element: frame_mbs_only_flag. */
	cedrus_enc_h264_coded_bit(dev, 1);

	/* Syntax element: direct_8x8_inference_flag. */
	cedrus_enc_h264_coded_bit(dev, 0);

	if (selection->width != pix_format->width ||
	    selection->height != pix_format->height) {
		crop_left = selection->left;
		crop_right = pix_format->width - selection->width -
			     selection->left;
		crop_top = selection->top;
		crop_bottom = pix_format->height - selection->height -
			      selection->top;

		/* Syntax element: frame_cropping_flag. */
		cedrus_enc_h264_coded_bit(dev, 1);

		/* Syntax element: frame_crop_left_offset. */
		cedrus_enc_h264_coded_ue(dev, crop_left / 2);

		/* Syntax element: frame_crop_right_offset. */
		cedrus_enc_h264_coded_ue(dev, crop_right / 2);

		/* Syntax element: frame_crop_top_offset. */
		cedrus_enc_h264_coded_ue(dev, crop_top / 2);

		/* Syntax element: frame_crop_bottom_offset. */
		cedrus_enc_h264_coded_ue(dev, crop_bottom / 2);

	} else {
		/* Syntax element: frame_cropping_flag. */
		cedrus_enc_h264_coded_bit(dev, 0);
	}

	/* Syntax element: vui_parameters_present_flag. */
	cedrus_enc_h264_coded_bit(dev, 1);

	if (h264_ctx->vui_sar_enable) {
		u8 vui_sar_idc =
			cedrus_enc_h264_vui_sar_idc(h264_ctx->vui_sar_idc);

		/* Syntax element: aspect_ratio_info_present_flag. */
		cedrus_enc_h264_coded_bit(dev, 1);

		/* Syntax element: aspect_ratio_idc. */
		cedrus_enc_h264_coded_u8(dev, vui_sar_idc);

		if (h264_ctx->vui_sar_idc ==
		    V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED) {
			/* Syntax element: sar_width. */
			cedrus_enc_h264_coded_u16(dev,
						  h264_ctx->vui_ext_sar_width);

			/* Syntax element: sar_height. */
			cedrus_enc_h264_coded_u16(dev,
						  h264_ctx->vui_ext_sar_height);
		}
	} else {
		/* Syntax element: aspect_ratio_info_present_flag. */
		cedrus_enc_h264_coded_bit(dev, 0);
	}

	/* Syntax element: overscan_info_present_flag. */
	cedrus_enc_h264_coded_bit(dev, 0);

	/* Syntax element: video_signal_type_present_flag. */
	cedrus_enc_h264_coded_bit(dev, 0);

	/* Syntax element: chroma_loc_info_present_flag. */
	cedrus_enc_h264_coded_bit(dev, 0);

	/* Syntax element: timing_info_present_flag. */
	cedrus_enc_h264_coded_bit(dev, 1);

	/* Syntax element: num_units_in_tick. */
	cedrus_enc_h264_coded_u32(dev, timeperframe->denominator);

	/* A frame requires two ticks in H.264. */
	/* Syntax element: time_scale. */
	cedrus_enc_h264_coded_u32(dev, timeperframe->numerator * 2);

	/* Syntax element: fixed_frame_rate_flag. */
	cedrus_enc_h264_coded_bit(dev, 1);

	/* Syntax element: nal_hrd_parameters_present_flag. */
	cedrus_enc_h264_coded_bit(dev, 0);

	/* Syntax element: vcl_hrd_parameters_present_flag. */
	cedrus_enc_h264_coded_bit(dev, 0);

	/* Syntax element: pic_struct_present_flag. */
	cedrus_enc_h264_coded_bit(dev, 0);

	/* Syntax element: bitstream_restriction_flag. */
	cedrus_enc_h264_coded_bit(dev, 0);

	/* Syntax element: rbsp_stop_one_bit. */
	cedrus_enc_h264_coded_bit(dev, 1);

	cedrus_enc_h264_coded_align(dev);
}

static void cedrus_enc_h264_job_configure_pps(struct cedrus_context *cedrus_ctx)
{
	struct cedrus_device *dev = cedrus_ctx->proc->dev;
	struct cedrus_enc_h264_job *job = cedrus_ctx->engine_job;
	struct cedrus_enc_h264_context *h264_ctx = cedrus_ctx->engine_ctx;
	struct cedrus_enc_h264_state *state = &h264_ctx->state;
	u8 header;

	/* Syntax element: Annex-B start code. */
	cedrus_enc_h264_coded_u32(dev, 0x1);

	header = cedrus_enc_h264_nalu_header(CENDRUS_ENC_H264_NALU_TYPE_PPS, 3);

	/* Syntax element: NALU header. */
	cedrus_enc_h264_coded_u8(dev, header);

	/* Syntax element: pic_parameter_set_id. */
	cedrus_enc_h264_coded_ue(dev, job->pic_parameter_set_id);

	/* Syntax element: seq_parameter_set_id. */
	cedrus_enc_h264_coded_ue(dev, job->seq_parameter_set_id);

	/* Syntax element: entropy_coding_mode_flag. */
	cedrus_enc_h264_coded_bit(dev, job->entropy_coding_mode_flag);

	/* Syntax element: bottom_field_pic_order_in_frame_present_flag. */
	cedrus_enc_h264_coded_bit(dev, 0);

	/* Syntax element: num_slice_groups_minus1. */
	cedrus_enc_h264_coded_ue(dev, 0);

	/* Syntax element: num_ref_idx_l0_default_active_minus1. */
	cedrus_enc_h264_coded_ue(dev, 0);

	/* Syntax element: num_ref_idx_l1_default_active_minus1. */
	cedrus_enc_h264_coded_ue(dev, 0);

	/* Syntax element: weighted_pred_flag. */
	cedrus_enc_h264_coded_bit(dev, 0);

	/* Syntax element: weighted_bipred_idc. */
	cedrus_enc_h264_coded_append(dev, 0, 2);

	/* Syntax element: pic_init_qp_minus26. */
	cedrus_enc_h264_coded_se(dev, state->qp_init - 26);

	/* Syntax element: pic_init_qs_minus26. */
	cedrus_enc_h264_coded_se(dev, state->qp_init - 26);

	/* Syntax element: chroma_qp_index_offset. */
	cedrus_enc_h264_coded_se(dev, job->chroma_qp_index_offset);

	/* Syntax element: deblocking_filter_control_present_flag. */
	cedrus_enc_h264_coded_bit(dev, 1);

	/* Syntax element: constrained_intra_pred_flag. */
	cedrus_enc_h264_coded_bit(dev, 0);

	/* Syntax element: redundant_pic_cnt_present_flag. */
	cedrus_enc_h264_coded_bit(dev, 0);

	/* Syntax element: rbsp_stop_one_bit. */
	cedrus_enc_h264_coded_bit(dev, 1);

	cedrus_enc_h264_coded_align(dev);
}

static void
cedrus_enc_h264_job_configure_slice_header(struct cedrus_context *cedrus_ctx)
{
	struct cedrus_device *dev = cedrus_ctx->proc->dev;
	struct cedrus_enc_h264_job *job = cedrus_ctx->engine_job;
	struct cedrus_enc_h264_context *h264_ctx = cedrus_ctx->engine_ctx;
	struct cedrus_enc_h264_state *state = &h264_ctx->state;
	u8 slice_type;
	u8 nalu_type;
	u8 header;

	/* Syntax element: Annex-B start code. */
	cedrus_enc_h264_coded_u32(dev, 0x1);

	if (job->frame_type == CEDRUS_ENC_H264_FRAME_TYPE_IDR)
		nalu_type = CENDRUS_ENC_H264_NALU_TYPE_SLICE_IDR;
	else
		nalu_type = CENDRUS_ENC_H264_NALU_TYPE_SLICE_NON_IDR;

	header = cedrus_enc_h264_nalu_header(nalu_type, job->nal_ref_idc);

	/* Syntax element: NALU header. */
	cedrus_enc_h264_coded_u8(dev, header);

	/* Syntax element: first_mb_in_slice. */
	cedrus_enc_h264_coded_ue(dev, 0);

	/* Syntax element: slice_type. */
	if (job->frame_type == CEDRUS_ENC_H264_FRAME_TYPE_IDR ||
	    job->frame_type == CEDRUS_ENC_H264_FRAME_TYPE_I)
		slice_type = CEDRUS_ENC_H264_SLICE_TYPE_I;
	else
		slice_type = CEDRUS_ENC_H264_SLICE_TYPE_P;

	cedrus_enc_h264_coded_ue(dev, slice_type);

	/* Syntax element: pic_parameter_set_id. */
	cedrus_enc_h264_coded_ue(dev, job->pic_parameter_set_id);

	/* Syntax element: frame_num. */
	cedrus_enc_h264_coded_append(dev, job->frame_num,
				    h264_ctx->log2_max_frame_num);

	if (job->frame_type == CEDRUS_ENC_H264_FRAME_TYPE_IDR) {
		/* Syntax element: idr_pic_id. */
		cedrus_enc_h264_coded_ue(dev, job->idr_pic_id);
	}

	if (h264_ctx->pic_order_cnt_type == 0) {
		/* Syntax element: pic_order_cnt_lsb. */
		cedrus_enc_h264_coded_append(dev, job->pic_order_cnt_lsb,
					    h264_ctx->log2_max_pic_order_cnt_lsb);
	}

	if (slice_type == CEDRUS_ENC_H264_SLICE_TYPE_P) {
		/* Syntax element: num_ref_idx_active_override_flag. */
		cedrus_enc_h264_coded_bit(dev, 0);

		/* Syntax element: ref_pic_list_modification_flag_l0. */
		cedrus_enc_h264_coded_bit(dev, 0);
	}

	/* XXX: only for pictures marked as reference. */
	if (job->frame_type == CEDRUS_ENC_H264_FRAME_TYPE_IDR) {
		/* Syntax element: no_output_of_prior_pics_flag. */
		cedrus_enc_h264_coded_bit(dev, 0);

		/* Syntax element: long_term_reference_flag. */
		cedrus_enc_h264_coded_bit(dev, 0);
	} else {
		/* Syntax element: adaptive_ref_pic_marking_mode_flag. */
		cedrus_enc_h264_coded_bit(dev, 0);
	}

	if (slice_type != CEDRUS_ENC_H264_SLICE_TYPE_I &&
	    job->entropy_coding_mode_flag) {
		/* Syntax element: cabac_init_idc. */
		cedrus_enc_h264_coded_ue(dev, job->cabac_init_idc);
	}

	/* Syntax element: slice_qp_delta. */
	cedrus_enc_h264_coded_se(dev, job->qp - state->qp_init);

	/* Syntax element: disable_deblocking_filter_idc. */
	cedrus_enc_h264_coded_ue(dev, job->disable_deblocking_filter_idc);

	if (job->disable_deblocking_filter_idc != 1) {
		/* Syntax element: slice_alpha_c0_offset_div2. */
		cedrus_enc_h264_coded_se(dev, job->slice_alpha_c0_offset_div2);

		/* Syntax element: slice_beta_offset_div2. */
		cedrus_enc_h264_coded_se(dev, job->slice_beta_offset_div2);
	}
}

static void cedrus_enc_h264_job_configure_headers(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	struct cedrus_enc_h264_context *h264_ctx = ctx->engine_ctx;
	struct cedrus_enc_h264_state *state = &h264_ctx->state;
	bool active = true;

	/* Disable emulation-prevention 0x3 byte. */
	cedrus_enc_h264_coded_eptb(dev, 0);

	while (active) {
		switch (state->step) {
		case CEDRUS_ENC_H264_STEP_START:
			state->step = CEDRUS_ENC_H264_STEP_SPS;
			break;
		case CEDRUS_ENC_H264_STEP_SPS:
			cedrus_enc_h264_job_configure_sps(ctx);
			state->step = CEDRUS_ENC_H264_STEP_PPS;
			break;
		case CEDRUS_ENC_H264_STEP_PPS:
			cedrus_enc_h264_job_configure_pps(ctx);
			state->step = CEDRUS_ENC_H264_STEP_SLICE;
			break;
		case CEDRUS_ENC_H264_STEP_SLICE:
			cedrus_enc_h264_job_configure_slice_header(ctx);
			active = false;
			break;
		}
	}

	/* Enable emulation-prevention 0x3 byte. */
	cedrus_enc_h264_coded_eptb(dev, 1);

	/* Wait for sync idle. */
	cedrus_poll(dev, VE_RESET_REG,
		    VE_RESET_CACHE_SYNC_IDLE |
		    VE_RESET_SYNC_IDLE);
}

static int cedrus_enc_h264_job_configure(struct cedrus_context *cedrus_ctx)
{
	struct cedrus_device *dev = cedrus_ctx->proc->dev;
	struct cedrus_enc_h264_job *job = cedrus_ctx->engine_job;
	struct cedrus_enc_h264_context *h264_ctx = cedrus_ctx->engine_ctx;
	struct cedrus_enc_h264_buffer *h264_buffer =
		cedrus_job_engine_buffer(cedrus_ctx);
	struct v4l2_pix_format *pix_format =
		&cedrus_ctx->v4l2.format_picture.fmt.pix;
	unsigned int stride_mbs_div_48;
	unsigned int luma_size;
	unsigned int size;
	dma_addr_t addr;
	u32 value;

	cedrus_write(dev, VE_ENC_AVC_STARTTRIG_REG, 0);

	/* Configure coded buffer. */

	cedrus_write(dev, VE_ENC_AVC_STM_BIT_OFFSET_REG, 0);

	cedrus_job_buffer_coded_dma(cedrus_ctx, &addr, &size);

	cedrus_write(dev, VE_ENC_AVC_STM_START_ADDR_REG, addr);
	cedrus_write(dev, VE_ENC_AVC_STM_END_ADDR_REG, addr + size - 1);

	cedrus_write(dev, VE_ENC_AVC_STM_BIT_MAX_REG, size * 8);

	cedrus_write(dev, VE_ENC_AVC_STM_BIT_LEN_REG, 0);
	cedrus_write(dev, VE_ENC_AVC_HEADER_BITS_REG, 0);
	cedrus_write(dev, VE_ENC_AVC_RESIDUAL_BITS_REG, 0);

	/* Produce H.264 headers. */

	cedrus_enc_h264_job_configure_headers(cedrus_ctx);

	/* Configure macroblock info buffer. */

	cedrus_write(dev, VE_ENC_AVC_MB_INFO_ADDR_REG, h264_ctx->mb_info_dma);

	/* Clear motion vector buffer. */

	cedrus_write(dev, VE_ENC_AVC_MV_BUF_ADDR_REG, 0);

	/* Configure reconstruction buffer. */

	cedrus_write(dev, VE_ENC_AVC_REC_ADDR_Y_REG, h264_buffer->rec_dma);
	cedrus_write(dev, VE_ENC_AVC_REC_ADDR_C_REG, h264_buffer->rec_dma +
		     h264_buffer->rec_luma_size);

	if (job->frame_type == CEDRUS_ENC_H264_FRAME_TYPE_P) {
		addr = h264_ctx->rec_last_dma;
		luma_size = h264_ctx->rec_last_luma_size;
	} else {
		/* XXX: is this really needed? */
		addr = h264_buffer->rec_dma;
		luma_size = h264_buffer->rec_luma_size;
	}

	cedrus_write(dev, VE_ENC_AVC_REF0_ADDR_Y_REG, addr);
	cedrus_write(dev, VE_ENC_AVC_REF0_ADDR_C_REG, addr + luma_size);

	h264_ctx->rec_last_dma = h264_buffer->rec_dma;
	h264_ctx->rec_last_luma_size = h264_buffer->rec_luma_size;

	/* Configure subpixel buffers. */

	cedrus_write(dev, VE_ENC_AVC_SUBPIX_ADDR_NEW_REG,
		     h264_buffer->subpix_dma);

	if (!h264_ctx->subpix_last_dma)
		h264_ctx->subpix_last_dma = h264_buffer->subpix_dma;

	/* XXX: is this for the last reference or the last encoded frame? */
	cedrus_write(dev, VE_ENC_AVC_SUBPIX_ADDR_LAST_REG,
		     h264_ctx->subpix_last_dma);

	h264_ctx->subpix_last_dma = h264_buffer->subpix_dma;

	/* Configure deblocking filter buffer. */

	cedrus_write(dev, VE_ENC_AVC_DEBLK_ADDR_REG, 0);

	/* Configure cyclic intra refresh. */

	cedrus_write(dev, VE_ENC_AVC_CYCLIC_INTRA_REFRESH_REG, 0);

	/* Configure encode parameters. */

	/*
	 * Frame num is always set to 0 here, regardless of the slice header
	 * element value.
	 */
	value = VE_ENC_AVC_PARA0_FRAME_NUM(0) |
		VE_ENC_AVC_PARA0_BETA_OFFSET_DIV2(job->slice_beta_offset_div2) |
		VE_ENC_AVC_PARA0_ALPHA_OFFSET_DIV2(job->slice_alpha_c0_offset_div2) |
		VE_ENC_AVC_PARA0_FIX_MODE_NUM(job->cabac_init_idc) |
		VE_ENC_AVC_PARA0_REF_PIC_TYPE_FRAME |
		VE_ENC_AVC_PARA0_PIC_TYPE_FRAME;

	if (job->entropy_coding_mode_flag)
		value |= VE_ENC_AVC_PARA0_ENTROPY_CODING_CABAC;
	else
		value |= VE_ENC_AVC_PARA0_ENTROPY_CODING_CAVLC;

	switch (job->frame_type) {
	case CEDRUS_ENC_H264_FRAME_TYPE_IDR:
	case CEDRUS_ENC_H264_FRAME_TYPE_I:
		value |= VE_ENC_AVC_PARA0_SLICE_TYPE_I;
		break;
	case CEDRUS_ENC_H264_FRAME_TYPE_P:
		value |= VE_ENC_AVC_PARA0_SLICE_TYPE_P;
		break;
	}

	cedrus_write(dev, VE_ENC_AVC_PARA0_REG, value);

	stride_mbs_div_48 = DIV_ROUND_UP(pix_format->bytesperline / 16, 48);

	cedrus_write(dev, VE_ENC_AVC_PARA1_REG,
		     VE_ENC_AVC_PARA1_QP_CHROMA_OFFSET0(job->chroma_qp_index_offset) |
		     VE_ENC_AVC_PARA1_STRIDE_MBS_DIV_48(stride_mbs_div_48) |
		     VE_ENC_AVC_PARA1_RC_MODE_FIXED |
		     VE_ENC_AVC_PARA1_FIXED_QP(job->qp));

	cedrus_write(dev, VE_ENC_AVC_PARA2_REG, 0);

	/* Dynamic motion estimation is disabled. */
	cedrus_write(dev, VE_ENC_AVC_DYNAMIC_ME_PAR0_REG, 0);
	cedrus_write(dev, VE_ENC_AVC_DYNAMIC_ME_PAR1_REG, 0);

	/* Configure rate-control parameters. */

	cedrus_write(dev, VE_ENC_AVC_RC_INIT_REG, 0);
	cedrus_write(dev, VE_ENC_AVC_RC_MAD_TH0_REG, 0);
	cedrus_write(dev, VE_ENC_AVC_RC_MAD_TH1_REG, 0);
	cedrus_write(dev, VE_ENC_AVC_RC_MAD_TH2_REG, 0);
	cedrus_write(dev, VE_ENC_AVC_RC_MAD_TH3_REG, 0);

	/* Configure motion estimation parameters. */

	cedrus_write(dev, VE_ENC_AVC_ME_PARA_REG,
		     VE_ENC_AVC_ME_PARA_WB_MV_INFO_DIS |
		     VE_ENC_AVC_ME_PARA_FME_SEARCH_LEVEL(2));

	/* Clear statistics. */

	cedrus_write(dev, VE_ENC_AVC_MAD_REG, 0);
	cedrus_write(dev, VE_ENC_AVC_OVERTIME_MB_REG, 0);
	cedrus_write(dev, VE_ENC_AVC_ME_INFO_REG, 0);

	return 0;
}

static void cedrus_enc_h264_job_trigger(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;

	/* Enable interrupt. */

	cedrus_write(dev, VE_ENC_AVC_INT_EN_REG,
		     VE_ENC_AVC_INT_EN_STALL |
		     VE_ENC_AVC_INT_EN_FINISH);

	/* Trigger encode start. */

	cedrus_write(dev, VE_ENC_AVC_STARTTRIG_REG,
		     VE_ENC_AVC_STARTTRIG_ENCODE_MODE_H264 |
		     VE_ENC_AVC_STARTTRIG_TYPE_ENC_START);
}

static void cedrus_enc_h264_job_finish(struct cedrus_context *ctx, int state)
{
	struct cedrus_device *dev = ctx->proc->dev;
	struct vb2_v4l2_buffer *v4l2_buffer = ctx->job.buffer_coded;
	struct vb2_buffer *vb2_buffer = &v4l2_buffer->vb2_buf;
	struct cedrus_enc_h264_job *job = ctx->engine_job;
	u32 length;

	if (state != VB2_BUF_STATE_DONE) {
		vb2_set_plane_payload(vb2_buffer, 0, 0);
		return;
	}

	length = cedrus_read(dev, VE_ENC_AVC_STM_BIT_LEN_REG);

	WARN_ON(length % 8);
	length /= 8;

	WARN_ON(length > vb2_plane_size(vb2_buffer, 0));

	vb2_set_plane_payload(vb2_buffer, 0, length);

	switch (job->frame_type) {
	case CEDRUS_ENC_H264_FRAME_TYPE_IDR:
	case CEDRUS_ENC_H264_FRAME_TYPE_I:
		v4l2_buffer->flags |= V4L2_BUF_FLAG_KEYFRAME;
		break;
	case CEDRUS_ENC_H264_FRAME_TYPE_P:
		v4l2_buffer->flags |= V4L2_BUF_FLAG_PFRAME;
		break;
	}
}

/* IRQ */

static int cedrus_enc_h264_irq_status(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	u32 status;

	status = cedrus_read(dev, VE_ENC_AVC_STATUS_REG);
	if (!(status & VE_ENC_AVC_STATUS_MASK))
		return CEDRUS_IRQ_NONE;

	if (status & VE_ENC_AVC_STATUS_FINISH)
		return CEDRUS_IRQ_SUCCESS;

	return CEDRUS_IRQ_ERROR;
}

static void cedrus_enc_h264_irq_clear(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;

	cedrus_write(dev, VE_ENC_AVC_STATUS_REG, VE_ENC_AVC_STATUS_MASK);
}

static void cedrus_enc_h264_irq_disable(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;

	cedrus_write(dev, VE_ENC_AVC_INT_EN_REG, 0);
}

/* Engine */

static const struct cedrus_engine_ops cedrus_enc_h264_ops = {
	.ctrl_validate		= cedrus_enc_h264_ctrl_validate,
	.ctrl_prepare		= cedrus_enc_h264_ctrl_prepare,

	.format_prepare		= cedrus_enc_format_coded_prepare,
	.format_configure	= cedrus_enc_format_coded_configure,

	.setup			= cedrus_enc_h264_setup,
	.cleanup		= cedrus_enc_h264_cleanup,

	.buffer_setup		= cedrus_enc_h264_buffer_setup,
	.buffer_cleanup		= cedrus_enc_h264_buffer_cleanup,

	.job_prepare		= cedrus_enc_h264_job_prepare,
	.job_configure		= cedrus_enc_h264_job_configure,
	.job_trigger		= cedrus_enc_h264_job_trigger,
	.job_finish		= cedrus_enc_h264_job_finish,

	.irq_status		= cedrus_enc_h264_irq_status,
	.irq_clear		= cedrus_enc_h264_irq_clear,
	.irq_disable		= cedrus_enc_h264_irq_disable,
};

static const struct v4l2_ctrl_config cedrus_enc_h264_ctrl_configs[] = {
	/* Queue */

	{
		.id		= V4L2_CID_MIN_BUFFERS_FOR_OUTPUT,
		.step		= 1,
		.min		= 1,
		.max		= 32,
		.def		= 1,
		.flags		= V4L2_CTRL_FLAG_VOLATILE,
	},

	/* Bitstream */

	{
		.id		= V4L2_CID_MPEG_VIDEO_HEADER_MODE,
		.min		= V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
		.max		= V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
		.def		= V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE,
		.min		= V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE,
		.max		= V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE,
		.def		= V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR,
		.step		= 1,
		.min		= 0,
		.max		= 1,
		.def		= 0,
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_FRAME_SKIP_MODE,
		.min		= V4L2_MPEG_VIDEO_FRAME_SKIP_MODE_DISABLED,
		.max		= V4L2_MPEG_VIDEO_FRAME_SKIP_MODE_DISABLED,
		.def		= V4L2_MPEG_VIDEO_FRAME_SKIP_MODE_DISABLED,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_ENABLE,
		.def		= 0,
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_IDC,
		.min		= V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_UNSPECIFIED,
		.max		= V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED,
		.def		= V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_UNSPECIFIED,
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_WIDTH,
		.step		= 1,
		.min		= 1,
		.max		= USHRT_MAX,
		.def		= 1,
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_HEIGHT,
		.step		= 1,
		.min		= 1,
		.max		= USHRT_MAX,
		.def		= 1,
		.ops		= &cedrus_context_ctrl_ops,
	},

	/* Profile/Level */

	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.min		= V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
		.max		= V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
		.def		= V4L2_MPEG_VIDEO_H264_PROFILE_MAIN,
		.menu_skip_mask	= BIT(V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED) |
				  BIT(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10) |
				  BIT(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422) |
				  BIT(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE) |
				  BIT(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10_INTRA) |
				  BIT(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422_INTRA) |
				  BIT(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_INTRA) |
				  BIT(V4L2_MPEG_VIDEO_H264_PROFILE_CAVLC_444_INTRA) |
				  BIT(V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_BASELINE) |
				  BIT(V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_HIGH) |
				  BIT(V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_HIGH_INTRA) |
				  BIT(V4L2_MPEG_VIDEO_H264_PROFILE_STEREO_HIGH) |
				  BIT(V4L2_MPEG_VIDEO_H264_PROFILE_MULTIVIEW_HIGH) |
				  BIT(V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH),
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_LEVEL,
		.min		= V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		.max		= V4L2_MPEG_VIDEO_H264_LEVEL_6_2,
		.def		= V4L2_MPEG_VIDEO_H264_LEVEL_3_1,
		.ops		= &cedrus_context_ctrl_ops,
	},

	/* Features */

	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE,
		.min		= V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC,
		.max		= V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC,
		.def		= V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC,
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_CHROMA_QP_INDEX_OFFSET,
		.step		= 1,
		.min		= 0,
		.max		= 7,
		.def		= 4,
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE,
		.min		= V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED,
		.max		= V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY,
		.def		= V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED,
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA,
		.step		= 1,
		.min		= -6,
		.max		= 6,
		.def		= 0,
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA,
		.step		= 1,
		.min		= -6,
		.max		= 6,
		.def		= 0,
		.ops		= &cedrus_context_ctrl_ops,
	},

	/* Quality and Rate Control */

	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_MIN_QP,
		.step		= 1,
		.min		= 0,
		.max		= 51,
		.def		= 10,
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
		.step		= 1,
		.min		= 0,
		.max		= 51,
		.def		= 40,
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
		.step		= 1,
		.min		= 0,
		.max		= 51,
		.def		= 26,
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP,
		.step		= 1,
		.min		= 0,
		.max		= 51,
		.def		= 28,
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_GOP_CLOSURE,
		.def		= 1,
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_GOP_SIZE,
		.step		= 1,
		.min		= 1,
		.max		= USHRT_MAX,
		.def		= 12,
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_H264_I_PERIOD,
		.step		= 1,
		.min		= 1,
		.max		= USHRT_MAX,
		.def		= 12,
		.ops		= &cedrus_context_ctrl_ops,
	},
	{
		.id		= V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,
		.ops		= &cedrus_context_ctrl_ops,
	},
};

static const struct v4l2_frmsize_stepwise cedrus_enc_h264_frmsize = {
	.min_width	= 16,
	.max_width	= 4096,
	.step_width	= 16,

	.min_height	= 16,
	.max_height	= 4096,
	.step_height	= 16,
};

const struct cedrus_engine cedrus_enc_h264 = {
	.codec			= CEDRUS_CODEC_H264,
	.role			= CEDRUS_ROLE_ENCODER,
	.capabilities		= CEDRUS_CAPABILITY_H264_ENC,

	.ops			= &cedrus_enc_h264_ops,

	.pixelformat		= V4L2_PIX_FMT_H264,
	.ctrl_configs		= cedrus_enc_h264_ctrl_configs,
	.ctrl_configs_count	= ARRAY_SIZE(cedrus_enc_h264_ctrl_configs),
	.frmsize		= &cedrus_enc_h264_frmsize,

	.ctx_size		= sizeof(struct cedrus_enc_h264_context),
	.job_size		= sizeof(struct cedrus_enc_h264_job),
	.buffer_size		= sizeof(struct cedrus_enc_h264_buffer),
};
