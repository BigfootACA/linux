// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2013 Jens Kuske <jenskuske@gmail.com>
 * Copyright 2018-2023 Bootlin
 * Author: Maxime Ripard <maxime.ripard@bootlin.com>
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>

#include "cedrus.h"
#include "cedrus_context.h"
#include "cedrus_dec.h"
#include "cedrus_dec_h264.h"
#include "cedrus_engine.h"
#include "cedrus_proc.h"
#include "cedrus_regs.h"

/* Helpers */

static void cedrus_dec_h264_mv_col_buf_dma(struct cedrus_buffer *cedrus_buffer,
					   dma_addr_t *top_addr,
					   dma_addr_t *bottom_addr)
{
	struct cedrus_dec_h264_buffer *h264_buffer =
		cedrus_buffer->engine_buffer;
	dma_addr_t addr;

	addr = h264_buffer->mv_col_buf_dma;
	*top_addr = addr;

	addr += h264_buffer->mv_col_buf_size / 2;
	*bottom_addr = addr;
}

/* Context */

static int cedrus_dec_h264_setup(struct cedrus_context *cedrus_ctx)
{
	struct device *dev = cedrus_ctx->proc->dev->dev;
	struct cedrus_dec_h264_context *h264_ctx = cedrus_ctx->engine_ctx;
	struct v4l2_pix_format *pix_format =
		&cedrus_ctx->v4l2.format_coded.fmt.pix;
	unsigned int pic_info_buf_size;
	int ret;

	/*
	 * NOTE: All buffers allocated here are only used by HW, so we
	 * can add DMA_ATTR_NO_KERNEL_MAPPING flag when allocating them.
	 */

	/* Formula for picture buffer size is taken from CedarX source. */

	if (pix_format->width > 2048)
		pic_info_buf_size = CEDRUS_DEC_H264_FRAME_NUM * 0x4000;
	else
		pic_info_buf_size = CEDRUS_DEC_H264_FRAME_NUM * 0x1000;

	/*
	 * FIXME: If V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY is set,
	 * there is no need to multiply by 2.
	 */
	pic_info_buf_size += pix_format->height * 2 * 64;

	if (pic_info_buf_size < CEDRUS_DEC_H264_PIC_INFO_BUF_SIZE_MIN)
		pic_info_buf_size = CEDRUS_DEC_H264_PIC_INFO_BUF_SIZE_MIN;

	h264_ctx->pic_info_buf_size = pic_info_buf_size;
	h264_ctx->pic_info_buf =
		dma_alloc_attrs(dev, h264_ctx->pic_info_buf_size,
				&h264_ctx->pic_info_buf_dma,
				GFP_KERNEL, DMA_ATTR_NO_KERNEL_MAPPING);
	if (!h264_ctx->pic_info_buf)
		return -ENOMEM;

	/*
	 * That buffer is supposed to be 16kiB in size, and be aligned
	 * on 16kiB as well. However, dma_alloc_attrs provides the
	 * guarantee that we'll have a DMA address aligned on the
	 * smallest page order that is greater to the requested size,
	 * so we don't have to overallocate.
	 */
	h264_ctx->neighbor_info_buf =
		dma_alloc_attrs(dev, CEDRUS_DEC_H264_NEIGHBOR_INFO_BUF_SIZE,
				&h264_ctx->neighbor_info_buf_dma,
				GFP_KERNEL, DMA_ATTR_NO_KERNEL_MAPPING);
	if (!h264_ctx->neighbor_info_buf) {
		ret = -ENOMEM;
		goto error_pic_info_buf;
	}

	if (pix_format->width > 2048) {
		/*
		 * Formulas for deblock and intra prediction buffer sizes
		 * are taken from CedarX source.
		 */

		h264_ctx->deblk_buf_size =
			ALIGN(pix_format->width, 32) * 12;
		h264_ctx->deblk_buf =
			dma_alloc_attrs(dev, h264_ctx->deblk_buf_size,
					&h264_ctx->deblk_buf_dma,
					GFP_KERNEL, DMA_ATTR_NO_KERNEL_MAPPING);
		if (!h264_ctx->deblk_buf) {
			ret = -ENOMEM;
			goto error_neighbor_info_buf;
		}

		/*
		 * NOTE: Multiplying by two deviates from CedarX logic, but it
		 * is for some unknown reason needed for H264 4K decoding on H6.
		 */
		h264_ctx->intra_pred_buf_size =
			ALIGN(pix_format->width, 64) * 5 * 2;
		h264_ctx->intra_pred_buf =
			dma_alloc_attrs(dev, h264_ctx->intra_pred_buf_size,
					&h264_ctx->intra_pred_buf_dma,
					GFP_KERNEL, DMA_ATTR_NO_KERNEL_MAPPING);
		if (!h264_ctx->intra_pred_buf) {
			ret = -ENOMEM;
			goto error_deblk_buf;
		}
	}

	return 0;

error_deblk_buf:
	dma_free_attrs(dev, h264_ctx->deblk_buf_size,
		       h264_ctx->deblk_buf,
		       h264_ctx->deblk_buf_dma,
		       DMA_ATTR_NO_KERNEL_MAPPING);

error_neighbor_info_buf:
	dma_free_attrs(dev, CEDRUS_DEC_H264_NEIGHBOR_INFO_BUF_SIZE,
		       h264_ctx->neighbor_info_buf,
		       h264_ctx->neighbor_info_buf_dma,
		       DMA_ATTR_NO_KERNEL_MAPPING);

error_pic_info_buf:
	dma_free_attrs(dev, h264_ctx->pic_info_buf_size,
		       h264_ctx->pic_info_buf,
		       h264_ctx->pic_info_buf_dma,
		       DMA_ATTR_NO_KERNEL_MAPPING);

	return ret;
}

static void cedrus_dec_h264_cleanup(struct cedrus_context *cedrus_ctx)
{
	struct device *dev = cedrus_ctx->proc->dev->dev;
	struct cedrus_dec_h264_context *h264_ctx = cedrus_ctx->engine_ctx;

	dma_free_attrs(dev, h264_ctx->pic_info_buf_size,
		       h264_ctx->pic_info_buf,
		       h264_ctx->pic_info_buf_dma,
		       DMA_ATTR_NO_KERNEL_MAPPING);

	dma_free_attrs(dev, CEDRUS_DEC_H264_NEIGHBOR_INFO_BUF_SIZE,
		       h264_ctx->neighbor_info_buf,
		       h264_ctx->neighbor_info_buf_dma,
		       DMA_ATTR_NO_KERNEL_MAPPING);

	if (h264_ctx->deblk_buf_size)
		dma_free_attrs(dev, h264_ctx->deblk_buf_size,
			       h264_ctx->deblk_buf,
			       h264_ctx->deblk_buf_dma,
			       DMA_ATTR_NO_KERNEL_MAPPING);

	if (h264_ctx->intra_pred_buf_size)
		dma_free_attrs(dev, h264_ctx->intra_pred_buf_size,
			       h264_ctx->intra_pred_buf,
			       h264_ctx->intra_pred_buf_dma,
			       DMA_ATTR_NO_KERNEL_MAPPING);
}

/* Buffer */

static void cedrus_dec_h264_buffer_cleanup(struct cedrus_context *cedrus_ctx,
					   struct cedrus_buffer *cedrus_buffer)
{
	struct device *dev = cedrus_ctx->proc->dev->dev;
	struct cedrus_dec_h264_buffer *h264_buffer =
		cedrus_buffer->engine_buffer;

	if (h264_buffer->mv_col_buf_size) {
		dma_free_attrs(dev, h264_buffer->mv_col_buf_size,
			       h264_buffer->mv_col_buf,
			       h264_buffer->mv_col_buf_dma,
			       DMA_ATTR_NO_KERNEL_MAPPING);

		h264_buffer->mv_col_buf_size = 0;
	}
}

/* Job */

static int cedrus_dec_h264_job_prepare(struct cedrus_context *ctx)
{
	struct cedrus_dec_h264_job *job = ctx->engine_job;
	u32 id;

	id = V4L2_CID_STATELESS_H264_SPS;
	job->sps = cedrus_context_ctrl_data(ctx, id);

	id = V4L2_CID_STATELESS_H264_PPS;
	job->pps = cedrus_context_ctrl_data(ctx, id);

	id = V4L2_CID_STATELESS_H264_SCALING_MATRIX;
	job->scaling_matrix = cedrus_context_ctrl_data(ctx, id);

	id = V4L2_CID_STATELESS_H264_SLICE_PARAMS;
	job->slice_params = cedrus_context_ctrl_data(ctx, id);

	id = V4L2_CID_STATELESS_H264_PRED_WEIGHTS;
	job->pred_weights = cedrus_context_ctrl_data(ctx, id);

	id = V4L2_CID_STATELESS_H264_DECODE_PARAMS;
	job->decode_params = cedrus_context_ctrl_data(ctx, id);

	return 0;
}

static void cedrus_h264_write_sram(struct cedrus_context *ctx,
				   unsigned int off,
				   const void *data, size_t len)
{
	struct cedrus_device *dev = ctx->proc->dev;
	const u32 *buffer = data;
	size_t count = DIV_ROUND_UP(len, 4);

	cedrus_write(dev, VE_AVC_SRAM_PORT_OFFSET, off << 2);

	while (count--)
		cedrus_write(dev, VE_AVC_SRAM_PORT_DATA, *buffer++);
}

static void cedrus_fill_ref_pic(struct cedrus_context *ctx,
				struct cedrus_buffer *cedrus_buffer,
				unsigned int top_field_order_cnt,
				unsigned int bottom_field_order_cnt,
				struct cedrus_dec_h264_sram_ref_pic *pic)
{
	struct cedrus_dec_h264_buffer *h264_buffer =
		cedrus_buffer->engine_buffer;
	dma_addr_t luma_addr, chroma_addr;
	dma_addr_t mv_col_buf_top_addr, mv_col_buf_bottom_addr;

	cedrus_buffer_picture_dma(ctx, cedrus_buffer, &luma_addr, &chroma_addr);
	cedrus_dec_h264_mv_col_buf_dma(cedrus_buffer, &mv_col_buf_top_addr,
				       &mv_col_buf_bottom_addr);

	pic->top_field_order_cnt = cpu_to_le32(top_field_order_cnt);
	pic->bottom_field_order_cnt = cpu_to_le32(bottom_field_order_cnt);
	pic->frame_info = cpu_to_le32(h264_buffer->pic_type << 8);

	pic->luma_ptr = cpu_to_le32(luma_addr);
	pic->chroma_ptr = cpu_to_le32(chroma_addr);
	pic->mv_col_top_ptr = cpu_to_le32(mv_col_buf_top_addr);
	pic->mv_col_bot_ptr = cpu_to_le32(mv_col_buf_bottom_addr);
}

static int cedrus_write_frame_list(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	struct cedrus_dec_h264_job *h264_job = ctx->engine_job;
	const struct v4l2_ctrl_h264_decode_params *decode =
		h264_job->decode_params;
	const struct v4l2_ctrl_h264_sps *sps = h264_job->sps;
	struct v4l2_pix_format *pix_format = &ctx->v4l2.format_coded.fmt.pix;
	struct cedrus_dec_h264_sram_ref_pic pic_list[CEDRUS_DEC_H264_FRAME_NUM];
	struct cedrus_buffer *cedrus_buffer_picture;
	struct cedrus_dec_h264_buffer *h264_buffer_picture;
	unsigned long used_dpbs = 0;
	u64 timestamp;
	unsigned int position;
	int output = -1;
	unsigned int i;

	cedrus_buffer_picture = cedrus_job_buffer_picture(ctx);
	h264_buffer_picture = cedrus_buffer_picture->engine_buffer;

	timestamp = cedrus_buffer_timestamp(cedrus_buffer_picture);

	memset(pic_list, 0, sizeof(pic_list));

	for (i = 0; i < ARRAY_SIZE(decode->dpb); i++) {
		const struct v4l2_h264_dpb_entry *dpb = &decode->dpb[i];
		struct cedrus_buffer *cedrus_buffer_ref;
		struct cedrus_dec_h264_buffer *h264_buffer_ref;

		if (!(dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID))
			continue;

		cedrus_buffer_ref =
			cedrus_buffer_picture_find(ctx, dpb->reference_ts);
		if (!cedrus_buffer_ref)
			continue;

		h264_buffer_ref = cedrus_buffer_ref->engine_buffer;

		position = h264_buffer_ref->position;
		used_dpbs |= BIT(position);

		if (timestamp == dpb->reference_ts) {
			output = position;
			continue;
		}

		if (!(dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE))
			continue;

		cedrus_fill_ref_pic(ctx, cedrus_buffer_ref,
				    dpb->top_field_order_cnt,
				    dpb->bottom_field_order_cnt,
				    &pic_list[position]);
	}

	if (output >= 0)
		position = output;
	else
		position = find_first_zero_bit(&used_dpbs, CEDRUS_DEC_H264_FRAME_NUM);

	h264_buffer_picture->position = position;

	/*
	 * FIXME: This should be done when allocating buffers, using values from
	 * controls provided after selecting the format.
	 */
	if (!h264_buffer_picture->mv_col_buf_size) {
		const struct v4l2_ctrl_h264_sps *sps = h264_job->sps;
		unsigned int field_size;

		field_size = DIV_ROUND_UP(pix_format->width, 16) *
			DIV_ROUND_UP(pix_format->height, 16) * 16;
		if (!(sps->flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE))
			field_size = field_size * 2;
		if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY))
			field_size = field_size * 2;

		h264_buffer_picture->mv_col_buf_size = field_size * 2;
		/* Buffer is never accessed by CPU, so we can skip kernel mapping. */
		h264_buffer_picture->mv_col_buf =
			dma_alloc_attrs(dev->dev,
					h264_buffer_picture->mv_col_buf_size,
					&h264_buffer_picture->mv_col_buf_dma,
					GFP_KERNEL, DMA_ATTR_NO_KERNEL_MAPPING);

		if (!h264_buffer_picture->mv_col_buf) {
			h264_buffer_picture->mv_col_buf_size = 0;
			return -ENOMEM;
		}
	}

	if (decode->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC)
		h264_buffer_picture->pic_type = CEDRUS_DEC_H264_PIC_TYPE_FIELD;
	else if (sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD)
		h264_buffer_picture->pic_type = CEDRUS_DEC_H264_PIC_TYPE_MBAFF;
	else
		h264_buffer_picture->pic_type = CEDRUS_DEC_H264_PIC_TYPE_FRAME;

	cedrus_fill_ref_pic(ctx, cedrus_buffer_picture,
			    decode->top_field_order_cnt,
			    decode->bottom_field_order_cnt,
			    &pic_list[position]);

	cedrus_h264_write_sram(ctx, CEDRUS_DEC_H264_SRAM_FRAMEBUFFER_LIST,
			       pic_list, sizeof(pic_list));

	cedrus_write(dev, VE_H264_OUTPUT_FRAME_IDX, position);

	return 0;
}

static void _cedrus_write_ref_list(struct cedrus_context *ctx,
				   const struct v4l2_h264_reference *ref_list,
				   u8 num_ref, unsigned int sram)
{
	struct cedrus_dec_h264_job *h264_job = ctx->engine_job;
	const struct v4l2_ctrl_h264_decode_params *decode =
		h264_job->decode_params;
	u8 sram_array[CEDRUS_DEC_H264_MAX_REF_IDX];
	unsigned int i;
	size_t size;

	memset(sram_array, 0, sizeof(sram_array));

	for (i = 0; i < num_ref; i++) {
		const struct cedrus_buffer *cedrus_buffer_ref;
		struct cedrus_dec_h264_buffer *h264_buffer_ref;
		const struct v4l2_h264_dpb_entry *dpb;
		u8 dpb_idx;

		dpb_idx = ref_list[i].index;
		dpb = &decode->dpb[dpb_idx];

		if (!(dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE))
			continue;

		cedrus_buffer_ref =
			cedrus_buffer_picture_find(ctx, dpb->reference_ts);
		if (!cedrus_buffer_ref)
			continue;

		h264_buffer_ref = cedrus_buffer_ref->engine_buffer;

		sram_array[i] |= h264_buffer_ref->position << 1;
		if (ref_list[i].fields == V4L2_H264_BOTTOM_FIELD_REF)
			sram_array[i] |= BIT(0);
	}

	size = min_t(size_t, ALIGN(num_ref, 4), sizeof(sram_array));
	cedrus_h264_write_sram(ctx, sram, &sram_array, size);
}

static void cedrus_write_ref_list0(struct cedrus_context *ctx)
{
	struct cedrus_dec_h264_job *h264_job = ctx->engine_job;
	const struct v4l2_ctrl_h264_slice_params *slice =
		h264_job->slice_params;

	_cedrus_write_ref_list(ctx, slice->ref_pic_list0,
			       slice->num_ref_idx_l0_active_minus1 + 1,
			       CEDRUS_DEC_H264_SRAM_REF_LIST_0);
}

static void cedrus_write_ref_list1(struct cedrus_context *ctx)
{
	struct cedrus_dec_h264_job *h264_job = ctx->engine_job;
	const struct v4l2_ctrl_h264_slice_params *slice =
		h264_job->slice_params;

	_cedrus_write_ref_list(ctx, slice->ref_pic_list1,
			       slice->num_ref_idx_l1_active_minus1 + 1,
			       CEDRUS_DEC_H264_SRAM_REF_LIST_1);
}

static void cedrus_write_scaling_lists(struct cedrus_context *ctx)
{
	struct cedrus_dec_h264_job *h264_job = ctx->engine_job;
	const struct v4l2_ctrl_h264_scaling_matrix *scaling_matrix =
		h264_job->scaling_matrix;
	const struct v4l2_ctrl_h264_pps *pps = h264_job->pps;

	if (!(pps->flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT))
		return;

	cedrus_h264_write_sram(ctx, CEDRUS_DEC_H264_SRAM_SCALING_LIST_8x8_0,
			       scaling_matrix->scaling_list_8x8[0],
			       sizeof(scaling_matrix->scaling_list_8x8[0]));

	cedrus_h264_write_sram(ctx, CEDRUS_DEC_H264_SRAM_SCALING_LIST_8x8_1,
			       scaling_matrix->scaling_list_8x8[1],
			       sizeof(scaling_matrix->scaling_list_8x8[1]));

	cedrus_h264_write_sram(ctx, CEDRUS_DEC_H264_SRAM_SCALING_LIST_4x4,
			       scaling_matrix->scaling_list_4x4,
			       sizeof(scaling_matrix->scaling_list_4x4));
}

static void cedrus_write_pred_weight_table(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	struct cedrus_dec_h264_job *h264_job = ctx->engine_job;
	const struct v4l2_ctrl_h264_pred_weights *pred_weights =
		h264_job->pred_weights;
	int i, j, k;

	cedrus_write(dev, VE_H264_SHS_WP,
		     ((pred_weights->chroma_log2_weight_denom & 0x7) << 4) |
		     ((pred_weights->luma_log2_weight_denom & 0x7) << 0));

	cedrus_write(dev, VE_AVC_SRAM_PORT_OFFSET,
		     CEDRUS_DEC_H264_SRAM_PRED_WEIGHT_TABLE << 2);

	for (i = 0; i < ARRAY_SIZE(pred_weights->weight_factors); i++) {
		const struct v4l2_h264_weight_factors *factors =
			&pred_weights->weight_factors[i];

		for (j = 0; j < ARRAY_SIZE(factors->luma_weight); j++) {
			u32 val;

			val = (((u32)factors->luma_offset[j] & 0x1ff) << 16) |
				(factors->luma_weight[j] & 0x1ff);
			cedrus_write(dev, VE_AVC_SRAM_PORT_DATA, val);
		}

		for (j = 0; j < ARRAY_SIZE(factors->chroma_weight); j++) {
			for (k = 0; k < ARRAY_SIZE(factors->chroma_weight[0]); k++) {
				u32 val;

				val = (((u32)factors->chroma_offset[j][k] & 0x1ff) << 16) |
					(factors->chroma_weight[j][k] & 0x1ff);
				cedrus_write(dev, VE_AVC_SRAM_PORT_DATA, val);
			}
		}
	}
}

/*
 * It turns out that using VE_H264_VLD_OFFSET to skip bits is not reliable. In
 * rare cases frame is not decoded correctly. However, setting offset to 0 and
 * skipping appropriate amount of bits with flush bits trigger always works.
 */
static void cedrus_skip_bits(struct cedrus_device *dev, int num)
{
	int count = 0;

	while (count < num) {
		int tmp = min(num - count, 32);

		cedrus_write(dev, VE_H264_TRIGGER_TYPE,
			     VE_H264_TRIGGER_TYPE_FLUSH_BITS |
			     VE_H264_TRIGGER_TYPE_N_BITS(tmp));
		/* XXX: use poll helper */
		while (cedrus_read(dev, VE_H264_STATUS) & VE_H264_STATUS_VLD_BUSY)
			udelay(1);

		count += tmp;
	}
}

static void cedrus_set_params(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	struct cedrus_dec_h264_context *h264_ctx = ctx->engine_ctx;
	struct cedrus_dec_h264_job *h264_job = ctx->engine_job;
	const struct v4l2_ctrl_h264_decode_params *decode =
		h264_job->decode_params;
	const struct v4l2_ctrl_h264_slice_params *slice =
		h264_job->slice_params;
	const struct v4l2_ctrl_h264_pps *pps = h264_job->pps;
	const struct v4l2_ctrl_h264_sps *sps = h264_job->sps;
	struct v4l2_m2m_ctx *m2m_ctx = ctx->v4l2.fh.m2m_ctx;
	struct v4l2_pix_format *pix_format = &ctx->v4l2.format_coded.fmt.pix;
	dma_addr_t coded_addr;
	unsigned int coded_size;
	unsigned int pic_width_in_mbs;
	bool mbaff_pic;
	u32 value;

	cedrus_job_buffer_coded_dma(ctx, &coded_addr, &coded_size);

	cedrus_write(dev, VE_H264_VLD_OFFSET, 0);
	cedrus_write(dev, VE_H264_VLD_LEN, coded_size * 8);

	cedrus_write(dev, VE_H264_VLD_END, coded_addr + coded_size);
	cedrus_write(dev, VE_H264_VLD_ADDR,
		     VE_H264_VLD_ADDR_VAL(coded_addr) |
		     VE_H264_VLD_ADDR_FIRST | VE_H264_VLD_ADDR_VALID |
		     VE_H264_VLD_ADDR_LAST);

	if (pix_format->width > 2048) {
		cedrus_write(dev, VE_BUF_CTRL,
			     VE_BUF_CTRL_INTRAPRED_MIXED_RAM |
			     VE_BUF_CTRL_DBLK_MIXED_RAM);
		cedrus_write(dev, VE_DBLK_DRAM_BUF_ADDR,
			     h264_ctx->deblk_buf_dma);
		cedrus_write(dev, VE_INTRAPRED_DRAM_BUF_ADDR,
			     h264_ctx->intra_pred_buf_dma);
	} else {
		cedrus_write(dev, VE_BUF_CTRL,
			     VE_BUF_CTRL_INTRAPRED_INT_SRAM |
			     VE_BUF_CTRL_DBLK_INT_SRAM);
	}

	/*
	 * FIXME: Since the bitstream parsing is done in software, and
	 * in userspace, this shouldn't be needed anymore. But it
	 * turns out that removing it breaks the decoding process,
	 * without any clear indication why.
	 */
	cedrus_write(dev, VE_H264_TRIGGER_TYPE,
		     VE_H264_TRIGGER_TYPE_INIT_SWDEC);

	cedrus_skip_bits(dev, slice->header_bit_size);

	if (V4L2_H264_CTRL_PRED_WEIGHTS_REQUIRED(pps, slice))
		cedrus_write_pred_weight_table(ctx);

	if (slice->slice_type == V4L2_H264_SLICE_TYPE_P ||
	    slice->slice_type == V4L2_H264_SLICE_TYPE_SP ||
	    slice->slice_type == V4L2_H264_SLICE_TYPE_B)
		cedrus_write_ref_list0(ctx);

	if (slice->slice_type == V4L2_H264_SLICE_TYPE_B)
		cedrus_write_ref_list1(ctx);

	// picture parameters
	/*
	 * FIXME: the kernel headers are allowing the default value to
	 * be passed, but the libva doesn't give us that.
	 */
	value = ((slice->num_ref_idx_l0_active_minus1 & 0x1f) << 10) |
		((slice->num_ref_idx_l1_active_minus1 & 0x1f) << 5) |
		((pps->weighted_bipred_idc & 0x3) << 2);

	if (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE)
		value |= VE_H264_PPS_ENTROPY_CODING_MODE;
	if (pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED)
		value |= VE_H264_PPS_WEIGHTED_PRED;
	if (pps->flags & V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED)
		value |= VE_H264_PPS_CONSTRAINED_INTRA_PRED;
	if (pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE)
		value |= VE_H264_PPS_TRANSFORM_8X8_MODE;

	cedrus_write(dev, VE_H264_PPS, value);

	// sequence parameters
	value = ((sps->chroma_format_idc & 0x7) << 19) |
		((sps->pic_width_in_mbs_minus1 & 0xff) << 8) |
		(sps->pic_height_in_map_units_minus1 & 0xff);

	if (sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)
		value |= VE_H264_SPS_MBS_ONLY;
	if (sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD)
		value |= VE_H264_SPS_MB_ADAPTIVE_FRAME_FIELD;
	if (sps->flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE)
		value |= VE_H264_SPS_DIRECT_8X8_INFERENCE;

	cedrus_write(dev, VE_H264_SPS, value);

	mbaff_pic = !(decode->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC) &&
		    (sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD);
	pic_width_in_mbs = sps->pic_width_in_mbs_minus1 + 1;

	// slice parameters
	value = (((slice->first_mb_in_slice % pic_width_in_mbs) & 0xff) << 24) |
		((((slice->first_mb_in_slice / pic_width_in_mbs) *
		   (mbaff_pic + 1)) & 0xff) << 16) |
		((slice->slice_type & 0xf) << 8) |
		(slice->cabac_init_idc & 0x3);

	if (decode->nal_ref_idc)
		value |= BIT(12);
	if (m2m_ctx->new_frame)
		value |= VE_H264_SHS_FIRST_SLICE_IN_PIC;
	if (decode->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC)
		value |= VE_H264_SHS_FIELD_PIC;
	if (decode->flags & V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD)
		value |= VE_H264_SHS_BOTTOM_FIELD;
	if (slice->flags & V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED)
		value |= VE_H264_SHS_DIRECT_SPATIAL_MV_PRED;

	cedrus_write(dev, VE_H264_SHS, value);

	value = VE_H264_SHS2_NUM_REF_IDX_ACTIVE_OVRD |
		((slice->num_ref_idx_l0_active_minus1 & 0x1f) << 24) |
		((slice->num_ref_idx_l1_active_minus1 & 0x1f) << 16) |
		((slice->disable_deblocking_filter_idc & 0x3) << 8) |
		((slice->slice_alpha_c0_offset_div2 & 0xf) << 4) |
		(slice->slice_beta_offset_div2 & 0xf);

	cedrus_write(dev, VE_H264_SHS2, value);

	value = ((pps->second_chroma_qp_index_offset & 0x3f) << 16) |
		((pps->chroma_qp_index_offset & 0x3f) << 8) |
		((pps->pic_init_qp_minus26 + 26 + slice->slice_qp_delta) & 0x3f);

	if (!(pps->flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT))
		value |= VE_H264_SHS_QP_SCALING_MATRIX_DEFAULT;

	cedrus_write(dev, VE_H264_SHS_QP, value);

	// clear status flags
	/* XXX: maybe reuse irq clear function */
	value = cedrus_read(dev, VE_H264_STATUS);
	cedrus_write(dev, VE_H264_STATUS, value);

	// enable int
	/* XXX: Add H264 enable bit (0 value) */
	cedrus_write(dev, VE_H264_CTRL,
		     VE_H264_CTRL_SLICE_DECODE_INT |
		     VE_H264_CTRL_DECODE_ERR_INT |
		     VE_H264_CTRL_VLD_DATA_REQ_INT);
}

static int cedrus_dec_h264_job_configure(struct cedrus_context *cedrus_ctx)
{
	struct cedrus_device *dev = cedrus_ctx->proc->dev;
	struct cedrus_dec_h264_context *h264_ctx = cedrus_ctx->engine_ctx;
	int ret;

	cedrus_write(dev, VE_H264_SDROT_CTRL, 0);
	cedrus_write(dev, VE_H264_EXTRA_BUFFER1,
		     h264_ctx->pic_info_buf_dma);
	cedrus_write(dev, VE_H264_EXTRA_BUFFER2,
		     h264_ctx->neighbor_info_buf_dma);

	cedrus_write_scaling_lists(cedrus_ctx);
	ret = cedrus_write_frame_list(cedrus_ctx);
	if (ret)
		return ret;

	cedrus_set_params(cedrus_ctx);

	return 0;
}

static void cedrus_dec_h264_job_trigger(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;

	cedrus_write(dev, VE_H264_TRIGGER_TYPE,
		     VE_H264_TRIGGER_TYPE_AVC_SLICE_DECODE);
}

/* IRQ */

static int cedrus_dec_h264_irq_status(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	u32 status;

	status = cedrus_read(dev, VE_H264_STATUS);
	status &= VE_H264_STATUS_INT_MASK;

	if (!status)
		return CEDRUS_IRQ_NONE;

	if  (!(status & VE_H264_CTRL_SLICE_DECODE_INT) ||
	     status & VE_H264_STATUS_VLD_DATA_REQ_INT ||
	     status & VE_H264_STATUS_DECODE_ERR_INT)
		return CEDRUS_IRQ_ERROR;

	return CEDRUS_IRQ_SUCCESS;
}

static void cedrus_dec_h264_irq_clear(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;

	cedrus_write(dev, VE_H264_STATUS, VE_H264_STATUS_INT_MASK);
}

static void cedrus_dec_h264_irq_disable(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	u32 value;

	value = cedrus_read(dev, VE_H264_CTRL);
	value &= ~VE_H264_CTRL_INT_MASK;

	cedrus_write(dev, VE_H264_CTRL, value);
}

/* Engine */

static const struct cedrus_engine_ops cedrus_dec_h264_ops = {
	.format_prepare		= cedrus_dec_format_coded_prepare,
	.format_configure	= cedrus_dec_format_coded_configure,

	.setup			= cedrus_dec_h264_setup,
	.cleanup		= cedrus_dec_h264_cleanup,

	.buffer_cleanup		= cedrus_dec_h264_buffer_cleanup,

	.job_prepare		= cedrus_dec_h264_job_prepare,
	.job_configure		= cedrus_dec_h264_job_configure,
	.job_trigger		= cedrus_dec_h264_job_trigger,

	.irq_status		= cedrus_dec_h264_irq_status,
	.irq_clear		= cedrus_dec_h264_irq_clear,
	.irq_disable		= cedrus_dec_h264_irq_disable,
};

static const struct v4l2_ctrl_config cedrus_dec_h264_ctrl_configs[] = {
	{
		.id	= V4L2_CID_STATELESS_H264_SPS,
	},
	{
		.id	= V4L2_CID_STATELESS_H264_PPS,
	},
	{
		.id	= V4L2_CID_STATELESS_H264_SCALING_MATRIX,
	},
	{
		.id	= V4L2_CID_STATELESS_H264_SLICE_PARAMS,
	},
	{
		.id	= V4L2_CID_STATELESS_H264_PRED_WEIGHTS,
	},
	{
		.id	= V4L2_CID_STATELESS_H264_DECODE_PARAMS,
	},
	/*
	 * We only expose supported profiles information,
	 * and not levels as it's not clear what is supported
	 * for each hardware/core version.
	 */
	{
		.id	= V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.min	= V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
		.def	= V4L2_MPEG_VIDEO_H264_PROFILE_MAIN,
		.max	= V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
		.menu_skip_mask =
			BIT(V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED),
	},
	{
		.id	= V4L2_CID_STATELESS_H264_DECODE_MODE,
		.max	= V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
		.def	= V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
	},
	{
		.id	= V4L2_CID_STATELESS_H264_START_CODE,
		.max	= V4L2_STATELESS_H264_START_CODE_NONE,
		.def	= V4L2_STATELESS_H264_START_CODE_NONE,
	},
};

static const struct v4l2_frmsize_stepwise cedrus_dec_h264_frmsize = {
	.min_width	= 16,
	.max_width	= 3840,
	.step_width	= 16,

	.min_height	= 16,
	.max_height	= 3840,
	.step_height	= 16,
};

const struct cedrus_engine cedrus_dec_h264 = {
	.codec			= CEDRUS_CODEC_H264,
	.role			= CEDRUS_ROLE_DECODER,
	.capabilities		= CEDRUS_CAPABILITY_H264_DEC,

	.ops			= &cedrus_dec_h264_ops,

	.pixelformat		= V4L2_PIX_FMT_H264_SLICE,
	.slice_based		= true,
	.ctrl_configs		= cedrus_dec_h264_ctrl_configs,
	.ctrl_configs_count	= ARRAY_SIZE(cedrus_dec_h264_ctrl_configs),
	.frmsize		= &cedrus_dec_h264_frmsize,

	.ctx_size		= sizeof(struct cedrus_dec_h264_context),
	.job_size		= sizeof(struct cedrus_dec_h264_job),
	.buffer_size		= sizeof(struct cedrus_dec_h264_buffer),
};
