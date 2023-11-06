// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2013 Jens Kuske <jenskuske@gmail.com>
 * Copyright 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright 2018-2023 Bootlin
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
#include "cedrus_dec_h265.h"
#include "cedrus_engine.h"
#include "cedrus_proc.h"
#include "cedrus_regs.h"

/* Helpers */

static void cedrus_dec_h265_mv_col_buf_dma(struct cedrus_buffer *cedrus_buffer,
					   dma_addr_t *top_addr,
					   dma_addr_t *bottom_addr)
{
	struct cedrus_dec_h265_buffer *h265_buffer =
		cedrus_buffer->engine_buffer;
	dma_addr_t addr;

	addr = h265_buffer->mv_col_buf_dma;
	*top_addr = addr;

	addr += h265_buffer->mv_col_buf_size / 2;
	*bottom_addr = addr;
}

static void cedrus_dec_h265_sram_offset_write(struct cedrus_device *dev,
					      u32 offset)
{
	cedrus_write(dev, VE_DEC_H265_SRAM_OFFSET, offset);
}

static void cedrus_dec_h265_sram_data_write(struct cedrus_device *dev,
					    void *data, unsigned int size)
{
	u32 *word = data;

	WARN_ON((size % sizeof(u32)) != 0);

	while (size >= sizeof(u32)) {
		cedrus_write(dev, VE_DEC_H265_SRAM_DATA, *word++);
		size -= sizeof(u32);
	}
}

static void cedrus_dec_h265_bits_skip(struct cedrus_device *dev,
				      unsigned int count)
{
	unsigned int written = 0;
	int ret;

	while (written < count) {
		unsigned int skip_count =
			min_t(unsigned int, count - written, 32);

		cedrus_write(dev, VE_DEC_H265_TRIGGER,
			     VE_DEC_H265_TRIGGER_FLUSH_BITS |
			     VE_DEC_H265_TRIGGER_TYPE_N_BITS(skip_count));

		ret = cedrus_poll_cleared(dev, VE_DEC_H265_STATUS,
					  VE_DEC_H265_STATUS_VLD_BUSY);
		if (ret)
			dev_err_ratelimited(dev->dev,
					    "timed out waiting to skip bits\n");

		written += skip_count;
	}
}

static u32 cedrus_dec_h265_bits_read(struct cedrus_device *dev,
				     unsigned int count)
{
	cedrus_write(dev, VE_DEC_H265_TRIGGER,
		     VE_DEC_H265_TRIGGER_SHOW_BITS |
		     VE_DEC_H265_TRIGGER_TYPE_N_BITS(count));

	/* XXX: check return code. */
	cedrus_poll_cleared(dev, VE_DEC_H265_STATUS,
			    VE_DEC_H265_STATUS_VLD_BUSY);

	return cedrus_read(dev, VE_DEC_H265_BITS_READ);
}

/* Context */

static int cedrus_dec_h265_setup(struct cedrus_context *cedrus_ctx)
{
	struct device *dev = cedrus_ctx->proc->dev->dev;
	struct cedrus_dec_h265_context *h265_ctx = cedrus_ctx->engine_ctx;
	int ret;

	/* Buffer is never accessed by CPU, so we can skip kernel mapping. */
	h265_ctx->neighbor_info_buf =
		dma_alloc_attrs(dev, CEDRUS_DEC_H265_NEIGHBOR_INFO_BUF_SIZE,
				&h265_ctx->neighbor_info_buf_addr, GFP_KERNEL,
				DMA_ATTR_NO_KERNEL_MAPPING);
	if (!h265_ctx->neighbor_info_buf)
		return -ENOMEM;

	/*
	 * FIXME: This might be faster with a cache-enabled allocation and
	 * explicit sync.
	 */
	h265_ctx->entry_points_buf =
		dma_alloc_coherent(dev, CEDRUS_DEC_H265_ENTRY_POINTS_BUF_SIZE,
				   &h265_ctx->entry_points_buf_addr,
				   GFP_KERNEL);
	if (!h265_ctx->entry_points_buf) {
		ret = -ENOMEM;
		goto error_neighbor_info_buf;
	}

	return 0;

error_neighbor_info_buf:
	dma_free_attrs(dev, CEDRUS_DEC_H265_NEIGHBOR_INFO_BUF_SIZE,
		       h265_ctx->neighbor_info_buf,
		       h265_ctx->neighbor_info_buf_addr,
		       DMA_ATTR_NO_KERNEL_MAPPING);

	return ret;

}

static void cedrus_dec_h265_cleanup(struct cedrus_context *cedrus_ctx)
{
	struct device *dev = cedrus_ctx->proc->dev->dev;
	struct cedrus_dec_h265_context *h265_ctx = cedrus_ctx->engine_ctx;

	dma_free_attrs(dev, CEDRUS_DEC_H265_NEIGHBOR_INFO_BUF_SIZE,
		       h265_ctx->neighbor_info_buf,
		       h265_ctx->neighbor_info_buf_addr,
		       DMA_ATTR_NO_KERNEL_MAPPING);

	dma_free_coherent(dev, CEDRUS_DEC_H265_ENTRY_POINTS_BUF_SIZE,
			  h265_ctx->entry_points_buf,
			  h265_ctx->entry_points_buf_addr);
}

/* Buffer */

static void cedrus_dec_h265_buffer_cleanup(struct cedrus_context *cedrus_ctx,
					   struct cedrus_buffer *cedrus_buffer)
{
	struct device *dev = cedrus_ctx->proc->dev->dev;
	struct cedrus_dec_h265_buffer *h265_buffer =
		cedrus_buffer->engine_buffer;

	if (h265_buffer->mv_col_buf_size) {
		dma_free_attrs(dev, h265_buffer->mv_col_buf_size,
			       h265_buffer->mv_col_buf,
			       h265_buffer->mv_col_buf_dma,
			       DMA_ATTR_NO_KERNEL_MAPPING);

		h265_buffer->mv_col_buf_size = 0;
	}
}

/* Job */

static int cedrus_dec_h265_job_prepare(struct cedrus_context *ctx)
{
	struct cedrus_dec_h265_job *job = ctx->engine_job;
	u32 id;

	id = V4L2_CID_STATELESS_HEVC_SPS;
	job->sps = cedrus_context_ctrl_data(ctx, id);

	id = V4L2_CID_STATELESS_HEVC_PPS;
	job->pps = cedrus_context_ctrl_data(ctx, id);

	id = V4L2_CID_STATELESS_HEVC_SCALING_MATRIX;
	job->scaling_matrix = cedrus_context_ctrl_data(ctx, id);

	id = V4L2_CID_STATELESS_HEVC_SLICE_PARAMS;
	job->slice_params = cedrus_context_ctrl_data(ctx, id);

	id = V4L2_CID_STATELESS_HEVC_ENTRY_POINT_OFFSETS;
	job->entry_point_offsets = cedrus_context_ctrl_data(ctx, id);
	job->entry_point_offsets_count =
		cedrus_context_ctrl_array_count(ctx, id);

	id = V4L2_CID_STATELESS_HEVC_DECODE_PARAMS;
	job->decode_params = cedrus_context_ctrl_data(ctx, id);

	return 0;
}

static void
cedrus_dec_h265_frame_info_write_single(struct cedrus_context *ctx,
					struct cedrus_buffer *buffer,
					unsigned int index, bool field_pic,
					u32 top_pic_order_cnt,
					u32 bottom_pic_order_cnt)
{
	struct cedrus_device *dev = ctx->proc->dev;
	struct cedrus_dec_h265_sram_frame_info frame_info = { 0 };
	dma_addr_t luma_addr, chroma_addr;
	dma_addr_t mv_col_buf_top_addr, mv_col_buf_bottom_addr;
	u32 sram_offset;

	cedrus_buffer_picture_dma(ctx, buffer, &luma_addr, &chroma_addr);

	luma_addr = VE_DEC_H265_SRAM_DATA_ADDR_BASE(luma_addr);
	chroma_addr = VE_DEC_H265_SRAM_DATA_ADDR_BASE(chroma_addr);

	cedrus_dec_h265_mv_col_buf_dma(buffer, &mv_col_buf_top_addr,
				       &mv_col_buf_bottom_addr);

	mv_col_buf_top_addr =
		VE_DEC_H265_SRAM_DATA_ADDR_BASE(mv_col_buf_top_addr);
	mv_col_buf_bottom_addr =
		VE_DEC_H265_SRAM_DATA_ADDR_BASE(mv_col_buf_bottom_addr);

	frame_info.luma_addr = cpu_to_le32(luma_addr);
	frame_info.chroma_addr = cpu_to_le32(chroma_addr);
	frame_info.top_pic_order_cnt = cpu_to_le32(top_pic_order_cnt);
	frame_info.top_mv_col_buf_addr = cpu_to_le32(mv_col_buf_top_addr);

	if (field_pic) {
		frame_info.bottom_pic_order_cnt =
			cpu_to_le32(bottom_pic_order_cnt);
		frame_info.bottom_mv_col_buf_addr =
			cpu_to_le32(mv_col_buf_bottom_addr);
	} else {
		frame_info.bottom_pic_order_cnt =
			cpu_to_le32(top_pic_order_cnt);
		frame_info.bottom_mv_col_buf_addr =
			cpu_to_le32(mv_col_buf_top_addr);
	}

	sram_offset = VE_DEC_H265_SRAM_OFFSET_FRAME_INFO +
		      VE_DEC_H265_SRAM_OFFSET_FRAME_INFO_UNIT * index;

	cedrus_dec_h265_sram_offset_write(dev, sram_offset);
	cedrus_dec_h265_sram_data_write(dev, &frame_info, sizeof(frame_info));
}

static void
cedrus_dec_h265_frame_info_write_dpb(struct cedrus_context *ctx,
				     const struct v4l2_hevc_dpb_entry *dpb,
				     u8 num_active_dpb_entries)
{
	unsigned int i;

	for (i = 0; i < num_active_dpb_entries; i++) {
		struct cedrus_buffer *buffer =
			cedrus_buffer_picture_find(ctx, dpb[i].timestamp);

		if (WARN_ON(!buffer))
			continue;

		cedrus_dec_h265_frame_info_write_single(ctx, buffer, i,
							dpb[i].field_pic,
							dpb[i].pic_order_cnt_val,
							dpb[i].pic_order_cnt_val);
	}
}

static void
cedrus_dec_h265_ref_pic_list_write(struct cedrus_device *dev,
				   const struct v4l2_hevc_dpb_entry *dpb,
				   const u8 list[], u8 num_ref_idx_active,
				   u32 sram_offset)
{
	unsigned int i;
	u32 word = 0;

	cedrus_dec_h265_sram_offset_write(dev, sram_offset);

	for (i = 0; i < num_ref_idx_active; i++) {
		unsigned int shift = (i % 4) * 8;
		unsigned int index = list[i];
		u8 value = list[i];

		if (dpb[index].flags & V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE)
			value |= VE_DEC_H265_SRAM_REF_PIC_LIST_LT_REF;

		/* Each SRAM word gathers up to 4 references. */
		word |= value << shift;

		/* Write the word to SRAM and clear it for the next batch. */
		if ((i % 4) == 3 || i == (num_ref_idx_active - 1)) {
			cedrus_dec_h265_sram_data_write(dev, &word,
							sizeof(word));
			word = 0;
		}
	}
}

static void cedrus_dec_h265_pred_weight_write(struct cedrus_device *dev,
					      const s8 delta_luma_weight[],
					      const s8 luma_offset[],
					      const s8 delta_chroma_weight[][2],
					      const s8 chroma_offset[][2],
					      u8 num_ref_idx_active,
					      u32 sram_luma_offset,
					      u32 sram_chroma_offset)
{
	struct cedrus_dec_h265_sram_pred_weight pred_weight[2];
	unsigned int i, j;

	memset(pred_weight, 0, sizeof(pred_weight));

	/* Luma prediction weight. */

	cedrus_dec_h265_sram_offset_write(dev, sram_luma_offset);

	for (i = 0; i < num_ref_idx_active; i++) {
		unsigned int index = i % 2;

		pred_weight[index].delta_weight = delta_luma_weight[i];
		pred_weight[index].offset = luma_offset[i];

		if (index == 1 || i == (num_ref_idx_active - 1))
			cedrus_dec_h265_sram_data_write(dev,
							(u32 *)pred_weight,
							sizeof(pred_weight));
	}

	memset(pred_weight, 0, sizeof(pred_weight));

	/* Chroma prediction weight. */

	cedrus_dec_h265_sram_offset_write(dev, sram_chroma_offset);

	for (i = 0; i < num_ref_idx_active; i++) {
		for (j = 0; j < 2; j++) {
			pred_weight[j].delta_weight = delta_chroma_weight[i][j];
			pred_weight[j].offset = chroma_offset[i][j];
		}

		cedrus_dec_h265_sram_data_write(dev, (u32 *)pred_weight,
						sizeof(pred_weight));
	}
}

static void
cedrus_dec_h265_scaling_list_write(struct cedrus_context *cedrus_ctx)
{
	struct cedrus_device *dev = cedrus_ctx->proc->dev;
	struct cedrus_dec_h265_job *h265_job = cedrus_ctx->engine_job;
	const struct v4l2_ctrl_hevc_scaling_matrix *scaling =
		h265_job->scaling_matrix;
	u32 i, j, k, value;

	cedrus_write(dev, VE_DEC_H265_SCALING_LIST_DC_COEF0,
		     (scaling->scaling_list_dc_coef_32x32[1] << 24) |
		     (scaling->scaling_list_dc_coef_32x32[0] << 16) |
		     (scaling->scaling_list_dc_coef_16x16[1] << 8) |
		     (scaling->scaling_list_dc_coef_16x16[0] << 0));

	cedrus_write(dev, VE_DEC_H265_SCALING_LIST_DC_COEF1,
		     (scaling->scaling_list_dc_coef_16x16[5] << 24) |
		     (scaling->scaling_list_dc_coef_16x16[4] << 16) |
		     (scaling->scaling_list_dc_coef_16x16[3] << 8) |
		     (scaling->scaling_list_dc_coef_16x16[2] << 0));

	cedrus_dec_h265_sram_offset_write(dev,
					  VE_DEC_H265_SRAM_OFFSET_SCALING_LISTS);

	for (i = 0; i < 6; i++) {
		for (j = 0; j < 8; j++) {
			for (k = 0; k < 8; k += 4) {
				value = ((u32)scaling->scaling_list_8x8[i][j + (k + 3) * 8] << 24) |
					((u32)scaling->scaling_list_8x8[i][j + (k + 2) * 8] << 16) |
					((u32)scaling->scaling_list_8x8[i][j + (k + 1) * 8] << 8) |
					scaling->scaling_list_8x8[i][j + k * 8];

				cedrus_write(dev, VE_DEC_H265_SRAM_DATA, value);
			}
		}
	}

	for (i = 0; i < 2; i++) {
		for (j = 0; j < 8; j++) {
			for (k = 0; k < 8; k += 4) {
				value = ((u32)scaling->scaling_list_32x32[i][j + (k + 3) * 8] << 24) |
					((u32)scaling->scaling_list_32x32[i][j + (k + 2) * 8] << 16) |
					((u32)scaling->scaling_list_32x32[i][j + (k + 1) * 8] << 8) |
					scaling->scaling_list_32x32[i][j + k * 8];

				cedrus_write(dev, VE_DEC_H265_SRAM_DATA, value);
			}
		}
	}

	for (i = 0; i < 6; i++) {
		for (j = 0; j < 8; j++) {
			for (k = 0; k < 8; k += 4) {
				value = ((u32)scaling->scaling_list_16x16[i][j + (k + 3) * 8] << 24) |
					((u32)scaling->scaling_list_16x16[i][j + (k + 2) * 8] << 16) |
					((u32)scaling->scaling_list_16x16[i][j + (k + 1) * 8] << 8) |
					scaling->scaling_list_16x16[i][j + k * 8];

				cedrus_write(dev, VE_DEC_H265_SRAM_DATA, value);
			}
		}
	}

	for (i = 0; i < 6; i++) {
		for (j = 0; j < 4; j++) {
			value = ((u32)scaling->scaling_list_4x4[i][j + 12] << 24) |
				((u32)scaling->scaling_list_4x4[i][j + 8] << 16) |
				((u32)scaling->scaling_list_4x4[i][j + 4] << 8) |
				scaling->scaling_list_4x4[i][j];

			cedrus_write(dev, VE_DEC_H265_SRAM_DATA, value);
		}
	}
}

static int cedrus_h265_is_low_delay(struct cedrus_dec_h265_job *h265_job)
{
	const struct v4l2_ctrl_hevc_slice_params *slice_params =
		h265_job->slice_params;
	const struct v4l2_ctrl_hevc_decode_params *decode_params =
		h265_job->decode_params;
	const struct v4l2_hevc_dpb_entry *dpb = decode_params->dpb;
	s32 poc = decode_params->pic_order_cnt_val;
	int i;

	for (i = 0; i < slice_params->num_ref_idx_l0_active_minus1 + 1; i++)
		if (dpb[slice_params->ref_idx_l0[i]].pic_order_cnt_val > poc)
			return 1;

	if (slice_params->slice_type != V4L2_HEVC_SLICE_TYPE_B)
		return 0;

	for (i = 0; i < slice_params->num_ref_idx_l1_active_minus1 + 1; i++)
		if (dpb[slice_params->ref_idx_l1[i]].pic_order_cnt_val > poc)
			return 1;

	return 0;
}

static void cedrus_dec_h265_tiles_write(struct cedrus_context *cedrus_ctx,
					unsigned int ctb_addr_x,
					unsigned int ctb_addr_y)
{
	struct cedrus_device *dev = cedrus_ctx->proc->dev;
	struct cedrus_dec_h265_context *h265_ctx = cedrus_ctx->engine_ctx;
	struct cedrus_dec_h265_job *h265_job = cedrus_ctx->engine_job;
	const struct v4l2_ctrl_hevc_slice_params *slice_params =
		h265_job->slice_params;
	const struct v4l2_ctrl_hevc_pps *pps = h265_job->pps;
	const u32 *entry_points = h265_job->entry_point_offsets;
	u32 num_entry_point_offsets = slice_params->num_entry_point_offsets;
	u32 *entry_points_buf = h265_ctx->entry_points_buf;
	int i, x, tx, y, ty;

	for (x = 0, tx = 0; tx < pps->num_tile_columns_minus1 + 1; tx++) {
		if (x + pps->column_width_minus1[tx] + 1 > ctb_addr_x)
			break;

		x += pps->column_width_minus1[tx] + 1;
	}

	for (y = 0, ty = 0; ty < pps->num_tile_rows_minus1 + 1; ty++) {
		if (y + pps->row_height_minus1[ty] + 1 > ctb_addr_y)
			break;

		y += pps->row_height_minus1[ty] + 1;
	}

	cedrus_write(dev, VE_DEC_H265_TILE_START_CTB, (y << 16) | (x << 0));
	cedrus_write(dev, VE_DEC_H265_TILE_END_CTB,
		     ((y + pps->row_height_minus1[ty]) << 16) |
		     ((x + pps->column_width_minus1[tx]) << 0));

	if (pps->flags & V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED) {
		for (i = 0; i < num_entry_point_offsets; i++)
			entry_points_buf[i] = entry_points[i];
	} else {
		for (i = 0; i < num_entry_point_offsets; i++) {
			if (tx + 1 >= pps->num_tile_columns_minus1 + 1) {
				x = 0;
				tx = 0;
				y += pps->row_height_minus1[ty++] + 1;
			} else {
				x += pps->column_width_minus1[tx++] + 1;
			}

			entry_points_buf[i * 4 + 0] = entry_points[i];
			entry_points_buf[i * 4 + 1] = 0x0;
			entry_points_buf[i * 4 + 2] = (y << 16) | (x << 0);
			entry_points_buf[i * 4 + 3] =
				((y + pps->row_height_minus1[ty]) << 16) |
				((x + pps->column_width_minus1[tx]) << 0);
		}
	}
}

static int cedrus_dec_h265_job_configure(struct cedrus_context *cedrus_ctx)
{
	struct cedrus_device *dev = cedrus_ctx->proc->dev;
	struct cedrus_dec_h265_context *h265_ctx = cedrus_ctx->engine_ctx;
	struct cedrus_dec_h265_job *h265_job = cedrus_ctx->engine_job;
	const struct v4l2_ctrl_hevc_sps *sps = h265_job->sps;
	const struct v4l2_ctrl_hevc_pps *pps = h265_job->pps;
	const struct v4l2_ctrl_hevc_slice_params *slice_params =
		h265_job->slice_params;
	const struct v4l2_hevc_pred_weight_table *pred_weight_table =
		&slice_params->pred_weight_table;
	const struct v4l2_ctrl_hevc_decode_params *decode_params =
		h265_job->decode_params;
	const struct v4l2_hevc_dpb_entry *dpb = decode_params->dpb;
	struct v4l2_m2m_ctx *m2m_ctx = cedrus_ctx->v4l2.fh.m2m_ctx;
	struct v4l2_pix_format *pix_format =
		&cedrus_ctx->v4l2.format_coded.fmt.pix;
	unsigned int width_in_ctb_luma, ctb_size_luma;
	unsigned int log2_max_luma_coding_block_size;
	unsigned int ctb_addr_x, ctb_addr_y;
	struct cedrus_buffer *cedrus_buffer_picture;
	struct cedrus_dec_h265_buffer *h265_buffer_picture;
	dma_addr_t coded_addr;
	unsigned int coded_size;
	u32 chroma_log2_weight_denom;
	u32 num_entry_point_offsets;
	u32 output_index;
	bool output_field_pic;
	u8 padding;
	int count;
	u32 value;

	cedrus_buffer_picture = cedrus_job_buffer_picture(cedrus_ctx);
	h265_buffer_picture = cedrus_buffer_picture->engine_buffer;

	/*
	 * If entry points offsets are present, we should get exactly the same
	 * count from the slice params and the controls array.
	 */
	num_entry_point_offsets = slice_params->num_entry_point_offsets;
	if (num_entry_point_offsets &&
	    num_entry_point_offsets != h265_job->entry_point_offsets_count)
		return -ERANGE;

	log2_max_luma_coding_block_size =
		sps->log2_min_luma_coding_block_size_minus3 + 3 +
		sps->log2_diff_max_min_luma_coding_block_size;
	ctb_size_luma = 1UL << log2_max_luma_coding_block_size;
	width_in_ctb_luma =
		DIV_ROUND_UP(sps->pic_width_in_luma_samples, ctb_size_luma);

	/* MV column buffer size and allocation. */
	/*
	 * FIXME: This should be done when allocating buffers, using values from
	 * controls provided after selecting the format.
	 */
	if (!h265_buffer_picture->mv_col_buf_size) {
		/*
		 * Each CTB requires a MV col buffer with a specific unit size.
		 * Since the address is given with missing lsb bits, 1 KiB is
		 * added to each buffer to ensure proper alignment.
		 */
		h265_buffer_picture->mv_col_buf_size =
			DIV_ROUND_UP(pix_format->width, ctb_size_luma) *
			DIV_ROUND_UP(pix_format->height, ctb_size_luma) *
			CEDRUS_DEC_H265_MV_COL_BUF_UNIT_CTB_SIZE + SZ_1K;

		/* Buffer is never accessed by CPU, so we can skip kernel mapping. */
		h265_buffer_picture->mv_col_buf =
			dma_alloc_attrs(dev->dev,
					h265_buffer_picture->mv_col_buf_size,
					&h265_buffer_picture->mv_col_buf_dma,
					GFP_KERNEL, DMA_ATTR_NO_KERNEL_MAPPING);
		if (!h265_buffer_picture->mv_col_buf) {
			h265_buffer_picture->mv_col_buf_size = 0;
			return -ENOMEM;
		}
	}

	cedrus_job_buffer_coded_dma(cedrus_ctx, &coded_addr, &coded_size);

	/* Source offset and length in bits. */

	cedrus_write(dev, VE_DEC_H265_BITS_OFFSET, 0);
	cedrus_write(dev, VE_DEC_H265_BITS_LEN, coded_size * 8);

	/* Source beginning and end addresses. */

	value = VE_DEC_H265_BITS_ADDR_BASE(coded_addr) |
		VE_DEC_H265_BITS_ADDR_VALID_SLICE_DATA |
		VE_DEC_H265_BITS_ADDR_LAST_SLICE_DATA |
		VE_DEC_H265_BITS_ADDR_FIRST_SLICE_DATA;

	cedrus_write(dev, VE_DEC_H265_BITS_ADDR, value);

	value = VE_DEC_H265_BITS_END_ADDR_BASE(coded_addr + coded_size);

	cedrus_write(dev, VE_DEC_H265_BITS_END_ADDR, value);

	/* Coding tree block address. */

	ctb_addr_x = slice_params->slice_segment_addr % width_in_ctb_luma;
	ctb_addr_y = slice_params->slice_segment_addr / width_in_ctb_luma;

	value = VE_DEC_H265_DEC_CTB_ADDR_X(ctb_addr_x) |
		VE_DEC_H265_DEC_CTB_ADDR_Y(ctb_addr_y);

	cedrus_write(dev, VE_DEC_H265_DEC_CTB_ADDR, value);

	if ((pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) ||
	    (pps->flags & V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED)) {
		cedrus_dec_h265_tiles_write(cedrus_ctx, ctb_addr_x, ctb_addr_y);
	} else {
		cedrus_write(dev, VE_DEC_H265_TILE_START_CTB, 0);
		cedrus_write(dev, VE_DEC_H265_TILE_END_CTB, 0);
	}

	/* Clear the number of correctly-decoded coding tree blocks. */
	if (m2m_ctx->new_frame)
		cedrus_write(dev, VE_DEC_H265_DEC_CTB_NUM, 0);

	/* Initialize bitstream access. */
	cedrus_write(dev, VE_DEC_H265_TRIGGER, VE_DEC_H265_TRIGGER_INIT_SWDEC);

	/*
	 * Cedrus expects that bitstream pointer is actually at the end of the
	 * slice header instead of start of slice data. Padding is 8 bits at
	 * most (one bit set to 1 and at most seven bits set to 0), so we have
	 * to inspect only one byte before slice data.
	 */

	if (slice_params->data_byte_offset == 0)
		return -EOPNOTSUPP;

	cedrus_dec_h265_bits_skip(dev,
				  (slice_params->data_byte_offset - 1) * 8);

	padding = cedrus_dec_h265_bits_read(dev, 8);

	/* XXX: rbsp final 1 bit? */
	/* at least one bit must be set in that byte */
	if (padding == 0)
		return -EINVAL;

	for (count = 0; count < 8; count++)
		if (padding & (1 << count))
			break;

	/* Include the one bit. */
	count++;

	cedrus_dec_h265_bits_skip(dev, 8 - count);

	/* Bitstream parameters. */

	value = VE_DEC_H265_DEC_NAL_HDR_NAL_UNIT_TYPE(slice_params->nal_unit_type) |
		VE_DEC_H265_DEC_NAL_HDR_NUH_TEMPORAL_ID_PLUS1(slice_params->nuh_temporal_id_plus1);

	cedrus_write(dev, VE_DEC_H265_DEC_NAL_HDR, value);

	/* SPS. */

	value = VE_DEC_H265_DEC_SPS_HDR_MAX_TRANSFORM_HIERARCHY_DEPTH_INTRA(sps->max_transform_hierarchy_depth_intra) |
		VE_DEC_H265_DEC_SPS_HDR_MAX_TRANSFORM_HIERARCHY_DEPTH_INTER(sps->max_transform_hierarchy_depth_inter) |
		VE_DEC_H265_DEC_SPS_HDR_LOG2_DIFF_MAX_MIN_TRANSFORM_BLOCK_SIZE(sps->log2_diff_max_min_luma_transform_block_size) |
		VE_DEC_H265_DEC_SPS_HDR_LOG2_MIN_TRANSFORM_BLOCK_SIZE_MINUS2(sps->log2_min_luma_transform_block_size_minus2) |
		VE_DEC_H265_DEC_SPS_HDR_LOG2_DIFF_MAX_MIN_LUMA_CODING_BLOCK_SIZE(sps->log2_diff_max_min_luma_coding_block_size) |
		VE_DEC_H265_DEC_SPS_HDR_LOG2_MIN_LUMA_CODING_BLOCK_SIZE_MINUS3(sps->log2_min_luma_coding_block_size_minus3) |
		VE_DEC_H265_DEC_SPS_HDR_BIT_DEPTH_CHROMA_MINUS8(sps->bit_depth_chroma_minus8) |
		VE_DEC_H265_DEC_SPS_HDR_BIT_DEPTH_LUMA_MINUS8(sps->bit_depth_luma_minus8) |
		VE_DEC_H265_DEC_SPS_HDR_CHROMA_FORMAT_IDC(sps->chroma_format_idc);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_SPS_HDR_FLAG_STRONG_INTRA_SMOOTHING_ENABLE,
				  V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED,
				  sps->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_SPS_HDR_FLAG_SPS_TEMPORAL_MVP_ENABLED,
				  V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED,
				  sps->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_SPS_HDR_FLAG_SAMPLE_ADAPTIVE_OFFSET_ENABLED,
				  V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET,
				  sps->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_SPS_HDR_FLAG_AMP_ENABLED,
				  V4L2_HEVC_SPS_FLAG_AMP_ENABLED, sps->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_SPS_HDR_FLAG_SEPARATE_COLOUR_PLANE,
				  V4L2_HEVC_SPS_FLAG_SEPARATE_COLOUR_PLANE,
				  sps->flags);

	cedrus_write(dev, VE_DEC_H265_DEC_SPS_HDR, value);

	value = VE_DEC_H265_DEC_PCM_CTRL_LOG2_DIFF_MAX_MIN_PCM_LUMA_CODING_BLOCK_SIZE(sps->log2_diff_max_min_pcm_luma_coding_block_size) |
		VE_DEC_H265_DEC_PCM_CTRL_LOG2_MIN_PCM_LUMA_CODING_BLOCK_SIZE_MINUS3(sps->log2_min_pcm_luma_coding_block_size_minus3) |
		VE_DEC_H265_DEC_PCM_CTRL_PCM_SAMPLE_BIT_DEPTH_CHROMA_MINUS1(sps->pcm_sample_bit_depth_chroma_minus1) |
		VE_DEC_H265_DEC_PCM_CTRL_PCM_SAMPLE_BIT_DEPTH_LUMA_MINUS1(sps->pcm_sample_bit_depth_luma_minus1);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_PCM_CTRL_FLAG_PCM_ENABLED,
				  V4L2_HEVC_SPS_FLAG_PCM_ENABLED, sps->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_PCM_CTRL_FLAG_PCM_LOOP_FILTER_DISABLED,
				  V4L2_HEVC_SPS_FLAG_PCM_LOOP_FILTER_DISABLED,
				  sps->flags);

	cedrus_write(dev, VE_DEC_H265_DEC_PCM_CTRL, value);

	/* PPS. */

	value = VE_DEC_H265_DEC_PPS_CTRL0_PPS_CR_QP_OFFSET(pps->pps_cr_qp_offset) |
		VE_DEC_H265_DEC_PPS_CTRL0_PPS_CB_QP_OFFSET(pps->pps_cb_qp_offset) |
		VE_DEC_H265_DEC_PPS_CTRL0_INIT_QP_MINUS26(pps->init_qp_minus26) |
		VE_DEC_H265_DEC_PPS_CTRL0_DIFF_CU_QP_DELTA_DEPTH(pps->diff_cu_qp_delta_depth);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_PPS_CTRL0_FLAG_CU_QP_DELTA_ENABLED,
				  V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED,
				  pps->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_PPS_CTRL0_FLAG_TRANSFORM_SKIP_ENABLED,
				  V4L2_HEVC_PPS_FLAG_TRANSFORM_SKIP_ENABLED,
				  pps->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_PPS_CTRL0_FLAG_CONSTRAINED_INTRA_PRED,
				  V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED,
				  pps->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_PPS_CTRL0_FLAG_SIGN_DATA_HIDING_ENABLED,
				  V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED,
				  pps->flags);

	cedrus_write(dev, VE_DEC_H265_DEC_PPS_CTRL0, value);

	value = VE_DEC_H265_DEC_PPS_CTRL1_LOG2_PARALLEL_MERGE_LEVEL_MINUS2(pps->log2_parallel_merge_level_minus2);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_PPS_CTRL1_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED,
				  V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED,
				  pps->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_PPS_CTRL1_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED,
				  V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED,
				  pps->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_PPS_CTRL1_FLAG_ENTROPY_CODING_SYNC_ENABLED,
				  V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED,
				  pps->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_PPS_CTRL1_FLAG_TILES_ENABLED,
				  V4L2_HEVC_PPS_FLAG_TILES_ENABLED,
				  pps->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_PPS_CTRL1_FLAG_TRANSQUANT_BYPASS_ENABLED,
				  V4L2_HEVC_PPS_FLAG_TRANSQUANT_BYPASS_ENABLED,
				  pps->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_PPS_CTRL1_FLAG_WEIGHTED_BIPRED,
				  V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED, pps->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_PPS_CTRL1_FLAG_WEIGHTED_PRED,
				  V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED, pps->flags);

	cedrus_write(dev, VE_DEC_H265_DEC_PPS_CTRL1, value);

	/* Slice Parameters. */

	value = VE_DEC_H265_DEC_SLICE_HDR_INFO0_PICTURE_TYPE(slice_params->pic_struct) |
	      VE_DEC_H265_DEC_SLICE_HDR_INFO0_FIVE_MINUS_MAX_NUM_MERGE_CAND(slice_params->five_minus_max_num_merge_cand) |
	      VE_DEC_H265_DEC_SLICE_HDR_INFO0_NUM_REF_IDX_L1_ACTIVE_MINUS1(slice_params->num_ref_idx_l1_active_minus1) |
	      VE_DEC_H265_DEC_SLICE_HDR_INFO0_NUM_REF_IDX_L0_ACTIVE_MINUS1(slice_params->num_ref_idx_l0_active_minus1) |
	      VE_DEC_H265_DEC_SLICE_HDR_INFO0_COLLOCATED_REF_IDX(slice_params->collocated_ref_idx) |
	      VE_DEC_H265_DEC_SLICE_HDR_INFO0_COLOUR_PLANE_ID(slice_params->colour_plane_id) |
	      VE_DEC_H265_DEC_SLICE_HDR_INFO0_SLICE_TYPE(slice_params->slice_type);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_SLICE_HDR_INFO0_FLAG_COLLOCATED_FROM_L0,
				  V4L2_HEVC_SLICE_PARAMS_FLAG_COLLOCATED_FROM_L0,
				  slice_params->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_SLICE_HDR_INFO0_FLAG_CABAC_INIT,
				  V4L2_HEVC_SLICE_PARAMS_FLAG_CABAC_INIT,
				  slice_params->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_SLICE_HDR_INFO0_FLAG_MVD_L1_ZERO,
				  V4L2_HEVC_SLICE_PARAMS_FLAG_MVD_L1_ZERO,
				  slice_params->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_SLICE_HDR_INFO0_FLAG_SLICE_SAO_CHROMA,
				  V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_CHROMA,
				  slice_params->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_SLICE_HDR_INFO0_FLAG_SLICE_SAO_LUMA,
				  V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA,
				  slice_params->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_SLICE_HDR_INFO0_FLAG_SLICE_TEMPORAL_MVP_ENABLE,
				  V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED,
				  slice_params->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_SLICE_HDR_INFO0_FLAG_DEPENDENT_SLICE_SEGMENT,
				  V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT,
				  slice_params->flags);

	if (m2m_ctx->new_frame)
		value |= VE_DEC_H265_DEC_SLICE_HDR_INFO0_FLAG_FIRST_SLICE_SEGMENT_IN_PIC;

	cedrus_write(dev, VE_DEC_H265_DEC_SLICE_HDR_INFO0, value);

	value = VE_DEC_H265_DEC_SLICE_HDR_INFO1_SLICE_TC_OFFSET_DIV2(slice_params->slice_tc_offset_div2) |
		VE_DEC_H265_DEC_SLICE_HDR_INFO1_SLICE_BETA_OFFSET_DIV2(slice_params->slice_beta_offset_div2) |
		VE_DEC_H265_DEC_SLICE_HDR_INFO1_SLICE_CR_QP_OFFSET(slice_params->slice_cr_qp_offset) |
		VE_DEC_H265_DEC_SLICE_HDR_INFO1_SLICE_CB_QP_OFFSET(slice_params->slice_cb_qp_offset) |
		VE_DEC_H265_DEC_SLICE_HDR_INFO1_SLICE_QP_DELTA(slice_params->slice_qp_delta);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_SLICE_HDR_INFO1_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED,
				  V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED,
				  slice_params->flags);

	value |= VE_DEC_H265_FLAG(VE_DEC_H265_DEC_SLICE_HDR_INFO1_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED,
				  V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED,
				  slice_params->flags);

	if (slice_params->slice_type != V4L2_HEVC_SLICE_TYPE_I &&
	    !cedrus_h265_is_low_delay(h265_job))
		value |= VE_DEC_H265_DEC_SLICE_HDR_INFO1_FLAG_SLICE_NOT_LOW_DELAY;

	cedrus_write(dev, VE_DEC_H265_DEC_SLICE_HDR_INFO1, value);

	chroma_log2_weight_denom = pred_weight_table->luma_log2_weight_denom +
				   pred_weight_table->delta_chroma_log2_weight_denom;

	value = VE_DEC_H265_DEC_SLICE_HDR_INFO2_NUM_ENTRY_POINT_OFFSETS(num_entry_point_offsets) |
		VE_DEC_H265_DEC_SLICE_HDR_INFO2_CHROMA_LOG2_WEIGHT_DENOM(chroma_log2_weight_denom) |
		VE_DEC_H265_DEC_SLICE_HDR_INFO2_LUMA_LOG2_WEIGHT_DENOM(pred_weight_table->luma_log2_weight_denom);

	cedrus_write(dev, VE_DEC_H265_DEC_SLICE_HDR_INFO2, value);

	value = VE_DEC_H265_ENTRY_POINT_OFFSET_ADDR_BASE(h265_ctx->entry_points_buf_addr);
	cedrus_write(dev, VE_DEC_H265_ENTRY_POINT_OFFSET_ADDR, value);

	/* Decoded picture size. */

	/* XXX: maybe use destination size here. */
	value = VE_DEC_H265_DEC_PIC_SIZE_WIDTH(pix_format->width) |
		VE_DEC_H265_DEC_PIC_SIZE_HEIGHT(pix_format->height);

	cedrus_write(dev, VE_DEC_H265_DEC_PIC_SIZE, value);

	/* Scaling list. */

	if (sps->flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED) {
		cedrus_dec_h265_scaling_list_write(cedrus_ctx);
		value = VE_DEC_H265_SCALING_LIST_CTRL0_FLAG_ENABLED;
	} else {
		value = VE_DEC_H265_SCALING_LIST_CTRL0_DEFAULT;
	}
	cedrus_write(dev, VE_DEC_H265_SCALING_LIST_CTRL0, value);

	/* Neightbor information address. */
	value = VE_DEC_H265_NEIGHBOR_INFO_ADDR_BASE(h265_ctx->neighbor_info_buf_addr);
	cedrus_write(dev, VE_DEC_H265_NEIGHBOR_INFO_ADDR, value);

	/* Write decoded picture buffer in pic list. */
	cedrus_dec_h265_frame_info_write_dpb(cedrus_ctx, dpb,
					     decode_params->num_active_dpb_entries);

	/* Destination picture. */

	output_index = V4L2_HEVC_DPB_ENTRIES_NUM_MAX;
	output_field_pic = (slice_params->pic_struct != 0) ? true : false;

	cedrus_dec_h265_frame_info_write_single(cedrus_ctx,
						cedrus_buffer_picture,
						output_index, output_field_pic,
						slice_params->slice_pic_order_cnt,
						slice_params->slice_pic_order_cnt);

	cedrus_write(dev, VE_DEC_H265_OUTPUT_FRAME_IDX, output_index);

	/* Reference picture list 0 (for P/B frames). */
	if (slice_params->slice_type != V4L2_HEVC_SLICE_TYPE_I) {
		cedrus_dec_h265_ref_pic_list_write(dev, dpb,
						   slice_params->ref_idx_l0,
						   slice_params->num_ref_idx_l0_active_minus1 + 1,
						   VE_DEC_H265_SRAM_OFFSET_REF_PIC_LIST0);

		if ((pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED) ||
		    (pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED))
			cedrus_dec_h265_pred_weight_write(dev,
							  pred_weight_table->delta_luma_weight_l0,
							  pred_weight_table->luma_offset_l0,
							  pred_weight_table->delta_chroma_weight_l0,
							  pred_weight_table->chroma_offset_l0,
							  slice_params->num_ref_idx_l0_active_minus1 + 1,
							  VE_DEC_H265_SRAM_OFFSET_PRED_WEIGHT_LUMA_L0,
							  VE_DEC_H265_SRAM_OFFSET_PRED_WEIGHT_CHROMA_L0);
	}

	/* Reference picture list 1 (for B frames). */
	if (slice_params->slice_type == V4L2_HEVC_SLICE_TYPE_B) {
		cedrus_dec_h265_ref_pic_list_write(dev, dpb,
						   slice_params->ref_idx_l1,
						   slice_params->num_ref_idx_l1_active_minus1 + 1,
						   VE_DEC_H265_SRAM_OFFSET_REF_PIC_LIST1);

		if (pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED)
			cedrus_dec_h265_pred_weight_write(dev,
							  pred_weight_table->delta_luma_weight_l1,
							  pred_weight_table->luma_offset_l1,
							  pred_weight_table->delta_chroma_weight_l1,
							  pred_weight_table->chroma_offset_l1,
							  slice_params->num_ref_idx_l1_active_minus1 + 1,
							  VE_DEC_H265_SRAM_OFFSET_PRED_WEIGHT_LUMA_L1,
							  VE_DEC_H265_SRAM_OFFSET_PRED_WEIGHT_CHROMA_L1);
	}


	/* Enable relevant interrupts. */
	cedrus_write(dev, VE_DEC_H265_CTRL, VE_DEC_H265_CTRL_IRQ_MASK);

	return 0;
}

static void cedrus_dec_h265_job_trigger(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;

	cedrus_write(dev, VE_DEC_H265_TRIGGER, VE_DEC_H265_TRIGGER_DEC_SLICE);
}

/* IRQ */

static int cedrus_dec_h265_irq_status(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	u32 status;

	status = cedrus_read(dev, VE_DEC_H265_STATUS);
	status &= VE_DEC_H265_STATUS_CHECK_MASK;

	if (!status)
		return CEDRUS_IRQ_NONE;

	if  (!(status & VE_DEC_H265_STATUS_SUCCESS) ||
	     status & VE_DEC_H265_STATUS_CHECK_ERROR)
		return CEDRUS_IRQ_ERROR;

	return CEDRUS_IRQ_SUCCESS;
}

static void cedrus_dec_h265_irq_clear(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;

	cedrus_write(dev, VE_DEC_H265_STATUS, VE_DEC_H265_STATUS_CHECK_MASK);
}

static void cedrus_dec_h265_irq_disable(struct cedrus_context *ctx)
{
	struct cedrus_device *dev = ctx->proc->dev;
	u32 value;

	value = cedrus_read(dev, VE_DEC_H265_CTRL);
	value &= ~VE_DEC_H265_CTRL_IRQ_MASK;

	cedrus_write(dev, VE_DEC_H265_CTRL, value);
}

/* Engine */

static const struct cedrus_engine_ops cedrus_dec_h265_ops = {
	.format_prepare		= cedrus_dec_format_coded_prepare,
	.format_configure	= cedrus_dec_format_coded_configure,

	.setup			= cedrus_dec_h265_setup,
	.cleanup		= cedrus_dec_h265_cleanup,

	.buffer_cleanup		= cedrus_dec_h265_buffer_cleanup,

	.job_prepare		= cedrus_dec_h265_job_prepare,
	.job_configure		= cedrus_dec_h265_job_configure,
	.job_trigger		= cedrus_dec_h265_job_trigger,

	.irq_status		= cedrus_dec_h265_irq_status,
	.irq_clear		= cedrus_dec_h265_irq_clear,
	.irq_disable		= cedrus_dec_h265_irq_disable,
};

static const struct v4l2_ctrl_config cedrus_dec_h265_ctrl_configs[] = {
	{
		.id	= V4L2_CID_STATELESS_HEVC_SPS,
	},
	{
		.id	= V4L2_CID_STATELESS_HEVC_PPS,
	},
	{
		.id	= V4L2_CID_STATELESS_HEVC_SCALING_MATRIX,
	},
	{
		.id	= V4L2_CID_STATELESS_HEVC_SLICE_PARAMS,
		/* The driver can only handle 1 entry per slice for now. */
		.dims	= { 1 },

	},
	{
		.id	= V4L2_CID_STATELESS_HEVC_ENTRY_POINT_OFFSETS,
		/* Maximum 256 entry point offsets per slice. */
		.dims	= { 256 },
		.max	= 0xffffffff,
		.step	= 1,
	},
	{
		.id	= V4L2_CID_STATELESS_HEVC_DECODE_PARAMS,
	},
	{
		.id	= V4L2_CID_STATELESS_HEVC_DECODE_MODE,
		.max	= V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED,
		.def	= V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED,
	},
	{
		.id	= V4L2_CID_STATELESS_HEVC_START_CODE,
		.max	= V4L2_STATELESS_HEVC_START_CODE_NONE,
		.def	= V4L2_STATELESS_HEVC_START_CODE_NONE,
	},
};

static const struct v4l2_frmsize_stepwise cedrus_dec_h265_frmsize = {
	.min_width	= 16,
	.max_width	= 3840,
	.step_width	= 16,

	.min_height	= 16,
	.max_height	= 3840,
	.step_height	= 16,
};

const struct cedrus_engine cedrus_dec_h265 = {
	.codec			= CEDRUS_CODEC_H265,
	.role			= CEDRUS_ROLE_DECODER,
	.capabilities		= CEDRUS_CAPABILITY_H265_DEC,

	.ops			= &cedrus_dec_h265_ops,

	.pixelformat		= V4L2_PIX_FMT_HEVC_SLICE,
	.slice_based		= true,
	.ctrl_configs		= cedrus_dec_h265_ctrl_configs,
	.ctrl_configs_count	= ARRAY_SIZE(cedrus_dec_h265_ctrl_configs),
	.frmsize		= &cedrus_dec_h265_frmsize,

	.ctx_size		= sizeof(struct cedrus_dec_h265_context),
	.job_size		= sizeof(struct cedrus_dec_h265_job),
	.buffer_size		= sizeof(struct cedrus_dec_h265_buffer),
};
