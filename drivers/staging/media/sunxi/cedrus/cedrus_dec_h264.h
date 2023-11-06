/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2018-2023 Bootlin
 * Author: Maxime Ripard <maxime.ripard@bootlin.com>
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _CEDRUS_DEC_H264_H_
#define _CEDRUS_DEC_H264_H_

#include <media/v4l2-ctrls.h>

#define CEDRUS_DEC_H264_MAX_REF_IDX		32
#define CEDRUS_DEC_H264_FRAME_NUM		18

#define CEDRUS_DEC_H264_NEIGHBOR_INFO_BUF_SIZE	(32 * SZ_1K)
#define CEDRUS_DEC_H264_PIC_INFO_BUF_SIZE_MIN	(130 * SZ_1K)

struct cedrus_dec_h264_context {
	void		*pic_info_buf;
	dma_addr_t	pic_info_buf_dma;
	ssize_t		pic_info_buf_size;

	void		*neighbor_info_buf;
	dma_addr_t	neighbor_info_buf_dma;

	void		*deblk_buf;
	dma_addr_t	deblk_buf_dma;
	ssize_t		deblk_buf_size;

	void		*intra_pred_buf;
	dma_addr_t	intra_pred_buf_dma;
	ssize_t		intra_pred_buf_size;
};

struct cedrus_dec_h264_job {
	const struct v4l2_ctrl_h264_sps			*sps;
	const struct v4l2_ctrl_h264_pps			*pps;
	const struct v4l2_ctrl_h264_scaling_matrix	*scaling_matrix;
	const struct v4l2_ctrl_h264_slice_params	*slice_params;
	const struct v4l2_ctrl_h264_pred_weights	*pred_weights;
	const struct v4l2_ctrl_h264_decode_params	*decode_params;
};

struct cedrus_dec_h264_buffer {
	unsigned int	position;
	unsigned int	pic_type;

	void		*mv_col_buf;
	dma_addr_t	mv_col_buf_dma;
	ssize_t		mv_col_buf_size;
};

enum cedrus_dec_h264_pic_type {
	CEDRUS_DEC_H264_PIC_TYPE_FRAME	= 0,
	CEDRUS_DEC_H264_PIC_TYPE_FIELD,
	CEDRUS_DEC_H264_PIC_TYPE_MBAFF,
};

/* XXX: move to regs */
enum cedrus_dec_h264_sram {
	CEDRUS_DEC_H264_SRAM_PRED_WEIGHT_TABLE	= 0x000,
	CEDRUS_DEC_H264_SRAM_FRAMEBUFFER_LIST	= 0x100,
	CEDRUS_DEC_H264_SRAM_REF_LIST_0		= 0x190,
	CEDRUS_DEC_H264_SRAM_REF_LIST_1		= 0x199,
	CEDRUS_DEC_H264_SRAM_SCALING_LIST_8x8_0	= 0x200,
	CEDRUS_DEC_H264_SRAM_SCALING_LIST_8x8_1	= 0x210,
	CEDRUS_DEC_H264_SRAM_SCALING_LIST_4x4	= 0x220,
};

/* XXX: move to regs */
struct cedrus_dec_h264_sram_ref_pic {
	__le32	top_field_order_cnt;
	__le32	bottom_field_order_cnt;
	__le32	frame_info;
	__le32	luma_ptr;
	__le32	chroma_ptr;
	__le32	mv_col_top_ptr;
	__le32	mv_col_bot_ptr;
	__le32	reserved;
} __packed;

extern const struct cedrus_engine cedrus_dec_h264;

#endif
