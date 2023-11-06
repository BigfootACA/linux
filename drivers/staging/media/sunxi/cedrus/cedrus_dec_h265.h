/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2013 Jens Kuske <jenskuske@gmail.com>
 * Copyright 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright 2018-2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _CEDRUS_DEC_H265_H_
#define _CEDRUS_DEC_H265_H_

#include <media/v4l2-ctrls.h>

/*
 * These are the sizes for side buffers required by the hardware for storing
 * internal decoding metadata. They match the values used by the early BSP
 * implementations, that were initially exposed in libvdpau-sunxi.
 * Subsequent BSP implementations seem to double the neighbor info buffer size
 * for the H6 SoC, which may be related to 10 bit H265 support.
 */
#define CEDRUS_DEC_H265_NEIGHBOR_INFO_BUF_SIZE		(794 * SZ_1K)
#define CEDRUS_DEC_H265_ENTRY_POINTS_BUF_SIZE		(4 * SZ_1K)
#define CEDRUS_DEC_H265_MV_COL_BUF_UNIT_CTB_SIZE	160

struct cedrus_dec_h265_context {
	void		*neighbor_info_buf;
	dma_addr_t	neighbor_info_buf_addr;

	void		*entry_points_buf;
	dma_addr_t	entry_points_buf_addr;
};

struct cedrus_dec_h265_job {
	const struct v4l2_ctrl_hevc_sps			*sps;
	const struct v4l2_ctrl_hevc_pps			*pps;
	const struct v4l2_ctrl_hevc_scaling_matrix	*scaling_matrix;
	const struct v4l2_ctrl_hevc_slice_params	*slice_params;
	const u32					*entry_point_offsets;
	u32						entry_point_offsets_count;
	const struct v4l2_ctrl_hevc_decode_params	*decode_params;
};

struct cedrus_dec_h265_buffer {
	void		*mv_col_buf;
	dma_addr_t	mv_col_buf_dma;
	ssize_t		mv_col_buf_size;
};

/* XXX: move to regs */
struct cedrus_dec_h265_sram_frame_info {
	__le32	top_pic_order_cnt;
	__le32	bottom_pic_order_cnt;
	__le32	top_mv_col_buf_addr;
	__le32	bottom_mv_col_buf_addr;
	__le32	luma_addr;
	__le32	chroma_addr;
} __packed;

/* XXX: move to regs */
struct cedrus_dec_h265_sram_pred_weight {
	__s8	delta_weight;
	__s8	offset;
} __packed;

extern const struct cedrus_engine cedrus_dec_h265;

#endif
