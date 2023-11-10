/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _CEDRUS_ENC_H264_H_
#define _CEDRUS_ENC_H264_H_

#include <media/v4l2-ctrls.h>

#define CENDRUS_ENC_H264_NALU_TYPE_SLICE_NON_IDR	1
#define CENDRUS_ENC_H264_NALU_TYPE_SLICE_IDR		5
#define CENDRUS_ENC_H264_NALU_TYPE_SPS			7
#define CENDRUS_ENC_H264_NALU_TYPE_PPS			8
#define CENDRUS_ENC_H264_NALU_TYPE_AUD			9

#define CEDRUS_ENC_H264_SLICE_TYPE_I		2
#define CEDRUS_ENC_H264_SLICE_TYPE_B		1
#define CEDRUS_ENC_H264_SLICE_TYPE_P		0

#define CEDRUS_ENC_H264_CONSTRAINT_SET0_FLAG	BIT(7)
#define CEDRUS_ENC_H264_CONSTRAINT_SET1_FLAG	BIT(6)
#define CEDRUS_ENC_H264_CONSTRAINT_SET2_FLAG	BIT(5)
#define CEDRUS_ENC_H264_CONSTRAINT_SET3_FLAG	BIT(4)
#define CEDRUS_ENC_H264_CONSTRAINT_SET4_FLAG	BIT(3)
#define CEDRUS_ENC_H264_CONSTRAINT_SET5_FLAG	BIT(2)

enum cedrus_enc_h264_frame_type {
	CEDRUS_ENC_H264_FRAME_TYPE_IDR,
	CEDRUS_ENC_H264_FRAME_TYPE_I,
	CEDRUS_ENC_H264_FRAME_TYPE_P,
	CEDRUS_ENC_H264_FRAME_TYPE_B,
};

enum cedrus_enc_h264_step {
	CEDRUS_ENC_H264_STEP_START,
	CEDRUS_ENC_H264_STEP_SPS,
	CEDRUS_ENC_H264_STEP_PPS,
	CEDRUS_ENC_H264_STEP_SLICE,
};

struct cedrus_enc_h264_job {
	unsigned int	nal_ref_idc;
	unsigned int	frame_type;
	unsigned int	frame_num;
	unsigned int	pic_order_cnt_lsb;
	unsigned int	qp;

	unsigned int	seq_parameter_set_id;
	unsigned int	pic_parameter_set_id;
	unsigned int	idr_pic_id;

	unsigned int	profile_idc;
	unsigned int	level_idc;
	unsigned int	constraint_set_flags;
	unsigned int	entropy_coding_mode_flag;
	unsigned int	chroma_qp_index_offset;
	unsigned int	disable_deblocking_filter_idc;
	int		slice_alpha_c0_offset_div2;
	int		slice_beta_offset_div2;

	unsigned int	cabac_init_idc;
};

struct cedrus_enc_h264_state {
	unsigned int	step;

	unsigned int	gop_index;
	unsigned int	frame_num;
	unsigned int	pic_order_cnt_lsb;

	unsigned int	qp_init;
};

struct cedrus_enc_h264_context {
	struct cedrus_enc_h264_state	state;

	void				*mb_info;
	dma_addr_t			mb_info_dma;
	unsigned int			mb_info_size;

	dma_addr_t			subpix_last_dma;

	dma_addr_t			rec_last_dma;
	unsigned int			rec_last_luma_size;

	unsigned int			width_mbs;
	unsigned int			height_mbs;

	unsigned int			pic_order_cnt_type;
	unsigned int			log2_max_pic_order_cnt_lsb;
	unsigned int			log2_max_frame_num;

	int				prepend_sps_pps_idr;
	int				profile;
	int				level;
	int				vui_sar_enable;
	int				vui_sar_idc;
	int				vui_ext_sar_width;
	int				vui_ext_sar_height;
	int				entropy_mode;
	int				chroma_qp_index_offset;
	int				loop_filter_mode;
	int				loop_filter_alpha;
	int				loop_filter_beta;
	int				qp_min;
	int				qp_max;
	int				qp_i;
	int				qp_p;
	int				gop_closure;
	int				gop_size;
	int				gop_open_i_period;
	bool				force_key_frame;

	struct v4l2_ctrl		*entropy_mode_ctrl;
};

struct cedrus_enc_h264_buffer {
	void		*rec;
	dma_addr_t	rec_dma;
	unsigned int	rec_size;
	unsigned int	rec_luma_size;
	unsigned int	rec_chroma_size;

	void		*subpix;
	dma_addr_t	subpix_dma;
	unsigned int	subpix_size;
};

extern const struct cedrus_engine cedrus_enc_h264;

static inline u8 cedrus_enc_h264_nalu_header(u8 type, u8 ref_idc)
{
	return (type & GENMASK(4, 0)) | ((ref_idc << 5) & GENMASK(6, 5));
}
#endif
