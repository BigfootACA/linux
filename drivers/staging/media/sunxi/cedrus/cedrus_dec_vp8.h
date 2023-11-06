/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2019 Jernej Skrabec <jernej.skrabec@siol.net>
 * Copyright 2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _CEDRUS_DEC_VP8_H_
#define _CEDRUS_DEC_VP8_H_

#include <media/v4l2-ctrls.h>

#define CEDRUS_DEC_VP8_ENTROPY_PROBS_SIZE	0x2400

struct cedrus_dec_vp8_context {
	unsigned int	last_frame_p_type;
	unsigned int	last_filter_type;
	unsigned int	last_sharpness_level;

	u8		*entropy_probs_buf;
	dma_addr_t	entropy_probs_buf_dma;
};

struct cedrus_dec_vp8_job {
	const struct v4l2_ctrl_vp8_frame	*frame;
};

extern const struct cedrus_engine cedrus_dec_vp8;

#endif
