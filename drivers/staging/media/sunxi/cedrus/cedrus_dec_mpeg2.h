/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2016 Florent Revest <florent.revest@free-electrons.com>
 * Copyright 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright 2018-2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _CEDRUS_DEC_MPEG2_H_
#define _CEDRUS_DEC_MPEG2_H_

#include <media/v4l2-ctrls.h>

struct cedrus_dec_mpeg2_job {
	const struct v4l2_ctrl_mpeg2_sequence		*sequence;
	const struct v4l2_ctrl_mpeg2_picture		*picture;
	const struct v4l2_ctrl_mpeg2_quantisation	*quantisation;
};

extern const struct cedrus_engine cedrus_dec_mpeg2;

#endif
