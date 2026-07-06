/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef QCOM_JENC_HDR_H
#define QCOM_JENC_HDR_H

#include <linux/types.h>

#include <media/v4l2-jpeg.h>

#include "qcom_jenc_defs.h"

#define JPEG_HEADER_MAX	1024

struct qcom_jenc_header {
	u8  data[JPEG_HEADER_MAX];
	u32 size;
	u32 sof_offset;
	u32 dqt_luma_offs;
	u32 dqt_chroma_offs;
};

struct jpeg_record_hdr {
	u8 marker[2];
	u8 length[2];
} __packed;

struct jpeg_dqt_header {
	u8 index;
	u8 value[V4L2_JPEG_PIXELS_IN_BLOCK];
} __packed;

struct jpeg_soi_app0 {
	u8 soi[2];
	u8 app0_marker[2];
	u8 app0_length[2];
	u8 jfif_id[5];
	u8 version[2];
	u8 units;
	u8 density_x[2];
	u8 density_y[2];
	u8 thumb_x;
	u8 thumb_y;
} __packed;

struct jpeg_sof0_mono {
	u8 precision;
	u8 height[2];
	u8 width[2];
	u8 components;

	u8 y_id;
	u8 y_sampling;
	u8 y_qtable;
} __packed;

struct jpeg_sof0_color {
	u8 precision;
	u8 height[2];
	u8 width[2];
	u8 components;

	u8 y_id;
	u8 y_sampling;
	u8 y_qtable;

	u8 cb_id;
	u8 cb_sampling;
	u8 cb_qtable;

	u8 cr_id;
	u8 cr_sampling;
	u8 cr_qtable;
} __packed;

struct jpeg_sos_hdr {
	u8 sos_marker[2];
	u8 sos_length[2];
	u8 components;
} __packed;

struct jpeg_sos_mono {
	u8 components;

	u8 y_id;
	u8 y_tables;

	u8 spectral[2];
	u8 approx;
} __packed;

struct jpeg_sos_color {
	u8 components;

	u8 y_id;
	u8 y_tables;

	u8 cb_id;
	u8 cb_tables;

	u8 cr_id;
	u8 cr_tables;

	u8 spectral[2];
	u8 approx;
} __packed;

struct jenc_context;

int qcom_jenc_header_init(struct qcom_jenc_header *c, u32 fourcc);

void qcom_jenc_dqts_emit(const struct qcom_jenc_header *c, u8 *dst);

u32 qcom_jenc_header_emit(const struct qcom_jenc_header *c, u8 *dst, u32 dst_size, u16 width,
			  u16 height);

#endif /* QCOM_JENC_HDR_H */
