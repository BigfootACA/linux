// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/errno.h>
#include <linux/string.h>

#include <media/jpeg.h>
#include <media/v4l2-jpeg.h>

#include "qcom_jenc_dev.h"
#include "qcom_jenc_hdr.h"

/*
 * The elements defined in this header are specified
 * in the ITU-T T.81 / JPEG specification.
 *
 * https://www.w3.org/Graphics/JPEG/itu-t81.pdf
 */

#define JFIF_HEADER_WIDTH_OFFS		0x07
#define JFIF_HEADER_HEIGHT_OFFS		0x05

#define JFIF_APP0_LENGTH_HI		0x00
#define JFIF_APP0_LENGTH_LO		0x10
#define JFIF_IDENT_TERM		0x00
#define JFIF_VERSION_MAJOR		0x01
#define JFIF_VERSION_MINOR		0x01
#define JFIF_DENSITY_HI			0x00
#define JFIF_DENSITY_LO			0x01
#define JFIF_THUMBNAIL_SIZE		0x00

#define JPEG_SEG_LEN_HI			0x00
#define JPEG_LEN_DQT_LUMA_LO		0x43
#define JPEG_LEN_DQT_CHROMA_LO		0x43
#define JPEG_LEN_SOF0_MONO_LO		0x0b
#define JPEG_LEN_SOF0_COLOR_LO		0x11
#define JPEG_LEN_DHT_MONO_LO		0xd2
#define JPEG_LEN_DHT_COLOR_HI		0x01
#define JPEG_LEN_DHT_COLOR_LO		0xa2
#define JPEG_LEN_SOS_MONO_LO		0x08
#define JPEG_LEN_SOS_COLOR_LO		0x0c

struct jpeg_header_buf {
	u8  *ptr;
	u32 size;
	u32 pos;
};

static const struct jpeg_soi_app0 soi_app0 = {
	.soi		= { JPEG_MARKER_PREFIX, JPEG_MARKER_SOI },
	.app0_marker	= { JPEG_MARKER_PREFIX, JPEG_MARKER_APP0 },
	.app0_length	= { JFIF_APP0_LENGTH_HI, JFIF_APP0_LENGTH_LO },
	.jfif_id	= { 'J', 'F', 'I', 'F', JFIF_IDENT_TERM },
	.version	= { JFIF_VERSION_MAJOR, JFIF_VERSION_MINOR },
	.units		= 0x00,
	.density_x	= { JFIF_DENSITY_HI, JFIF_DENSITY_LO },
	.density_y	= { JFIF_DENSITY_HI, JFIF_DENSITY_LO },
	.thumb_x	= JFIF_THUMBNAIL_SIZE,
	.thumb_y	= JFIF_THUMBNAIL_SIZE,
};

static const struct jpeg_record_hdr dqt_luma_hdr = {
	.marker = { JPEG_MARKER_PREFIX, JPEG_MARKER_DQT },
	.length = { JPEG_SEG_LEN_HI, JPEG_LEN_DQT_LUMA_LO }
};

static const struct jpeg_record_hdr dqt_chroma_hdr = {
	.marker = { JPEG_MARKER_PREFIX, JPEG_MARKER_DQT },
	.length = { JPEG_SEG_LEN_HI, JPEG_LEN_DQT_CHROMA_LO }
};

/* Luminance quantization table */
static const struct jpeg_dqt_header dqt_luma_data = {
	.index = 0x00,
};

/* Chrominance quantization table */
static const struct jpeg_dqt_header dqt_chroma_data = {
	.index = 0x01,
};

static const struct jpeg_record_hdr  sof0_mono_hdr = {
	.marker	= { JPEG_MARKER_PREFIX, JPEG_MARKER_SOF0 },
	.length	= { JPEG_SEG_LEN_HI, JPEG_LEN_SOF0_MONO_LO },
};

static const struct jpeg_sof0_mono sof0_mono_data = {
	.precision	= JPEG_SAMPLE_PRECISION_BITS,
	.height		= { 0x00, 0x00 },
	.width		= { 0x00, 0x00 },
	.components	= JPEG_COMP_MONO,
	.y_id		= 1,
	.y_sampling	= JPEG_SAMPLING_H1V1,
	.y_qtable	= JPEG_QTABLE_LUMA,
};

static const struct jpeg_record_hdr  sof0_color_hdr = {
	.marker	= { JPEG_MARKER_PREFIX, JPEG_MARKER_SOF0 },
	.length	= { JPEG_SEG_LEN_HI, JPEG_LEN_SOF0_COLOR_LO },
};

static const struct jpeg_sof0_color sof0_color_data = {
	.precision	= JPEG_SAMPLE_PRECISION_BITS,
	.height		= { 0x00, 0x00 },
	.width		= { 0x00, 0x00 },
	.components	= JPEG_COMP_COLOR,
	.y_id		= 1,
	.y_sampling	= JPEG_SAMPLING_H2V2,
	.y_qtable	= JPEG_QTABLE_LUMA,
	.cb_id		= 2,
	.cb_sampling	= JPEG_SAMPLING_H1V1,
	.cb_qtable	= JPEG_QTABLE_CHROMA,
	.cr_id		= 3,
	.cr_sampling	= JPEG_SAMPLING_H1V1,
	.cr_qtable	= JPEG_QTABLE_CHROMA,
};

static const struct jpeg_record_hdr coeff_mono_hdr = {
	.marker = { JPEG_MARKER_PREFIX, JPEG_MARKER_DHT },
	.length = { JPEG_SEG_LEN_HI, JPEG_LEN_DHT_MONO_LO },
};

static const struct jpeg_record_hdr coeff_color_hdr = {
	.marker	= { JPEG_MARKER_PREFIX, JPEG_MARKER_DHT },
	.length	= { JPEG_LEN_DHT_COLOR_HI, JPEG_LEN_DHT_COLOR_LO },
};

static const struct jpeg_record_hdr sos_mono_hdr = {
	.marker	= { JPEG_MARKER_PREFIX, JPEG_MARKER_SOS },
	.length	= { JPEG_SEG_LEN_HI, JPEG_LEN_SOS_MONO_LO },
};

static const struct jpeg_sos_mono sos_mono_data = {
	.components	= JPEG_COMP_MONO,
	.y_id		= 1,
	.y_tables	= JPEG_TABLE_SEL_LUMA,
	.spectral	= { JPEG_SPECTRAL_START, JPEG_SPECTRAL_END },
	.approx		= JPEG_APPROX_HIGH_LOW,
};

static const struct jpeg_record_hdr sos_color_hdr = {
	.marker	= { JPEG_MARKER_PREFIX, JPEG_MARKER_SOS },
	.length	= { JPEG_SEG_LEN_HI, JPEG_LEN_SOS_COLOR_LO },
};

static const struct jpeg_sos_color sos_color_data = {
	.components	= JPEG_COMP_COLOR,
	.y_id		= 1,
	.y_tables	= JPEG_TABLE_SEL_LUMA,
	.cb_id		= 2,
	.cb_tables	= JPEG_TABLE_SEL_CHROMA,
	.cr_id		= 3,
	.cr_tables	= JPEG_TABLE_SEL_CHROMA,
	.spectral	= { JPEG_SPECTRAL_START, JPEG_SPECTRAL_END },
	.approx		= JPEG_APPROX_HIGH_LOW,
};

static inline int jb_put_mem(struct jpeg_header_buf *hdr, const void *src, u32 len)
{
	if (len > hdr->size - hdr->pos)
		return -ENOSPC;

	memcpy(hdr->ptr + hdr->pos, src, len);
	hdr->pos += len;

	return 0;
}

static int jb_put_dht(struct jpeg_header_buf *hdr, u8 index, const u8 *table, u32 len)
{
	u8 data[1 + V4L2_JPEG_REF_HT_AC_LEN];

	if (len > V4L2_JPEG_REF_HT_AC_LEN)
		return -EINVAL;

	data[0] = index;
	memcpy(&data[1], table, len);

	return jb_put_mem(hdr, data, len + 1);
}

static inline void patch_u16be(u8 *buf, u32 off, u16 v)
{
	buf[off]	= (v >> 8) & 0xff;
	buf[off + 1]	=  v & 0xff;
}

int qcom_jenc_header_init(struct qcom_jenc_header *c, u32 fourcc)
{
	int rc;
	struct jpeg_header_buf hdr = {
		.ptr = c->data,
		.size = sizeof(c->data),
		.pos = 0,
	};

	c->sof_offset	= 0;
	c->dqt_luma_offs = 0;
	c->dqt_chroma_offs = 0;

	rc = jb_put_mem(&hdr, &soi_app0, sizeof(soi_app0));
	if (rc)
		return rc;

	/* luma DQT is always present */
	rc = jb_put_mem(&hdr, &dqt_luma_hdr, sizeof(dqt_luma_hdr));
	if (rc)
		return rc;

	c->dqt_luma_offs = hdr.pos;
	rc = jb_put_mem(&hdr, &dqt_luma_data, sizeof(dqt_luma_data));
	if (rc)
		return rc;

	/* chroma DQT only for color images */
	if (fourcc != V4L2_PIX_FMT_GREY) {
		rc = jb_put_mem(&hdr, &dqt_chroma_hdr, sizeof(dqt_chroma_hdr));
		if (rc)
			return rc;

		c->dqt_chroma_offs = hdr.pos;
		rc = jb_put_mem(&hdr, &dqt_chroma_data, sizeof(dqt_chroma_data));
		if (rc)
			return rc;
	}

	/* store the offset of the SOF record for later use */
	c->sof_offset = hdr.pos;

	if (fourcc != V4L2_PIX_FMT_GREY) {
		rc = jb_put_mem(&hdr, &sof0_color_hdr, sizeof(sof0_color_hdr));
		if (rc)
			return rc;
		rc = jb_put_mem(&hdr, &sof0_color_data, sizeof(sof0_color_data));
		if (rc)
			return rc;
		rc = jb_put_mem(&hdr, &coeff_color_hdr, sizeof(coeff_color_hdr));
		if (rc)
			return rc;
		rc = jb_put_dht(&hdr, JPEG_DC_HT_INDEX_LUMA,
				v4l2_jpeg_ref_table_luma_dc_ht,
				ARRAY_SIZE(v4l2_jpeg_ref_table_luma_dc_ht));
		if (rc)
			return rc;
		rc = jb_put_dht(&hdr, JPEG_AC_HT_INDEX_LUMA,
				v4l2_jpeg_ref_table_luma_ac_ht,
				ARRAY_SIZE(v4l2_jpeg_ref_table_luma_ac_ht));
		if (rc)
			return rc;
		rc = jb_put_dht(&hdr, JPEG_DC_HT_INDEX_CHROMA,
				v4l2_jpeg_ref_table_chroma_dc_ht,
				ARRAY_SIZE(v4l2_jpeg_ref_table_chroma_dc_ht));
		if (rc)
			return rc;
		rc = jb_put_dht(&hdr, JPEG_AC_HT_INDEX_CHROMA,
				v4l2_jpeg_ref_table_chroma_ac_ht,
				ARRAY_SIZE(v4l2_jpeg_ref_table_chroma_ac_ht));
		if (rc)
			return rc;
		rc = jb_put_mem(&hdr, &sos_color_hdr, sizeof(sos_color_hdr));
		if (rc)
			return rc;
		rc = jb_put_mem(&hdr, &sos_color_data, sizeof(sos_color_data));
		if (rc)
			return rc;
	} else {
		rc = jb_put_mem(&hdr, &sof0_mono_hdr, sizeof(sof0_mono_hdr));
		if (rc)
			return rc;
		rc = jb_put_mem(&hdr, &sof0_mono_data, sizeof(sof0_mono_data));
		if (rc)
			return rc;
		rc = jb_put_mem(&hdr, &coeff_mono_hdr, sizeof(coeff_mono_hdr));
		if (rc)
			return rc;
		rc = jb_put_dht(&hdr, JPEG_DC_HT_INDEX_LUMA,
				v4l2_jpeg_ref_table_luma_dc_ht,
				ARRAY_SIZE(v4l2_jpeg_ref_table_luma_dc_ht));
		if (rc)
			return rc;
		rc = jb_put_dht(&hdr, JPEG_AC_HT_INDEX_LUMA,
				v4l2_jpeg_ref_table_luma_ac_ht,
				ARRAY_SIZE(v4l2_jpeg_ref_table_luma_ac_ht));
		if (rc)
			return rc;
		rc = jb_put_mem(&hdr, &sos_mono_hdr, sizeof(sos_mono_hdr));
		if (rc)
			return rc;
		rc = jb_put_mem(&hdr, &sos_mono_data, sizeof(sos_mono_data));
		if (rc)
			return rc;
	}

	c->size = hdr.pos;

	return 0;
}

void qcom_jenc_dqts_emit(const struct qcom_jenc_header *c, u8 *dst)
{
	/* Propagate DQT tables into the JPEG header */
	if (c->dqt_luma_offs) {
		u32 luma_offs = c->dqt_luma_offs + sizeof(dqt_luma_data.index);

		memcpy(dst + luma_offs, &c->data[luma_offs], sizeof(dqt_luma_data.value));
	}

	if (c->dqt_chroma_offs) {
		u32 chroma_offs = c->dqt_chroma_offs + sizeof(dqt_chroma_data.index);

		memcpy(dst + chroma_offs, &c->data[chroma_offs], sizeof(dqt_chroma_data.value));
	}
}

u32 qcom_jenc_header_emit(const struct qcom_jenc_header *c, u8 *dst, u32 dst_size, u16 width,
			  u16 height)
{
	/* Copy JFIF into JPEG header and update actual image size */
	if (dst_size < c->size)
		return 0;

	memcpy(dst, c->data, c->size);

	/* Update output image size */
	patch_u16be(dst, c->sof_offset + JFIF_HEADER_WIDTH_OFFS, width);
	patch_u16be(dst, c->sof_offset + JFIF_HEADER_HEIGHT_OFFS, height);

	return c->size;
}
