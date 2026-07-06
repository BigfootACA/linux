/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _MEDIA_JPEG_H_
#define _MEDIA_JPEG_H_

/* JPEG markers */
#define JPEG_MARKER_TEM		0x01
#define JPEG_MARKER_SOF0	0xc0
#define JPEG_MARKER_DHT		0xc4
#define JPEG_MARKER_RST		0xd0
#define JPEG_MARKER_SOI		0xd8
#define JPEG_MARKER_EOI		0xd9
#define JPEG_MARKER_SOS		0xda
#define JPEG_MARKER_DQT		0xdb
#define JPEG_MARKER_DRI		0xdd
#define JPEG_MARKER_DHP		0xde
#define JPEG_MARKER_APP0	0xe0
#define JPEG_MARKER_COM		0xfe

/* JPEG marker prefix byte */
#define JPEG_MARKER_PREFIX		0xff

/* JPEG baseline sample precision (bits per sample) */
#define JPEG_SAMPLE_PRECISION_BITS	8

/* Number of components: greyscale (1) and colour YCbCr (3) */
#define JPEG_COMP_MONO			1
#define JPEG_COMP_COLOR			3

/* Chroma subsampling factors encoded in SOF0 sampling byte */
#define JPEG_SAMPLING_H1V1		0x11
#define JPEG_SAMPLING_H2V2		0x22

/* Quantization table destination identifiers */
#define JPEG_QTABLE_LUMA		0
#define JPEG_QTABLE_CHROMA		1

/* Huffman table class/destination identifiers for DC and AC tables */
#define JPEG_DC_HT_INDEX_LUMA		0x00
#define JPEG_DC_HT_INDEX_CHROMA		0x01
#define JPEG_AC_HT_INDEX_LUMA		0x10
#define JPEG_AC_HT_INDEX_CHROMA		0x11

/* SOS spectral selection and approximation fields (baseline JPEG) */
#define JPEG_SPECTRAL_START		0x00
#define JPEG_SPECTRAL_END		0x3f
#define JPEG_APPROX_HIGH_LOW		0x00

/* SOS table selector bytes (luma: DC=0/AC=0, chroma: DC=1/AC=1) */
#define JPEG_TABLE_SEL_LUMA		0x00
#define JPEG_TABLE_SEL_CHROMA		0x11

#endif /* _MEDIA_JPEG_H_ */
