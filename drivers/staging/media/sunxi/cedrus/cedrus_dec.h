/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _CEDRUS_DEC_H_
#define _CEDRUS_DEC_H_

#include <linux/videodev2.h>

struct cedrus_device;
struct cedrus_context;

/* Format */

int cedrus_dec_format_coded_prepare(struct cedrus_context *ctx,
				    struct v4l2_format *format);
int cedrus_dec_format_coded_configure(struct cedrus_context *ctx);
int cedrus_dec_format_picture_configure(struct cedrus_context *ctx);

/* Decoder */

int cedrus_dec_setup(struct cedrus_device *dev);
void cedrus_dec_cleanup(struct cedrus_device *dev);

#endif
