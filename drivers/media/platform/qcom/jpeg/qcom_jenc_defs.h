/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef QCOM_JENC_DEFS_H
#define QCOM_JENC_DEFS_H

#include <linux/types.h>
#include <uapi/linux/v4l2-controls.h>

/* Offline JPEG encoder constraints */
#define QCOM_JPEG_HW_MAX_WIDTH	8192
#define QCOM_JPEG_HW_MAX_HEIGHT	8192
#define QCOM_JPEG_HW_MIN_WIDTH	256
#define QCOM_JPEG_HW_MIN_HEIGHT	256

#define QCOM_JPEG_HW_DEF_HSTEP	16
#define QCOM_JPEG_HW_DEF_VSTEP	16

#define QCOM_JPEG_HW_DEF_WIDTH	1920
#define QCOM_JPEG_HW_DEF_HEIGHT	1088

#define QCOM_JPEG_MAX_PLANES	3

#define QCOM_JPEG_QUALITY_MIN	1
#define QCOM_JPEG_QUALITY_DEF	98
#define QCOM_JPEG_QUALITY_MAX	100
#define QCOM_JPEG_QUALITY_MID	(QCOM_JPEG_QUALITY_MAX / 2)
#define QCOM_JPEG_QUALITY_UNT	1

#define QCOM_JPEG_FPS_MIN	1
#define QCOM_JPEG_FPS_MAX	240
#define QCOM_JPEG_FPS_DEF	30
#define QCOM_JPEG_FPS_UNT	1

#endif /* QCOM_JENC_DEFS_H */
