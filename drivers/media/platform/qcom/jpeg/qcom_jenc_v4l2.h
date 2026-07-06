/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef QCOM_JENC_V4L2_H
#define QCOM_JENC_V4L2_H

#include <linux/types.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

struct qcom_jenc_dev;

int qcom_jpeg_v4l2_register(struct qcom_jenc_dev *jenc);

#endif /* QCOM_JENC_V4L2_H */
