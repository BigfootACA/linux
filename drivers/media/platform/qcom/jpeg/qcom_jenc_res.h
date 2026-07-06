/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef QCOM_JENC_RES_H
#define QCOM_JENC_RES_H

#include "qcom_jenc_defs.h"

struct qcom_dev_resources {
	const struct qcom_jpeg_hw_ops	*hw_ops;
	unsigned long			ref_clk_hz;
	unsigned long			ref_throughput_mpps;
};

extern const struct qcom_dev_resources qcom_t165_t480_jpeg_drvdata;

#endif	/* QCOM_JENC_RES_H */
