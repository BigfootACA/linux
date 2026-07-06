// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include "qcom_jenc_ops.h"
#include "qcom_jenc_res.h"

#include "qcom_jenc_v420_hw_info.h"

const struct qcom_dev_resources qcom_t165_t480_jpeg_drvdata = {
	.hw_ops			= &qcom_jpeg_default_ops,
	.ref_clk_hz		= 600000000UL,
	.ref_throughput_mpps	= 110UL,
};
