/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _DT_BINDINGS_CLK_QCOM_EVA_CC_SM8750_H
#define _DT_BINDINGS_CLK_QCOM_EVA_CC_SM8750_H

/* EVA_CC clocks */
#define EVA_CC_AHB_CLK_SRC			0
#define EVA_CC_MVS0_CLK				1
#define EVA_CC_MVS0_CLK_SRC			2
#define EVA_CC_MVS0_DIV_CLK_SRC			3
#define EVA_CC_MVS0_FREERUN_CLK			4
#define EVA_CC_MVS0_SHIFT_CLK			5
#define EVA_CC_MVS0C_CLK			6
#define EVA_CC_MVS0C_DIV2_DIV_CLK_SRC		7
#define EVA_CC_MVS0C_FREERUN_CLK		8
#define EVA_CC_MVS0C_SHIFT_CLK			9
#define EVA_CC_PLL0				10
#define EVA_CC_SLEEP_CLK_SRC			11
#define EVA_CC_XO_CLK_SRC			12

/* EVA_CC power domains */
#define EVA_CC_MVS0_GDSC			0
#define EVA_CC_MVS0C_GDSC			1

/* EVA_CC resets */
#define EVA_CC_INTERFACE_BCR			0
#define EVA_CC_MVS0_BCR				1
#define EVA_CC_MVS0_FREERUN_CLK_ARES		2
#define EVA_CC_MVS0C_CLK_ARES			3
#define EVA_CC_MVS0C_BCR			4
#define EVA_CC_MVS0C_FREERUN_CLK_ARES		5

#endif
