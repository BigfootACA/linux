// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/math64.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-sg.h>

#include "qcom_jenc_defs.h"
#include "qcom_jenc_dev.h"
#include "qcom_jenc_ops.h"
#include "qcom_jenc_v420_hw_info.h"

/* Per-plane register offset lookup arrays (indexed by plane number 0..2) */
static const u32 jpeg_v420_fe_pntr[] = {
	JPEG_V420_REG_FE_PNTR_0,
	JPEG_V420_REG_FE_PNTR_1,
	JPEG_V420_REG_FE_PNTR_2,
};

static const u32 jpeg_v420_fe_offs[] = {
	JPEG_V420_REG_FE_OFFS_0,
	JPEG_V420_REG_FE_OFFS_1,
	JPEG_V420_REG_FE_OFFS_2,
};

static const u32 jpeg_v420_fe_bsize[] = {
	JPEG_V420_REG_FE_BSIZE_0,
	JPEG_V420_REG_FE_BSIZE_1,
	JPEG_V420_REG_FE_BSIZE_2,
};

static const u32 jpeg_v420_fe_stride[] = {
	JPEG_V420_REG_FE_STRIDE_0,
	JPEG_V420_REG_FE_STRIDE_1,
	JPEG_V420_REG_FE_STRIDE_2,
};

static const u32 jpeg_v420_fe_hinit[] = {
	JPEG_V420_REG_FE_HINIT_0,
	JPEG_V420_REG_FE_HINIT_1,
	JPEG_V420_REG_FE_HINIT_2,
};

static const u32 jpeg_v420_fe_vinit[] = {
	JPEG_V420_REG_FE_VINIT_0,
	JPEG_V420_REG_FE_VINIT_1,
	JPEG_V420_REG_FE_VINIT_2,
};

static const u32 jpeg_v420_we_pntr[] = {
	JPEG_V420_REG_WE_PNTR_0,
	JPEG_V420_REG_WE_PNTR_1,
	JPEG_V420_REG_WE_PNTR_2,
};

static const u32 jpeg_v420_we_stride[] = {
	JPEG_V420_REG_WE_STRIDE_0,
	JPEG_V420_REG_WE_STRIDE_1,
	JPEG_V420_REG_WE_STRIDE_2,
};

static const u32 jpeg_v420_we_hinit[] = {
	JPEG_V420_REG_WE_HINIT_0,
	JPEG_V420_REG_WE_HINIT_1,
	JPEG_V420_REG_WE_HINIT_2,
};

static const u32 jpeg_v420_we_hstep[] = {
	JPEG_V420_REG_WE_HSTEP_0,
	JPEG_V420_REG_WE_HSTEP_1,
	JPEG_V420_REG_WE_HSTEP_2,
};

static const u32 jpeg_v420_we_vinit[] = {
	JPEG_V420_REG_WE_VINIT_0,
	JPEG_V420_REG_WE_VINIT_1,
	JPEG_V420_REG_WE_VINIT_2,
};

static const u32 jpeg_v420_we_vstep[] = {
	JPEG_V420_REG_WE_VSTEP_0,
	JPEG_V420_REG_WE_VSTEP_1,
	JPEG_V420_REG_WE_VSTEP_2,
};

static const u32 jpeg_v420_scale_hstep[] = {
	JPEG_V420_REG_SCALE_HSTEP_0,
	JPEG_V420_REG_SCALE_HSTEP_1,
	JPEG_V420_REG_SCALE_HSTEP_2,
};

static const u32 jpeg_v420_scale_vstep[] = {
	JPEG_V420_REG_SCALE_VSTEP_0,
	JPEG_V420_REG_SCALE_VSTEP_1,
	JPEG_V420_REG_SCALE_VSTEP_2,
};

static const u32 jpeg_v420_scale_out_cfg[] = {
	JPEG_V420_REG_SCALE_OUT_CFG_0,
	JPEG_V420_REG_SCALE_OUT_CFG_1,
	JPEG_V420_REG_SCALE_OUT_CFG_2,
};

#define JPEG_RESET_TIMEOUT_MS	300
#define JPEG_STOP_TIMEOUT_MS	200

#define JPEG_DQT_SHIFT		20

#define JPEG_MCU_BLOCK_8	8
#define JPEG_MCU_BLOCK_16	16
#define JPEG_MCU_BLOCK_128	128
#define JPEG_MCU_BLOCK_256	256

#define JPEG_DEFAULT_SCALE_STEP	0x200000

#define JPEG_CLR_U32	(0U)
#define JPEG_SET_U32	(~0U)

/*
 *  JPEG | V4L2
 *  ---- | -------
 *  H1V1 | GREY
 *  H1V2 | YUV422M
 *  H2V1 | NV16M
 *  H2V2 | NV12M
 */
enum qcom_jpeg_encode_fmt {
	JPEG_ENCODE_H1V1 = 0,
	JPEG_ENCODE_H1V2,
	JPEG_ENCODE_H2V1,
	JPEG_ENCODE_H2V2,
	JPEG_ENCODE_MONO,
};

enum qcom_jpeg_memory_fmt {
	JPEG_MEM_FMT_PLANAR	 = 0x0,
	JPEG_MEM_FMT_PPLANAR	 = 0x1,
	JPEG_MEM_FMT_MONO	 = 0x2,
	JPEG_MEM_FMT_COEFFICIENT = 0x3
};

enum jpeg_mal_bounds {
	JPEG_CFG_MAL_BOUND_32_BYTES	= 0x0,
	JPEG_CFG_MAL_BOUND_64_BYTES	= 0x1,
	JPEG_CFG_MAL_BOUND_128_BYTES	= 0x2,
	JPEG_CFG_MAL_BOUND_256_BYTES	= 0x3,
	JPEG_CFG_MAL_BOUND_512_BYTES	= 0x4,
	JPEG_CFG_MAL_BOUND_1K_BYTES	= 0x5,
	JPEG_CFG_MAL_BOUND_2K_BYTES	= 0x6,
	JPEG_CFG_MAL_BOUND_4K_BYTES	= 0x7
};

struct qcom_jpeg_scale_blocks {
	u8 w_block[QCOM_JPEG_MAX_PLANES];
	u8 h_block[QCOM_JPEG_MAX_PLANES];
};

struct qcom_jpeg_mal_boundary {
	u32 bytes;
	int boundary;
};

struct qcom_jpeg_formats {
	enum qcom_jpeg_encode_fmt encode;
	enum qcom_jpeg_memory_fmt memory;
	u32 fourcc;
};

/*
 * Luminance quantization table defined by CCITT T.81.
 * See: https://www.w3.org/Graphics/JPEG/itu-t81.pdf
 */
static const u8 t81k1_dct_luma_table[V4L2_JPEG_PIXELS_IN_BLOCK] = {
	16,  11,  10,  16,  24,  40,  51,  61,
	12,  12,  14,  19,  26,  58,  60,  55,
	14,  13,  16,  24,  40,  57,  69,  56,
	14,  17,  22,  29,  51,  87,  80,  62,
	18,  22,  37,  56,  68, 109, 103,  77,
	24,  35,  55,  64,  81, 104, 113,  92,
	49,  64,  78,  87, 103, 121, 120, 101,
	72,  92,  95,  98, 112, 100, 103,  99
};

/*
 * Chrominance quantization table defined by CCITT T.81.
 * See: https://www.w3.org/Graphics/JPEG/itu-t81.pdf
 */
static const u8 t81k2_dct_chroma_table[V4L2_JPEG_PIXELS_IN_BLOCK] = {
	17,  18,  24,  47,  99,  99,  99,  99,
	18,  21,  26,  66,  99,  99,  99,  99,
	24,  26,  56,  99,  99,  99,  99,  99,
	47,  66,  99,  99,  99,  99,  99,  99,
	99,  99,  99,  99,  99,  99,  99,  99,
	99,  99,  99,  99,  99,  99,  99,  99,
	99,  99,  99,  99,  99,  99,  99,  99,
	99,  99,  99,  99,  99,  99,  99,  99
};

/*
 * Zig-zag scan order for quantized DCT coefficients
 * as defined by CCITT T.81.
 * See: https://www.w3.org/Graphics/JPEG/itu-t81.pdf
 */
static const u8 t81a6_dct_zig_zag_table[] = {
	 0,  1,  5,  6, 14, 15, 27, 28,
	 2,  4,  7, 13, 16, 26, 29, 42,
	 3,  8, 12, 17, 25, 30, 41, 43,
	 9, 11, 18, 24, 31, 40, 44, 53,
	10, 19, 23, 32, 39, 45, 52, 54,
	20, 22, 33, 38, 46, 51, 55, 60,
	21, 34, 37, 47, 50, 56, 59, 61,
	35, 36, 48, 49, 57, 58, 62, 63
};

static const struct qcom_jpeg_formats jpeg_encode_fmt[] = {
	{
		.fourcc = V4L2_PIX_FMT_GREY,
		.encode = JPEG_ENCODE_MONO,
		.memory = JPEG_MEM_FMT_MONO
	},
	{
		.fourcc = V4L2_PIX_FMT_JPEG,
		.encode = JPEG_ENCODE_H1V1,
		.memory = JPEG_MEM_FMT_PPLANAR
	},
	{
		.fourcc = V4L2_PIX_FMT_YUV422M,
		.encode = JPEG_ENCODE_H1V2,
		.memory = JPEG_MEM_FMT_PLANAR
	},
	{
		.fourcc = V4L2_PIX_FMT_YVU422M,
		.encode = JPEG_ENCODE_H1V2,
		.memory = JPEG_MEM_FMT_PLANAR
	},
	{
		.fourcc = V4L2_PIX_FMT_NV16M,
		.encode = JPEG_ENCODE_H2V1,
		.memory = JPEG_MEM_FMT_PPLANAR
	},
	{
		.fourcc = V4L2_PIX_FMT_NV61M,
		.encode = JPEG_ENCODE_H2V1,
		.memory = JPEG_MEM_FMT_PPLANAR
	},
	{
		.fourcc = V4L2_PIX_FMT_NV12M,
		.encode = JPEG_ENCODE_H2V2,
		.memory = JPEG_MEM_FMT_PPLANAR
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21M,
		.encode = JPEG_ENCODE_H2V2,
		.memory = JPEG_MEM_FMT_PPLANAR
	}
};

static const struct qcom_jpeg_mal_boundary jpeg_mal_bounds[] = {
	{ .bytes =   32, .boundary = JPEG_CFG_MAL_BOUND_32_BYTES  },
	{ .bytes =   64, .boundary = JPEG_CFG_MAL_BOUND_64_BYTES  },
	{ .bytes =  128, .boundary = JPEG_CFG_MAL_BOUND_128_BYTES },
	{ .bytes =  256, .boundary = JPEG_CFG_MAL_BOUND_256_BYTES },
	{ .bytes =  512, .boundary = JPEG_CFG_MAL_BOUND_512_BYTES },
	{ .bytes = 1024, .boundary = JPEG_CFG_MAL_BOUND_1K_BYTES  },
	{ .bytes = 2048, .boundary = JPEG_CFG_MAL_BOUND_2K_BYTES  },
	{ .bytes = 4096, .boundary = JPEG_CFG_MAL_BOUND_4K_BYTES  }
};

static const struct qcom_jpeg_scale_blocks jpeg_mcu_blocks[] = {
	[JPEG_ENCODE_H1V1] = {
		.w_block = { JPEG_MCU_BLOCK_8, JPEG_MCU_BLOCK_8, JPEG_MCU_BLOCK_8 },
		.h_block = { JPEG_MCU_BLOCK_8, JPEG_MCU_BLOCK_8, JPEG_MCU_BLOCK_8 },
	},
	[JPEG_ENCODE_H1V2] = {
		.w_block = { JPEG_MCU_BLOCK_8, JPEG_MCU_BLOCK_8, JPEG_MCU_BLOCK_8  },
		.h_block = { JPEG_MCU_BLOCK_16, JPEG_MCU_BLOCK_8, JPEG_MCU_BLOCK_8 },
	},
	[JPEG_ENCODE_H2V1] = {
		.w_block = { JPEG_MCU_BLOCK_16, JPEG_MCU_BLOCK_8, JPEG_MCU_BLOCK_8 },
		.h_block = { JPEG_MCU_BLOCK_8, JPEG_MCU_BLOCK_8, JPEG_MCU_BLOCK_8  },
	},
	[JPEG_ENCODE_H2V2] = {
		.w_block = { JPEG_MCU_BLOCK_16, JPEG_MCU_BLOCK_8, JPEG_MCU_BLOCK_8 },
		.h_block = { JPEG_MCU_BLOCK_16, JPEG_MCU_BLOCK_8, JPEG_MCU_BLOCK_8 },
	},
	[JPEG_ENCODE_MONO] = {
		.w_block = { JPEG_MCU_BLOCK_8 },
		.h_block = { JPEG_MCU_BLOCK_8 }
	},
};

static inline int jpeg_get_memory_fmt(u32 fourcc)
{
	u32 fi;

	for (fi = 0; fi < ARRAY_SIZE(jpeg_encode_fmt); fi++) {
		if (jpeg_encode_fmt[fi].fourcc == fourcc)
			return jpeg_encode_fmt[fi].memory;
	}

	return -EINVAL;
}

static inline int jpeg_get_encode_fmt(u32 fourcc)
{
	u32 fi;

	for (fi = 0; fi < ARRAY_SIZE(jpeg_encode_fmt); fi++) {
		if (jpeg_encode_fmt[fi].fourcc == fourcc)
			return jpeg_encode_fmt[fi].encode;
	}

	return -EINVAL;
}

static inline int jpeg_get_mal_boundary(u32 width, const struct qcom_jpeg_mal_boundary *table,
					u32 count)
{
	u32 bi;

	for (bi = 0; bi < count; bi++) {
		if (table[bi].bytes > width)
			break;
	}

	if (!bi)
		return table[0].boundary;

	if (bi >= count)
		return table[count - 1].boundary;

	return table[bi - 1].boundary;
}

static inline int jpeg_get_mcu_geometry(enum qcom_jpeg_encode_fmt fmt, u32 width, u32 height,
					u32 *blk_w, u32 *blk_h, u32 *mcu_cols, u32 *mcu_rows)
{
	const struct qcom_jpeg_scale_blocks *blks;
	u32 bw = 0, bh = 0;
	u8 pln;

	blks = &jpeg_mcu_blocks[fmt];

	for (pln = 0; pln < QCOM_JPEG_MAX_PLANES; pln++) {
		bw = max(bw, blks->w_block[pln]);
		bh = max(bh, blks->h_block[pln]);
	}

	if (!bw || !bh)
		return -EINVAL;

	if (blk_w)
		*blk_w = bw;
	if (blk_h)
		*blk_h = bh;

	if (mcu_cols)
		*mcu_cols = ALIGN(width, bw) / bw;

	if (mcu_rows)
		*mcu_rows = ALIGN(height, bh) / bh;

	return 0;
}

static inline u32 jpeg_io_read(struct qcom_jenc_dev *jenc, u32 offset)
{
	return readl(jenc->jpeg_base + offset);
}

static inline void jpeg_io_write(struct qcom_jenc_dev *jenc, u32 offset, u32 value)
{
	writel(value, jenc->jpeg_base + offset);
}

/*
 * Runtime bitfield helpers (for non-constant masks).
 *
 * Requirements:
 *  - mask must be non-zero
 *  - mask must be contiguous (e.g. 0x7u << n)
 */

static inline u32 jpeg_bits_get(u32 mask, u32 val)
{
	/* __ffs(0) is undefined; fail-safe on invalid masks. */
	if (WARN_ON_ONCE(!mask))
		return 0;

	return (val & mask) >> __ffs(mask);
}

static inline u32 jpeg_bits_set(u32 mask, u32 val)
{
	/* __ffs(0) is undefined; fail-safe on invalid masks. */
	if (WARN_ON_ONCE(!mask))
		return 0;

	return (val << __ffs(mask)) & mask;
}

static inline u32 jpeg_rd_bits(struct qcom_jenc_dev *jenc, u32 offs, u32 mask)
{
	u32 reg = jpeg_io_read(jenc, offs);

	return jpeg_bits_get(mask, reg);
}

/*
 * Read-modify-write (for R/W registers)
 */
static inline void jpeg_rw_bits(struct qcom_jenc_dev *jenc, u32 offs, u32 mask, u32 val)
{
	u32 reg = jpeg_io_read(jenc, offs);

	reg &= ~mask;
	reg |= jpeg_bits_set(mask, val);

	jpeg_io_write(jenc, offs, reg);
}

/*
 * Write-only variant (for write only registers)
 */
static inline void jpeg_wo_bits(struct qcom_jenc_dev *jenc, u32 offs, u32 mask, u32 val)
{
	jpeg_io_write(jenc, offs, jpeg_bits_set(mask, val));
}

static u8 jpeg_calculate_dqt(struct jenc_context *ectx, u8 dqt_value)
{
	u8 calc_val;
	u64 ratio;

	ratio = (QCOM_JPEG_QUALITY_MAX - ectx->quality_requested) << JPEG_DQT_SHIFT;
	ratio = max_t(u64, 1, ratio);
	do_div(ratio, QCOM_JPEG_QUALITY_MID);

	calc_val = DIV64_U64_ROUND_CLOSEST(ratio * dqt_value, 1LU << JPEG_DQT_SHIFT);

	return max_t(u8, 1, calc_val);
}

/*
 * jpeg_update_dqt_cache - compute scaled DQT coefficients and store them in
 * the software JPEG header cache (hdr_cache).  Safe to call from buf_prepare
 * before the hardware is powered on; no MMIO access is performed here.
 */
static void jpeg_update_dqt_cache(struct jenc_context *ectx)
{
	u8 *base;
	u8 dqt_val, idx;
	int i;

	/* Luma DQT cache update */
	if (ectx->hdr_cache.dqt_luma_offs) {
		base = &ectx->hdr_cache.data[ectx->hdr_cache.dqt_luma_offs + 1];
		for (i = 0; i < ARRAY_SIZE(t81k1_dct_luma_table); i++) {
			dqt_val = jpeg_calculate_dqt(ectx, t81k1_dct_luma_table[i]);
			idx = t81a6_dct_zig_zag_table[i];
			base[idx] = dqt_val;
		}
	}

	/* Chroma DQT cache update */
	if (ectx->hdr_cache.dqt_chroma_offs) {
		base = &ectx->hdr_cache.data[ectx->hdr_cache.dqt_chroma_offs + 1];
		for (i = 0; i < ARRAY_SIZE(t81k2_dct_chroma_table); i++) {
			dqt_val = jpeg_calculate_dqt(ectx, t81k2_dct_chroma_table[i]);
			idx = t81a6_dct_zig_zag_table[i];
			base[idx] = dqt_val;
		}
	}
}

/*
 * jpeg_upload_dmi_table - write the scaled DQT coefficients to the hardware
 * DMI registers.  Must only be called from the job execution path where
 * runtime PM has already been acquired (pm_runtime_resume_and_get).
 *
 * Reads precomputed values from hdr_cache (populated by jpeg_update_dqt_cache)
 * to avoid redundant per-coefficient recalculation on the hot encode path.
 */
static void jpeg_upload_dmi_table(struct jenc_context *ectx)
{
	const u8 *luma_qt = &ectx->hdr_cache.data[ectx->hdr_cache.dqt_luma_offs + 1];
	u32 pcfg = JPEG_DMI_ENABLE_UPLOAD;
	u32 addr = 0x00000000;
	u32 reg_val;
	int i;

	/* DMI upload start sequence */
	jpeg_io_write(ectx->jenc, JPEG_V420_REG_DMI_ADDR, addr);
	jpeg_io_write(ectx->jenc, JPEG_V420_REG_DMI_CFG, pcfg);

	/* DMI Luma upload - values are stored in zigzag order in hdr_cache */
	for (i = 0; i < ARRAY_SIZE(t81k1_dct_luma_table); i++) {
		reg_val = div_u64(U16_MAX + 1U, luma_qt[i]);
		reg_val = clamp_t(u32, reg_val, 0, U16_MAX);
		jpeg_io_write(ectx->jenc, JPEG_V420_REG_DMI_DATA, reg_val);
	}

	/* DMI Chroma upload - only present for color formats */
	if (ectx->hdr_cache.dqt_chroma_offs) {
		const u8 *chroma_qt = &ectx->hdr_cache.data[ectx->hdr_cache.dqt_chroma_offs + 1];

		for (i = 0; i < ARRAY_SIZE(t81k2_dct_chroma_table); i++) {
			reg_val = div_u64(U16_MAX + 1U, chroma_qt[i]);
			reg_val = clamp_t(u32, reg_val, 0, U16_MAX);
			jpeg_io_write(ectx->jenc, JPEG_V420_REG_DMI_DATA, reg_val);
		}
	}

	/* DMI upload end sequence */
	jpeg_io_write(ectx->jenc, JPEG_V420_REG_DMI_CFG, addr);

	ectx->quality_programmed = ectx->quality_requested;

	dev_dbg(ectx->dev, "quality_programmed=%d\n", ectx->quality_programmed);
}

static void jpeg_sync_sg(struct device *dev,
			 struct qcom_jpeg_buff *frame,
			 enum dma_data_direction direction, bool for_device)
{
	u8 pln;

	for (pln = 0; pln < QCOM_JPEG_MAX_PLANES; pln++) {
		struct sg_table *sgt = frame->plns[pln].sgt;

		if (!frame->plns[pln].dma || !sgt)
			break;

		if (for_device)
			dma_sync_sgtable_for_device(dev, sgt, direction);
		else
			dma_sync_sgtable_for_cpu(dev, sgt, direction);
	}
}

static int jpeg_init(struct qcom_jenc_dev *jenc)
{
	unsigned long rtime;
	u32 hw_ver;

	if (WARN_ON_ONCE(!jenc || !jenc->dev || !jenc->jpeg_base))
		return -EINVAL;

	jpeg_wo_bits(jenc, JPEG_V420_REG_INT_CLR, JPEG_V420_IRQ_STATUS_ALL_BITS, JPEG_SET_U32);

	reinit_completion(&jenc->reset_complete);
	jpeg_rw_bits(jenc, JPEG_V420_REG_INT_MASK,
		     JPEG_V420_IRQ_STATUS_RESET_ACK_MSK, JPEG_SET_U32);

	jpeg_wo_bits(jenc, JPEG_V420_REG_RESET_CMD,
		     JPEG_V420_RST_CMD_FE_RESET_MSK			|
		     JPEG_V420_RST_CMD_WE_RESET_MSK			|
		     JPEG_V420_RST_CMD_ENCODER_RESET_MSK		|
		     JPEG_V420_RST_CMD_BLOCK_FORMATTER_RST_MSK	|
		     JPEG_V420_RST_CMD_SCALE_RESET_MSK			|
		     JPEG_V420_RST_CMD_REGISTER_RESET_MSK		|
		     JPEG_V420_RST_CMD_MISR_RESET_MSK			|
		     JPEG_V420_RST_CMD_CORE_RESET_MSK			|
		     JPEG_V420_RST_CMD_JPEG_V420_DOMAIN_RESET_MSK,
		     JPEG_SET_U32);

	rtime = wait_for_completion_timeout(&jenc->reset_complete,
					    msecs_to_jiffies(JPEG_RESET_TIMEOUT_MS));
	if (!rtime) {
		dev_err(jenc->dev, "encoder HW reset timeout\n");
		return -ETIME;
	}

	hw_ver = jpeg_io_read(jenc, JPEG_V420_REG_HW_VERSION);
	dev_dbg(jenc->dev, "JPEG HW encoder version %d.%d.%d\n",
		jpeg_bits_get(JPEG_V420_HW_VER_MAJOR_MSK, hw_ver),
		jpeg_bits_get(JPEG_V420_HW_VER_MINOR_MSK, hw_ver),
		jpeg_bits_get(JPEG_V420_HW_VER_STEP_MSK, hw_ver));

	jpeg_wo_bits(jenc, JPEG_V420_REG_HW_CMD, JPEG_V420_CMD_CLR_RD_PLN0_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_RD_PLN1_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_RD_PLN2_QUEUE_MSK, JPEG_SET_U32);
	jpeg_wo_bits(jenc, JPEG_V420_REG_HW_CMD, JPEG_V420_CMD_CLR_RD_PLN0_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_RD_PLN1_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_RD_PLN2_QUEUE_MSK, JPEG_CLR_U32);

	jpeg_wo_bits(jenc, JPEG_V420_REG_HW_CMD, JPEG_V420_CMD_CLR_WR_PLN0_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_WR_PLN1_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_WR_PLN2_QUEUE_MSK, JPEG_SET_U32);
	jpeg_wo_bits(jenc, JPEG_V420_REG_HW_CMD, JPEG_V420_CMD_CLR_WR_PLN0_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_WR_PLN1_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_WR_PLN2_QUEUE_MSK, JPEG_CLR_U32);

	jpeg_wo_bits(jenc, JPEG_V420_REG_INT_CLR, JPEG_V420_IRQ_STATUS_ALL_BITS, JPEG_SET_U32);
	jpeg_wo_bits(jenc, JPEG_V420_REG_INT_MASK, JPEG_V420_IRQ_STATUS_ALL_BITS, JPEG_SET_U32);

	return 0;
}

static int jpeg_exec(struct qcom_jenc_dev *jenc)
{
	jpeg_wo_bits(jenc, JPEG_V420_REG_HW_CMD, JPEG_V420_CMD_HW_START_MSK, 1);

	return 0;
}

static void jpeg_stop(struct qcom_jenc_dev *jenc)
{
	jpeg_wo_bits(jenc, JPEG_V420_REG_HW_CMD, JPEG_V420_CMD_HW_START_MSK, 0);

	jpeg_wo_bits(jenc, JPEG_V420_REG_HW_CMD, JPEG_V420_CMD_CLR_RD_PLN0_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_RD_PLN1_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_RD_PLN2_QUEUE_MSK, JPEG_SET_U32);
	jpeg_wo_bits(jenc, JPEG_V420_REG_HW_CMD, JPEG_V420_CMD_CLR_RD_PLN0_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_RD_PLN1_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_RD_PLN2_QUEUE_MSK, JPEG_CLR_U32);

	jpeg_wo_bits(jenc, JPEG_V420_REG_HW_CMD, JPEG_V420_CMD_CLR_WR_PLN0_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_WR_PLN1_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_WR_PLN2_QUEUE_MSK, JPEG_SET_U32);
	jpeg_wo_bits(jenc, JPEG_V420_REG_HW_CMD, JPEG_V420_CMD_CLR_WR_PLN0_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_WR_PLN1_QUEUE_MSK |
		     JPEG_V420_CMD_CLR_WR_PLN2_QUEUE_MSK, JPEG_CLR_U32);
}

static int jpeg_deinit(struct qcom_jenc_dev *jenc)
{
	unsigned long rtime;

	jpeg_wo_bits(jenc, JPEG_V420_REG_INT_CLR, JPEG_V420_IRQ_STATUS_ALL_BITS, JPEG_SET_U32);

	reinit_completion(&jenc->stop_complete);
	jpeg_rw_bits(jenc, JPEG_V420_REG_INT_MASK, JPEG_V420_IRQ_STATUS_STOP_ACK_MSK, JPEG_SET_U32);

	jpeg_wo_bits(jenc, JPEG_V420_REG_HW_CMD, JPEG_V420_CMD_HW_STOP_MSK, 1);

	rtime = wait_for_completion_timeout(&jenc->stop_complete,
					    msecs_to_jiffies(JPEG_STOP_TIMEOUT_MS));
	jpeg_wo_bits(jenc, JPEG_V420_REG_INT_MASK, JPEG_V420_IRQ_STATUS_ALL_BITS, JPEG_CLR_U32);
	jpeg_wo_bits(jenc, JPEG_V420_REG_INT_CLR, JPEG_V420_IRQ_STATUS_ALL_BITS, JPEG_SET_U32);

	if (!rtime) {
		dev_err(jenc->dev, "encoder HW stop timeout\n");
		return -ETIME;
	}

	return 0;
}

static int jpeg_apply_fe_addr(struct jenc_context *ectx, struct qcom_jenc_queue *q,
			      struct vb2_buffer *vb)
{
	struct qcom_jpeg_buff *frame = &q->buff[vb->index];
	struct v4l2_pix_format_mplane *fmt = &q->vf;
	struct qcom_jenc_dev *jenc = ectx->jenc;
	unsigned long flags;
	u8 pln;

	jpeg_sync_sg(jenc->dev, frame, DMA_TO_DEVICE, true);

	for (pln = 0; pln < fmt->num_planes; pln++) {
		if (!frame->plns[pln].sgt || !frame->plns[pln].sgt->sgl)
			break;

		jpeg_io_write(jenc, jpeg_v420_fe_pntr[pln], frame->plns[pln].dma);
		jpeg_io_write(jenc, jpeg_v420_fe_offs[pln], 0);

		dev_dbg(jenc->dev, "FE pln=%d addr=%pad idx:%d\n",
			pln, &frame->plns[pln].dma, vb->index);
	}

	spin_lock_irqsave(&jenc->hw_lock, flags);
	q->buff_id = vb->index;
	spin_unlock_irqrestore(&jenc->hw_lock, flags);

	return 0;
}

static int jpeg_store_fe_next(struct jenc_context *ectx, struct vb2_buffer *vb2)
{
	struct qcom_jenc_queue *q = &ectx->bufq[TYPE2QID(vb2->type)];
	struct qcom_jpeg_buff *buff = &q->buff[vb2->index];
	u8 pln;

	for (pln = 0; pln < q->vf.num_planes; pln++) {
		buff->plns[pln].sgt = vb2_dma_sg_plane_desc(vb2, pln);
		if (!buff->plns[pln].sgt || !buff->plns[pln].sgt->sgl)
			return -EINVAL;

		buff->plns[pln].dma = sg_dma_address(buff->plns[pln].sgt->sgl);
		if (!buff->plns[pln].dma)
			return -EINVAL;

		buff->plns[pln].size = vb2_plane_size(vb2, pln);
		if (!buff->plns[pln].size)
			return -EINVAL;
	}

	return 0;
}

static int jpeg_setup_fe_size(struct jenc_context *ectx, struct qcom_jenc_queue *q)
{
	struct v4l2_pix_format_mplane *sfmt = &q->vf;
	struct qcom_jenc_dev *jenc = ectx->jenc;
	u8 pln;

	for (pln = 0; pln < QCOM_JPEG_MAX_PLANES; pln++) {
		jpeg_rw_bits(jenc, jpeg_v420_fe_bsize[pln],
			     JPEG_V420_PLN_RD_BUFF_SIZE_WIDTH_MSK, 0);
		jpeg_rw_bits(jenc, jpeg_v420_fe_bsize[pln],
			     JPEG_V420_PLN_RD_BUFF_SIZE_HEIGHT_MSK, 0);
		jpeg_rw_bits(jenc, jpeg_v420_fe_stride[pln], JPEG_V420_PLN_RD_STRIDE_STRIDE_MSK, 0);
	}

	for (pln = 0; pln < sfmt->num_planes; pln++) {
		jpeg_rw_bits(jenc, jpeg_v420_fe_bsize[pln], JPEG_V420_PLN_RD_BUFF_SIZE_WIDTH_MSK,
			     sfmt->width  - 1);
		jpeg_rw_bits(jenc, jpeg_v420_fe_bsize[pln], JPEG_V420_PLN_RD_BUFF_SIZE_HEIGHT_MSK,
			     sfmt->height  - 1);
		jpeg_rw_bits(jenc, jpeg_v420_fe_stride[pln], JPEG_V420_PLN_RD_STRIDE_STRIDE_MSK,
			     sfmt->plane_fmt[pln].bytesperline);

		dev_dbg(ectx->dev, "FE pln=%d width=%d height=%d stride=%d\n",
			pln,
			jpeg_rd_bits(jenc, jpeg_v420_fe_bsize[pln],
				     JPEG_V420_PLN_RD_BUFF_SIZE_WIDTH_MSK),
			jpeg_rd_bits(jenc, jpeg_v420_fe_bsize[pln],
				     JPEG_V420_PLN_RD_BUFF_SIZE_HEIGHT_MSK),
			jpeg_rd_bits(jenc, jpeg_v420_fe_stride[pln],
				     JPEG_V420_PLN_RD_STRIDE_STRIDE_MSK));
	}

	return 0;
}

static int jpeg_setup_fe_hinit(struct jenc_context *ectx, struct qcom_jenc_queue *q)
{
	struct qcom_jenc_dev *jenc = ectx->jenc;
	u8 pln;

	for (pln = 0; pln < QCOM_JPEG_MAX_PLANES; pln++)
		jpeg_io_write(jenc, jpeg_v420_fe_hinit[pln], 0);

	return 0;
}

static int jpeg_setup_fe_vinit(struct jenc_context *ectx, struct qcom_jenc_queue *q)
{
	struct qcom_jenc_dev *jenc = ectx->jenc;
	u8 pln;

	for (pln = 0; pln < QCOM_JPEG_MAX_PLANES; pln++)
		jpeg_io_write(jenc, jpeg_v420_fe_vinit[pln], 0);

	return 0;
}

static int jpeg_setup_fe_params(struct jenc_context *ectx, struct qcom_jenc_queue *q)
{
	struct v4l2_pix_format_mplane *sfmt = &q->vf;
	struct qcom_jenc_dev *jenc = ectx->jenc;
	u8 expected_planes, pln;
	int rval;

	jpeg_rw_bits(jenc, JPEG_V420_REG_FE_CFG, JPEG_V420_FE_CFG_MAL_EN_MSK, 1);
	jpeg_rw_bits(jenc, JPEG_V420_REG_FE_CFG, JPEG_V420_FE_CFG_BOTTOM_VPAD_EN_MSK, 1);

	rval = jpeg_get_memory_fmt(sfmt->pixelformat);
	if (rval < 0) {
		dev_err(ectx->dev, "invalid memory format for v4l2 format: %p4cc\n",
			&sfmt->pixelformat);
		return -EINVAL;
	}

	switch (rval) {
	case JPEG_MEM_FMT_MONO:
		expected_planes = 1;
		break;
	case JPEG_MEM_FMT_PPLANAR:
		expected_planes = 2;
		break;
	case JPEG_MEM_FMT_PLANAR:
		expected_planes = 3;
		break;
	default:
		return -EINVAL;
	}

	if (sfmt->num_planes != expected_planes) {
		dev_err(ectx->dev, "plane mismatch fmt=%u expected=%u got=%u\n",
			rval, expected_planes, sfmt->num_planes);
		return -EINVAL;
	}

	jpeg_rw_bits(jenc, JPEG_V420_REG_FE_CFG, JPEG_V420_FE_CFG_MEMORY_FORMAT_MSK, rval);

	jpeg_rw_bits(jenc, JPEG_V420_REG_FE_CFG, JPEG_V420_FE_CFG_PLN0_EN_MSK, 0);
	jpeg_rw_bits(jenc, JPEG_V420_REG_FE_CFG, JPEG_V420_FE_CFG_PLN1_EN_MSK, 0);
	jpeg_rw_bits(jenc, JPEG_V420_REG_FE_CFG, JPEG_V420_FE_CFG_PLN2_EN_MSK, 0);

	/* Scaling is not supported; always configure for 1:1 */
	jpeg_rw_bits(jenc, JPEG_V420_REG_FE_CFG, JPEG_V420_FE_CFG_SIXTEEN_MCU_EN_MSK, 1);
	jpeg_rw_bits(jenc, JPEG_V420_REG_FE_CFG, JPEG_V420_FE_CFG_MCUS_PER_BLOCK_MSK, 0);

	dev_dbg(ectx->dev, "sixteen MCU enabled=%d, %d MCU per blocks\n",
		jpeg_rd_bits(jenc, JPEG_V420_REG_FE_CFG, JPEG_V420_FE_CFG_SIXTEEN_MCU_EN_MSK),
		jpeg_rd_bits(jenc, JPEG_V420_REG_FE_CFG, JPEG_V420_FE_CFG_MCUS_PER_BLOCK_MSK));

	rval = jpeg_get_mal_boundary(sfmt->width, jpeg_mal_bounds, ARRAY_SIZE(jpeg_mal_bounds));
	if (rval < 0) {
		dev_err(ectx->dev, "failed to get FE mal boundary width=%u\n",
			sfmt->width);
		return -EINVAL;
	}
	jpeg_rw_bits(jenc, JPEG_V420_REG_FE_CFG, JPEG_V420_FE_CFG_MAL_BOUNDARY_MSK, rval);

	dev_dbg(ectx->dev, "optimal FE mal boundary=%d\n",
		jpeg_rd_bits(jenc, JPEG_V420_REG_FE_CFG, JPEG_V420_FE_CFG_MAL_BOUNDARY_MSK));

	rval = jpeg_get_encode_fmt(sfmt->pixelformat);
	if (rval < 0) {
		dev_err(ectx->dev, "unsupported encode format %p4cc\n",
			&sfmt->pixelformat);
		return -EINVAL;
	}

	switch (rval) {
	case JPEG_ENCODE_MONO:
	case JPEG_ENCODE_H1V1:
	case JPEG_ENCODE_H2V1:
		jpeg_rw_bits(jenc, JPEG_V420_REG_FE_VBPAD_CFG, JPEG_V420_FE_VBPAD_CFG_BLOCK_ROW_MSK,
			     DIV_ROUND_UP(sfmt->height, JPEG_MCU_BLOCK_8));
		break;
	case JPEG_ENCODE_H1V2:
	case JPEG_ENCODE_H2V2:
		jpeg_rw_bits(jenc, JPEG_V420_REG_FE_VBPAD_CFG, JPEG_V420_FE_VBPAD_CFG_BLOCK_ROW_MSK,
			     DIV_ROUND_UP(sfmt->height, JPEG_MCU_BLOCK_16));
		break;
	default:
		dev_err(ectx->dev, "unsupported encode format: %d\n", rval);
		return -EINVAL;
	}

	dev_dbg(ectx->dev, "FE vpad config=%d\n",
		jpeg_rd_bits(jenc, JPEG_V420_REG_FE_VBPAD_CFG,
			     JPEG_V420_FE_VBPAD_CFG_BLOCK_ROW_MSK));

	if (sfmt->pixelformat == V4L2_PIX_FMT_NV21M || sfmt->pixelformat == V4L2_PIX_FMT_NV61M)
		jpeg_rw_bits(jenc, JPEG_V420_REG_FE_CFG, JPEG_V420_FE_CFG_CBCR_ORDER_MSK, 1);
	else
		jpeg_rw_bits(jenc, JPEG_V420_REG_FE_CFG, JPEG_V420_FE_CFG_CBCR_ORDER_MSK, 0);

	for (pln = 0; pln < sfmt->num_planes; pln++) {
		if (sfmt->width && sfmt->height) {
			switch (pln) {
			case 0:
				jpeg_rw_bits(jenc, JPEG_V420_REG_FE_CFG,
					     JPEG_V420_FE_CFG_PLN0_EN_MSK, 1);
				break;
			case 1:
				jpeg_rw_bits(jenc, JPEG_V420_REG_FE_CFG,
					     JPEG_V420_FE_CFG_PLN1_EN_MSK, 1);
				break;
			case 2:
				jpeg_rw_bits(jenc, JPEG_V420_REG_FE_CFG,
					     JPEG_V420_FE_CFG_PLN2_EN_MSK, 1);
				break;
			}
		}
	}

	jpeg_rw_bits(jenc, JPEG_V420_REG_CORE_CFG, JPEG_V420_CORE_CFG_FE_ENABLE_MSK, 1);

	return 0;
}

static int jpeg_setup_fe(struct jenc_context *ectx, struct qcom_jenc_queue *q)
{
	int rc;

	rc = jpeg_setup_fe_size(ectx, q);
	if (rc)
		return rc;

	rc = jpeg_setup_fe_hinit(ectx, q);
	if (rc)
		return rc;

	rc = jpeg_setup_fe_vinit(ectx, q);
	if (rc)
		return rc;

	rc = jpeg_setup_fe_params(ectx, q);
	if (rc)
		return rc;

	return 0;
}

static int jpeg_ensure_header_cache(struct jenc_context *ectx)
{
	struct qcom_jenc_queue *sq = &ectx->bufq[JENC_SRC_QUEUE];
	int rc;

	if (ectx->hdr_cache.size)
		return 0;

	rc = qcom_jenc_header_init(&ectx->hdr_cache, sq->vf.pixelformat);
	if (rc) {
		dev_err(ectx->dev, "JFIF header lazy init failed\n");
		return rc;
	}

	return 0;
}

static int jpeg_apply_we_addr(struct jenc_context *ectx, struct qcom_jenc_queue *q,
			      struct vb2_buffer *vb)
{
	struct qcom_jpeg_buff *frame = &q->buff[vb->index];
	struct qcom_jenc_dev *jenc = ectx->jenc;
	void *mptr = vb2_plane_vaddr(vb, 0);
	dma_addr_t dma = frame->plns[0].dma;
	unsigned long flags;
	size_t header_size;
	int rc;
	u8 pln = 0;

	if (WARN_ON_ONCE(!dma))
		return -EPERM;

	if (WARN_ON_ONCE(!mptr))
		return -EPERM;

	rc = jpeg_ensure_header_cache(ectx);
	if (rc)
		return rc;

	/* Update DQT cache if quality changed, then upload to HW DMI. */
	mutex_lock(&ectx->quality_mutex);
	if (!ectx->hdr_cache.size || ectx->quality_programmed != ectx->quality_requested)
		jpeg_update_dqt_cache(ectx);
	jpeg_upload_dmi_table(ectx);
	mutex_unlock(&ectx->quality_mutex);

	/*
	 * Invalidate stale CPU cache lines before writing the JPEG header
	 * with the CPU into the destination buffer.
	 */
	jpeg_sync_sg(jenc->dev, frame, DMA_BIDIRECTIONAL, false);

	header_size = min_t(size_t, vb->planes[0].length, ectx->hdr_cache.size);
	header_size = qcom_jenc_header_emit(&ectx->hdr_cache, mptr,
					    (u32)header_size, (u16)q->vf.width,
					    (u16)q->vf.height);

	dma += header_size;
	qcom_jenc_dqts_emit(&ectx->hdr_cache, mptr);

	/*
	 * Flush CPU writes to the header before handing the buffer to the
	 * hardware DMA engine.
	 */
	jpeg_sync_sg(jenc->dev, frame, DMA_BIDIRECTIONAL, true);

	jpeg_io_write(jenc, jpeg_v420_we_pntr[pln], dma);

	dev_dbg(jenc->dev, "WE pln=%d addr=%pad idx:%d\n",
		pln, &dma, vb->index);

	spin_lock_irqsave(&jenc->hw_lock, flags);
	q->buff_id = vb->index;
	spin_unlock_irqrestore(&jenc->hw_lock, flags);

	return 0;
}

static int jpeg_store_we_next(struct jenc_context *ectx, struct vb2_buffer *vb2)
{
	struct qcom_jenc_queue *q = &ectx->bufq[TYPE2QID(vb2->type)];
	struct qcom_jpeg_buff *frame = &q->buff[vb2->index];
	struct sg_table *sgt;
	dma_addr_t dma;

	sgt = vb2_dma_sg_plane_desc(vb2, 0);
	if (!sgt || !sgt->sgl)
		return -EINVAL;

	dma = sg_dma_address(sgt->sgl);
	if (!dma)
		return -EINVAL;

	if (!vb2_plane_vaddr(vb2, 0))
		return -EINVAL;

	frame->plns[0].sgt = sgt;
	frame->plns[0].dma = dma;
	frame->plns[0].size = vb2_plane_size(vb2, 0);

	return 0;
}

static int jpeg_setup_we_size(struct jenc_context *ectx, struct qcom_jenc_queue *q)
{
	struct v4l2_pix_format_mplane *dfmt = &q->vf;
	struct qcom_jenc_dev *jenc = ectx->jenc;
	u8 pln;

	for (pln = 0; pln < QCOM_JPEG_MAX_PLANES; pln++)
		jpeg_rw_bits(jenc, jpeg_v420_we_stride[pln], JPEG_V420_PLN_WR_STRIDE_STRIDE_MSK, 0);

	jpeg_io_write(jenc, JPEG_V420_REG_WE_BSIZE_0, dfmt->plane_fmt[0].sizeimage);

	dev_dbg(ectx->dev, "WE size=%u\n", dfmt->plane_fmt[0].sizeimage);

	return 0;
}

static int jpeg_setup_we_hinit(struct jenc_context *ectx, struct qcom_jenc_queue *q)
{
	struct v4l2_pix_format_mplane *dfmt = &q->vf;
	struct qcom_jenc_dev *jenc = ectx->jenc;
	u8 pln;

	if (!dfmt->width) {
		dev_err(ectx->dev, "invalid destination width=%d\n", dfmt->width);
		return -EINVAL;
	}

	for (pln = 0; pln < QCOM_JPEG_MAX_PLANES; pln++) {
		jpeg_rw_bits(jenc, jpeg_v420_we_hinit[pln], JPEG_V420_PLN_WR_HINIT_INTEGER_MSK, 0);
		jpeg_rw_bits(jenc, jpeg_v420_we_hstep[pln], JPEG_V420_PLN_WR_HSTEP_INTEGER_MSK, 0);
	}

	jpeg_rw_bits(jenc, JPEG_V420_REG_WE_HSTEP_0,
		     JPEG_V420_PLN_WR_HSTEP_INTEGER_MSK, dfmt->width);

	dev_dbg(ectx->dev, "WE hstep=%u\n",
		jpeg_rd_bits(jenc, JPEG_V420_REG_WE_HSTEP_0, JPEG_V420_PLN_WR_HSTEP_INTEGER_MSK));

	return 0;
}

static int jpeg_setup_we_vinit(struct jenc_context *ectx, struct qcom_jenc_queue *q)
{
	struct v4l2_pix_format_mplane *dfmt = &q->vf;
	struct qcom_jenc_dev *jenc = ectx->jenc;
	u8 pln;

	if (!dfmt->height) {
		dev_err(ectx->dev, "invalid destination height=%d\n", dfmt->height);
		return -EINVAL;
	}

	for (pln = 0; pln < QCOM_JPEG_MAX_PLANES; pln++) {
		jpeg_rw_bits(jenc, jpeg_v420_we_vinit[pln], JPEG_V420_PLN_WR_VINIT_INTEGER_MSK, 0);
		jpeg_rw_bits(jenc, jpeg_v420_we_vstep[pln], JPEG_V420_PLN_WR_VSTEP_INTEGER_MSK, 0);
	}

	jpeg_rw_bits(jenc, JPEG_V420_REG_WE_VSTEP_0,
		     JPEG_V420_PLN_WR_VSTEP_INTEGER_MSK, dfmt->height);

	dev_dbg(ectx->dev, "WE vstep=%u\n",
		jpeg_rd_bits(jenc, JPEG_V420_REG_WE_VSTEP_0, JPEG_V420_PLN_WR_VSTEP_INTEGER_MSK));

	return 0;
}

static int jpeg_setup_we_params(struct jenc_context *ectx, struct qcom_jenc_queue *q)
{
	struct v4l2_pix_format_mplane *dfmt = &q->vf;
	struct qcom_jenc_dev *jenc = ectx->jenc;
	u32 blk_w = 0, blk_h = 0, mcu_cols = 0, mcu_rows = 0;
	int rval;

	rval = jpeg_get_memory_fmt(dfmt->pixelformat);
	if (rval < 0) {
		dev_err(ectx->dev, "invalid memory format for v4l2 format: %p4cc\n",
			&dfmt->pixelformat);
		return -EINVAL;
	}
	jpeg_rw_bits(jenc, JPEG_V420_REG_WE_CFG, JPEG_V420_WE_CFG_MEMORY_FORMAT_MSK, rval);

	rval = jpeg_get_mal_boundary(dfmt->width, jpeg_mal_bounds, ARRAY_SIZE(jpeg_mal_bounds));
	if (rval < 0) {
		dev_err(ectx->dev, "failed to get WE mal boundary width=%u\n",
			dfmt->width);
		return -EINVAL;
	}
	jpeg_rw_bits(jenc, JPEG_V420_REG_WE_CFG, JPEG_V420_WE_CFG_MAL_BOUNDARY_MSK, rval);

	dev_dbg(ectx->dev, "optimal WE mal boundary=%d\n",
		jpeg_rd_bits(jenc, JPEG_V420_REG_WE_CFG, JPEG_V420_WE_CFG_MAL_BOUNDARY_MSK));

	rval = jpeg_get_encode_fmt(dfmt->pixelformat);
	if (rval < 0) {
		dev_err(ectx->dev, "unsupported encode format %p4cc\n",
			&dfmt->pixelformat);
		return rval;
	}

	rval = jpeg_get_mcu_geometry(rval, dfmt->width, dfmt->height, &blk_w, &blk_h,
				     &mcu_cols, &mcu_rows);
	if (rval < 0) {
		dev_err(ectx->dev, "invalid MCU geometry mcu_cols=%d mcu_rows=%d\n",
			mcu_cols, mcu_rows);
		return rval;
	}

	dev_dbg(ectx->dev, "WE blk_w=%u blk_h=%u cols=%u rows=%u\n",
		blk_w, blk_h, mcu_cols, mcu_rows);

	jpeg_rw_bits(jenc, JPEG_V420_REG_WE_BLOCKS_0,
		     JPEG_V420_PLN_WR_BLK_CFG_BLOCKS_PER_ROW_MSK, mcu_rows - 1);
	jpeg_rw_bits(jenc, JPEG_V420_REG_WE_BLOCKS_0,
		     JPEG_V420_PLN_WR_BLK_CFG_BLOCKS_PER_COL_MSK, mcu_cols - 1);

	jpeg_rw_bits(jenc, JPEG_V420_REG_WE_CFG, JPEG_V420_WE_CFG_CBCR_ORDER_MSK, 1);
	jpeg_rw_bits(jenc, JPEG_V420_REG_WE_CFG, JPEG_V420_WE_CFG_MAL_EN_MSK, 1);
	jpeg_rw_bits(jenc, JPEG_V420_REG_WE_CFG, JPEG_V420_WE_CFG_POP_BUFF_ON_EOS_MSK, 1);
	jpeg_rw_bits(jenc, JPEG_V420_REG_WE_CFG, JPEG_V420_WE_CFG_PLN0_EN_MSK, 1);

	jpeg_rw_bits(jenc, JPEG_V420_REG_CORE_CFG, JPEG_V420_CORE_CFG_MODE_MSK, 1);
	jpeg_rw_bits(jenc, JPEG_V420_REG_CORE_CFG, JPEG_V420_CORE_CFG_WE_ENABLE_MSK, 1);

	return 0;
}

static int jpeg_setup_we(struct jenc_context *ectx, struct qcom_jenc_queue *q)
{
	int rc;

	rc = jpeg_setup_we_size(ectx, q);
	if (rc)
		return rc;

	rc = jpeg_setup_we_hinit(ectx, q);
	if (rc)
		return rc;

	rc = jpeg_setup_we_vinit(ectx, q);
	if (rc)
		return rc;

	return jpeg_setup_we_params(ectx, q);
}

static int jpeg_setup_scale(struct jenc_context *ectx)
{
	struct qcom_jenc_queue *sq = &ectx->bufq[JENC_SRC_QUEUE];
	struct qcom_jenc_queue *dq = &ectx->bufq[JENC_DST_QUEUE];
	struct v4l2_pix_format_mplane *sfmt = &sq->vf;
	struct v4l2_pix_format_mplane *dfmt = &dq->vf;
	struct qcom_jenc_dev *jenc = ectx->jenc;
	u32 blk_w = 0, blk_h = 0, mcu_cols = 0, mcu_rows = 0;
	int rval;
	u8 pln;

	jpeg_rw_bits(jenc, JPEG_V420_REG_RESET_CMD, JPEG_V420_RST_CMD_SCALE_RESET_MSK, 1);

	/* explicit no scaling */
	jpeg_rw_bits(jenc, JPEG_V420_REG_SCALE_CFG, JPEG_V420_SCALE_CFG_HSCALE_ENABLE_MSK, 0);
	jpeg_rw_bits(jenc, JPEG_V420_REG_SCALE_CFG, JPEG_V420_SCALE_CFG_VSCALE_ENABLE_MSK, 0);

	for (pln = 0; pln < QCOM_JPEG_MAX_PLANES; pln++) {
		jpeg_io_write(jenc, jpeg_v420_scale_hstep[pln], JPEG_DEFAULT_SCALE_STEP);
		jpeg_io_write(jenc, jpeg_v420_scale_vstep[pln], JPEG_DEFAULT_SCALE_STEP);
	}

	rval = jpeg_get_encode_fmt(sfmt->pixelformat);
	if (rval < 0) {
		dev_err(ectx->dev, "unsupported encode format %p4cc\n",
			&sfmt->pixelformat);
		return -EINVAL;
	}

	rval = jpeg_get_mcu_geometry(rval, dfmt->width, dfmt->height, &blk_w, &blk_h,
				     &mcu_cols, &mcu_rows);
	if (rval < 0) {
		dev_err(ectx->dev, "invalid MCU geometry blk_w=%d blk_h=%d\n",
			blk_w, blk_h);
		return -EINVAL;
	}

	dev_dbg(ectx->dev, "scale blk_w=%u blk_h=%u cols=%u rows=%u\n",
		blk_w, blk_h, mcu_cols, mcu_rows);

	for (pln = 0; pln < sq->vf.num_planes; pln++) {
		jpeg_rw_bits(jenc, jpeg_v420_scale_out_cfg[pln],
			     JPEG_V420_SCALE_OUT_CFG_BLOCK_WIDTH_MSK, blk_w - 1);
		jpeg_rw_bits(jenc, jpeg_v420_scale_out_cfg[pln],
			     JPEG_V420_SCALE_OUT_CFG_BLOCK_HEIGHT_MSK, blk_h - 1);
	}

	dev_dbg(ectx->dev, "scale src=%ux%u dst=%ux%u enable=%d/%d\n",
		sfmt->width, sfmt->height, dfmt->width, dfmt->height,
		jpeg_rd_bits(jenc, JPEG_V420_REG_SCALE_CFG, JPEG_V420_SCALE_CFG_HSCALE_ENABLE_MSK),
		jpeg_rd_bits(jenc, JPEG_V420_REG_SCALE_CFG, JPEG_V420_SCALE_CFG_VSCALE_ENABLE_MSK));

	/* Disabled, but must be configured */
	jpeg_rw_bits(jenc, JPEG_V420_REG_CORE_CFG, JPEG_V420_CORE_CFG_SCALE_ENABLE_MSK, 0);

	return 0;
}

static int jpeg_setup_encode(struct jenc_context *ectx)
{
	struct qcom_jenc_queue *sq = &ectx->bufq[JENC_SRC_QUEUE];
	struct v4l2_pix_format_mplane *sfmt = &sq->vf;
	struct qcom_jenc_dev *jenc = ectx->jenc;
	u32 blk_w = 0, blk_h = 0, mcu_cols = 0, mcu_rows = 0;
	int rval;

	if (!sfmt->width || !sfmt->height)
		return -EINVAL;

	jpeg_rw_bits(jenc, JPEG_V420_REG_RESET_CMD, JPEG_V420_RST_CMD_ENCODER_RESET_MSK, 1);

	rval = jpeg_get_encode_fmt(sfmt->pixelformat);
	if (rval < 0) {
		dev_err(ectx->dev, "unsupported encode format %p4cc\n",
			&sfmt->pixelformat);
		return -EINVAL;
	}
	jpeg_rw_bits(jenc, JPEG_V420_REG_ENC_CFG, JPEG_V420_ENC_CFG_IMAGE_FORMAT_MSK, rval);

	rval = jpeg_get_mcu_geometry(rval, sfmt->width, sfmt->height, &blk_w, &blk_h,
				     &mcu_cols, &mcu_rows);
	if (rval < 0) {
		dev_err(ectx->dev, "invalid MCU geometry mcu_cols=%d mcu_rows=%d\n",
			mcu_cols, mcu_rows);
		return -EINVAL;
	}

	dev_dbg(ectx->dev, "encode blk_w=%u blk_h=%u cols=%u rows=%u\n",
		blk_w, blk_h, mcu_cols, mcu_rows);

	jpeg_rw_bits(jenc, JPEG_V420_REG_ENC_IMG_SIZE,
		     JPEG_V420_ENC_IMG_SIZE_ENCODE_WIDTH_MSK, mcu_cols - 1);
	jpeg_rw_bits(jenc, JPEG_V420_REG_ENC_IMG_SIZE,
		     JPEG_V420_ENC_IMG_SIZE_ENCODE_HEIGHT_MSK, mcu_rows - 1);

	dev_dbg(ectx->dev, "encode width=%d height=%d\n",
		jpeg_rd_bits(jenc, JPEG_V420_REG_ENC_IMG_SIZE,
			     JPEG_V420_ENC_IMG_SIZE_ENCODE_WIDTH_MSK),
		jpeg_rd_bits(jenc, JPEG_V420_REG_ENC_IMG_SIZE,
			     JPEG_V420_ENC_IMG_SIZE_ENCODE_HEIGHT_MSK));

	jpeg_rw_bits(jenc, JPEG_V420_REG_ENC_CFG, JPEG_V420_ENC_CFG_APPLY_EOI_MSK, 1);
	jpeg_rw_bits(jenc, JPEG_V420_REG_CORE_CFG, JPEG_V420_CORE_CFG_ENC_ENABLE_MSK, 1);

	return 0;
}

static irqreturn_t op_jpeg_irq_bot(int irq, void *data)
{
	struct qcom_jenc_dev *jenc = data;
	unsigned long flags;
	u32 irq_status;
	u32 irq_mask;

	irq_status = READ_ONCE(jenc->pending_irq_status);

	irq_mask = JPEG_V420_IRQ_STATUS_SESSION_DONE_MSK;
	if (jpeg_bits_get(irq_mask, irq_status)) {
		struct jenc_context *ctx;

		spin_lock_irqsave(&jenc->hw_lock, flags);
		ctx = jenc->actx;
		jenc->actx = NULL;
		spin_unlock_irqrestore(&jenc->hw_lock, flags);

		if (ctx && ctx->bufq[JENC_DST_QUEUE].buff_id >= 0) {
			struct qcom_jenc_queue *dq = &ctx->bufq[JENC_DST_QUEUE];
			struct qcom_jpeg_buff *frame = &dq->buff[dq->buff_id];
			size_t out_size = jpeg_rd_bits(jenc, JPEG_V420_REG_ENC_OUT_SIZE,
						       JPEG_V420_STATUS_OUT_SIZE_BYTES_MSK);

			dev_dbg(jenc->dev, "complete idx:%d addr=%pad size=%zu\n",
				dq->buff_id, &frame->plns[0].dma, out_size);

			jenc->enc_hw_irq_cb(ctx, VB2_BUF_STATE_DONE,
					    out_size + ctx->hdr_cache.size);
		}

		jpeg_stop(jenc);
	}

	irq_mask = JPEG_V420_IRQ_STATUS_SESION_ERROR_MSK;
	if (jpeg_bits_get(irq_mask, irq_status)) {
		struct jenc_context *ctx;

		spin_lock_irqsave(&jenc->hw_lock, flags);
		ctx = jenc->actx;
		jenc->actx = NULL;
		spin_unlock_irqrestore(&jenc->hw_lock, flags);

		dev_err(jenc->dev, "encoder hardware failure=0x%x\n",
			jpeg_bits_get(irq_mask, irq_status));
		if (ctx)
			jenc->enc_hw_irq_cb(ctx, VB2_BUF_STATE_ERROR, 0);

		jpeg_stop(jenc);
	}

	return IRQ_HANDLED;
}

static irqreturn_t op_jpeg_irq_top(int irq, void *data)
{
	struct qcom_jenc_dev *jenc = data;
	unsigned long flags;
	u32 irq_status;
	u32 irq_mask;

	spin_lock_irqsave(&jenc->hw_lock, flags);

	irq_status = jpeg_io_read(jenc, JPEG_V420_REG_INT_STATUS);
	jpeg_wo_bits(jenc, JPEG_V420_REG_INT_CLR, JPEG_V420_IRQ_STATUS_ALL_BITS, irq_status);

	irq_mask = JPEG_V420_IRQ_STATUS_RESET_ACK_MSK;
	if (jpeg_bits_get(irq_mask, irq_status)) {
		complete(&jenc->reset_complete);
		spin_unlock_irqrestore(&jenc->hw_lock, flags);
		return IRQ_HANDLED;
	}

	irq_mask = JPEG_V420_IRQ_STATUS_STOP_ACK_MSK;
	if (jpeg_bits_get(irq_mask, irq_status)) {
		complete(&jenc->stop_complete);
		dev_dbg(jenc->dev, "hardware stop acknowledged\n");
		spin_unlock_irqrestore(&jenc->hw_lock, flags);
		return IRQ_HANDLED;
	}

	WRITE_ONCE(jenc->pending_irq_status, irq_status);

	spin_unlock_irqrestore(&jenc->hw_lock, flags);

	return IRQ_WAKE_THREAD;
}

static void op_jpeg_get_hw_caps(struct qcom_jenc_dev *jenc, u32 *caps)
{
	u32 hw_caps;

	hw_caps = jpeg_io_read(jenc, JPEG_V420_REG_HW_CAPABILITY);
	dev_dbg(jenc->dev, "CAPS: encode=%d decode=%d upscale=%d downscale=%d\n",
		jpeg_bits_get(JPEG_V420_HW_CAP_ENCODE_MSK, hw_caps),
		jpeg_bits_get(JPEG_V420_HW_CAP_DECODE_MSK, hw_caps),
		jpeg_bits_get(JPEG_V420_HW_CAP_UPSCALE_MSK, hw_caps),
		jpeg_bits_get(JPEG_V420_HW_CAP_DOWNSCALE_MSK, hw_caps));

	*caps = hw_caps;
}

static struct qcom_jenc_queue *op_jpeg_get_buff_queue(struct jenc_context *ectx,
						      enum qcom_enc_qid id)
{
	return &ectx->bufq[id];
}

static int op_jpeg_queue_setup(struct jenc_context *ectx, enum qcom_enc_qid id)
{
	int rc;

	if (id == JENC_SRC_QUEUE) {
		struct qcom_jenc_queue *q = &ectx->bufq[id];

		rc = qcom_jenc_header_init(&ectx->hdr_cache, q->vf.pixelformat);
		if (rc) {
			dev_err(ectx->dev, "JFIF header init failed\n");
			return rc;
		}
	}

	return 0;
}

static int op_jpeg_src_fmt_update(struct jenc_context *ectx, u32 old_fourcc, u32 new_fourcc)
{
	bool old_is_mono = (old_fourcc == V4L2_PIX_FMT_GREY);
	bool new_is_mono = (new_fourcc == V4L2_PIX_FMT_GREY);
	int rc;

	/* header layout changes only for mono <-> color source format switch */
	if (old_is_mono == new_is_mono)
		return 0;

	rc = qcom_jenc_header_init(&ectx->hdr_cache, new_fourcc);
	if (rc) {
		dev_err(ectx->dev, "JFIF header reinit failed\n");
		return rc;
	}

	/* force DQT upload after source profile switch */
	mutex_lock(&ectx->quality_mutex);
	ectx->quality_programmed = 0;
	mutex_unlock(&ectx->quality_mutex);

	return 0;
}

static int op_jpeg_buffer_prepare(struct jenc_context *ectx, struct vb2_buffer *vb2)
{
	int rc;

	if (V4L2_TYPE_IS_OUTPUT(vb2->type)) {
		rc = jpeg_store_fe_next(ectx, vb2);
		if (rc)
			dev_err(ectx->dev, "cannot set up fetch engine address\n");
	} else {
		rc = jpeg_store_we_next(ectx, vb2);
		if (rc)
			dev_err(ectx->dev, "cannot set up write engine address\n");
	}

	return rc;
}

static int op_jpeg_process_exec(struct qcom_jenc_dev *jenc, struct jenc_context *ectx,
				struct vb2_buffer *vb)
{
	struct qcom_jenc_queue *sq = &ectx->bufq[JENC_SRC_QUEUE];
	struct qcom_jenc_queue *dq = &ectx->bufq[JENC_DST_QUEUE];
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&jenc->hw_lock, flags);
	jenc->actx = ectx;
	spin_unlock_irqrestore(&jenc->hw_lock, flags);

	if (V4L2_TYPE_IS_OUTPUT(vb->type)) {
		rc = jpeg_setup_fe(ectx, sq);
		if (rc)
			goto err_clear_ctx;

		rc = jpeg_apply_fe_addr(ectx, sq, vb);
		if (rc)
			goto err_clear_ctx;
	} else {
		rc = jpeg_setup_we(ectx, dq);
		if (rc)
			goto err_clear_ctx;

		rc = jpeg_apply_we_addr(ectx, dq, vb);
		if (rc)
			goto err_clear_ctx;
	}

	/* Both src and dst buffers queued for this frame — trigger encode. */
	if (sq->sequence == dq->sequence) {
		rc = jpeg_setup_scale(ectx);
		if (rc)
			goto err_clear_ctx;

		rc = jpeg_setup_encode(ectx);
		if (rc)
			goto err_clear_ctx;

		jpeg_exec(jenc);
	}

	return 0;

err_clear_ctx:
	spin_lock_irqsave(&jenc->hw_lock, flags);
	if (jenc->actx == ectx)
		jenc->actx = NULL;
	spin_unlock_irqrestore(&jenc->hw_lock, flags);

	return rc;
}

static int op_jpeg_prepare(struct qcom_jenc_dev *jenc)
{
	jpeg_rw_bits(jenc, JPEG_V420_REG_RESET_CMD, JPEG_V420_RST_CMD_ENCODER_RESET_MSK, 1);
	jpeg_rw_bits(jenc, JPEG_V420_REG_RESET_CMD, JPEG_V420_RST_CMD_BLOCK_FORMATTER_RST_MSK, 1);
	jpeg_rw_bits(jenc, JPEG_V420_REG_RESET_CMD, JPEG_V420_RST_CMD_CORE_RESET_MSK, 1);

	return 0;
}

static unsigned long jpeg_calc_optimal_clk(struct qcom_jenc_dev *jenc,
					   u32 width, u32 height, u32 fps)
{
	const struct qcom_dev_resources *res = jenc->res;
	unsigned long ref_pixel_rate;
	unsigned long optimal_clk;
	unsigned long pixel_rate;

	if (!width || !height || !fps)
		return jenc->max_freq;

	ref_pixel_rate = res->ref_throughput_mpps * 1000000UL;
	pixel_rate = (unsigned long)width * height * fps;

	optimal_clk = DIV_ROUND_UP_ULL(pixel_rate * res->ref_clk_hz, ref_pixel_rate);

	return min_t(unsigned long, jenc->max_freq, optimal_clk);
}

static unsigned long jpeg_get_optimal_frequency(struct jenc_context *ectx)
{
	struct qcom_jenc_queue *sq = &ectx->bufq[JENC_SRC_QUEUE];
	struct qcom_jenc_dev *jenc = ectx->jenc;
	u32 fps;

	/*
	 * If adaptive performance scaling is not requested always run at
	 * highest clock level for deterministic latency.
	 */
	if (!v4l2_ctrl_g_ctrl(ectx->perf_level_auto_ctl))
		return jenc->max_freq;

	fps = (u32)v4l2_ctrl_g_ctrl(ectx->fps_target_ctl);
	fps = clamp(fps, (u32)QCOM_JPEG_FPS_MIN, (u32)QCOM_JPEG_FPS_MAX);

	return jpeg_calc_optimal_clk(jenc, sq->vf.width, sq->vf.height, fps);
}

static int op_jpeg_acquire(struct jenc_context *ectx, struct vb2_queue *q)
{
	struct qcom_jenc_queue *sq = &ectx->bufq[JENC_SRC_QUEUE];
	struct qcom_jenc_queue *dq = &ectx->bufq[JENC_DST_QUEUE];
	struct qcom_jenc_dev *jenc = ectx->jenc;
	int rc;

	/* reset per-context stream state for each (re)acquire */
	sq->sequence = 0;
	sq->buff_id = -1;
	dq->sequence = 0;
	dq->buff_id = -1;
	/*
	 * Recreate JPEG header lazily per destination buffer to tolerate
	 * different valid V4L2 call orders (e.g. STREAMON before first QBUF).
	 */
	ectx->hdr_cache.size = 0;
	/* force DQT upload on first frame after (re)acquire */
	mutex_lock(&ectx->quality_mutex);
	ectx->quality_programmed = 0;
	mutex_unlock(&ectx->quality_mutex);

	if (atomic_inc_return(&jenc->ref_count) == 1) {
		jenc->opt_freq = jpeg_get_optimal_frequency(ectx);

		dev_dbg(jenc->dev, "freq=%lu src=%ux%u\n", jenc->opt_freq,
			sq->vf.width, sq->vf.height);

		rc = pm_runtime_resume_and_get(jenc->dev);
		if (rc < 0) {
			dev_err(jenc->dev, "PM runtime get failed\n");
			atomic_dec(&jenc->ref_count);
			return rc;
		}

		enable_irq(jenc->irq);

		rc = jpeg_init(jenc);
		if (rc) {
			dev_err(jenc->dev, "hardware init failed\n");
			disable_irq(jenc->irq);
			atomic_dec(&jenc->ref_count);
			pm_runtime_put_sync(jenc->dev);
			return rc;
		}
	}

	return 0;
}

static int op_jpeg_release(struct jenc_context *ectx, struct vb2_queue *q)
{
	struct qcom_jenc_dev *jenc = ectx->jenc;
	int rc = 0;
	int pm_rc;
	int ref;

	ref = atomic_dec_if_positive(&jenc->ref_count);
	if (ref < 0) {
		WARN_ON_ONCE(1);
		return 0;
	}

	if (!ref) {
		rc = jpeg_deinit(jenc);
		if (rc)
			dev_err(jenc->dev, "hardware exit failed\n");

		disable_irq(jenc->irq);

		pm_rc = pm_runtime_put_sync(jenc->dev);
		if (pm_rc < 0) {
			dev_err(jenc->dev, "PM runtime put failed\n");
			if (!rc)
				rc = pm_rc;
		}

		dev_dbg(jenc->dev, "JPEG HW encoder released\n");
	}

	return rc;
}

const struct qcom_jpeg_hw_ops qcom_jpeg_default_ops = {
	.hw_get_cap	= op_jpeg_get_hw_caps,
	.hw_acquire	= op_jpeg_acquire,
	.hw_release	= op_jpeg_release,
	.hw_prepare	= op_jpeg_prepare,
	.get_queue	= op_jpeg_get_buff_queue,
	.queue_setup	= op_jpeg_queue_setup,
	.src_fmt_update	= op_jpeg_src_fmt_update,
	.buf_prepare	= op_jpeg_buffer_prepare,
	.process_exec	= op_jpeg_process_exec,
	.hw_irq_top	= op_jpeg_irq_top,
	.hw_irq_bot	= op_jpeg_irq_bot
};
