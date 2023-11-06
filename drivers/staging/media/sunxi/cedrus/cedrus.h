/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cedrus Video Engine Driver
 *
 * Copyright 2016 Florent Revest <florent.revest@free-electrons.com>
 * Copyright 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright 2018-2023 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _CEDRUS_H_
#define _CEDRUS_H_

#include <linux/iopoll.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#include "cedrus_context.h"
#include "cedrus_proc.h"

#define CEDRUS_NAME		"cedrus"
#define CEDRUS_DESCRIPTION	"Allwinner Cedrus Video Engine Driver"

#define CEDRUS_WIDTH_MIN	16U
#define CEDRUS_WIDTH_MAX	4096U
#define CEDRUS_HEIGHT_MIN	16U
#define CEDRUS_HEIGHT_MAX	2304U

enum cedrus_codec {
	CEDRUS_CODEC_MPEG2,
	CEDRUS_CODEC_H264,
	CEDRUS_CODEC_H265,
	CEDRUS_CODEC_VP8,
};

enum cedrus_irq_status {
	CEDRUS_IRQ_NONE,
	CEDRUS_IRQ_ERROR,
	CEDRUS_IRQ_SUCCESS,
};

enum cedrus_capability {
	CEDRUS_CAPABILITY_UNTILED	= BIT(0),
	CEDRUS_CAPABILITY_MPEG2_DEC	= BIT(1),
	CEDRUS_CAPABILITY_H264_DEC	= BIT(2),
	CEDRUS_CAPABILITY_H265_DEC	= BIT(3),
	CEDRUS_CAPABILITY_H265_10_DEC	= BIT(4),
	CEDRUS_CAPABILITY_VP8_DEC	= BIT(5),
};

struct cedrus_context;

struct cedrus_variant {
	unsigned int	capabilities;
	unsigned int	clock_mod_rate;
};

struct cedrus_format {
	unsigned int	pixelformat;
	unsigned int	capabilities;
	int		type;
};

struct cedrus_v4l2 {
	struct v4l2_device	v4l2_dev;
	struct media_device	media_dev;
	struct v4l2_m2m_dev	*m2m_dev;
};

struct cedrus_device {
	struct device		*dev;

	struct cedrus_v4l2	v4l2;
	struct cedrus_proc	dec;

	void __iomem		*io_base;
	struct clk		*clock_ahb;
	struct clk		*clock_mod;
	struct clk		*clock_ram;
	struct reset_control	*reset;

	unsigned int		capabilities;

	struct delayed_work	watchdog_work;
};

/* Capabilities */

static inline bool cedrus_capabilities_check(struct cedrus_device *dev,
					     unsigned int capabilities)
{
	return (dev->capabilities & capabilities) == capabilities;
}

/* I/O */

static inline void cedrus_write(struct cedrus_device *dev, u32 reg, u32 val)
{
	writel(val, dev->io_base + reg);
}

static inline u32 cedrus_read(struct cedrus_device *dev, u32 reg)
{
	return readl(dev->io_base + reg);
}

static inline int cedrus_poll(struct cedrus_device *dev, u32 reg, u32 bits)
{
	u32 value;

	return readl_poll_timeout_atomic(dev->io_base + reg, value,
					 (value & bits) == bits, 10, 1000);
}

static inline int cedrus_poll_cleared(struct cedrus_device *dev, u32 reg,
				      u32 bits)
{
	u32 value;

	return readl_poll_timeout_atomic(dev->io_base + reg, value,
					 (value & bits) == 0, 10, 1000);
}

/* Buffer */

static inline u64 cedrus_buffer_timestamp(struct cedrus_buffer *buffer)
{
	return buffer->m2m_buffer.vb.vb2_buf.timestamp;
}

static inline void cedrus_buffer_picture_dma(struct cedrus_context *ctx,
					     struct cedrus_buffer *cedrus_buffer,
					     dma_addr_t *luma_addr,
					     dma_addr_t *chroma_addr)
{
	struct v4l2_pix_format *pix_format = &ctx->v4l2.format_picture.fmt.pix;
	struct vb2_buffer *vb2_buffer = &cedrus_buffer->m2m_buffer.vb.vb2_buf;
	dma_addr_t addr;

	addr = vb2_dma_contig_plane_dma_addr(vb2_buffer, 0);
	*luma_addr = addr;

	addr += pix_format->bytesperline * pix_format->height;
	*chroma_addr = addr;
}

static inline void cedrus_buffer_coded_dma(struct cedrus_context *ctx,
					   struct cedrus_buffer *cedrus_buffer,
					   dma_addr_t *addr, unsigned int *size)
{
	struct vb2_buffer *vb2_buffer = &cedrus_buffer->m2m_buffer.vb.vb2_buf;

	*addr = vb2_dma_contig_plane_dma_addr(vb2_buffer, 0);
	*size = vb2_get_plane_payload(vb2_buffer, 0);
}

static inline struct cedrus_buffer *
cedrus_buffer_picture_find(struct cedrus_context *ctx, u64 timestamp)
{
	struct vb2_buffer *vb2_buffer;
	struct vb2_v4l2_buffer *v4l2_buffer;

	if (WARN_ON(!ctx->job.queue_picture))
		return NULL;

	vb2_buffer = vb2_find_buffer(ctx->job.queue_picture, timestamp);
	if (!vb2_buffer)
		return NULL;

	v4l2_buffer = to_vb2_v4l2_buffer(vb2_buffer);

	return container_of(v4l2_buffer, struct cedrus_buffer, m2m_buffer.vb);
}

static inline struct cedrus_buffer *
cedrus_buffer_from_vb2(const struct vb2_buffer *vb2_buffer)
{
	const struct vb2_v4l2_buffer *v4l2_buffer =
		to_vb2_v4l2_buffer(vb2_buffer);

	return container_of(v4l2_buffer, struct cedrus_buffer, m2m_buffer.vb);
}

#endif
