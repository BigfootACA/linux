/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef QCOM_JENC_DEV_H
#define QCOM_JENC_DEV_H

#include <linux/device.h>
#include <linux/irqreturn.h>
#include <linux/mutex.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>

#include "qcom_jenc_defs.h"
#include "qcom_jenc_hdr.h"
#include "qcom_jenc_res.h"

#define QCOM_JPEG_ENC_NAME "qcom-jpeg-enc"

#define TYPE2QID(t) \
	(V4L2_TYPE_IS_OUTPUT(t) ? JENC_SRC_QUEUE : JENC_DST_QUEUE)

enum qcom_enc_qid {
	JENC_SRC_QUEUE = 0,
	JENC_DST_QUEUE,
	JENC_QUEUE_MAX
};

struct jenc_enc_format {
	u32 type;
	u32 fourcc;
};

struct qcom_jpeg_buff {
	struct {
		struct sg_table		*sgt;
		dma_addr_t		dma;
		unsigned long		size;

	} plns[QCOM_JPEG_MAX_PLANES];
};

struct qcom_jenc_queue {
	struct v4l2_pix_format_mplane	vf;
	u32				sequence;
	struct qcom_jpeg_buff		buff[VB2_MAX_FRAME];
	int				buff_id;
};

struct qcom_jenc_dev {
	struct device			*dev;
	struct device			*camss_dev;
	struct v4l2_device		v4l2_dev;
	struct v4l2_m2m_dev		*m2m_dev;
	struct video_device		*vdev;
	const struct qcom_dev_resources	*res;
	int				irq;
	void __iomem			*jpeg_base;
	struct clk_bulk_data		*clks;
	int				num_clks;
	unsigned long			max_freq;
	unsigned long			opt_freq;
	/* device mutex lock */
	struct mutex			dev_mutex;
	atomic_t			ref_count;
	struct completion		reset_complete;
	struct completion		stop_complete;
	/* encoder hardware lock */
	spinlock_t			hw_lock;
	struct jenc_context		*actx;

	u32				pending_irq_status;

	void (*enc_hw_irq_cb)
		(void *data, enum vb2_buffer_state ev, size_t out_size);
};

struct jenc_context {
	struct device		 *dev;
	struct qcom_jenc_dev	 *jenc;
	struct v4l2_fh		 fh;

	/* quality update lock */
	struct mutex		 quality_mutex;
	struct v4l2_ctrl	 *quality_ctl;
	u32			 quality_requested;
	u32			 quality_programmed;
	struct v4l2_ctrl	 *perf_level_auto_ctl;
	struct v4l2_ctrl	 *fps_target_ctl;
	struct v4l2_ctrl_handler ctrl_hdl;

	/* session context lock */
	struct mutex		 ctx_lock;

	bool			 is_stopping;
	bool			 hw_acquired;

	struct qcom_jenc_queue	bufq[JENC_QUEUE_MAX];
	struct qcom_jenc_header	hdr_cache;

	struct work_struct finish_work;
	struct work_struct stop_work;
};

#endif
