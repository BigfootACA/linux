/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef QCOM_JENC_OPS_H
#define QCOM_JENC_OPS_H

#include <linux/device.h>
#include <linux/types.h>
#include <media/videobuf2-core.h>

#include "qcom_jenc_dev.h"

/*
 * JENC encoder hardware operations.
 */
struct qcom_jpeg_hw_ops {
	void (*hw_get_cap)
		(struct qcom_jenc_dev *jenc_dev, u32 *hw_caps);

	int (*hw_acquire)
		(struct jenc_context *ectx, struct vb2_queue *queue);

	int (*hw_release)
		(struct jenc_context *ectx, struct vb2_queue *queue);

	int (*hw_prepare)
		(struct qcom_jenc_dev *jenc);

	struct qcom_jenc_queue * (*get_queue)
		(struct jenc_context *ectx, enum qcom_enc_qid id);

	int (*queue_setup)
		(struct jenc_context *ectx, enum qcom_enc_qid id);

	int (*src_fmt_update)
		(struct jenc_context *ectx, u32 old_fourcc, u32 new_fourcc);

	int (*buf_prepare)
		(struct jenc_context *ectx, struct vb2_buffer *vb2);

	int (*process_exec)
		(struct qcom_jenc_dev *jenc, struct jenc_context *ectx, struct vb2_buffer *vb2);

	irqreturn_t (*hw_irq_top)(int irq_num, void *data);
	irqreturn_t (*hw_irq_bot)(int irq_num, void *data);
};

extern const struct qcom_jpeg_hw_ops qcom_jpeg_default_ops;

#endif /* QCOM_JENC_OPS_H */
