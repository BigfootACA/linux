// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-sg.h>
#include <media/videobuf2-v4l2.h>

#include "qcom_jenc_defs.h"
#include "qcom_jenc_dev.h"
#include "qcom_jenc_ops.h"
#include "qcom_jenc_v4l2.h"

static const struct v4l2_frmsizeenum jpeg_def_frmsize = {
	.stepwise = {
		.min_width	= QCOM_JPEG_HW_MIN_WIDTH,
		.max_width	= QCOM_JPEG_HW_MAX_WIDTH,
		.step_width	= QCOM_JPEG_HW_DEF_HSTEP,
		.min_height	= QCOM_JPEG_HW_MIN_HEIGHT,
		.max_height	= QCOM_JPEG_HW_MAX_HEIGHT,
		.step_height	= QCOM_JPEG_HW_DEF_VSTEP,
	},
	.type = V4L2_FRMSIZE_TYPE_STEPWISE
};

static const struct jenc_enc_format jpeg_src_formats[] = {
	{
		.fourcc	= V4L2_PIX_FMT_NV12M,
		.type	= V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	},
	{
		.fourcc	= V4L2_PIX_FMT_NV21M,
		.type	= V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	},
	{
		.fourcc	= V4L2_PIX_FMT_GREY,
		.type	= V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	},
};

#define JPEG_SRC_FMT_COUNT ARRAY_SIZE(jpeg_src_formats)

static const struct jenc_enc_format jpeg_dst_formats[] = {
	{
		.fourcc	= V4L2_PIX_FMT_JPEG,
		.type	= V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	}
};

#define JPEG_DST_FMT_COUNT ARRAY_SIZE(jpeg_dst_formats)

static inline struct jenc_context *jpeg_file2ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct jenc_context, fh);
}

static struct qcom_jenc_queue *jpeg_get_bufq(struct jenc_context *ectx, enum qcom_enc_qid id)
{
	return &ectx->bufq[id];
}

static bool jpeg_v4l2_queues_busy(struct jenc_context *ctx)
{
	struct vb2_queue *out_q;
	struct vb2_queue *cap_q;

	out_q = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);

	cap_q = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	return vb2_is_busy(out_q) || vb2_is_busy(cap_q);
}

static bool jpeg_is_invalid_src(struct jenc_context *ectx, u32 type)
{
	bool is_invalid = (type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);

	if (is_invalid)
		dev_dbg(ectx->dev, "invalid src type or format\n");

	return is_invalid;
}

static bool jpeg_is_invalid_dst(struct jenc_context *ectx, u32 type)
{
	bool is_invalid = (type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	if (is_invalid)
		dev_dbg(ectx->dev, "invalid dst type or format\n");

	return is_invalid;
}

static const struct jenc_enc_format *jpeg_find_pix_format(enum qcom_enc_qid id, u32 fourcc)
{
	const struct jenc_enc_format *efmt;
	unsigned int i, count;

	if (id == JENC_SRC_QUEUE) {
		count = JPEG_SRC_FMT_COUNT;
		efmt = &jpeg_src_formats[0];
	} else {
		count = JPEG_DST_FMT_COUNT;
		efmt = &jpeg_dst_formats[0];
	}

	for (i = 0; i < count; i++) {
		if (efmt[i].fourcc == fourcc)
			return &efmt[i];
	}

	return NULL;
}

static const struct jenc_enc_format *jpeg_get_format(struct jenc_context *ectx,
						     enum qcom_enc_qid qid, u32 pixelformat)
{
	const struct jenc_enc_format *efmt = jpeg_find_pix_format(qid, pixelformat);

	if (!efmt)
		efmt = (qid == JENC_SRC_QUEUE) ? &jpeg_src_formats[0] : &jpeg_dst_formats[0];

	return efmt;
}

static int jpeg_update_src_planes(const struct jenc_enc_format *ef, struct v4l2_format *v4f)
{
	const struct v4l2_format_info *info = v4l2_format_info(ef->fourcc);
	struct v4l2_pix_format_mplane *f = &v4f->fmt.pix_mp;
	int rc;

	if (!info)
		return -EINVAL;

	f->pixelformat = ef->fourcc;

	f->field	= V4L2_FIELD_NONE;
	f->colorspace	= V4L2_COLORSPACE_SRGB;
	f->xfer_func	= V4L2_MAP_XFER_FUNC_DEFAULT(f->colorspace);
	f->ycbcr_enc	= V4L2_MAP_YCBCR_ENC_DEFAULT(f->colorspace);
	f->quantization =
		V4L2_MAP_QUANTIZATION_DEFAULT(f->ycbcr_enc == V4L2_YCBCR_ENC_601,
					      f->colorspace, f->ycbcr_enc);

	v4l2_apply_frmsize_constraints(&f->width, &f->height, &jpeg_def_frmsize.stepwise);

	rc = v4l2_fill_pixfmt_mp(f, ef->fourcc, f->width, f->height);
	if (rc)
		return rc;

	return 0;
}

static void jpeg_update_dst_plane(const struct jenc_enc_format *ef, struct v4l2_format *v4f)
{
	struct v4l2_pix_format_mplane *f = &v4f->fmt.pix_mp;
	u32 nbx, nby;
	u64 size;

	f->pixelformat  = ef->fourcc;
	f->field	= V4L2_FIELD_NONE;
	f->colorspace	= V4L2_COLORSPACE_SRGB;
	f->xfer_func	= V4L2_MAP_XFER_FUNC_DEFAULT(f->colorspace);
	f->ycbcr_enc	= V4L2_MAP_YCBCR_ENC_DEFAULT(f->colorspace);
	f->quantization =
		V4L2_MAP_QUANTIZATION_DEFAULT(f->ycbcr_enc == V4L2_YCBCR_ENC_601,
					      f->colorspace, f->ycbcr_enc);

	v4l2_apply_frmsize_constraints(&f->width, &f->height, &jpeg_def_frmsize.stepwise);

	/*
	 * JPEG is a variable-size format. The output size cannot be derived
	 * from bits per point or line stride.
	 *
	 * Provide a conservative upper bound based on worst-case entropy
	 * coding of 8x8 DCT blocks:
	 *
	 * - Each 8x8 block has 64 coefficients (1 DC + 63 AC).
	 * - In worst-case (high-entropy input, low quantization), all
	 *   coefficients may be non-zero.
	 * - Huffman coding then emits (code + magnitude bits) per coefficient,
	 *   which can approach ~2 bytes per coefficient in the worst case.
	 *
	 * => Worst-case is 64 coefficients * 2 bytes = 128 bytes per 8x8 block
	 * => approximately 2 bytes per point
	 *
	 * This bound implicitly covers byte stuffing (0xFF escaping) and is
	 * conservative with respect to subsampled formats (e.g. 4:2:0).
	 *
	 * Additional margin is added for headers and alignment.
	 *
	 * Note: This is a conservative upper bound, not an exact size.
	 */

	nbx = DIV_ROUND_UP(f->width,  8);
	nby = DIV_ROUND_UP(f->height, 8);

	size = nbx * nby * 128;
	size += JPEG_HEADER_MAX; /* JPEG header written by CPU before HW DMA */
	size += SZ_4K;           /* safety margin and alignment */

	f->plane_fmt[0].bytesperline = 0;
	f->plane_fmt[0].sizeimage    = ALIGN(size, SZ_4K);
}

static int jpeg_enum_fmt_src(struct v4l2_fmtdesc *f)
{
	if (f->index >= JPEG_SRC_FMT_COUNT)
		return -EINVAL;

	f->pixelformat = jpeg_src_formats[f->index].fourcc;

	return 0;
}

static int jpeg_enum_fmt_dst(struct v4l2_fmtdesc *f)
{
	if (f->index >= JPEG_DST_FMT_COUNT)
		return -EINVAL;

	f->pixelformat = jpeg_dst_formats[f->index].fourcc;

	return 0;
}

static int jpeg_v4l2_try_format(struct jenc_context *ectx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pm = &f->fmt.pix_mp;
	const struct jenc_enc_format *ef;
	int rc;

	/* The function always returns valid driver format */
	ef = jpeg_get_format(ectx, TYPE2QID(f->type), pm->pixelformat);

	dev_dbg(ectx->dev, "type=%d %p4cc\n",
		TYPE2QID(f->type), &ef->fourcc);

	if (V4L2_TYPE_IS_CAPTURE(f->type)) {
		f->fmt.pix_mp.num_planes = 1;

		jpeg_update_dst_plane(ef, f);

		dev_dbg(ectx->dev, "\tImage: %dx%d Size:%9d\n", f->fmt.pix_mp.width,
			f->fmt.pix_mp.height, f->fmt.pix_mp.plane_fmt[0].sizeimage);
	} else {
		const struct v4l2_format_info *info = v4l2_format_info(ef->fourcc);
		u8 pln = 0;

		if (!info)
			return -EINVAL;

		f->fmt.pix_mp.num_planes = info->comp_planes;

		rc = jpeg_update_src_planes(ef, f);
		if (rc)
			return rc;

		for (pln = 0; pln < f->fmt.pix_mp.num_planes; pln++)
			dev_dbg(ectx->dev, "\tImage: %dx%d BPL:%5d Size:%9d\n",
				pm->width, pm->height, pm->plane_fmt[pln].bytesperline,
				pm->plane_fmt[pln].sizeimage);
	}

	return 0;
}

static int jpeg_v4l2_set_defaults(struct jenc_context *ectx)
{
	struct qcom_jenc_queue *sq = jpeg_get_bufq(ectx, JENC_SRC_QUEUE);
	struct qcom_jenc_queue *dq = jpeg_get_bufq(ectx, JENC_DST_QUEUE);
	struct v4l2_format f = {0};
	int rc;

	f.type			 = jpeg_src_formats->type;
	f.fmt.pix_mp.pixelformat = jpeg_src_formats->fourcc;
	f.fmt.pix_mp.width	 = QCOM_JPEG_HW_DEF_WIDTH;
	f.fmt.pix_mp.height	 = QCOM_JPEG_HW_DEF_HEIGHT;

	rc = jpeg_v4l2_try_format(ectx, &f);
	if (rc)
		return rc;

	sq->vf = f.fmt.pix_mp;

	f.type			 = jpeg_dst_formats->type;
	f.fmt.pix_mp.pixelformat = jpeg_dst_formats->fourcc;
	f.fmt.pix_mp.width	 = QCOM_JPEG_HW_DEF_WIDTH;
	f.fmt.pix_mp.height	 = QCOM_JPEG_HW_DEF_HEIGHT;

	rc = jpeg_v4l2_try_format(ectx, &f);
	if (rc)
		return rc;

	dq->vf = f.fmt.pix_mp;

	return 0;
}

static int jpeg_v4l2_set_format(struct jenc_context *ectx, struct v4l2_format *f)
{
	const struct qcom_jpeg_hw_ops *hw = ectx->jenc->res->hw_ops;
	struct qcom_jenc_queue *q = jpeg_get_bufq(ectx, TYPE2QID(f->type));
	struct qcom_jenc_queue *sq = jpeg_get_bufq(ectx, JENC_SRC_QUEUE);
	struct v4l2_pix_format_mplane *pm = &f->fmt.pix_mp;
	u32 old_src_fourcc = sq->vf.pixelformat;
	int rc;

	if (jpeg_v4l2_queues_busy(ectx))
		return -EBUSY;

	if (!v4l2_m2m_get_vq(ectx->fh.m2m_ctx, f->type)) {
		dev_err(ectx->dev, "cannot get video queue\n");
		return -EINVAL;
	}

	rc = jpeg_v4l2_try_format(ectx, f);
	if (rc)
		return rc;

	/*
	 * Because scaling is not supported, source and destination image
	 * sizes must be equal.
	 */
	if (V4L2_TYPE_IS_CAPTURE(f->type)) {
		/* Adjust source size to match capture size */
		if (pm->width != sq->vf.width || pm->height != sq->vf.height) {
			struct v4l2_format nf = {0};

			nf.type			  = jpeg_src_formats->type;
			nf.fmt.pix_mp.pixelformat = sq->vf.pixelformat;
			nf.fmt.pix_mp.width	  = pm->width;
			nf.fmt.pix_mp.height	  = pm->height;

			rc = jpeg_v4l2_try_format(ectx, &nf);
			if (rc)
				return rc;

			sq->vf = nf.fmt.pix_mp;
		}

	} else {
		struct qcom_jenc_queue *dq = jpeg_get_bufq(ectx, JENC_DST_QUEUE);
		struct v4l2_format nf = {0};

		/* Adjust destination size to match source size */
		if (pm->width != dq->vf.width || pm->height != dq->vf.height) {
			nf.type			  = jpeg_dst_formats->type;
			nf.fmt.pix_mp.pixelformat = dq->vf.pixelformat;
			nf.fmt.pix_mp.width	  = pm->width;
			nf.fmt.pix_mp.height	  = pm->height;

			rc = jpeg_v4l2_try_format(ectx, &nf);
			if (rc)
				return rc;

			dq->vf = nf.fmt.pix_mp;

			/*
			 * The horizontal alignment of the destination is larger, and the
			 * result after adjustment may still differ. In this case, the
			 * requested image size should also be modified.
			 */
			if (pm->width != nf.fmt.pix_mp.width ||
			    pm->height != nf.fmt.pix_mp.height) {
				pm->width  = nf.fmt.pix_mp.width;
				pm->height = nf.fmt.pix_mp.height;
			}
		}
	}

	q->vf = *pm;

	if (V4L2_TYPE_IS_OUTPUT(f->type) && hw->src_fmt_update) {
		rc = hw->src_fmt_update(ectx, old_src_fourcc, q->vf.pixelformat);
		if (rc)
			return rc;
	}

	return 0;
}

static void jpeg_v4l2_get_format(struct jenc_context *ectx, struct v4l2_format *f)
{
	struct qcom_jenc_queue *q = jpeg_get_bufq(ectx, TYPE2QID(f->type));

	f->fmt.pix_mp = q->vf;
}

static void jpeg_v4l2_work_stop(struct jenc_context *ctx, enum vb2_buffer_state buff_state);

static void jpeg_finish_work(struct work_struct *work)
{
	struct jenc_context *ctx = container_of(work, struct jenc_context, finish_work);

	v4l2_m2m_job_finish(ctx->jenc->m2m_dev, ctx->fh.m2m_ctx);
}

static void jpeg_stop_work(struct work_struct *work)
{
	struct jenc_context *ctx = container_of(work, struct jenc_context, stop_work);
	struct qcom_jenc_dev *jenc = ctx->jenc;

	mutex_lock(&jenc->dev_mutex);
	jpeg_v4l2_work_stop(ctx, VB2_BUF_STATE_ERROR);
	mutex_unlock(&jenc->dev_mutex);
}

static void jpeg_v4l2_work_done(struct jenc_context *ctx, size_t out_size)
{
	struct vb2_v4l2_buffer *vb;

	vb = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	if (vb)
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_DONE);

	vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	if (vb) {
		vb2_set_plane_payload(&vb->vb2_buf, 0, out_size);
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_DONE);
	}

	schedule_work(&ctx->finish_work);
}

static void jpeg_v4l2_work_stop(struct jenc_context *ctx, enum vb2_buffer_state buff_state)
{
	bool was_stopping = ctx->is_stopping;
	struct vb2_v4l2_buffer *vb;

	ctx->is_stopping = false;

	/* Drain CAPTURE queue; signal EOS on last buffer if V4L2_ENC_CMD_STOP. */
	while ((vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx))) {
		if (was_stopping && !v4l2_m2m_last_dst_buf(ctx->fh.m2m_ctx)) {
			vb2_set_plane_payload(&vb->vb2_buf, 0, 0);
			v4l2_m2m_last_buffer_done(ctx->fh.m2m_ctx, vb);
		} else {
			v4l2_m2m_buf_done(vb, buff_state);
		}
	}

	while ((vb = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(vb, buff_state);

	/* Always call job_finish to let m2m schedule the next job. */
	schedule_work(&ctx->finish_work);
}

static void jpeg_v4l2_process_cb(void *priv, enum vb2_buffer_state ev, size_t out_size)
{
	struct jenc_context *ctx = priv;
	struct qcom_jenc_dev *jenc = ctx->jenc;

	/* threaded IRQ path */
	mutex_lock(&jenc->dev_mutex);

	if (ev == VB2_BUF_STATE_DONE && out_size)
		jpeg_v4l2_work_done(ctx, out_size);
	else
		jpeg_v4l2_work_stop(ctx, ev);

	mutex_unlock(&jenc->dev_mutex);
}

static int jpeg_v4l2_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct jenc_context *ectx = container_of(ctrl->handler, struct jenc_context, ctrl_hdl);

	switch (ctrl->id) {
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		mutex_lock(&ectx->quality_mutex);
		ectx->quality_requested = ctrl->val;
		mutex_unlock(&ectx->quality_mutex);
		break;
	case V4L2_CID_QCOM_JPEG_PERF_LEVEL_AUTO:
		/* value is read via v4l2_ctrl_g_ctrl() in jpeg_select_perf_level() */
		break;
	case V4L2_CID_QCOM_JPEG_FPS_TARGET:
		/* value is read via v4l2_ctrl_g_ctrl() in jpeg_select_perf_level() */
		break;
	default:
		dev_err(ectx->dev, "invalid control id=%#x\n", ctrl->id);
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops qcom_jpeg_v4l2_ctrl_ops = {
	.s_ctrl = jpeg_v4l2_s_ctrl,
};

static const struct v4l2_ctrl_config qcom_jpeg_perf_level_auto_cfg = {
	.ops	= &qcom_jpeg_v4l2_ctrl_ops,
	.id	= V4L2_CID_QCOM_JPEG_PERF_LEVEL_AUTO,
	.name	= "perf_level_auto",
	.type	= V4L2_CTRL_TYPE_BOOLEAN,
	.min	= 0,
	.max	= 1,
	.step	= 1,
	.def	= 0,
};

static const struct v4l2_ctrl_config qcom_jpeg_fps_target_cfg = {
	.ops	= &qcom_jpeg_v4l2_ctrl_ops,
	.id	= V4L2_CID_QCOM_JPEG_FPS_TARGET,
	.name	= "fps_target",
	.type	= V4L2_CTRL_TYPE_INTEGER,
	.min	= QCOM_JPEG_FPS_MIN,
	.max	= QCOM_JPEG_FPS_MAX,
	.step	= QCOM_JPEG_FPS_UNT,
	.def	= QCOM_JPEG_FPS_DEF,
};

static int bop_jpeg_vb2_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
				    unsigned int *plns_per_buff, unsigned int sizes[],
				    struct device *alloc_devs[])
{
	struct jenc_context *ectx = vb2_get_drv_priv(vq);
	struct qcom_jenc_dev *jenc = ectx->jenc;
	const struct qcom_jpeg_hw_ops *hw = jenc->res->hw_ops;
	struct qcom_jenc_queue *q;
	int pln;

	q = hw->get_queue(ectx, TYPE2QID(vq->type));
	if (!q || !q->vf.num_planes)
		return -EINVAL;

	if (*plns_per_buff) {
		if (*plns_per_buff != q->vf.num_planes)
			return -EINVAL;

		for (pln = 0; pln < q->vf.num_planes; ++pln) {
			if (sizes[pln] < q->vf.plane_fmt[pln].sizeimage)
				return -EINVAL;
		}

		return 0;
	}

	*plns_per_buff = q->vf.num_planes;
	for (pln = 0; pln < q->vf.num_planes; ++pln) {
		sizes[pln] = q->vf.plane_fmt[pln].sizeimage;
		dev_dbg(ectx->dev, "queue=%d size[%d]=%d\n", TYPE2QID(vq->type),
			pln, sizes[pln]);
	}

	return hw->queue_setup(ectx, TYPE2QID(vq->type));
}

static int bop_jpeg_vb2_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (vbuf->field == V4L2_FIELD_ANY)
		vbuf->field = V4L2_FIELD_NONE;

	if (vbuf->field != V4L2_FIELD_NONE)
		return -EINVAL;

	return 0;
}

static int bop_jpeg_vb2_buf_prepare(struct vb2_buffer *vb)
{
	struct jenc_context *ectx = vb2_get_drv_priv(vb->vb2_queue);
	struct qcom_jenc_dev *jenc = ectx->jenc;
	const struct qcom_jpeg_hw_ops *hw = jenc->res->hw_ops;
	struct qcom_jenc_queue *q = &ectx->bufq[TYPE2QID(vb->type)];
	int pln;
	int rc;

	if (vb->num_planes != q->vf.num_planes)
		return -EINVAL;

	for (pln = 0; pln < q->vf.num_planes; pln++) {
		if (q->vf.plane_fmt[pln].sizeimage == 0)
			return -EINVAL;

		if (vb2_plane_size(vb, pln) < q->vf.plane_fmt[pln].sizeimage)
			return -EINVAL;
	}

	rc = hw->buf_prepare(ectx, vb);
	if (rc) {
		dev_err_ratelimited(ectx->dev, "buffer prepare failed\n");
		return rc;
	}

	return 0;
}

static void bop_jpeg_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct jenc_context *ectx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	v4l2_m2m_buf_queue(ectx->fh.m2m_ctx, vbuf);
}

static int bop_jpeg_vb2_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct jenc_context *ectx = vb2_get_drv_priv(q);
	struct qcom_jenc_dev *jenc = ectx->jenc;
	const struct qcom_jpeg_hw_ops *hw = jenc->res->hw_ops;
	struct qcom_jenc_queue *sq = jpeg_get_bufq(ectx, JENC_SRC_QUEUE);
	struct qcom_jenc_queue *dq = jpeg_get_bufq(ectx, JENC_DST_QUEUE);
	u32 hw_caps;
	u8 pln;
	int rc;

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		dev_dbg(ectx->dev, "%p4cc %dx%d\n",
			&sq->vf.pixelformat, sq->vf.width, sq->vf.height);

		for (pln = 0; pln < sq->vf.num_planes; pln++) {
			dev_dbg(ectx->dev, "\tpln=%d %dx%d bpl:%d size:%d\n", pln,
				sq->vf.width, sq->vf.height,
				sq->vf.plane_fmt[pln].bytesperline,
				sq->vf.plane_fmt[pln].sizeimage);
		}
	} else {
		dev_dbg(ectx->dev, "%p4cc %dx%d\n",
			&dq->vf.pixelformat,
			dq->vf.width, dq->vf.height);
	}

	mutex_lock(&jenc->dev_mutex);

	/*
	 * Header cache is initialized lazily on CAPTURE QBUF, so valid V4L2
	 * orders like STREAMON before first QBUF still get a proper JPEG header.
	 */
	if (!ectx->hw_acquired) {
		rc = hw->hw_acquire(ectx, q);
		if (!rc) {
			ectx->hw_acquired = true;
			hw->hw_get_cap(jenc, &hw_caps);
			dev_dbg(ectx->dev, "hw_caps=0x%x\n", hw_caps);
		}
	} else {
		rc = 0;
	}

	mutex_unlock(&jenc->dev_mutex);

	return rc;
}

static void bop_jpeg_vb2_stop_streaming(struct vb2_queue *q)
{
	struct jenc_context *ectx = vb2_get_drv_priv(q);
	struct qcom_jenc_dev *jenc = ectx->jenc;
	const struct qcom_jpeg_hw_ops *hw = jenc->res->hw_ops;
	bool do_release;

	mutex_lock(&jenc->dev_mutex);

	jpeg_v4l2_work_stop(ectx, VB2_BUF_STATE_ERROR);

	do_release = ectx->hw_acquired;
	ectx->hw_acquired = false;

	mutex_unlock(&jenc->dev_mutex);

	/*
	 * hw_release() calls jpeg_deinit() which issues HW_STOP and waits for
	 * the STOP_ACK interrupt completion.  It must be called outside
	 * dev_mutex to avoid a deadlock: the threaded IRQ handler acquires
	 * dev_mutex before signalling the completion.
	 */
	if (do_release)
		hw->hw_release(ectx, q);
}

static const struct vb2_ops qcom_jpeg_v4l2_vb2_ops = {
	.queue_setup		= bop_jpeg_vb2_queue_setup,
	.buf_out_validate	= bop_jpeg_vb2_buf_out_validate,
	.buf_prepare		= bop_jpeg_vb2_buf_prepare,
	.buf_queue		= bop_jpeg_vb2_buf_queue,
	.start_streaming	= bop_jpeg_vb2_start_streaming,
	.stop_streaming		= bop_jpeg_vb2_stop_streaming,
};

static void mop_jpeg_m2m_job_abort(void *priv)
{
	struct jenc_context *ectx = priv;
	struct qcom_jenc_dev *jenc = ectx->jenc;

	mutex_lock(&jenc->dev_mutex);

	jpeg_v4l2_work_stop(ectx, VB2_BUF_STATE_ERROR);

	mutex_unlock(&jenc->dev_mutex);
}

static void mop_jpeg_m2m_job_run(void *priv)
{
	struct jenc_context *ectx = priv;
	struct qcom_jenc_dev *jenc = ectx->jenc;
	const struct qcom_jpeg_hw_ops *hw = jenc->res->hw_ops;
	struct vb2_v4l2_buffer *src_vb, *dst_vb;
	struct qcom_jenc_queue *sq, *dq;

	mutex_lock(&jenc->dev_mutex);

	src_vb = v4l2_m2m_next_src_buf(ectx->fh.m2m_ctx);
	dst_vb = v4l2_m2m_next_dst_buf(ectx->fh.m2m_ctx);

	if (!src_vb || !dst_vb)
		goto err_stop;

	if (hw->hw_prepare(jenc))
		goto err_stop;

	sq = hw->get_queue(ectx, TYPE2QID(src_vb->vb2_buf.type));
	src_vb->sequence = sq->sequence++;
	if (hw->process_exec(jenc, ectx, &src_vb->vb2_buf))
		goto err_stop;

	dq = hw->get_queue(ectx, TYPE2QID(dst_vb->vb2_buf.type));
	dst_vb->sequence = dq->sequence++;
	if (hw->process_exec(jenc, ectx, &dst_vb->vb2_buf))
		goto err_stop;

	v4l2_m2m_buf_copy_metadata(src_vb, dst_vb);

	mutex_unlock(&jenc->dev_mutex);
	return;

err_stop:
	jpeg_v4l2_work_stop(ectx, VB2_BUF_STATE_ERROR);
	mutex_unlock(&jenc->dev_mutex);
}

static const struct v4l2_m2m_ops qcom_jpeg_v4l2_m2m_ops = {
	.device_run	= mop_jpeg_m2m_job_run,
	.job_abort	= mop_jpeg_m2m_job_abort,
};

static int iop_jpeg_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, QCOM_JPEG_ENC_NAME, sizeof(cap->driver));
	strscpy(cap->card, QCOM_JPEG_ENC_NAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s", QCOM_JPEG_ENC_NAME);

	return 0;
}

static int iop_jpeg_enum_fmt_vid_dst(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	struct jenc_context *ectx = jpeg_file2ctx(file);

	if (jpeg_is_invalid_dst(ectx, f->type))
		return -EINVAL;

	return jpeg_enum_fmt_dst(f);
}

static int iop_jpeg_enum_framesizes(struct file *file, void *priv, struct v4l2_frmsizeenum *fsize)
{
	const struct jenc_enc_format *efmt;

	if (fsize->index != 0)
		return -EINVAL;

	efmt = jpeg_find_pix_format(JENC_SRC_QUEUE, fsize->pixel_format);
	if (efmt) {
		fsize->type	= jpeg_def_frmsize.type;
		fsize->stepwise	= jpeg_def_frmsize.stepwise;
		return 0;
	}

	efmt = jpeg_find_pix_format(JENC_DST_QUEUE, fsize->pixel_format);
	if (efmt) {
		fsize->type	= jpeg_def_frmsize.type;
		fsize->stepwise	= jpeg_def_frmsize.stepwise;
		return 0;
	}

	return -EINVAL;
}

static int iop_jpeg_enum_fmt_vid_src(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	struct jenc_context *ectx = jpeg_file2ctx(file);

	if (jpeg_is_invalid_src(ectx, f->type))
		return -EINVAL;

	return jpeg_enum_fmt_src(f);
}

static int iop_jpeg_get_fmt_vid_dst(struct file *file, void *priv, struct v4l2_format *f)
{
	struct jenc_context *ectx = jpeg_file2ctx(file);

	if (jpeg_is_invalid_dst(ectx, f->type))
		return -EINVAL;

	jpeg_v4l2_get_format(ectx, f);

	return 0;
}

static int iop_jpeg_try_fmt_vid_dst(struct file *file, void *priv, struct v4l2_format *f)
{
	struct jenc_context *ectx = jpeg_file2ctx(file);

	if (jpeg_is_invalid_dst(ectx, f->type))
		return -EINVAL;

	return jpeg_v4l2_try_format(ectx, f);
}

static int iop_jpeg_set_fmt_vid_dst(struct file *file, void *priv, struct v4l2_format *f)
{
	struct jenc_context *ectx = jpeg_file2ctx(file);

	if (jpeg_is_invalid_dst(ectx, f->type))
		return -EINVAL;

	return jpeg_v4l2_set_format(ectx, f);
}

static int iop_jpeg_get_fmt_vid_src(struct file *file, void *priv, struct v4l2_format *f)
{
	struct jenc_context *ectx = jpeg_file2ctx(file);

	if (jpeg_is_invalid_src(ectx, f->type))
		return -EINVAL;

	jpeg_v4l2_get_format(ectx, f);

	return 0;
}

static int iop_jpeg_try_fmt_vid_src(struct file *file, void *priv, struct v4l2_format *f)
{
	struct jenc_context *ectx = jpeg_file2ctx(file);

	if (jpeg_is_invalid_src(ectx, f->type))
		return -EINVAL;

	return jpeg_v4l2_try_format(ectx, f);
}

static int iop_jpeg_set_fmt_vid_src(struct file *file, void *priv, struct v4l2_format *f)
{
	struct jenc_context *ectx = jpeg_file2ctx(file);

	if (jpeg_is_invalid_src(ectx, f->type))
		return -EINVAL;

	return jpeg_v4l2_set_format(ectx, f);
}

static int iop_jpeg_encoder_command(struct file *file, void *priv, struct v4l2_encoder_cmd *ec)
{
	struct jenc_context *ectx = jpeg_file2ctx(file);
	struct vb2_queue *vq;
	int rc;

	if (ec->cmd == V4L2_ENC_CMD_STOP) {
		vq = v4l2_m2m_get_src_vq(ectx->fh.m2m_ctx);
		if (!vb2_is_streaming(vq))
			return 0;

		vq = v4l2_m2m_get_dst_vq(ectx->fh.m2m_ctx);
		if (!vb2_is_streaming(vq))
			return 0;

		rc = v4l2_m2m_ioctl_encoder_cmd(file, priv, ec);
		if (rc)
			return rc;

		ectx->is_stopping = true;
		schedule_work(&ectx->stop_work);

		return 0;
	}

	return v4l2_m2m_ioctl_encoder_cmd(file, priv, ec);
}

static const struct v4l2_ioctl_ops qcom_jpeg_v4l2_ioctl_ops = {
	.vidioc_querycap		= iop_jpeg_querycap,
	.vidioc_enum_fmt_vid_cap	= iop_jpeg_enum_fmt_vid_dst,
	.vidioc_enum_fmt_vid_out	= iop_jpeg_enum_fmt_vid_src,
	.vidioc_enum_framesizes		= iop_jpeg_enum_framesizes,

	.vidioc_g_fmt_vid_cap_mplane	= iop_jpeg_get_fmt_vid_dst,
	.vidioc_try_fmt_vid_cap_mplane	= iop_jpeg_try_fmt_vid_dst,
	.vidioc_s_fmt_vid_cap_mplane	= iop_jpeg_set_fmt_vid_dst,
	.vidioc_g_fmt_vid_out_mplane	= iop_jpeg_get_fmt_vid_src,
	.vidioc_try_fmt_vid_out_mplane	= iop_jpeg_try_fmt_vid_src,
	.vidioc_s_fmt_vid_out_mplane	= iop_jpeg_set_fmt_vid_src,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,

	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,

	.vidioc_encoder_cmd		= iop_jpeg_encoder_command,
	.vidioc_try_encoder_cmd		= v4l2_m2m_ioctl_try_encoder_cmd,
};

static int jpeg_v4l2_init_queue(void *priv, struct vb2_queue *sq, struct vb2_queue *dq)
{
	struct jenc_context *ectx = priv;
	int rc;

	sq->drv_priv		= ectx;
	sq->dev			= ectx->dev;
	sq->type		= V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	sq->io_modes		= VB2_MMAP | VB2_DMABUF | VB2_USERPTR;
	sq->buf_struct_size	= sizeof(struct v4l2_m2m_buffer);
	sq->ops			= &qcom_jpeg_v4l2_vb2_ops;
	sq->mem_ops		= &vb2_dma_sg_memops;
	sq->timestamp_flags	= V4L2_BUF_FLAG_TIMESTAMP_COPY;
	sq->lock		= &ectx->ctx_lock;
	sq->min_queued_buffers	= 1;

	rc = vb2_queue_init(sq);
	if (rc)
		return rc;

	dq->drv_priv		= ectx;
	dq->dev			= ectx->dev;
	dq->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dq->io_modes		= VB2_MMAP | VB2_DMABUF | VB2_USERPTR;
	dq->buf_struct_size	= sizeof(struct v4l2_m2m_buffer);
	dq->ops			= &qcom_jpeg_v4l2_vb2_ops;
	dq->mem_ops		= &vb2_dma_sg_memops;
	dq->timestamp_flags	= V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dq->lock		= &ectx->ctx_lock;
	dq->min_queued_buffers	= 1;

	rc = vb2_queue_init(dq);
	if (rc) {
		vb2_queue_release(sq);
		return rc;
	}

	return 0;
}

static int fop_jpeg_file_open(struct file *file)
{
	struct qcom_jenc_dev *jenc = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);
	struct jenc_context *ectx;
	int rc;

	ectx = kzalloc_obj(*ectx, GFP_KERNEL);
	if (!ectx)
		return -ENOMEM;

	ectx->dev  = jenc->dev;
	ectx->jenc = jenc;

	/* default quality if userspace does not set the control explicitly */
	ectx->quality_requested = QCOM_JPEG_QUALITY_DEF;
	ectx->quality_programmed = 0;

	mutex_init(&ectx->ctx_lock);
	mutex_init(&ectx->quality_mutex);

	INIT_WORK(&ectx->finish_work, jpeg_finish_work);
	INIT_WORK(&ectx->stop_work, jpeg_stop_work);

	rc = jpeg_v4l2_set_defaults(ectx);
	if (rc)
		goto err_unlock_free;

	v4l2_fh_init(&ectx->fh, vdev);
	v4l2_fh_add(&ectx->fh, file);

	v4l2_ctrl_handler_init(&ectx->ctrl_hdl, 3);
	ectx->quality_ctl = v4l2_ctrl_new_std(&ectx->ctrl_hdl,
					      &qcom_jpeg_v4l2_ctrl_ops,
					      V4L2_CID_JPEG_COMPRESSION_QUALITY,
					      QCOM_JPEG_QUALITY_MIN,
					      QCOM_JPEG_QUALITY_MAX,
					      QCOM_JPEG_QUALITY_UNT,
					      QCOM_JPEG_QUALITY_DEF);
	ectx->perf_level_auto_ctl =
		v4l2_ctrl_new_custom(&ectx->ctrl_hdl,
				     &qcom_jpeg_perf_level_auto_cfg, NULL);
	ectx->fps_target_ctl =
		v4l2_ctrl_new_custom(&ectx->ctrl_hdl,
				     &qcom_jpeg_fps_target_cfg, NULL);
	if (ectx->ctrl_hdl.error) {
		rc = ectx->ctrl_hdl.error;
		goto err_fh_exit;
	}

	ectx->fh.ctrl_handler = &ectx->ctrl_hdl;

	rc = v4l2_ctrl_handler_setup(&ectx->ctrl_hdl);
	if (rc)
		goto err_ctrl_handler_free;

	v4l2_m2m_get(jenc->m2m_dev);
	ectx->fh.m2m_ctx = v4l2_m2m_ctx_init(jenc->m2m_dev, ectx, &jpeg_v4l2_init_queue);
	if (IS_ERR(ectx->fh.m2m_ctx)) {
		rc = PTR_ERR(ectx->fh.m2m_ctx);
		v4l2_m2m_put(jenc->m2m_dev);
		goto err_ctrl_handler_free;
	}

	return 0;

err_ctrl_handler_free:
	v4l2_ctrl_handler_free(&ectx->ctrl_hdl);
err_fh_exit:
	v4l2_fh_del(&ectx->fh, file);
	v4l2_fh_exit(&ectx->fh);
err_unlock_free:
	mutex_destroy(&ectx->quality_mutex);
	mutex_destroy(&ectx->ctx_lock);
	kfree(ectx);

	return rc;
}

static int fop_jpeg_file_release(struct file *file)
{
	struct jenc_context *ectx = jpeg_file2ctx(file);
	struct v4l2_m2m_dev *m2m_dev = ectx->fh.m2m_ctx->m2m_dev;
	struct qcom_jenc_dev *jenc = ectx->jenc;
	unsigned long flags;

	/*
	 * Ensure the threaded IRQ handler cannot dereference this context
	 * after it is freed.  Clear jenc->actx under hw_lock before
	 * cancelling work and releasing the m2m context.
	 */
	spin_lock_irqsave(&jenc->hw_lock, flags);
	if (jenc->actx == ectx)
		jenc->actx = NULL;
	spin_unlock_irqrestore(&jenc->hw_lock, flags);

	cancel_work_sync(&ectx->stop_work);
	cancel_work_sync(&ectx->finish_work);

	v4l2_m2m_ctx_release(ectx->fh.m2m_ctx);
	v4l2_m2m_put(m2m_dev);
	v4l2_ctrl_handler_free(&ectx->ctrl_hdl);
	v4l2_fh_del(&ectx->fh, file);
	v4l2_fh_exit(&ectx->fh);
	mutex_destroy(&ectx->quality_mutex);
	mutex_destroy(&ectx->ctx_lock);
	kfree(ectx);

	return 0;
}

static const struct v4l2_file_operations qcom_jpeg_v4l2_file_ops = {
	.owner		= THIS_MODULE,
	.open		= fop_jpeg_file_open,
	.release	= fop_jpeg_file_release,
	.poll		= v4l2_m2m_fop_poll,
	.mmap		= v4l2_m2m_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

/* Release callback: free jenc after last fd is closed. */
static void jenc_v4l2_dev_release(struct v4l2_device *v4l2_dev)
{
	struct qcom_jenc_dev *jenc = container_of(v4l2_dev, struct qcom_jenc_dev, v4l2_dev);

	v4l2_device_unregister(&jenc->v4l2_dev);
	v4l2_m2m_put(jenc->m2m_dev);
	kfree(jenc);
}

int qcom_jpeg_v4l2_register(struct qcom_jenc_dev *jenc)
{
	int rc;

	mutex_lock(&jenc->dev_mutex);

	jenc->enc_hw_irq_cb = jpeg_v4l2_process_cb;

	jenc->m2m_dev = v4l2_m2m_init(&qcom_jpeg_v4l2_m2m_ops);
	if (IS_ERR(jenc->m2m_dev)) {
		dev_err(jenc->dev, "failed to init mem2mem device\n");
		rc = PTR_ERR(jenc->m2m_dev);
		goto err_mutex_unlock;
	}

	jenc->vdev = video_device_alloc();
	if (!jenc->vdev) {
		rc = -ENOMEM;
		goto err_m2m_release;
	}

	snprintf(jenc->vdev->name, sizeof(jenc->vdev->name), "%s", QCOM_JPEG_ENC_NAME);
	jenc->vdev->fops	= &qcom_jpeg_v4l2_file_ops;
	jenc->vdev->ioctl_ops	= &qcom_jpeg_v4l2_ioctl_ops;
	jenc->vdev->minor	= -1;
	jenc->vdev->release	= video_device_release;
	jenc->vdev->lock	= &jenc->dev_mutex;
	jenc->vdev->v4l2_dev	= &jenc->v4l2_dev;
	jenc->vdev->vfl_dir	= VFL_DIR_M2M;
	jenc->vdev->device_caps	= V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE;

	rc = video_register_device(jenc->vdev, VFL_TYPE_VIDEO, -1);
	if (rc) {
		dev_err(jenc->dev, "failed to register video device\n");
		goto err_video_device_release;
	}

	video_set_drvdata(jenc->vdev, jenc);

	jenc->v4l2_dev.release = jenc_v4l2_dev_release;

	mutex_unlock(&jenc->dev_mutex);

	dev_dbg(jenc->dev, "device registered as /dev/video%d\n", jenc->vdev->num);

	return 0;

err_video_device_release:
	video_device_release(jenc->vdev);
err_m2m_release:
	v4l2_m2m_release(jenc->m2m_dev);
err_mutex_unlock:
	mutex_unlock(&jenc->dev_mutex);

	return rc;
}

