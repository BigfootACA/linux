// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include <media/v4l2-mem2mem.h>

#include "qcom_jenc_dev.h"
#include "qcom_jenc_ops.h"
#include "qcom_jenc_res.h"
#include "qcom_jenc_v4l2.h"

enum jpeg_opp_clks_id {
	JPEG_OPP_CNOC_IDX = 0,
	JPEG_OPP_CORE_IDX
};

static const char * const opp_clk_names[] = {
	[JPEG_OPP_CNOC_IDX] = "cnoc_axi",
	[JPEG_OPP_CORE_IDX] = "jpeg",
	NULL,
};

static struct dev_pm_opp_config opp_config = {
	.clk_names = opp_clk_names,
	.config_clks = dev_pm_opp_config_clks_simple,
};

static int qcom_jpeg_opp_init(struct qcom_jenc_dev *jenc)
{
	struct dev_pm_opp *opp;
	int rc;

	rc = devm_pm_opp_set_config(jenc->dev, &opp_config);
	if (rc)
		return rc;

	rc = devm_pm_opp_of_add_table(jenc->dev);
	if (rc && rc != -ENODEV)
		return rc;

	/* initialize the maximum available frequency for the JPEG core */
	jenc->max_freq = ULONG_MAX;
	opp = dev_pm_opp_find_freq_floor_indexed(jenc->dev, &jenc->max_freq, JPEG_OPP_CORE_IDX);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	dev_pm_opp_put(opp);

	/* initialize the default optimized frequency for the JPEG core */
	jenc->opt_freq = jenc->max_freq;

	dev_dbg(jenc->dev, "JPEG max clocks is: %lu\n", jenc->max_freq);

	return 0;
}

static int qcom_jpeg_clk_init(struct qcom_jenc_dev *jenc)
{
	jenc->num_clks = devm_clk_bulk_get_all(jenc->dev, &jenc->clks);
	if (jenc->num_clks < 0)
		return jenc->num_clks;

	return 0;
}

static int qcom_jpeg_clk_on(struct qcom_jenc_dev *jenc)
{
	struct dev_pm_opp *opp;
	int rc;

	rc = clk_bulk_prepare_enable(jenc->num_clks, jenc->clks);
	if (rc)
		return rc;

	/* setup the OPP according to the calculated optimal frequency */
	opp = dev_pm_opp_find_freq_ceil_indexed(jenc->dev, &jenc->opt_freq, JPEG_OPP_CORE_IDX);
	if (IS_ERR(opp)) {
		rc = PTR_ERR(opp);
		goto err_clk_disable;
	}

	rc = dev_pm_opp_set_opp(jenc->dev, opp);
	if (rc)
		goto err_dev_pm_opp;

	dev_dbg(jenc->dev, "selected OPP clocks cnoc=%lu, core=%lu\n",
		dev_pm_opp_get_freq_indexed(opp, JPEG_OPP_CNOC_IDX),
		dev_pm_opp_get_freq_indexed(opp, JPEG_OPP_CORE_IDX));

	dev_pm_opp_put(opp);

	return 0;

err_dev_pm_opp:
	dev_pm_opp_put(opp);
err_clk_disable:
	clk_bulk_disable_unprepare(jenc->num_clks, jenc->clks);

	return rc;
}

static void qcom_jpeg_clk_off(struct qcom_jenc_dev *jenc)
{
	dev_pm_opp_set_opp(jenc->dev, NULL);
	clk_bulk_disable_unprepare(jenc->num_clks, jenc->clks);
	jenc->opt_freq = jenc->max_freq;
}

/* qcom_jpeg_camss_get - resume the parent CAMSS device */
static int qcom_jpeg_camss_get(struct qcom_jenc_dev *jenc)
{
	return pm_runtime_resume_and_get(jenc->camss_dev);
}

/* qcom_jpeg_camss_put - release the parent CAMSS device */
static void qcom_jpeg_camss_put(struct qcom_jenc_dev *jenc)
{
	pm_runtime_put_sync(jenc->camss_dev);
}

static int qcom_jpeg_pm_suspend(struct device *dev)
{
	struct qcom_jenc_dev *jenc = dev_get_drvdata(dev);

	qcom_jpeg_clk_off(jenc);
	qcom_jpeg_camss_put(jenc);

	return 0;
}

static int qcom_jpeg_pm_resume(struct device *dev)
{
	struct qcom_jenc_dev *jenc = dev_get_drvdata(dev);
	int rc;

	rc = qcom_jpeg_camss_get(jenc);
	if (rc)
		return rc;

	rc = qcom_jpeg_clk_on(jenc);
	if (rc) {
		qcom_jpeg_camss_put(jenc);
		return rc;
	}

	return 0;
}

static int qcom_jpeg_pm_system_suspend(struct device *dev)
{
	struct qcom_jenc_dev *jenc = dev_get_drvdata(dev);
	int rc;

	v4l2_m2m_suspend(jenc->m2m_dev);

	rc = pm_runtime_force_suspend(dev);
	if (rc)
		v4l2_m2m_resume(jenc->m2m_dev);

	return rc;
}

static int qcom_jpeg_pm_system_resume(struct device *dev)
{
	struct qcom_jenc_dev *jenc = dev_get_drvdata(dev);
	int rc;

	rc = pm_runtime_force_resume(dev);
	if (rc)
		return rc;

	v4l2_m2m_resume(jenc->m2m_dev);

	return 0;
}

static _DEFINE_DEV_PM_OPS(qcom_jpeg_pm_ops,
			  qcom_jpeg_pm_system_suspend, qcom_jpeg_pm_system_resume,
			  qcom_jpeg_pm_suspend, qcom_jpeg_pm_resume, NULL);

static int qcom_jpeg_probe(struct platform_device *pdev)
{
	const struct qcom_dev_resources *res;
	struct qcom_jenc_dev *jenc;
	int rc;

	jenc = devm_kzalloc(&pdev->dev, sizeof(*jenc), GFP_KERNEL);
	if (!jenc)
		return -ENOMEM;

	jenc->dev = &pdev->dev;
	jenc->camss_dev = pdev->dev.parent;
	platform_set_drvdata(pdev, jenc);
	rc = devm_mutex_init(&pdev->dev, &jenc->dev_mutex);
	if (rc)
		goto err_free_jenc;
	spin_lock_init(&jenc->hw_lock);
	init_completion(&jenc->reset_complete);
	init_completion(&jenc->stop_complete);

	res = device_get_match_data(jenc->dev);
	if (!res) {
		rc = dev_err_probe(jenc->dev, -ENODEV, "unsupported SoC\n");
		goto err_free_jenc;
	}
	jenc->res = res;

	if (!jenc->res->hw_ops) {
		rc = dev_err_probe(jenc->dev, -EINVAL, "missing hw resources\n");
		goto err_free_jenc;
	}

	rc = dma_set_mask_and_coherent(jenc->dev, DMA_BIT_MASK(32));
	if (rc) {
		dev_err_probe(jenc->dev, rc, "failed to set DMA mask\n");
		goto err_free_jenc;
	}

	jenc->jpeg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(jenc->jpeg_base)) {
		rc = dev_err_probe(jenc->dev, PTR_ERR(jenc->jpeg_base),
				   "failed to map JPEG resource\n");
		goto err_free_jenc;
	}

	rc = qcom_jpeg_opp_init(jenc);
	if (rc) {
		dev_err_probe(jenc->dev, rc, "failed to init OPP\n");
		goto err_free_jenc;
	}

	rc = qcom_jpeg_clk_init(jenc);
	if (rc) {
		dev_err_probe(jenc->dev, rc, "failed to init clocks\n");
		goto err_free_jenc;
	}

	jenc->irq = platform_get_irq(pdev, 0);
	if (jenc->irq < 0) {
		rc = dev_err_probe(jenc->dev, jenc->irq, "failed to get IRQ\n");
		goto err_free_jenc;
	}

	rc = devm_request_threaded_irq(jenc->dev, jenc->irq,
				       jenc->res->hw_ops->hw_irq_top,
				       jenc->res->hw_ops->hw_irq_bot,
				       IRQF_ONESHOT | IRQF_NO_AUTOEN, dev_name(jenc->dev), jenc);
	if (rc) {
		dev_err_probe(jenc->dev, rc, "failed to request IRQ\n");
		goto err_free_jenc;
	}

	rc = v4l2_device_register(jenc->dev, &jenc->v4l2_dev);
	if (rc) {
		dev_err_probe(jenc->dev, rc, "failed to register V4L2 device\n");
		goto err_free_jenc;
	}

	rc = devm_add_action_or_reset(jenc->dev,
				      (void (*)(void *))v4l2_device_unregister,
				      &jenc->v4l2_dev);
	if (rc)
		goto err_free_jenc;

	rc = devm_pm_runtime_enable(jenc->dev);
	if (rc)
		goto err_free_jenc;

	rc = qcom_jpeg_v4l2_register(jenc);
	if (rc) {
		dev_err_probe(jenc->dev, rc, "failed to register video device\n");
		goto err_free_jenc;
	}

	dev_dbg(jenc->dev, "Qualcomm JPEG encoder registered\n");

	return 0;

err_free_jenc:
	return rc;
}

static const struct of_device_id qcom_jpeg_of_match[] = {
	{
		.compatible	= "qcom,sm8250-jenc",
		.data		= &qcom_t165_t480_jpeg_drvdata
	},
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_jpeg_of_match);

static struct platform_driver qcom_jpeg_platform_driver = {
	.probe  = qcom_jpeg_probe,
	.driver = {
		.name = QCOM_JPEG_ENC_NAME,
		.of_match_table = qcom_jpeg_of_match,
		.pm             = pm_ptr(&qcom_jpeg_pm_ops),
	},
};

module_platform_driver(qcom_jpeg_platform_driver);

MODULE_DESCRIPTION("QCOM JPEG mem2mem V4L2 encoder");
MODULE_LICENSE("GPL");
