// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 BigfootACA
 */

#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/firmware/qcom/qcom_tzmem.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define QSEE_LOG_DIAG_FEATURE_ID		6
#define QSEE_LOG_LEGACY_SIZE		SZ_32K
#define QSEE_LOG_V2_SIZE			SZ_128K
#define QSEE_LOG_DIAG_MAJOR_SHIFT	22
#define QSEE_LOG_DIAG_MINOR_SHIFT	12
#define QSEE_LOG_DIAG_VERSION_MASK	GENMASK(9, 0)
#define QSEE_LOG_DIAG_MAJOR_V9		9
#define QSEE_LOG_DIAG_MINOR_V2		2
#define QSEE_LOG_DIAG_MINOR_V21		3
#define QSEE_LOG_DIAG_MINOR_V22		4

struct qcom_qsee_log {
	struct debugfs_blob_wrapper blob;
	struct dentry *debugfs_dir;
	void *vaddr;
	dma_addr_t dma_addr;
	size_t size;
	u64 bridge_handle;
};

static bool qcom_qsee_log_uses_v2(u64 version)
{
	u32 major = (version >> QSEE_LOG_DIAG_MAJOR_SHIFT) &
		    QSEE_LOG_DIAG_VERSION_MASK;
	u32 minor = (version >> QSEE_LOG_DIAG_MINOR_SHIFT) &
		    QSEE_LOG_DIAG_VERSION_MASK;

	return major == QSEE_LOG_DIAG_MAJOR_V9 &&
	       (minor == QSEE_LOG_DIAG_MINOR_V2 ||
		minor == QSEE_LOG_DIAG_MINOR_V21 ||
		minor == QSEE_LOG_DIAG_MINOR_V22);
}

static int qcom_qsee_log_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qcom_qsee_log *qsee_log;
	struct dentry *log_file;
	u64 diag_version = 0;
	u32 feature = QSEE_LOG_DIAG_FEATURE_ID;
	u32 major;
	u32 minor;
	int ret;

	if (!qcom_scm_is_available())
		return -EPROBE_DEFER;

	qsee_log = devm_kzalloc(dev, sizeof(*qsee_log), GFP_KERNEL);
	if (!qsee_log)
		return -ENOMEM;

	ret = qcom_scm_qseecom_get_feature_version(feature, &diag_version);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to query diagnostic version\n");

	qsee_log->size = qcom_qsee_log_uses_v2(diag_version) ?
			 QSEE_LOG_V2_SIZE : QSEE_LOG_LEGACY_SIZE;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set 32-bit DMA mask\n");

	qsee_log->vaddr = dma_alloc_coherent(dev, qsee_log->size,
					     &qsee_log->dma_addr,
					     GFP_KERNEL | GFP_DMA);
	if (!qsee_log->vaddr)
		return -ENOMEM;

	if (qsee_log->dma_addr > U32_MAX ||
	    qsee_log->size - 1 > U32_MAX - qsee_log->dma_addr) {
		dev_err(dev, "log buffer is outside the 32-bit address range\n");
		ret = -ERANGE;
		goto err_free_dma;
	}

	memset(qsee_log->vaddr, 0, qsee_log->size);
	ret = qcom_tzmem_shm_bridge_create(qsee_log->dma_addr, qsee_log->size,
					   &qsee_log->bridge_handle);
	if (ret)
		goto err_free_dma;

	qsee_log->debugfs_dir = debugfs_create_dir("qcom_qsee_log", NULL);
	if (IS_ERR_OR_NULL(qsee_log->debugfs_dir)) {
		ret = qsee_log->debugfs_dir ?
			PTR_ERR(qsee_log->debugfs_dir) : -ENOMEM;
		goto err_delete_bridge;
	}

	qsee_log->blob.data = qsee_log->vaddr;
	qsee_log->blob.size = qsee_log->size;
	log_file = debugfs_create_blob("log", 0400, qsee_log->debugfs_dir,
				       &qsee_log->blob);
	if (IS_ERR_OR_NULL(log_file)) {
		ret = log_file ? PTR_ERR(log_file) : -ENOMEM;
		goto err_remove_debugfs;
	}

	ret = qcom_scm_qseecom_register_log_buf(qsee_log->dma_addr,
						qsee_log->size);
	if (ret)
		goto err_remove_debugfs;

	major = (diag_version >> QSEE_LOG_DIAG_MAJOR_SHIFT) &
		QSEE_LOG_DIAG_VERSION_MASK;
	minor = (diag_version >> QSEE_LOG_DIAG_MINOR_SHIFT) &
		QSEE_LOG_DIAG_VERSION_MASK;
	dev_info(dev,
		 "registered %zu-byte log buffer at %pad, diagnostic version %u.%u\n",
		 qsee_log->size, &qsee_log->dma_addr, major, minor);
	platform_set_drvdata(pdev, qsee_log);

	return 0;

err_remove_debugfs:
	debugfs_remove_recursive(qsee_log->debugfs_dir);
err_delete_bridge:
	qcom_tzmem_shm_bridge_delete(qsee_log->bridge_handle);
err_free_dma:
	dma_free_coherent(dev, qsee_log->size, qsee_log->vaddr,
			  qsee_log->dma_addr);
	return ret;
}

static const struct of_device_id qcom_qsee_log_of_match[] = {
	{ .compatible = "qcom,qsee-log" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_qsee_log_of_match);

static struct platform_driver qcom_qsee_log_driver = {
	.probe = qcom_qsee_log_probe,
	.driver = {
		.name = "qcom-qsee-log",
		.of_match_table = qcom_qsee_log_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(qcom_qsee_log_driver);

MODULE_DESCRIPTION("Qualcomm QSEE raw log buffer driver");
MODULE_LICENSE("GPL");
