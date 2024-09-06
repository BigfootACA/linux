// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Junhao Xie <bigfoot@classfun.cn>
 */

#include <linux/completion.h>
#include <linux/hwmon.h>
#include <linux/mfd/photonicat-pmu.h>
#include <linux/module.h>
#include <linux/platform_device.h>

struct pcat_hwmon {
	struct pcat_pmu *pmu;
	struct notifier_block nb;
	struct device *hwmon;
	int temperature;
	struct completion initial_report;
};

static umode_t pcat_hwmon_is_visible(const void *data,
				     enum hwmon_sensor_types type,
				     u32 attr, int channel)
{
	return 0444;
}

static int pcat_hwmon_read(struct device *dev,
			   enum hwmon_sensor_types type,
			   u32 attr, int channel, long *val)
{
	struct pcat_hwmon *hwmon = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			*val = hwmon->temperature * 1000;
			break;
		default:
			return -EOPNOTSUPP;
		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static const struct hwmon_channel_info *pcat_hwmon_info[] = {
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT),
	NULL
};

static const struct hwmon_ops pcat_hwmon_hwmon_ops = {
	.is_visible = pcat_hwmon_is_visible,
	.read = pcat_hwmon_read,
};

static const struct hwmon_chip_info pcat_hwmon_chip_info = {
	.ops = &pcat_hwmon_hwmon_ops,
	.info = pcat_hwmon_info,
};

static int pcat_hwmon_notify(struct notifier_block *nb, unsigned long action,
			     void *data)
{
	struct pcat_hwmon *hwmon = container_of(nb, struct pcat_hwmon, nb);
	struct pcat_data *frame = data;
	struct pcat_data_cmd_status *status = frame->data;

	if (action != PCAT_CMD_STATUS_REPORT)
		return NOTIFY_DONE;

	hwmon->temperature = status->temp - 40;
	complete(&hwmon->initial_report);

	return NOTIFY_DONE;
}

static int pcat_hwmon_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct pcat_hwmon *hwmon;

	hwmon = devm_kzalloc(dev, sizeof(*hwmon), GFP_KERNEL);
	if (!hwmon)
		return -ENOMEM;

	hwmon->pmu = dev_get_drvdata(dev->parent);
	hwmon->nb.notifier_call = pcat_hwmon_notify;
	init_completion(&hwmon->initial_report);
	platform_set_drvdata(pdev, hwmon);

	ret = pcat_pmu_register_notify(hwmon->pmu, &hwmon->nb);
	if (ret)
		return ret;

	if (!wait_for_completion_timeout(&hwmon->initial_report,
					 msecs_to_jiffies(3000))) {
		ret = dev_err_probe(dev, -ETIMEDOUT,
				    "timeout waiting for initial report\n");
		goto error;
	}

	dev_info(dev, "Board Temperature: %d degress C\n", hwmon->temperature);

	hwmon->hwmon = devm_hwmon_device_register_with_info(
		dev, "pcat_pmu", hwmon, &pcat_hwmon_chip_info, NULL);

	if (IS_ERR(hwmon->hwmon)) {
		ret = PTR_ERR(hwmon->hwmon);
		dev_err_probe(&pdev->dev, ret,
			      "Failed to register hwmon device\n");
		goto error;
	}

	return 0;
error:
	pcat_pmu_unregister_notify(hwmon->pmu, &hwmon->nb);
	return ret;
}

static void pcat_hwmon_remove(struct platform_device *pdev)
{
	struct pcat_hwmon *hwmon = platform_get_drvdata(pdev);

	pcat_pmu_unregister_notify(hwmon->pmu, &hwmon->nb);
}

static struct platform_driver pcat_hwmon_driver = {
	.driver = {
		.name = "photonicat-hwmon",
	},
	.probe = pcat_hwmon_probe,
	.remove = pcat_hwmon_remove,
};
module_platform_driver(pcat_hwmon_driver);

MODULE_AUTHOR("Junhao Xie <bigfoot@classfun.cn>");
MODULE_ALIAS("platform:photonicat-hwmon");
MODULE_DESCRIPTION("Photonicat PMU Hardware Monitor");
MODULE_LICENSE("GPL");
