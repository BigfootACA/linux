// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Junhao Xie <bigfoot@classfun.cn>
 */

#include <linux/mfd/photonicat-pmu.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>

struct pcat_poweroff {
	struct device *dev;
	struct pcat_pmu *pmu;
	struct notifier_block nb;
};

static int pcat_do_poweroff(struct sys_off_data *data)
{
	struct pcat_poweroff *poweroff = data->cb_data;

	dev_info(poweroff->dev, "Host request PMU shutdown\n");
	pcat_pmu_write_data(poweroff->pmu, PCAT_CMD_HOST_REQUEST_SHUTDOWN,
			    NULL, 0);

	return NOTIFY_DONE;
}

static int pcat_poweroff_notify(struct notifier_block *nb, unsigned long action,
				void *data)
{
	struct pcat_poweroff *poweroff =
		container_of(nb, struct pcat_poweroff, nb);
	struct pcat_data *frame = data;
	const char *reason = "(unknown)";

	if (action != PCAT_CMD_PMU_REQUEST_SHUTDOWN)
		return NOTIFY_DONE;

	if (frame->size >= 1) {
		switch (*(enum pcat_shutdown_reason *)frame->data) {
		case PCAT_SHUTDOWN_BUTTON:
			reason = "power button";
			break;
		case PCAT_SHUTDOWN_POWER_LOW:
			reason = "battery voltage low";
			break;
		case PCAT_SHUTDOWN_UPGRADE:
			reason = "system upgrade";
			break;
		case PCAT_SHUTDOWN_OTHER:
			reason = "other reason";
			break;
		}
	}

	dev_info(poweroff->dev, "PMU request host shutdown: %s\n", reason);
	orderly_poweroff(true);

	return NOTIFY_DONE;
}

static int pcat_poweroff_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct pcat_poweroff *poweroff;

	poweroff = devm_kzalloc(dev, sizeof(*poweroff), GFP_KERNEL);
	if (!poweroff)
		return -ENOMEM;

	poweroff->dev = dev;
	poweroff->pmu = dev_get_drvdata(dev->parent);
	poweroff->nb.notifier_call = pcat_poweroff_notify;
	platform_set_drvdata(pdev, poweroff);

	ret = devm_register_sys_off_handler(&pdev->dev,
					    SYS_OFF_MODE_POWER_OFF,
					    SYS_OFF_PRIO_DEFAULT,
					    pcat_do_poweroff,
					    poweroff);
	if (ret)
		return ret;

	return pcat_pmu_register_notify(poweroff->pmu, &poweroff->nb);
}

static void pcat_poweroff_remove(struct platform_device *pdev)
{
	struct pcat_poweroff *poweroff = platform_get_drvdata(pdev);

	pcat_pmu_unregister_notify(poweroff->pmu, &poweroff->nb);
}

static struct platform_driver pcat_poweroff_driver = {
	.driver = {
		.name = "photonicat-poweroff",
	},
	.probe = pcat_poweroff_probe,
	.remove = pcat_poweroff_remove,
};
module_platform_driver(pcat_poweroff_driver);

MODULE_AUTHOR("Junhao Xie <bigfoot@classfun.cn>");
MODULE_ALIAS("platform:photonicat-poweroff");
MODULE_DESCRIPTION("Photonicat PMU Poweroff");
MODULE_LICENSE("GPL");
