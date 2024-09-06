// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Junhao Xie <bigfoot@classfun.cn>
 */

#include <linux/mfd/photonicat-pmu.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>

struct pcat_watchdog {
	struct pcat_pmu *pmu;
	struct watchdog_device wdd;
};

static const struct watchdog_info pcat_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "Photonicat PMU Watchdog",
};

static int pcat_wdt_setup(struct pcat_watchdog *data, int timeout)
{
	struct pcat_data_cmd_watchdog timeouts = { 60, 60, 0 };

	timeouts.running_timeout = MIN(255, MAX(0, timeout));

	return pcat_pmu_write_data(data->pmu, PCAT_CMD_WATCHDOG_TIMEOUT_SET,
				   &timeouts, sizeof(timeouts));
}

static int pcat_wdt_start(struct watchdog_device *wdev)
{
	struct pcat_watchdog *data = watchdog_get_drvdata(wdev);

	return pcat_wdt_setup(data, data->wdd.timeout);
}

static int pcat_wdt_stop(struct watchdog_device *wdev)
{
	struct pcat_watchdog *data = watchdog_get_drvdata(wdev);

	return pcat_wdt_setup(data, 0);
}

static int pcat_wdt_ping(struct watchdog_device *wdev)
{
	struct pcat_watchdog *data = watchdog_get_drvdata(wdev);

	return pcat_pmu_send(data->pmu, PCAT_CMD_HEARTBEAT, NULL, 0);
}

static int pcat_wdt_set_timeout(struct watchdog_device *wdev, unsigned int val)
{
	int ret = 0;
	struct pcat_watchdog *data = watchdog_get_drvdata(wdev);

	if (watchdog_active(&data->wdd))
		ret = pcat_wdt_setup(data, val);

	return ret;
}

static const struct watchdog_ops pcat_wdt_ops = {
	.owner = THIS_MODULE,
	.start = pcat_wdt_start,
	.stop = pcat_wdt_stop,
	.ping = pcat_wdt_ping,
	.set_timeout = pcat_wdt_set_timeout,
};

static int pcat_watchdog_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pcat_watchdog *watchdog;

	watchdog = devm_kzalloc(dev, sizeof(*watchdog), GFP_KERNEL);
	if (!watchdog)
		return -ENOMEM;

	watchdog->pmu = dev_get_drvdata(dev->parent);
	watchdog->wdd.info = &pcat_wdt_info;
	watchdog->wdd.ops = &pcat_wdt_ops;
	watchdog->wdd.timeout = 60;
	watchdog->wdd.max_timeout = U8_MAX;
	watchdog->wdd.min_timeout = 1;
	watchdog->wdd.parent = dev;

	watchdog_stop_on_reboot(&watchdog->wdd);
	watchdog_init_timeout(&watchdog->wdd, watchdog->wdd.timeout, dev);
	watchdog_set_drvdata(&watchdog->wdd, watchdog);
	platform_set_drvdata(pdev, watchdog);

	return devm_watchdog_register_device(dev, &watchdog->wdd);
}

static struct platform_driver pcat_watchdog_driver = {
	.driver = {
		.name = "photonicat-watchdog",
	},
	.probe = pcat_watchdog_probe,
};
module_platform_driver(pcat_watchdog_driver);

MODULE_AUTHOR("Junhao Xie <bigfoot@classfun.cn>");
MODULE_ALIAS("platform:photonicat-watchdog");
MODULE_DESCRIPTION("Photonicat PMU watchdog");
MODULE_LICENSE("GPL");
