// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Junhao Xie <bigfoot@classfun.cn>
 */

#include <linux/mfd/photonicat-pmu.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

struct pcat_leds {
	struct pcat_pmu *pmu;
	struct led_classdev cdev;
};

static int pcat_led_status_set(struct led_classdev *cdev,
			       enum led_brightness brightness)
{
	struct pcat_leds *leds = container_of(cdev, struct pcat_leds, cdev);
	struct pcat_data_cmd_led_setup setup = { 0, 0, 0 };

	if (brightness)
		setup.on_time = 100;
	else
		setup.down_time = 100;
	return pcat_pmu_write_data(leds->pmu, PCAT_CMD_NET_STATUS_LED_SETUP,
				   &setup, sizeof(setup));
}

static int pcat_leds_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pcat_leds *leds;

	leds = devm_kzalloc(dev, sizeof(*leds), GFP_KERNEL);
	if (!leds)
		return -ENOMEM;

	leds->pmu = dev_get_drvdata(dev->parent);
	platform_set_drvdata(pdev, leds);

	leds->cdev.name = "net-status";
	leds->cdev.max_brightness = 1;
	leds->cdev.brightness_set_blocking = pcat_led_status_set;

	return devm_led_classdev_register(dev, &leds->cdev);
}

static struct platform_driver pcat_leds_driver = {
	.driver = {
		.name = "photonicat-leds",
	},
	.probe = pcat_leds_probe,
};
module_platform_driver(pcat_leds_driver);

MODULE_AUTHOR("Junhao Xie <bigfoot@classfun.cn>");
MODULE_ALIAS("platform:photonicat-leds");
MODULE_DESCRIPTION("Photonicat PMU Status LEDs");
MODULE_LICENSE("GPL");
