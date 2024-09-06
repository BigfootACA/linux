// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Junhao Xie <bigfoot@classfun.cn>
 */

#include <linux/module.h>
#include <linux/mfd/photonicat-pmu.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

struct pcat_supply {
	struct device *dev;
	struct pcat_pmu *pmu;
	struct notifier_block nb;
	struct power_supply *bat_psy;
	struct power_supply *chg_psy;
	struct power_supply_battery_info *bat_info;
	u16 bat_microvolt;
	u16 chg_microvolt;
	struct completion initial_report;
};

static bool pcat_pmu_is_charger_online(struct pcat_supply *supply)
{
	return supply->chg_microvolt > 1000;
}

static bool pcat_pmu_is_battery_present(struct pcat_supply *supply)
{
	return supply->bat_microvolt > 1000;
}

static int pcat_pmu_get_battery_capacity(struct pcat_supply *supply)
{
	return power_supply_batinfo_ocv2cap(supply->bat_info,
					    supply->bat_microvolt * 1000, 20);
}

static int pcat_pmu_get_battery_energy(struct pcat_supply *supply)
{
	int capacity;

	capacity = pcat_pmu_get_battery_capacity(supply);
	if (capacity < 0)
		return 0;

	return supply->bat_info->energy_full_design_uwh / 100 * capacity;
}

static int pcat_pmu_get_battery_status(struct pcat_supply *supply)
{
	if (pcat_pmu_get_battery_capacity(supply) < 100) {
		if (pcat_pmu_is_charger_online(supply))
			return POWER_SUPPLY_STATUS_CHARGING;
		else
			return POWER_SUPPLY_STATUS_DISCHARGING;
	}

	return POWER_SUPPLY_STATUS_FULL;
}

static int pcat_pmu_get_battery_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct pcat_supply *supply = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = pcat_pmu_get_battery_capacity(supply);
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		val->intval = supply->bat_info->energy_full_design_uwh;
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		val->intval = pcat_pmu_get_battery_energy(supply);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = pcat_pmu_is_battery_present(supply);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = pcat_pmu_get_battery_status(supply);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = supply->bat_info->voltage_max_design_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = supply->bat_info->voltage_min_design_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = supply->bat_microvolt * 1000;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int pcat_pmu_get_charger_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct pcat_supply *supply = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pcat_pmu_is_charger_online(supply);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = supply->chg_microvolt * 1000;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property pcat_battery_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static enum power_supply_property pcat_charger_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_ONLINE,
};

static int pcat_supply_notify(struct notifier_block *nb, unsigned long action,
			      void *data)
{
	struct pcat_supply *supply = container_of(nb, struct pcat_supply, nb);
	struct pcat_data *frame = data;
	struct pcat_data_cmd_status *status = frame->data;

	if (action != PCAT_CMD_STATUS_REPORT)
		return NOTIFY_DONE;

	supply->bat_microvolt = status->battery_microvolt;
	supply->chg_microvolt = status->charger_microvolt;

	complete(&supply->initial_report);

	return NOTIFY_DONE;
}

static const struct power_supply_desc pcat_bat_desc = {
	.name		= "pcat_battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= pcat_battery_props,
	.num_properties	= ARRAY_SIZE(pcat_battery_props),
	.get_property	= pcat_pmu_get_battery_property,
};

static const struct power_supply_desc pcat_chg_desc = {
	.name		= "pcat_charger",
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.properties	= pcat_charger_props,
	.num_properties	= ARRAY_SIZE(pcat_charger_props),
	.get_property	= pcat_pmu_get_charger_property,
};

static int pcat_supply_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct pcat_supply *supply;
	struct power_supply_config psy_cfg = {};

	supply = devm_kzalloc(dev, sizeof(*supply), GFP_KERNEL);
	if (!supply)
		return -ENOMEM;

	supply->dev = dev;
	supply->pmu = dev_get_drvdata(dev->parent);
	supply->nb.notifier_call = pcat_supply_notify;
	init_completion(&supply->initial_report);
	psy_cfg.drv_data = supply;
	psy_cfg.of_node = dev->parent->of_node;
	platform_set_drvdata(pdev, supply);

	ret = pcat_pmu_register_notify(supply->pmu, &supply->nb);
	if (ret)
		return ret;

	if (!wait_for_completion_timeout(&supply->initial_report,
					 msecs_to_jiffies(3000))) {
		ret = dev_err_probe(dev, -ETIMEDOUT,
				    "timeout waiting for initial report\n");
		goto error;
	}

	dev_info(dev, "Battery Voltage: %u mV\n", supply->bat_microvolt);
	dev_info(dev, "Charger Voltage: %u mV\n", supply->chg_microvolt);

	supply->bat_psy = devm_power_supply_register(dev, &pcat_bat_desc,
						     &psy_cfg);
	if (IS_ERR(supply->bat_psy)) {
		ret = PTR_ERR(supply->bat_psy);
		dev_err_probe(dev, ret, "Failed to register battery supply\n");
		goto error;
	}

	supply->chg_psy = devm_power_supply_register(dev, &pcat_chg_desc,
						     &psy_cfg);
	if (IS_ERR(supply->chg_psy)) {
		ret = PTR_ERR(supply->chg_psy);
		dev_err_probe(dev, ret, "Failed to register charger supply\n");
		goto error;
	}

	ret = power_supply_get_battery_info(supply->bat_psy,
					    &supply->bat_info);
	if (ret) {
		dev_err_probe(dev, ret, "Unable to get battery info\n");
		goto error;
	}

	return 0;
error:
	pcat_pmu_unregister_notify(supply->pmu, &supply->nb);
	return ret;
}

static void pcat_supply_remove(struct platform_device *pdev)
{
	struct pcat_supply *supply = platform_get_drvdata(pdev);

	pcat_pmu_unregister_notify(supply->pmu, &supply->nb);
}

static struct platform_driver pcat_supply_driver = {
	.driver = {
		.name = "photonicat-supply",
	},
	.probe = pcat_supply_probe,
	.remove = pcat_supply_remove,
};
module_platform_driver(pcat_supply_driver);

MODULE_AUTHOR("Junhao Xie <bigfoot@classfun.cn>");
MODULE_ALIAS("platform:photonicat-supply");
MODULE_DESCRIPTION("Photonicat PMU Power Supply");
MODULE_LICENSE("GPL");
