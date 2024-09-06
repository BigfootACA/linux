// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Junhao Xie <bigfoot@classfun.cn>
 */

#include <linux/kernel.h>
#include <linux/mfd/photonicat-pmu.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>

struct pcat_rtc {
	struct pcat_pmu *pmu;
	struct notifier_block nb;
	struct rtc_device *rtc;
	struct pcat_data_cmd_date_time time;
	struct completion initial_report;
};

static bool pcat_time_to_rtc_time(const struct pcat_data_cmd_date_time *time,
				  struct rtc_time *tm)
{
	if (time->second >= 60 || time->minute >= 60 || time->hour >= 24 ||
	    time->day <= 0 || time->day > 31 || time->month <= 0 ||
	    time->month > 12 || time->year < 1900 || time->year > 9999)
		return false;

	tm->tm_sec = time->second;
	tm->tm_min = time->minute;
	tm->tm_hour = time->hour;
	tm->tm_mday = time->day;
	tm->tm_mon = time->month - 1;
	tm->tm_year = time->year - 1900;
	tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, time->year);
	tm->tm_wday = ((time->year * (365 % 7)) + ((time->year - 1) / 4) -
		       ((time->year - 1) / 100) + ((time->year - 1) / 400) +
		       tm->tm_yday) % 7;

	return true;
}

static void pcat_time_from_rtc_time(struct pcat_data_cmd_date_time *time,
				    const struct rtc_time *tm)
{
	time->year = tm->tm_year + 1900;
	time->month = tm->tm_mon + 1;
	time->day = tm->tm_mday;
	time->hour = tm->tm_hour;
	time->minute = tm->tm_min;
	time->second = tm->tm_sec;
}

static int pcat_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct pcat_rtc *rtc = dev_get_drvdata(dev);

	memset(tm, 0, sizeof(*tm));
	if (!pcat_time_to_rtc_time(&rtc->time, tm))
		return -EINVAL;

	return 0;
}

static int pcat_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	int ret;
	struct pcat_rtc *rtc = dev_get_drvdata(dev);
	struct pcat_data_cmd_date_time time;

	pcat_time_from_rtc_time(&time, tm);

	ret = pcat_pmu_write_data(rtc->pmu, PCAT_CMD_DATE_TIME_SYNC,
				  &time, sizeof(time));
	if (ret)
		return ret;

	memcpy(&rtc->time, &time, sizeof(rtc->time));

	return 0;
}

static const struct rtc_class_ops pcat_rtc_ops = {
	.read_time = pcat_rtc_read_time,
	.set_time = pcat_rtc_set_time,
};

static int pcat_rtc_notify(struct notifier_block *nb, unsigned long action,
			   void *data)
{
	struct pcat_rtc *rtc = container_of(nb, struct pcat_rtc, nb);
	struct pcat_data *frame = data;
	struct pcat_data_cmd_status *status = frame->data;

	if (action != PCAT_CMD_STATUS_REPORT)
		return NOTIFY_DONE;

	memcpy(&rtc->time, &status->time, sizeof(rtc->time));
	complete(&rtc->initial_report);

	return NOTIFY_DONE;
}

static int pcat_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pcat_rtc *rtc;
	int ret;

	rtc = devm_kzalloc(&pdev->dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc->pmu = dev_get_drvdata(dev->parent);
	rtc->nb.notifier_call = pcat_rtc_notify;
	init_completion(&rtc->initial_report);
	platform_set_drvdata(pdev, rtc);

	ret = pcat_pmu_register_notify(rtc->pmu, &rtc->nb);
	if (ret)
		return ret;

	if (!wait_for_completion_timeout(&rtc->initial_report,
					 msecs_to_jiffies(3000))) {
		ret = dev_err_probe(dev, -ETIMEDOUT,
				    "timeout waiting for initial report\n");
		goto error;
	}

	dev_info(dev, "RTC Time: %04d/%02d/%02d %02d:%02d:%02d\n",
		 rtc->time.year, rtc->time.month, rtc->time.day, rtc->time.hour,
		 rtc->time.minute, rtc->time.second);

	rtc->rtc = devm_rtc_device_register(&pdev->dev, "pcat-rtc",
					    &pcat_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc)) {
		ret = PTR_ERR(rtc->rtc);
		dev_err_probe(&pdev->dev, ret,
			      "Failed to register RTC device\n");
		goto error;
	}

	return 0;
error:
	pcat_pmu_unregister_notify(rtc->pmu, &rtc->nb);
	return ret;
}

static void pcat_rtc_remove(struct platform_device *pdev)
{
	struct pcat_rtc *rtc = platform_get_drvdata(pdev);

	pcat_pmu_unregister_notify(rtc->pmu, &rtc->nb);
}

static struct platform_driver pcat_rtc_driver = {
	.driver = {
		.name = "photonicat-rtc",
	},
	.probe = pcat_rtc_probe,
	.remove = pcat_rtc_remove,
};
module_platform_driver(pcat_rtc_driver);

MODULE_AUTHOR("Junhao Xie <bigfoot@classfun.cn>");
MODULE_ALIAS("platform:photonicat-rtc");
MODULE_DESCRIPTION("Photonicat PMU RTC");
MODULE_LICENSE("GPL");
