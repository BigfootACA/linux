// SPDX-License-Identifier: GPL-2.0+
/**
 * Driver for Allwinner H616 GPU power domain
 *
 * Copyright (c) 2025 Junhao Xie <bigfoot@classfun.cn>
 * 
 * Someone say it is "H616 GPU Power Domain", but I don't known what is this...
 * Some guys wrote it in U-Boot, but it's horrible.
 * Original value is 1, just write 0 to here.
 * If you dont do this, GPU_INT_MASK will hang.
 *
 * Never send this patch to mainline, this patch need refactor.
 *
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>

struct sun50i_gpu_pd {
	struct device *dev;
	struct generic_pm_domain genpd;
	struct generic_pm_domain *genpds[1];
	struct genpd_onecell_data cell;
	void __iomem *base;
};

#define to_sun50i_gpu_pd(_genpd) \
	container_of(_genpd, struct sun50i_gpu_pd, genpd)

static int sun50i_gpu_pd_power_on(struct generic_pm_domain *genpd)
{
	const struct sun50i_gpu_pd *pd = to_sun50i_gpu_pd(genpd);
	u32 val;

	val = readl(pd->base);
	val &= ~BIT(0);
	writel(val, pd->base);

	return 0;
}

static int sun50i_gpu_pd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sun50i_gpu_pd *pd;
	int ret;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;
	platform_set_drvdata(pdev, pd);
	pd->dev = dev;

	pd->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pd->base))
		return PTR_ERR(pd->base);

	pd->genpd.name		= "GPU";
	pd->genpd.power_on	= sun50i_gpu_pd_power_on;

	ret = pm_genpd_init(&pd->genpd, NULL, true);
	if (ret) {
		dev_err(dev, "Failed to add pd-domain: %d\n", ret);
		return ret;
	}

	pd->cell.num_domains = 1;
	pd->cell.domains = pd->genpds;
	pd->cell.domains[0] = &pd->genpd;

	ret = of_genpd_add_provider_onecell(dev->of_node, &pd->cell);
	if (ret)
		dev_warn(dev, "Failed to add provider: %d\n", ret);

	return 0;
}

static const struct of_device_id sun50i_gpu_pd_of_match[] = {
	{.compatible = "allwinner,sun50i-h616-gpu-pd", },
	{ }
};
MODULE_DEVICE_TABLE(of, sun50i_gpu_pd_of_match);

static struct platform_driver sun50i_gpu_pd_driver = {
	.probe	= sun50i_gpu_pd_probe,
	.driver	= {
		.name			= "sun50i-gpu-pd",
		.of_match_table		= sun50i_gpu_pd_of_match,
		/* Power domains cannot be removed while they are in use. */
		.suppress_bind_attrs	= true,
	},
};
module_platform_driver(sun50i_gpu_pd_driver);

MODULE_AUTHOR("Junhao Xie <bigfoot@classfun.cn>");
MODULE_DESCRIPTION("Allwinner H616 GPU power domain driver");
MODULE_LICENSE("GPL");
