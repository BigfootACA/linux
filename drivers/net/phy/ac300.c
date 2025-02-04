// SPDX-License-Identifier: GPL-2.0+
/**
 * Driver for X-Powers AC300 Ethernet PHY
 *
 * Copyright (c) 2025 Junhao Xie <bigfoot@classfun.cn>
 */

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <linux/phy.h>

#define AC300_EPHY_ID			0x00441400
#define AC300_EPHY_ID_MASK		0x0ffffff0
#define AC300_TOP_ID			0xc0000000
#define AC300_TOP_ID_MASK		0xffffffff

struct ac300_priv {
	uint16_t calib;
};

static int ac300_enable(struct phy_device *phydev)
{
	phy_write(phydev, 0x00, 0x1f83); /* release reset */
	phy_write(phydev, 0x00, 0x1fb7); /* clk gating (24MHz clock)*/
	phy_write(phydev, 0x05, 0xa819);
	phy_write(phydev, 0x06, 0x00);

	return 0;
}

static int ac300_disable(struct phy_device *phydev)
{
	phy_write(phydev, 0x00, 0x1f40);
	phy_write(phydev, 0x05, 0xa800);
	phy_write(phydev, 0x06, 0x01);

	return 0;
}

static void ac300_page(struct phy_device *phydev, int page)
{
	phy_write(phydev, 0x1f, page << 8);
}

static void disable_intelligent_ieee(struct phy_device *phydev)
{
	ac300_page(phydev, 1);
	phy_clear_bits(phydev, 0x17, BIT(3));	/* disable intelligent EEE */
}

static void disable_802_3az_ieee(struct phy_device *phydev)
{
	int value;

	ac300_page(phydev, 0);
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x3c);
	phy_write(phydev, 0xd, BIT(14) | 0x7);
	value = phy_read(phydev, 0xe);
	value &= ~BIT(1);
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x3c);
	phy_write(phydev, 0xd, BIT(14) | 0x7);
	phy_write(phydev, 0xe, value);

	ac300_page(phydev, 2);
	phy_write(phydev, 0x18, 0x0000);
}

static void ephy_config_default(struct phy_device *phydev)
{
	ac300_page(phydev, 1);
	phy_write(phydev, 0x12, 0x4824); /* Disable APS */

	ac300_page(phydev, 2);
	phy_write(phydev, 0x18, 0x0000); /* PHYAFE TRX optimization */

	ac300_page(phydev, 6);
	phy_write(phydev, 0x14, 0x708b); /* PHYAFE TX optimization */
	phy_write(phydev, 0x13, 0xF000); /* PHYAFE RX optimization */
	phy_write(phydev, 0x15, 0x1530);

	ac300_page(phydev, 8);
	phy_write(phydev, 0x18, 0x00bc); /* PHYAFE TRX optimization */
}

static void ephy_config_fixed(struct phy_device *phydev)
{
	ac300_page(phydev, 1);
	phy_write(phydev, 0x12, 0x4824); /*Disable APS */

	ac300_page(phydev, 2);
	phy_write(phydev, 0x18, 0x0000); /*PHYAFE TRX optimization */

	ac300_page(phydev, 6);
	phy_write(phydev, 0x14, 0x7809); /*PHYAFE TX optimization */
	phy_write(phydev, 0x13, 0xf000); /*PHYAFE RX optimization */
	phy_write(phydev, 0x10, 0x5523);
	phy_write(phydev, 0x15, 0x3533);

	ac300_page(phydev, 8);
	phy_write(phydev, 0x1d, 0x0844); /*disable auto offset */
	phy_write(phydev, 0x18, 0x00bc); /*PHYAFE TRX optimization */
}

static void ephy_config_cali(struct phy_device *phydev, u16 ephy_cali)
{
	int value;

	value = phy_read(phydev, 0x06);
	value &= ~(0x0F << 12);
	value |= (0x0F & (0x03 + ephy_cali)) << 12;
	phy_write(phydev, 0x06, value);
}

static int ac300_init(struct phy_device *phydev)
{
	int value;
	struct device *dev = &phydev->mdio.dev;
	struct ac300_priv *priv = phydev->priv;

	ac300_enable(phydev);
	msleep(100);

	ephy_config_cali(phydev, priv->calib);

	if (priv->calib & BIT(9)) {
		dev_dbg(dev, "use fixed config\n");
		ephy_config_fixed(phydev);
	} else {
		dev_dbg(dev, "use default config\n");
		ephy_config_default(phydev);
	}

	disable_intelligent_ieee(phydev); /* Disable Intelligent IEEE */
	disable_802_3az_ieee(phydev);     /* Disable 802.3az IEEE */

	ac300_page(phydev, 0);
	value = phy_read(phydev, 0x06);
	value |= BIT(11);
	value |= BIT(1);
	phy_write(phydev, 0x06, value); /* LED_POL 1: Low active */

	phy_set_bits(phydev, 0x13, BIT(12));

	return 0;
}

static void ac300_remove(struct phy_device *phydev)
{
	ac300_disable(phydev);
}

static int ac300_read_calibration(struct device *dev, u16 *out)
{
	struct nvmem_cell *cell;
	uint16_t *data;
	size_t len;
	int ret = 0;

	cell = devm_nvmem_cell_get(dev, "calibration");
	if (IS_ERR(cell)) {
		ret = PTR_ERR(cell);
		dev_err(dev, "Failed to get calibration nvmem cell: %d\n", ret);
		return ret;
	}

	data = nvmem_cell_read(cell, &len);
	if (IS_ERR(data)) {
		ret = PTR_ERR(cell);
		dev_err(dev, "Failed to read calibration data: %d\n", ret);
		return ret;
	}

	if (len < sizeof(uint16_t)) {
		ret = -EINVAL;
		dev_err(dev, "Bad nvmem calibration cell size\n");
		goto out;
	}

	*out = *data;
	dev_dbg(dev, "calibration value: 0x%x\n", *out);

out:
	kfree(data);
	return ret;
}

static int ac300_top_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct ac300_priv *priv;
	struct clk *clk;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	phydev->priv = priv;

	clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "Failed to request clock\n");

	ret = ac300_read_calibration(dev, &priv->calib);
	if (ret)
		return dev_err_probe(dev, PTR_ERR(clk),
				     "Failed to read calibration\n");

	ac300_init(phydev);

	return 0;
}

static struct phy_device *ac300_find_top(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct device_node *top_node;
	struct phy_device *top_phydev;

	top_node = of_parse_phandle(dev->of_node, "phy-top-handle", 0);
	if (!top_node) {
		dev_err(dev, "No phy-top-handle property\n");
		return ERR_PTR(-ENODEV);
	}

	top_phydev = of_phy_find_device(top_node);
	if (!top_phydev) {
		top_phydev = ERR_PTR(-EPROBE_DEFER);
		goto out_put_node;
	}

	if (!top_phydev->drv || top_phydev->drv->phy_id != AC300_TOP_ID) {
		dev_err(dev, "Bad top phy device\n");
		top_phydev = ERR_PTR(-EPROBE_DEFER);
		goto out_put_node;
	}

out_put_node:
	of_node_put(top_node);
	return top_phydev;
}

static int ac300_ephy_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct ac300_priv *priv, *priv_top;
	struct phy_device *top_phydev;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	phydev->priv = priv;

	top_phydev = ac300_find_top(phydev);
	if (IS_ERR(top_phydev))
		return dev_err_probe(dev, PTR_ERR(top_phydev),
				     "Failed to get ac300 top phy");

	priv_top = top_phydev->priv;
	priv->calib = priv_top->calib;

	return 0;
}

static struct phy_driver ac300_driver[] = {
	{
		.phy_id		= AC300_EPHY_ID,
		.phy_id_mask	= AC300_EPHY_ID_MASK,
		.name		= "X-Powers AC300 EPHY",
		.soft_reset	= genphy_soft_reset,
		.config_init	= ac300_init,
		.probe		= ac300_ephy_probe,
		.remove		= ac300_remove,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	},{
		.phy_id		= AC300_TOP_ID,
		.phy_id_mask	= AC300_TOP_ID_MASK,
		.name		= "X-Powers AC300 TOP",
		.soft_reset	= genphy_soft_reset,
		.probe		= ac300_top_probe,
		.remove		= ac300_remove,
		.suspend	= ac300_enable,
		.resume		= ac300_disable,
	},
};
module_phy_driver(ac300_driver);

static const struct mdio_device_id __maybe_unused ac300_phy_tbl[] = {
	{ AC300_EPHY_ID, AC300_EPHY_ID_MASK },
	{ AC300_TOP_ID, AC300_TOP_ID_MASK },
	{ }
};
MODULE_DEVICE_TABLE(mdio, ac300_phy_tbl);

MODULE_AUTHOR("Junhao Xie <bigfoot@classfun.cn>");
MODULE_DESCRIPTION("X-Powers AC300 Ethernet PHY driver");
MODULE_LICENSE("GPL");
