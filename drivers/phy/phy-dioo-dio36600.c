// SPDX-License-Identifier: GPL-2.0

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define DIO36600_REG_RESET_CTRL			0x01
#define  DIO36600_SOFT_RESET			BIT(0)

#define DIO36600_REG_LINK_CTRL1			0x02
#define  DIO36600_SPEED_CTRL			GENMASK(7, 6)
#define  DIO36600_ROLE_CTRL			GENMASK(5, 4)
#define  DIO36600_OP_MODE			GENMASK(2, 0)

#define DIO36600_REG_LINK_CTRL2			0x03
#define  DIO36600_AUTO_RESUME_EN		BIT(1)
#define  DIO36600_FORCE_ESE1			BIT(0)

#define DIO36600_REG_EUSB2_RX_CTRL		0x04
#define  DIO36600_EUSB2_RX_SQUELCH		GENMASK(5, 4)
#define  DIO36600_EUSB2_RX_EQ			GENMASK(2, 0)

#define DIO36600_REG_EUSB2_TX_CTRL		0x05
#define  DIO36600_EUSB2_TX_SWING		GENMASK(5, 4)
#define  DIO36600_EUSB2_TX_DE_EMPH		GENMASK(1, 0)

#define DIO36600_REG_USB2_RX_CTRL		0x06
#define  DIO36600_USB2_RX_SQUELCH		GENMASK(6, 4)
#define  DIO36600_USB2_RX_EQ			GENMASK(2, 0)

#define DIO36600_REG_USB2_TX_CTRL1		0x07
#define  DIO36600_USB2_TX_DE_EMPH_WIDTH		GENMASK(5, 4)
#define  DIO36600_USB2_TX_DE_EMPH		GENMASK(2, 0)

#define DIO36600_REG_USB2_TX_CTRL2		0x08
#define  DIO36600_USB2_FS_EDGE			BIT(6)
#define  DIO36600_USB2_HS_EDGE			GENMASK(5, 4)
#define  DIO36600_USB2_TX_SWING			GENMASK(2, 0)

#define DIO36600_REG_USB2_HS_TERM		0x09
#define  DIO36600_USB2_HS_TERM			GENMASK(2, 0)

#define DIO36600_REG_USB2_HS_DISC		0x0a
#define  DIO36600_USB2_HS_DISC			GENMASK(1, 0)

#define DIO36600_REG_RAP_SIGNATURE		0x0d

#define DIO36600_REG_VDX_CTRL			0x0e
#define  DIO36600_VDX_EN			BIT(0)

#define DIO36600_REG_DEVICE_STATUS		0x0f
#define  DIO36600_SPEED				GENMASK(3, 2)
#define  DIO36600_ROLE				GENMASK(1, 0)

#define DIO36600_REG_LINK_STATUS		0x10
#define  DIO36600_LINK_STATE			GENMASK(2, 0)

#define DIO36600_REG_REVISION_ID		0x13
#define DIO36600_REG_CHIP_ID_0			0x14
#define DIO36600_REG_CHIP_ID_1			0x15
#define DIO36600_REG_CHIP_ID_2			0x16

#define DIO36600_CHIP_ID			0x3660

#define DIO36600_RESET_US			10

#define DIO36600_STARTUP_US			500

#define DIO36600_NUM_SUPPLIES			2

#define DIO36600_RSVD				U32_MAX

struct dio36600_tune {
	const char *prop;
	u8 reg;
	u8 mask;
	u8 num_values;
	u32 values[8];
};

static const struct dio36600_tune dio36600_tunes[] = {
	{
		.prop = "dioo,eusb2-rx-squelch-threshold-microvolt",
		.reg = DIO36600_REG_EUSB2_RX_CTRL,
		.mask = DIO36600_EUSB2_RX_SQUELCH,
		.values = { 50000, 65000, 85000, 95000 },
		.num_values = 4,
	}, {
		.prop = "dioo,eusb2-rx-equalization-db",
		.reg = DIO36600_REG_EUSB2_RX_CTRL,
		.mask = DIO36600_EUSB2_RX_EQ,
		.values = { 0, 1, 2, 3, 4 },
		.num_values = 5,
	}, {
		.prop = "dioo,eusb2-tx-swing-microvolt",
		.reg = DIO36600_REG_EUSB2_TX_CTRL,
		.mask = DIO36600_EUSB2_TX_SWING,
		.values = { 180000, 200000, 220000, 240000 },
		.num_values = 4,
	}, {
		.prop = "dioo,eusb2-tx-de-emphasis-db",
		.reg = DIO36600_REG_EUSB2_TX_CTRL,
		.mask = DIO36600_EUSB2_TX_DE_EMPH,
		.values = { 0, 1, 2, 3 },
		.num_values = 4,
	}, {
		.prop = "dioo,usb2-rx-squelch-threshold-microvolt",
		.reg = DIO36600_REG_USB2_RX_CTRL,
		.mask = DIO36600_USB2_RX_SQUELCH,
		.values = { DIO36600_RSVD, 65000, 85000, 95000,
			    110000, 125000, 140000, 155000 },
		.num_values = 8,
	}, {
		.prop = "dioo,usb2-rx-equalization-db",
		.reg = DIO36600_REG_USB2_RX_CTRL,
		.mask = DIO36600_USB2_RX_EQ,
		.values = { 0, 1, 2, 3, 4 },
		.num_values = 5,
	}, {
		.prop = "dioo,usb2-tx-de-emphasis-width-percent",
		.reg = DIO36600_REG_USB2_TX_CTRL1,
		.mask = DIO36600_USB2_TX_DE_EMPH_WIDTH,
		.values = { 0, 50, 80 },
		.num_values = 3,
	}, {
		.prop = "dioo,usb2-tx-de-emphasis-db",
		.reg = DIO36600_REG_USB2_TX_CTRL1,
		.mask = DIO36600_USB2_TX_DE_EMPH,
		.values = { 0, 1, 2, 3, 4, 5, 6 },
		.num_values = 7,
	}, {
		.prop = "dioo,usb2-fs-rise-fall-time-ns",
		.reg = DIO36600_REG_USB2_TX_CTRL2,
		.mask = DIO36600_USB2_FS_EDGE,
		.values = { 20, 14 },
		.num_values = 2,
	}, {
		.prop = "dioo,usb2-hs-rise-fall-time-ps",
		.reg = DIO36600_REG_USB2_TX_CTRL2,
		.mask = DIO36600_USB2_HS_EDGE,
		.values = { 900, 830, 760 },
		.num_values = 3,
	}, {
		.prop = "dioo,usb2-tx-swing-microvolt",
		.reg = DIO36600_REG_USB2_TX_CTRL2,
		.mask = DIO36600_USB2_TX_SWING,
		.values = { 350000, 400000, 450000, 500000,
			    550000, 600000, 650000, 700000 },
		.num_values = 8,
	}, {
		.prop = "dioo,usb2-hs-termination-ohms",
		.reg = DIO36600_REG_USB2_HS_TERM,
		.mask = DIO36600_USB2_HS_TERM,
		.values = { 100, 95, 90, 85, 80 },
		.num_values = 5,
	}, {
		.prop = "dioo,usb2-hs-disconnect-threshold-microvolt",
		.reg = DIO36600_REG_USB2_HS_DISC,
		.mask = DIO36600_USB2_HS_DISC,
		.values = { 575000, 675000, 775000, 875000 },
		.num_values = 4,
	},
};

struct dio36600 {
	struct device *dev;
	struct regmap *regmap;
	struct phy *phy;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;

	u8 tune_mask[DIO36600_REG_USB2_HS_DISC + 1];
	u8 tune_val[DIO36600_REG_USB2_HS_DISC + 1];
};

static const struct regulator_bulk_data dio36600_supplies[DIO36600_NUM_SUPPLIES] = {
	{
		.supply = "vdd1v8",
		.init_load_uA = 44000,
	}, {
		.supply = "vdd3v3",
		.init_load_uA = 5000,
	}
};

static bool dio36600_readable_reg(struct device *dev, unsigned int reg)
{
	return reg <= DIO36600_REG_CHIP_ID_2;
}

static bool dio36600_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case DIO36600_REG_RESET_CTRL ... DIO36600_REG_USB2_HS_DISC:
	case DIO36600_REG_RAP_SIGNATURE:
	case DIO36600_REG_VDX_CTRL:
		return true;
	default:
		return false;
	}
}

static bool dio36600_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case DIO36600_REG_RESET_CTRL:
	case DIO36600_REG_LINK_CTRL2:
	case DIO36600_REG_VDX_CTRL:
	case DIO36600_REG_DEVICE_STATUS:
	case DIO36600_REG_LINK_STATUS:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config dio36600_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= DIO36600_REG_CHIP_ID_2,
	.readable_reg	= dio36600_readable_reg,
	.writeable_reg	= dio36600_writeable_reg,
	.volatile_reg	= dio36600_volatile_reg,
	.cache_type	= REGCACHE_NONE,
};

static int dio36600_validate_id(struct dio36600 *dio)
{
	u8 id[4];
	u16 chip_id;
	int ret;

	ret = regmap_bulk_read(dio->regmap, DIO36600_REG_REVISION_ID, id,
			       ARRAY_SIZE(id));
	if (ret)
		return ret;

	chip_id = id[2] << 8 | id[1];
	if (chip_id != DIO36600_CHIP_ID)
		return -ENODEV;

	return 0;
}

static int dio36600_configure(struct dio36600 *dio)
{
	unsigned int reg;
	int ret;

	/* The reset write is accepted but the device does not acknowledge it. */
	ret = regmap_write(dio->regmap, DIO36600_REG_RESET_CTRL,
			   DIO36600_SOFT_RESET);
	if (ret && ret != -EIO)
		return ret;

	fsleep(DIO36600_STARTUP_US);

	ret = dio36600_validate_id(dio);
	if (ret)
		return ret;

	for (reg = 0; reg < ARRAY_SIZE(dio->tune_mask); reg++) {
		if (!dio->tune_mask[reg])
			continue;

		ret = regmap_update_bits(dio->regmap, reg, dio->tune_mask[reg],
					 dio->tune_val[reg]);
		if (ret)
			return ret;
	}

	return 0;
}

static int dio36600_init(struct phy *phy)
{
	struct dio36600 *dio = phy_get_drvdata(phy);
	int ret;

	ret = regulator_bulk_enable(DIO36600_NUM_SUPPLIES, dio->supplies);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(dio->reset_gpio, 1);
	fsleep(DIO36600_RESET_US);
	gpiod_set_value_cansleep(dio->reset_gpio, 0);

	fsleep(DIO36600_STARTUP_US);

	ret = dio36600_configure(dio);
	if (ret) {
		dev_err(dio->dev, "failed to configure repeater: %d\n", ret);
		gpiod_set_value_cansleep(dio->reset_gpio, 1);
		regulator_bulk_disable(DIO36600_NUM_SUPPLIES, dio->supplies);
		return ret;
	}

	return 0;
}

static int dio36600_exit(struct phy *phy)
{
	struct dio36600 *dio = phy_get_drvdata(phy);

	gpiod_set_value_cansleep(dio->reset_gpio, 1);

	return regulator_bulk_disable(DIO36600_NUM_SUPPLIES, dio->supplies);
}

static const struct phy_ops dio36600_ops = {
	.init		= dio36600_init,
	.exit		= dio36600_exit,
	.owner		= THIS_MODULE,
};

static int dio36600_parse_tuning(struct dio36600 *dio)
{
	struct device_node *np = dio->dev->of_node;
	const struct dio36600_tune *tune;
	unsigned int i, code;
	u32 val;

	for (i = 0; i < ARRAY_SIZE(dio36600_tunes); i++) {
		tune = &dio36600_tunes[i];

		if (of_property_read_u32(np, tune->prop, &val))
			continue;

		for (code = 0; code < tune->num_values; code++)
			if (tune->values[code] == val)
				break;

		if (code == tune->num_values)
			return dev_err_probe(dio->dev, -EINVAL,
					     "unsupported %s: %u\n",
					     tune->prop, val);

		dio->tune_mask[tune->reg] |= tune->mask;
		dio->tune_val[tune->reg] |= field_prep(tune->mask, code);
	}

	if (of_property_read_bool(np, "dioo,auto-resume")) {
		dio->tune_mask[DIO36600_REG_LINK_CTRL2] |= DIO36600_AUTO_RESUME_EN;
		dio->tune_val[DIO36600_REG_LINK_CTRL2] |= DIO36600_AUTO_RESUME_EN;
	}

	return 0;
}

static int dio36600_check_id(struct dio36600 *dio)
{
	u8 id[4];
	u16 chip_id;
	int ret;

	ret = regulator_bulk_enable(DIO36600_NUM_SUPPLIES, dio->supplies);
	if (ret)
		return dev_err_probe(dio->dev, ret, "failed to enable supplies\n");

	gpiod_set_value_cansleep(dio->reset_gpio, 0);
	fsleep(DIO36600_STARTUP_US);

	ret = regmap_bulk_read(dio->regmap, DIO36600_REG_REVISION_ID, id,
			       ARRAY_SIZE(id));

	gpiod_set_value_cansleep(dio->reset_gpio, 1);
	regulator_bulk_disable(DIO36600_NUM_SUPPLIES, dio->supplies);

	if (ret)
		return dev_err_probe(dio->dev, ret, "failed to read chip ID\n");

	chip_id = id[2] << 8 | id[1];
	if (chip_id != DIO36600_CHIP_ID)
		return dev_err_probe(dio->dev, -ENODEV,
				     "unexpected chip ID %#06x\n", chip_id);

	dev_dbg(dio->dev, "revision %#04x, configuration %#04x\n", id[0], id[3]);

	return 0;
}

static int dio36600_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct phy_provider *phy_provider;
	struct dio36600 *dio;
	int ret;

	dio = devm_kzalloc(dev, sizeof(*dio), GFP_KERNEL);
	if (!dio)
		return -ENOMEM;

	dio->dev = dev;

	dio->regmap = devm_regmap_init_i2c(client, &dio36600_regmap_config);
	if (IS_ERR(dio->regmap))
		return dev_err_probe(dev, PTR_ERR(dio->regmap),
				     "failed to initialize regmap\n");

	dio->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dio->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(dio->reset_gpio),
				     "failed to get reset GPIO\n");

	ret = devm_regulator_bulk_get_const(dev, DIO36600_NUM_SUPPLIES,
					    dio36600_supplies, &dio->supplies);
	if (ret)
		return ret;

	ret = dio36600_parse_tuning(dio);
	if (ret)
		return ret;

	ret = dio36600_check_id(dio);
	if (ret)
		return ret;

	dio->phy = devm_phy_create(dev, dev->of_node, &dio36600_ops);
	if (IS_ERR(dio->phy))
		return dev_err_probe(dev, PTR_ERR(dio->phy),
				     "failed to create PHY\n");

	phy_set_drvdata(dio->phy, dio);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct i2c_device_id dio36600_table[] = {
	{ "dio36600" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, dio36600_table);

static const struct of_device_id dio36600_of_table[] = {
	{ .compatible = "dioo,dio36600" },
	{ }
};
MODULE_DEVICE_TABLE(of, dio36600_of_table);

static struct i2c_driver dio36600_driver = {
	.driver = {
		.name = "dio36600",
		.of_match_table = dio36600_of_table,
	},
	.probe = dio36600_probe,
	.id_table = dio36600_table,
};

module_i2c_driver(dio36600_driver);

MODULE_AUTHOR("Junhao Xie <bigfoot@radxa.com>");
MODULE_DESCRIPTION("Dioo DIO36600 eUSB2 repeater driver");
MODULE_LICENSE("GPL");
