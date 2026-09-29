// SPDX-License-Identifier: GPL-2.0

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>

#define SC96231_REG_CHIP_ID		0x0000
#define SC96231_REG_CUST_ID		0x0002
#define SC96231_REG_FW_VERSION		0x0004
#define SC96231_REG_CMD			0x0100
#define SC96231_REG_IRQ_EN		0x0104
#define SC96231_REG_IRQ_FLAG		0x0108
#define SC96231_REG_IRQ_CLR		0x010c
#define SC96231_REG_MODE		0x0114
#define SC96231_REG_VOUT		0x0160
#define SC96231_REG_IOUT		0x0164
#define SC96231_REG_VRECT		0x0168
#define SC96231_REG_TRANSMIT_POWER	0x0172
#define SC96231_REG_T_DIE		0x0178
#define SC96231_REG_IIC_CHECK		0x01cf
#define SC96231_REG_MAX			0x01ff

#define SC96231_CHIP_ID			0x6231

#define SC96231_CMD_CHIP_RESET		BIT(1)
#define SC96231_CMD_TX_DISABLE		BIT(18)
#define SC96231_CMD_TX_ENABLE		BIT(19)

#define SC96231_MODE_STANDBY		BIT(0)
#define SC96231_MODE_RECEIVER		BIT(1)
#define SC96231_MODE_TRANSMITTER	BIT(2)
#define SC96231_MODE_TX_PING		BIT(14)
#define SC96231_MODE_TX_POWER_TRANSFER	BIT(15)
#define SC96231_MODE_RUNNING		(SC96231_MODE_STANDBY | \
					 SC96231_MODE_RECEIVER | \
					 SC96231_MODE_TRANSMITTER)

#define SC96231_IIC_CHECK_PATTERN	0x55
#define SC96231_IIC_CHECK_DELAY_US	20000

#define SC96231_TX_SETTLE_MS		50

#define SC96231_WAKE_POLL_MS		20
#define SC96231_WAKE_TIMEOUT_MS		2000

struct sc96231 {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct gpio_desc *txon_gpio;
	struct gpio_desc *boost_gpio;
	/* Serialize transmitter state changes. */
	struct mutex lock;
	bool tx_enabled;
};

static int sc96231_read(struct sc96231 *sc, unsigned int reg, void *val,
			size_t len)
{
	unsigned int start = ALIGN_DOWN(reg, 4);
	size_t window = ALIGN(reg + len - start, 4);
	u8 buf[8];
	int ret;

	if (window > sizeof(buf))
		return -EINVAL;

	ret = regmap_raw_read(sc->regmap, start, buf, window);
	if (ret)
		return ret;

	memcpy(val, buf + (reg - start), len);

	return 0;
}

static int sc96231_write(struct sc96231 *sc, unsigned int reg, const void *val,
			 size_t len)
{
	unsigned int start = ALIGN_DOWN(reg, 4);
	size_t window = ALIGN(reg + len - start, 4);
	u8 buf[8];
	int ret;

	if (window > sizeof(buf))
		return -EINVAL;

	if (window != len) {
		ret = regmap_raw_read(sc->regmap, start, buf, window);
		if (ret)
			return ret;
	}

	memcpy(buf + (reg - start), val, len);

	return regmap_raw_write(sc->regmap, start, buf, window);
}

static int sc96231_read_u16(struct sc96231 *sc, unsigned int reg, u16 *val)
{
	__le16 raw;
	int ret;

	ret = sc96231_read(sc, reg, &raw, sizeof(raw));
	if (ret)
		return ret;

	*val = le16_to_cpu(raw);

	return 0;
}

static int sc96231_read_u32(struct sc96231 *sc, unsigned int reg, u32 *val)
{
	__le32 raw;
	int ret;

	ret = sc96231_read(sc, reg, &raw, sizeof(raw));
	if (ret)
		return ret;

	*val = le32_to_cpu(raw);

	return 0;
}

static int sc96231_write_cmd(struct sc96231 *sc, u32 cmd)
{
	__le32 raw = cpu_to_le32(cmd);

	return sc96231_write(sc, SC96231_REG_CMD, &raw, sizeof(raw));
}

static int sc96231_check_firmware(struct sc96231 *sc)
{
	u8 pattern = SC96231_IIC_CHECK_PATTERN;
	u8 readback;
	int ret;

	ret = sc96231_write(sc, SC96231_REG_IIC_CHECK, &pattern, sizeof(pattern));
	if (ret)
		return ret;

	usleep_range(SC96231_IIC_CHECK_DELAY_US,
		     SC96231_IIC_CHECK_DELAY_US + 1000);

	ret = sc96231_read(sc, SC96231_REG_IIC_CHECK, &readback, sizeof(readback));
	if (ret)
		return ret;

	if (readback != SC96231_IIC_CHECK_PATTERN) {
		dev_dbg(sc->dev, "firmware did not answer: %#x\n", readback);
		return -ENODEV;
	}

	return 0;
}

static int sc96231_set_tx(struct sc96231 *sc, bool enable)
{
	int ret;

	guard(mutex)(&sc->lock);

	if (sc->tx_enabled == enable)
		return 0;

	if (enable) {
		gpiod_set_value_cansleep(sc->boost_gpio, 1);

		ret = sc96231_write_cmd(sc, SC96231_CMD_TX_ENABLE);
		if (ret) {
			gpiod_set_value_cansleep(sc->boost_gpio, 0);
			return ret;
		}

		msleep(SC96231_TX_SETTLE_MS);
	} else {
		ret = sc96231_write_cmd(sc, SC96231_CMD_TX_DISABLE);

		gpiod_set_value_cansleep(sc->boost_gpio, 0);

		if (ret)
			return ret;
	}

	sc->tx_enabled = enable;

	return 0;
}

static int sc96231_get_property(struct power_supply *psy,
				enum power_supply_property prop,
				union power_supply_propval *val)
{
	struct sc96231 *sc = power_supply_get_drvdata(psy);
	u32 mode;
	u16 raw;
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Southchip";
		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SC96231";
		return 0;
	case POWER_SUPPLY_PROP_ONLINE:
		scoped_guard(mutex, &sc->lock)
			val->intval = sc->tx_enabled;
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = sc96231_read_u32(sc, SC96231_REG_MODE, &mode);
		if (ret)
			return ret;
		val->intval = !!(mode & SC96231_MODE_TX_POWER_TRANSFER);
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = sc96231_read_u16(sc, SC96231_REG_VOUT, &raw);
		if (ret)
			return ret;
		val->intval = raw * 1000;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = sc96231_read_u16(sc, SC96231_REG_IOUT, &raw);
		if (ret)
			return ret;
		val->intval = raw * 1000;
		return 0;
	case POWER_SUPPLY_PROP_POWER_NOW:
		ret = sc96231_read_u16(sc, SC96231_REG_TRANSMIT_POWER, &raw);
		if (ret)
			return ret;
		val->intval = raw * 1000;
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		ret = sc96231_read_u16(sc, SC96231_REG_T_DIE, &raw);
		if (ret)
			return ret;
		val->intval = (s16)raw;
		return 0;
	default:
		return -EINVAL;
	}
}

static int sc96231_set_property(struct power_supply *psy,
				enum power_supply_property prop,
				const union power_supply_propval *val)
{
	struct sc96231 *sc = power_supply_get_drvdata(psy);
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = sc96231_set_tx(sc, !!val->intval);
		if (!ret)
			power_supply_changed(sc->psy);
		return ret;
	default:
		return -EINVAL;
	}
}

static int sc96231_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property prop)
{
	return prop == POWER_SUPPLY_PROP_ONLINE;
}

static enum power_supply_property sc96231_properties[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static const struct power_supply_desc sc96231_psy_desc = {
	.name			= "sc96231-wireless-tx",
	.type			= POWER_SUPPLY_TYPE_WIRELESS,
	.properties		= sc96231_properties,
	.num_properties		= ARRAY_SIZE(sc96231_properties),
	.get_property		= sc96231_get_property,
	.set_property		= sc96231_set_property,
	.property_is_writeable	= sc96231_property_is_writeable,
};

static irqreturn_t sc96231_irq_handler(int irq, void *data)
{
	struct sc96231 *sc = data;
	__le32 raw;
	u32 flags;
	int ret;

	ret = sc96231_read_u32(sc, SC96231_REG_IRQ_FLAG, &flags);
	if (ret)
		return IRQ_NONE;

	if (!flags)
		return IRQ_NONE;

	dev_dbg(sc->dev, "interrupt flags %#x\n", flags);

	raw = cpu_to_le32(flags);
	sc96231_write(sc, SC96231_REG_IRQ_CLR, &raw, sizeof(raw));

	power_supply_changed(sc->psy);

	return IRQ_HANDLED;
}

static const struct regmap_config sc96231_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= SC96231_REG_MAX,
};

static int sc96231_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {};
	struct sc96231 *sc;
	u32 fw_version;
	u32 mode;
	u16 chip_id;
	unsigned int waited;
	int ret;

	sc = devm_kzalloc(dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->dev = dev;
	i2c_set_clientdata(client, sc);

	ret = devm_mutex_init(dev, &sc->lock);
	if (ret)
		return ret;

	sc->regmap = devm_regmap_init_i2c(client, &sc96231_regmap_config);
	if (IS_ERR(sc->regmap))
		return dev_err_probe(dev, PTR_ERR(sc->regmap),
				     "failed to init regmap\n");

	sc->txon_gpio = devm_gpiod_get(dev, "reverse-txon", GPIOD_OUT_HIGH);
	if (IS_ERR(sc->txon_gpio))
		return dev_err_probe(dev, PTR_ERR(sc->txon_gpio),
				     "failed to get txon gpio\n");

	sc->boost_gpio = devm_gpiod_get(dev, "reverse-boost", GPIOD_OUT_LOW);
	if (IS_ERR(sc->boost_gpio))
		return dev_err_probe(dev, PTR_ERR(sc->boost_gpio),
				     "failed to get boost gpio\n");

	for (waited = 0; ; waited += SC96231_WAKE_POLL_MS) {
		msleep(SC96231_WAKE_POLL_MS);

		ret = sc96231_read_u16(sc, SC96231_REG_CHIP_ID, &chip_id);
		if (!ret && chip_id != SC96231_CHIP_ID)
			ret = -ENODEV;
		if (!ret)
			ret = sc96231_check_firmware(sc);
		if (!ret)
			ret = sc96231_read_u32(sc, SC96231_REG_MODE, &mode);
		if (!ret && !(mode & SC96231_MODE_RUNNING))
			ret = -ENODEV;
		if (!ret)
			break;

		if (waited >= SC96231_WAKE_TIMEOUT_MS)
			return dev_err_probe(dev, ret,
					     "chip did not start in %u ms\n",
					     waited);
	}

	ret = sc96231_read_u32(sc, SC96231_REG_FW_VERSION, &fw_version);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read firmware version\n");

	dev_info(dev, "chip %#06x, firmware %#010x\n", chip_id, fw_version);

	psy_cfg.drv_data = sc;
	psy_cfg.fwnode = dev_fwnode(dev);
	sc->psy = devm_power_supply_register(dev, &sc96231_psy_desc, &psy_cfg);
	if (IS_ERR(sc->psy))
		return dev_err_probe(dev, PTR_ERR(sc->psy),
				     "failed to register power supply\n");

	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						sc96231_irq_handler,
						IRQF_ONESHOT, dev_name(dev), sc);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request irq\n");
	}

	return 0;
}

static void sc96231_remove(struct i2c_client *client)
{
	struct sc96231 *sc = i2c_get_clientdata(client);

	sc96231_set_tx(sc, false);
}

static const struct i2c_device_id sc96231_i2c_ids[] = {
	{ "sc96231" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc96231_i2c_ids);

static const struct of_device_id sc96231_of_match[] = {
	{ .compatible = "southchip,sc96231" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc96231_of_match);

static struct i2c_driver sc96231_driver = {
	.driver = {
		.name		= "sc96231-charger",
		.of_match_table	= sc96231_of_match,
	},
	.probe		= sc96231_probe,
	.remove		= sc96231_remove,
	.id_table	= sc96231_i2c_ids,
};
module_i2c_driver(sc96231_driver);

MODULE_AUTHOR("Junhao Xie <bigfoot@radxa.com>");
MODULE_DESCRIPTION("Southchip SC96231 wireless power transmitter driver");
MODULE_LICENSE("GPL");
