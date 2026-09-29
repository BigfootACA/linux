// SPDX-License-Identifier: GPL-2.0

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

#define SC8541_REG_VBAT_OVP		0x00
#define SC8541_REG_VBAT_OVP_ALM		0x01
#define SC8541_REG_IBAT_OCP		0x02
#define SC8541_REG_IBAT_OCP_ALM		0x03
#define SC8541_REG_IBUS_UCP		0x05
#define SC8541_REG_VBUS_OVP		0x06
#define SC8541_REG_VBUS_OVP_ALM		0x07
#define SC8541_REG_IBUS_OCP		0x08
#define SC8541_REG_TEMP_CTRL		0x0a
#define SC8541_REG_CONVERTER		0x0f
#define SC8541_REG_CTRL1		0x10
#define SC8541_REG_CTRL2		0x11
#define SC8541_REG_CTRL3		0x12
#define SC8541_REG_STAT1		0x17
#define SC8541_REG_FLAG1		0x18
#define SC8541_REG_FLAG_LAST		0x1c
#define SC8541_REG_DEVICE_ID		0x22
#define SC8541_REG_ADC_CTRL		0x23
#define SC8541_REG_ADC_BASE		0x25
#define SC8541_REG_ACDRV		0x40
#define SC8541_REG_PMID2OUT		0x42
#define SC8541_REG_MAX			0x42

#define SC8541_DEVICE_ID_VALUE		0x41

#define SC8541_ADC_SETTLE_MS		50

#define SC8541_VBAT_OVP_MIN_MV		3840
#define SC8541_VBAT_OVP_MAX_MV		5110
#define SC8541_VBAT_OVP_STEP_MV		10

#define SC8541_IBAT_OCP_MIN_MA		0
#define SC8541_IBAT_OCP_MAX_MA		12700
#define SC8541_IBAT_OCP_STEP_MA		100

#define SC8541_VBUS_OVP_MIN_MV		7000
#define SC8541_VBUS_OVP_MAX_MV		13350
#define SC8541_VBUS_OVP_STEP_MV		50

#define SC8541_IBUS_OCP_MIN_MA		1000
#define SC8541_IBUS_OCP_MAX_MA		8000
#define SC8541_IBUS_OCP_STEP_MA		250

enum sc8541_field {
	F_VBAT_OVP_DIS, F_VBAT_OVP,
	F_VBAT_OVP_ALM_DIS, F_VBAT_OVP_ALM,
	F_IBAT_OCP_DIS, F_IBAT_OCP,
	F_IBAT_OCP_ALM_DIS, F_IBAT_OCP_ALM,
	F_IBUS_UCP_DIS, F_VBUS_IN_RANGE_DIS,
	F_VBUS_PD_EN, F_VBUS_OVP,
	F_VBUS_OVP_ALM_DIS, F_VBUS_OVP_ALM,
	F_IBUS_OCP_DIS, F_IBUS_OCP,
	F_TSHUT_DIS, F_TSBUS_FLT_DIS, F_TSBAT_FLT_DIS,
	F_VAC1_OVP, F_VAC2_OVP, F_VAC1_PD_EN, F_VAC2_PD_EN,
	F_REG_RST, F_OTG_EN, F_CHG_EN, F_MODE,
	F_FSW_SET, F_WD_TIMEOUT, F_WD_TIMEOUT_DIS,
	F_IBAT_SNS_RES, F_SS_TIMEOUT, F_IBUS_UCP_FALL_DG,
	F_VOUT_OVP_DIS, F_VOUT_OVP, F_MS,
	F_CP_SWITCHING_STAT, F_VBUS_ERRORHI_STAT, F_VBUS_ERRORLO_STAT,
	F_DEVICE_ID,
	F_ADC_EN, F_ADC_RATE,
	F_PMID2OUT_UVP, F_PMID2OUT_OVP,
	F_MAX_FIELDS,
};

static const struct reg_field sc8541_reg_fields[F_MAX_FIELDS] = {
	[F_VBAT_OVP_DIS]	= REG_FIELD(SC8541_REG_VBAT_OVP, 7, 7),
	[F_VBAT_OVP]		= REG_FIELD(SC8541_REG_VBAT_OVP, 0, 6),
	[F_VBAT_OVP_ALM_DIS]	= REG_FIELD(SC8541_REG_VBAT_OVP_ALM, 7, 7),
	[F_VBAT_OVP_ALM]	= REG_FIELD(SC8541_REG_VBAT_OVP_ALM, 0, 6),
	[F_IBAT_OCP_DIS]	= REG_FIELD(SC8541_REG_IBAT_OCP, 7, 7),
	[F_IBAT_OCP]		= REG_FIELD(SC8541_REG_IBAT_OCP, 0, 6),
	[F_IBAT_OCP_ALM_DIS]	= REG_FIELD(SC8541_REG_IBAT_OCP_ALM, 7, 7),
	[F_IBAT_OCP_ALM]	= REG_FIELD(SC8541_REG_IBAT_OCP_ALM, 0, 6),
	[F_IBUS_UCP_DIS]	= REG_FIELD(SC8541_REG_IBUS_UCP, 7, 7),
	[F_VBUS_IN_RANGE_DIS]	= REG_FIELD(SC8541_REG_IBUS_UCP, 2, 2),
	[F_VBUS_PD_EN]		= REG_FIELD(SC8541_REG_VBUS_OVP, 7, 7),
	[F_VBUS_OVP]		= REG_FIELD(SC8541_REG_VBUS_OVP, 0, 6),
	[F_VBUS_OVP_ALM_DIS]	= REG_FIELD(SC8541_REG_VBUS_OVP_ALM, 7, 7),
	[F_VBUS_OVP_ALM]	= REG_FIELD(SC8541_REG_VBUS_OVP_ALM, 0, 6),
	[F_IBUS_OCP_DIS]	= REG_FIELD(SC8541_REG_IBUS_OCP, 7, 7),
	[F_IBUS_OCP]		= REG_FIELD(SC8541_REG_IBUS_OCP, 0, 4),
	[F_TSHUT_DIS]		= REG_FIELD(SC8541_REG_TEMP_CTRL, 7, 7),
	[F_TSBUS_FLT_DIS]	= REG_FIELD(SC8541_REG_TEMP_CTRL, 3, 3),
	[F_TSBAT_FLT_DIS]	= REG_FIELD(SC8541_REG_TEMP_CTRL, 2, 2),
	[F_VAC1_OVP]		= REG_FIELD(0x0e, 5, 7),
	[F_VAC2_OVP]		= REG_FIELD(0x0e, 2, 4),
	[F_VAC1_PD_EN]		= REG_FIELD(0x0e, 1, 1),
	[F_VAC2_PD_EN]		= REG_FIELD(0x0e, 0, 0),
	[F_REG_RST]		= REG_FIELD(SC8541_REG_CONVERTER, 7, 7),
	[F_OTG_EN]		= REG_FIELD(SC8541_REG_CONVERTER, 5, 5),
	[F_CHG_EN]		= REG_FIELD(SC8541_REG_CONVERTER, 4, 4),
	[F_MODE]		= REG_FIELD(SC8541_REG_CONVERTER, 3, 3),
	[F_FSW_SET]		= REG_FIELD(SC8541_REG_CTRL1, 5, 7),
	[F_WD_TIMEOUT]		= REG_FIELD(SC8541_REG_CTRL1, 3, 4),
	[F_WD_TIMEOUT_DIS]	= REG_FIELD(SC8541_REG_CTRL1, 2, 2),
	[F_IBAT_SNS_RES]	= REG_FIELD(SC8541_REG_CTRL2, 7, 7),
	[F_SS_TIMEOUT]		= REG_FIELD(SC8541_REG_CTRL2, 4, 6),
	[F_IBUS_UCP_FALL_DG]	= REG_FIELD(SC8541_REG_CTRL2, 2, 3),
	[F_VOUT_OVP_DIS]	= REG_FIELD(SC8541_REG_CTRL3, 7, 7),
	[F_VOUT_OVP]		= REG_FIELD(SC8541_REG_CTRL3, 5, 6),
	[F_MS]			= REG_FIELD(SC8541_REG_CTRL3, 0, 1),
	[F_CP_SWITCHING_STAT]	= REG_FIELD(SC8541_REG_STAT1, 6, 6),
	[F_VBUS_ERRORHI_STAT]	= REG_FIELD(SC8541_REG_STAT1, 4, 4),
	[F_VBUS_ERRORLO_STAT]	= REG_FIELD(SC8541_REG_STAT1, 3, 3),
	[F_DEVICE_ID]		= REG_FIELD(SC8541_REG_DEVICE_ID, 0, 7),
	[F_ADC_EN]		= REG_FIELD(SC8541_REG_ADC_CTRL, 7, 7),
	[F_ADC_RATE]		= REG_FIELD(SC8541_REG_ADC_CTRL, 6, 6),
	[F_PMID2OUT_UVP]	= REG_FIELD(SC8541_REG_PMID2OUT, 5, 7),
	[F_PMID2OUT_OVP]	= REG_FIELD(SC8541_REG_PMID2OUT, 2, 4),
};

enum sc8541_adc_channel {
	SC8541_ADC_IBUS,
	SC8541_ADC_VBUS,
	SC8541_ADC_VAC1,
	SC8541_ADC_VAC2,
	SC8541_ADC_VOUT,
	SC8541_ADC_VBAT,
	SC8541_ADC_IBAT,
	SC8541_ADC_TSBUS,
	SC8541_ADC_TSBAT,
	SC8541_ADC_TDIE,
	SC8541_ADC_MAX,
};

static const struct {
	int mul;
	int div;
} sc8541_adc_scale[SC8541_ADC_MAX] = {
	[SC8541_ADC_IBUS]	= { 25, 10 },
	[SC8541_ADC_VBUS]	= { 375, 100 },
	[SC8541_ADC_VAC1]	= { 5, 1 },
	[SC8541_ADC_VAC2]	= { 5, 1 },
	[SC8541_ADC_VOUT]	= { 125, 100 },
	[SC8541_ADC_VBAT]	= { 125, 100 },
	[SC8541_ADC_IBAT]	= { 3125, 1000 },
	[SC8541_ADC_TSBUS]	= { 9766, 100000 },
	[SC8541_ADC_TSBAT]	= { 9766, 100000 },
	[SC8541_ADC_TDIE]	= { 5, 1 },
};

struct sc8541_flag {
	u8 reg;
	u8 mask;
	const char *name;
};

static const struct sc8541_flag sc8541_flags[] = {
	{ 0x18, BIT(1), "VBUS OVP" },
	{ 0x18, BIT(4), "IBAT OCP" },
	{ 0x18, BIT(5), "VOUT OVP" },
	{ 0x18, BIT(7), "VBAT OVP" },
	{ 0x19, BIT(5), "IBUS UCP" },
	{ 0x19, BIT(7), "IBUS OCP" },
	{ 0x1a, BIT(6), "VAC2 OVP" },
	{ 0x1a, BIT(7), "VAC1 OVP" },
	{ 0x1b, BIT(0), "watchdog timeout" },
	{ 0x1b, BIT(2), "thermal shutdown" },
	{ 0x1b, BIT(6), "soft start timeout" },
};

struct sc8541_init {
	enum sc8541_field field;
	unsigned int value;
};

#define SC8541_VBAT_OVP_MV		4800
#define SC8541_VBAT_OVP_ALM_MV		4600
#define SC8541_VBUS_OVP_MV		13000
#define SC8541_IBUS_OCP_MA		7000

#define SC8541_VAC_OVP_CODE		3

static const struct sc8541_init sc8541_defaults[] = {
	{ F_VBAT_OVP_DIS, 0 },
	{ F_VBAT_OVP, (SC8541_VBAT_OVP_MV - SC8541_VBAT_OVP_MIN_MV) /
		      SC8541_VBAT_OVP_STEP_MV },
	{ F_VBAT_OVP_ALM, (SC8541_VBAT_OVP_ALM_MV - SC8541_VBAT_OVP_MIN_MV) /
			  SC8541_VBAT_OVP_STEP_MV },
	{ F_VBUS_OVP, (SC8541_VBUS_OVP_MV - SC8541_VBUS_OVP_MIN_MV) /
		      SC8541_VBUS_OVP_STEP_MV },
	{ F_IBUS_OCP, (SC8541_IBUS_OCP_MA - SC8541_IBUS_OCP_MIN_MA) /
		      SC8541_IBUS_OCP_STEP_MA },
	{ F_VAC1_OVP, SC8541_VAC_OVP_CODE },
	{ F_VAC2_OVP, SC8541_VAC_OVP_CODE },
	{ F_VBAT_OVP_ALM_DIS, 1 },
	{ F_IBAT_OCP_DIS, 0 },
	{ F_IBAT_OCP_ALM_DIS, 1 },
	{ F_VBUS_OVP_ALM_DIS, 1 },
	{ F_IBUS_UCP_DIS, 0 },
	{ F_IBUS_OCP_DIS, 0 },
	{ F_TSHUT_DIS, 0 },
	{ F_TSBUS_FLT_DIS, 1 },
	{ F_TSBAT_FLT_DIS, 1 },
	{ F_VOUT_OVP_DIS, 0 },
	{ F_WD_TIMEOUT_DIS, 1 },
	{ F_CHG_EN, 0 },
	{ F_OTG_EN, 0 },
};

struct sc8541 {
	struct device *dev;
	struct regmap *regmap;
	struct regmap_field *fields[F_MAX_FIELDS];
	struct power_supply *psy;
	bool adc_enabled;
};

static int sc8541_field_read(struct sc8541 *sc, enum sc8541_field field,
			     unsigned int *val)
{
	return regmap_field_read(sc->fields[field], val);
}

static int sc8541_field_write(struct sc8541 *sc, enum sc8541_field field,
			      unsigned int val)
{
	return regmap_field_write(sc->fields[field], val);
}

static unsigned int sc8541_val_to_reg(unsigned int val, unsigned int min,
				      unsigned int max, unsigned int step)
{
	val = clamp(val, min, max);

	return (val - min) / step;
}

static int sc8541_adc_read(struct sc8541 *sc, enum sc8541_adc_channel channel,
			   int *result)
{
	__be16 raw;
	int ret;

	if (!sc->adc_enabled) {
		ret = sc8541_field_write(sc, F_ADC_EN, 1);
		if (ret)
			return ret;

		sc->adc_enabled = true;
		msleep(SC8541_ADC_SETTLE_MS);
	}

	ret = regmap_bulk_read(sc->regmap,
			       SC8541_REG_ADC_BASE + channel * sizeof(raw),
			       &raw, sizeof(raw));
	if (ret)
		return ret;

	*result = be16_to_cpu(raw) * sc8541_adc_scale[channel].mul /
		  sc8541_adc_scale[channel].div;

	return 0;
}

static int sc8541_get_health(struct sc8541 *sc, int *health)
{
	unsigned int flag;
	unsigned int val;
	int ret;

	ret = regmap_read(sc->regmap, 0x1b, &val);
	if (ret)
		return ret;
	if (val & BIT(2)) {
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
		return 0;
	}

	ret = regmap_read(sc->regmap, 0x18, &flag);
	if (ret)
		return ret;
	if (flag & (BIT(1) | BIT(7))) {
		*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		return 0;
	}
	if (flag & BIT(4)) {
		*health = POWER_SUPPLY_HEALTH_OVERCURRENT;
		return 0;
	}

	*health = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int sc8541_get_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       union power_supply_propval *val)
{
	struct sc8541 *sc = power_supply_get_drvdata(psy);
	unsigned int reg;
	int value;
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Southchip";
		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SC8541";
		return 0;
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_STATUS:
		ret = sc8541_field_read(sc, F_CP_SWITCHING_STAT, &reg);
		if (ret)
			return ret;
		if (prop == POWER_SUPPLY_PROP_ONLINE)
			val->intval = reg;
		else
			val->intval = reg ? POWER_SUPPLY_STATUS_CHARGING :
					    POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = sc8541_field_read(sc, F_CP_SWITCHING_STAT, &reg);
		if (ret)
			return ret;
		val->intval = reg ? POWER_SUPPLY_CHARGE_TYPE_FAST :
				    POWER_SUPPLY_CHARGE_TYPE_NONE;
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		return sc8541_get_health(sc, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = sc8541_adc_read(sc, SC8541_ADC_VBUS, &value);
		if (ret)
			return ret;
		val->intval = value * 1000;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = sc8541_adc_read(sc, SC8541_ADC_IBUS, &value);
		if (ret)
			return ret;
		val->intval = value * 1000;
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		return sc8541_adc_read(sc, SC8541_ADC_TDIE, &val->intval);
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sc8541_field_read(sc, F_VBUS_OVP, &reg);
		if (ret)
			return ret;
		val->intval = (SC8541_VBUS_OVP_MIN_MV +
			       reg * SC8541_VBUS_OVP_STEP_MV) * 1000;
		return 0;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sc8541_field_read(sc, F_IBUS_OCP, &reg);
		if (ret)
			return ret;
		val->intval = (SC8541_IBUS_OCP_MIN_MA +
			       reg * SC8541_IBUS_OCP_STEP_MA) * 1000;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = sc8541_field_read(sc, F_VBAT_OVP, &reg);
		if (ret)
			return ret;
		val->intval = (SC8541_VBAT_OVP_MIN_MV +
			       reg * SC8541_VBAT_OVP_STEP_MV) * 1000;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = sc8541_field_read(sc, F_IBAT_OCP, &reg);
		if (ret)
			return ret;
		val->intval = (SC8541_IBAT_OCP_MIN_MA +
			       reg * SC8541_IBAT_OCP_STEP_MA) * 1000;
		return 0;
	default:
		return -EINVAL;
	}
}

static int sc8541_set_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       const union power_supply_propval *val)
{
	struct sc8541 *sc = power_supply_get_drvdata(psy);
	unsigned int reg;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		return sc8541_field_write(sc, F_CHG_EN, !!val->intval);
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		reg = sc8541_val_to_reg(val->intval / 1000,
					SC8541_VBUS_OVP_MIN_MV,
					SC8541_VBUS_OVP_MAX_MV,
					SC8541_VBUS_OVP_STEP_MV);
		return sc8541_field_write(sc, F_VBUS_OVP, reg);
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		reg = sc8541_val_to_reg(val->intval / 1000,
					SC8541_IBUS_OCP_MIN_MA,
					SC8541_IBUS_OCP_MAX_MA,
					SC8541_IBUS_OCP_STEP_MA);
		return sc8541_field_write(sc, F_IBUS_OCP, reg);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		reg = sc8541_val_to_reg(val->intval / 1000,
					SC8541_VBAT_OVP_MIN_MV,
					SC8541_VBAT_OVP_MAX_MV,
					SC8541_VBAT_OVP_STEP_MV);
		return sc8541_field_write(sc, F_VBAT_OVP, reg);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		reg = sc8541_val_to_reg(val->intval / 1000,
					SC8541_IBAT_OCP_MIN_MA,
					SC8541_IBAT_OCP_MAX_MA,
					SC8541_IBAT_OCP_STEP_MA);
		return sc8541_field_write(sc, F_IBAT_OCP, reg);
	default:
		return -EINVAL;
	}
}

static int sc8541_property_is_writeable(struct power_supply *psy,
					enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_property sc8541_properties[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
};

static const struct power_supply_desc sc8541_psy_desc = {
	.name			= "sc8541-charge-pump",
	.type			= POWER_SUPPLY_TYPE_MAINS,
	.properties		= sc8541_properties,
	.num_properties		= ARRAY_SIZE(sc8541_properties),
	.get_property		= sc8541_get_property,
	.set_property		= sc8541_set_property,
	.property_is_writeable	= sc8541_property_is_writeable,
};

static irqreturn_t sc8541_irq_handler(int irq, void *data)
{
	struct sc8541 *sc = data;
	unsigned int flags[SC8541_REG_FLAG_LAST - SC8541_REG_FLAG1 + 1];
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(flags); i++) {
		ret = regmap_read(sc->regmap, SC8541_REG_FLAG1 + i, &flags[i]);
		if (ret)
			return IRQ_NONE;
	}

	for (i = 0; i < ARRAY_SIZE(sc8541_flags); i++) {
		const struct sc8541_flag *flag = &sc8541_flags[i];

		if (flags[flag->reg - SC8541_REG_FLAG1] & flag->mask)
			dev_warn_ratelimited(sc->dev, "%s\n", flag->name);
	}

	power_supply_changed(sc->psy);

	return IRQ_HANDLED;
}

static bool sc8541_is_volatile_reg(struct device *dev, unsigned int reg)
{
	return reg >= SC8541_REG_STAT1;
}

static const struct regmap_config sc8541_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= SC8541_REG_MAX,
	.cache_type	= REGCACHE_MAPLE,
	.volatile_reg	= sc8541_is_volatile_reg,
};

static int sc8541_hw_init(struct sc8541 *sc)
{
	unsigned int i;
	int ret;

	ret = sc8541_field_write(sc, F_REG_RST, 1);
	if (ret)
		return ret;

	usleep_range(10000, 11000);
	regcache_drop_region(sc->regmap, 0, SC8541_REG_MAX);

	for (i = 0; i < ARRAY_SIZE(sc8541_defaults); i++) {
		ret = sc8541_field_write(sc, sc8541_defaults[i].field,
					 sc8541_defaults[i].value);
		if (ret)
			return ret;
	}

	ret = sc8541_field_write(sc, F_ADC_RATE, 0);
	if (ret)
		return ret;

	return sc8541_field_write(sc, F_ADC_EN, 1);
}

static int sc8541_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {};
	unsigned int device_id;
	struct sc8541 *sc;
	unsigned int i;
	int ret;

	sc = devm_kzalloc(dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->dev = dev;
	i2c_set_clientdata(client, sc);

	sc->regmap = devm_regmap_init_i2c(client, &sc8541_regmap_config);
	if (IS_ERR(sc->regmap))
		return dev_err_probe(dev, PTR_ERR(sc->regmap),
				     "failed to init regmap\n");

	for (i = 0; i < F_MAX_FIELDS; i++) {
		sc->fields[i] = devm_regmap_field_alloc(dev, sc->regmap,
						       sc8541_reg_fields[i]);
		if (IS_ERR(sc->fields[i]))
			return dev_err_probe(dev, PTR_ERR(sc->fields[i]),
					     "failed to alloc field %u\n", i);
	}

	ret = sc8541_field_read(sc, F_DEVICE_ID, &device_id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read device id\n");

	if (device_id != SC8541_DEVICE_ID_VALUE)
		return dev_err_probe(dev, -ENODEV,
				     "unexpected device id %#x\n", device_id);

	ret = sc8541_hw_init(sc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init device\n");

	sc->adc_enabled = true;

	psy_cfg.drv_data = sc;
	psy_cfg.fwnode = dev_fwnode(dev);
	sc->psy = devm_power_supply_register(dev, &sc8541_psy_desc, &psy_cfg);
	if (IS_ERR(sc->psy))
		return dev_err_probe(dev, PTR_ERR(sc->psy),
				     "failed to register power supply\n");

	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						sc8541_irq_handler,
						IRQF_ONESHOT,
						dev_name(dev), sc);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request irq\n");
	}

	return 0;
}

static void sc8541_remove(struct i2c_client *client)
{
	struct sc8541 *sc = i2c_get_clientdata(client);

	sc8541_field_write(sc, F_CHG_EN, 0);
	sc8541_field_write(sc, F_ADC_EN, 0);
}

static const struct i2c_device_id sc8541_i2c_ids[] = {
	{ "sc8541" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc8541_i2c_ids);

static const struct of_device_id sc8541_of_match[] = {
	{ .compatible = "southchip,sc8541" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc8541_of_match);

static struct i2c_driver sc8541_driver = {
	.driver = {
		.name		= "sc8541-charger",
		.of_match_table	= sc8541_of_match,
	},
	.probe		= sc8541_probe,
	.remove		= sc8541_remove,
	.id_table	= sc8541_i2c_ids,
};
module_i2c_driver(sc8541_driver);

MODULE_AUTHOR("Junhao Xie <bigfoot@radxa.com>");
MODULE_DESCRIPTION("Southchip SC8541 charge pump driver");
MODULE_LICENSE("GPL");
