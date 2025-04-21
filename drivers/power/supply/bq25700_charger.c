// SPDX-License-Identifier: GPL-2.0
/*
 * TI BQ257000 charger driver

 * Copyright (c) 2021 Rockchip Electronics Co. Ltd.
 * Copyright (c) 2025 BigfootACA <bigfoot@classfun.cn>
 *
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/seq_file.h>

enum bq25700_fields {
	EN_LWPWR, WDTWR_ADJ, IDPM_AUTO_DISABLE,
	EN_OOA, PWM_FREQ, EN_LEARN, IADP_GAIN, IBAT_GAIN,
	EN_LDO, EN_IDPM, CHRG_INHIBIT,/*reg12h*/
	CHARGE_CURRENT,/*reg14h*/
	MAX_CHARGE_VOLTAGE,/*reg15h*/

	AC_STAT, ICO_DONE, IN_VINDPM, IN_IINDPM, IN_FCHRG, IN_PCHRG, IN_OTG,
	F_ACOV, F_BATOC, F_ACOC, SYSOVP_STAT, F_LATCHOFF, F_OTG_OVP, F_OTG_OCP,
	/*reg20h*/
	STAT_COMP, STAT_ICRIT, STAT_INOM, STAT_IDCHG, STAT_VSYS, STAT_BAT_REMOV,
	STAT_ADP_REMOV,/*reg21h*/
	INPUT_CURRENT_DPM,/*reg22h*/
	OUTPUT_INPUT_VOL, OUTPUT_SYS_POWER,/*reg23h*/
	OUTPUT_DSG_CUR,	OUTPUT_CHG_CUR,/*reg24h*/
	OUTPUT_INPUT_CUR, OUTPUT_CMPIN_VOL,/*reg25h*/
	OUTPUT_SYS_VOL, OUTPUT_BAT_VOL,/*reg26h*/

	EN_IBAT, EN_PROCHOT_LPWR, EN_PSYS, RSNS_RAC, RSNS_RSR,
	PSYS_RATIO, CMP_REF,	CMP_POL, CMP_DEG, FORCE_LATCHOFF,
	EN_SHIP_DCHG, AUTO_WAKEUP_EN, /*reg30h*/
	PKPWR_TOVLD_REG, EN_PKPWR_IDPM, EN_PKPWR_VSYS, PKPWER_OVLD_STAT,
	PKPWR_RELAX_STAT, PKPWER_TMAX,	EN_EXTILIM, EN_ICHG_IDCHG, Q2_OCP,
	ACX_OCP, EN_ACOC, ACOC_VTH, EN_BATOC, BATCOC_VTH,/*reg31h*/
	EN_HIZ, RESET_REG, RESET_VINDPM, EN_OTG, EN_ICO_MODE, BATFETOFF_HIZ,
	PSYS_OTG_IDCHG,/*reg32h*/
	ILIM2_VTH, ICRIT_DEG, VSYS_VTH, EN_PROCHOT_EXT, PROCHOT_WIDTH,
	PROCHOT_CLEAR, INOM_DEG,/*reg33h*/
	IDCHG_VTH, IDCHG_DEG, PROCHOT_PROFILE_COMP, PROCHOT_PROFILE_ICRIT,
	PROCHOT_PROFILE_INOM, PROCHOT_PROFILE_IDCHG,
	PROCHOT_PROFILE_VSYS, PROCHOT_PROFILE_BATPRES, PROCHOT_PROFILE_ACOK,
	/*reg34h*/
	ADC_CONV, ADC_START, ADC_FULLSCALE, EN_ADC_CMPIN, EN_ADC_VBUS,
	EN_ADC_PSYS, EN_ADC_IIN, EN_ADC_IDCHG, EN_ADC_ICHG, EN_ADC_VSYS,
	EN_ADC_VBAT,/*reg35h*/

	OTG_VOLTAGE,/*reg3bh*/
	OTG_CURRENT,/*reg3ch*/
	INPUT_VOLTAGE,/*reg3dh*/
	MIN_SYS_VOLTAGE,/*reg3eh*/
	INPUT_CURRENT,/*reg3fh*/

	MANUFACTURE_ID,/*regfeh*/
	DEVICE_ID,/*regffh*/

	F_MAX_FIELDS
};

/* initial field values, converted to register values */
struct bq25700_init_data {
	u32 ichg;		/* charge current */
	u32 max_chg_vol;	/* max charge voltage */
	u32 input_current;	/* input current */
	u32 sys_min_voltage;	/* mininum system voltage */
	u32 otg_voltage;	/* OTG voltage */
	u32 otg_current;	/* OTG current */
};

struct bq25700_state {
	u8 ac_stat;
	u8 ico_done;
	u8 in_vindpm;
	u8 in_iindpm;
	u8 in_fchrg;
	u8 in_pchrg;
	u8 in_otg;
	u8 fault_acov;
	u8 fault_batoc;
	u8 fault_acoc;
	u8 sysovp_stat;
	u8 fault_latchoff;
	u8 fault_otg_ovp;
	u8 fault_otg_ocp;
};

/*
 * Most of the val -> idx conversions can be computed, given the minimum,
 * maximum and the step between values. For the rest of conversions, we use
 * lookup tables.
 */
enum bq25700_table_ids {
	/* range tables */
	TBL_CHGCURSET,
	TBL_CHGVOLSET,
	TBL_INCURLIMIT,
	TBL_INVOL,
	TBL_SYSPWR,
	TBL_BATCHGCUR,
	TBL_BATDISCUR,
	TBL_INCUR,
	TBL_CMPIN,
	TBL_SYSVOL,
	TBL_BATVOL,
	TBL_OTGVOLSET,
	TBL_OTGCURSET,
	TBL_INVOLSET,
	TBL_MINSYSVOLSET,
	TBL_MAX,
};

struct bq25700_range {
	u32 min;
	u32 max;
	u32 step;
};

struct bq25700_table {
	struct bq25700_range range[TBL_MAX];
};

struct bq25700_chip_info {
	const char *const manufacturer;
	const char *const model;
	const struct bq25700_table *table;
	const struct regmap_config *reg_cfg;
	const struct reg_field *reg_fields;
	u32 reg_fields_count;
	u32 i2c_func;
};

struct bq25700_device {
	struct device			 *dev;
	struct power_supply		 *supply_battery;
	struct power_supply		 *supply_charger;
	const struct bq25700_chip_info	 *info;
	struct power_supply_battery_info *battery;
	unsigned int			 irq;
	struct regulator_dev		 *otg_vbus_reg;
	struct regmap			 *regmap;
	struct regmap_field		 *rmap_fields[F_MAX_FIELDS];
	int				 chip_id;
	struct bq25700_init_data	 init_data;
	struct bq25700_state		 state;
	struct dentry			 *debugfs;
};

static const struct reg_field bq25700_reg_fields[] = {
	/*REG12*/
	[EN_LWPWR] = REG_FIELD(0x12, 15, 15),
	[WDTWR_ADJ] = REG_FIELD(0x12, 13, 14),
	[IDPM_AUTO_DISABLE] = REG_FIELD(0x12, 12, 12),
	[EN_OOA] = REG_FIELD(0x12, 10, 10),
	[PWM_FREQ] = REG_FIELD(0x12, 9, 9),
	[EN_LEARN] = REG_FIELD(0x12, 5, 5),
	[IADP_GAIN] = REG_FIELD(0x12, 4, 4),
	[IBAT_GAIN] = REG_FIELD(0x12, 3, 3),
	[EN_LDO] = REG_FIELD(0x12, 2, 2),
	[EN_IDPM] = REG_FIELD(0x12, 1, 1),
	[CHRG_INHIBIT] = REG_FIELD(0x12, 0, 0),
	/*REG0x14*/
	[CHARGE_CURRENT] = REG_FIELD(0x14, 6, 12),
	/*REG0x15*/
	[MAX_CHARGE_VOLTAGE] = REG_FIELD(0x15, 4, 14),
	/*REG20*/
	[AC_STAT] = REG_FIELD(0x20, 15, 15),
	[ICO_DONE] = REG_FIELD(0x20, 14, 14),
	[IN_VINDPM] = REG_FIELD(0x20, 12, 12),
	[IN_IINDPM] = REG_FIELD(0x20, 11, 11),
	[IN_FCHRG] = REG_FIELD(0x20, 10, 10),
	[IN_PCHRG] = REG_FIELD(0x20, 9, 9),
	[IN_OTG] = REG_FIELD(0x20, 8, 8),
	[F_ACOV] = REG_FIELD(0x20, 7, 7),
	[F_BATOC] = REG_FIELD(0x20, 6, 6),
	[F_ACOC] = REG_FIELD(0x20, 5, 5),
	[SYSOVP_STAT] = REG_FIELD(0x20, 4, 4),
	[F_LATCHOFF] = REG_FIELD(0x20, 2, 2),
	[F_OTG_OVP] = REG_FIELD(0x20, 1, 1),
	[F_OTG_OCP] = REG_FIELD(0x20, 0, 0),
	/*REG21*/
	[STAT_COMP] = REG_FIELD(0x21, 6, 6),
	[STAT_ICRIT] = REG_FIELD(0x21, 5, 5),
	[STAT_INOM] = REG_FIELD(0x21, 4, 4),
	[STAT_IDCHG] = REG_FIELD(0x21, 3, 3),
	[STAT_VSYS] = REG_FIELD(0x21, 2, 2),
	[STAT_BAT_REMOV] = REG_FIELD(0x21, 1, 1),
	[STAT_ADP_REMOV] = REG_FIELD(0x21, 0, 0),
	/*REG22*/
	[INPUT_CURRENT_DPM] = REG_FIELD(0x22, 8, 14),
	/*REG23H*/
	[OUTPUT_INPUT_VOL] = REG_FIELD(0x23, 8, 15),
	[OUTPUT_SYS_POWER] = REG_FIELD(0x23, 0, 7),
	/*REG24H*/
	[OUTPUT_DSG_CUR] = REG_FIELD(0x24, 8, 14),
	[OUTPUT_CHG_CUR] = REG_FIELD(0x24, 0, 6),
	/*REG25H*/
	[OUTPUT_INPUT_CUR] = REG_FIELD(0x25, 8, 15),
	[OUTPUT_CMPIN_VOL] = REG_FIELD(0x25, 0, 7),
	/*REG26H*/
	[OUTPUT_SYS_VOL] = REG_FIELD(0x26, 8, 15),
	[OUTPUT_BAT_VOL] = REG_FIELD(0x26, 0, 6),

	/*REG30*/
	[EN_IBAT] = REG_FIELD(0x30, 15, 15),
	[EN_PROCHOT_LPWR] = REG_FIELD(0x30, 13, 14),
	[EN_PSYS] = REG_FIELD(0x30, 12, 12),
	[RSNS_RAC] = REG_FIELD(0x30, 11, 11),
	[RSNS_RSR] = REG_FIELD(0x30, 10, 10),
	[PSYS_RATIO] = REG_FIELD(0x30, 9, 9),
	[CMP_REF] = REG_FIELD(0x30, 7, 7),
	[CMP_POL] = REG_FIELD(0x30, 6, 6),
	[CMP_DEG] = REG_FIELD(0x30, 4, 5),
	[FORCE_LATCHOFF] = REG_FIELD(0x30, 3, 3),
	[EN_SHIP_DCHG] = REG_FIELD(0x30, 1, 1),
	[AUTO_WAKEUP_EN] = REG_FIELD(0x30, 0, 0),
	/*REG31*/
	[PKPWR_TOVLD_REG] = REG_FIELD(0x31, 14, 15),
	[EN_PKPWR_IDPM] = REG_FIELD(0x31, 13, 13),
	[EN_PKPWR_VSYS] = REG_FIELD(0x31, 12, 12),
	[PKPWER_OVLD_STAT] = REG_FIELD(0x31, 11, 11),
	[PKPWR_RELAX_STAT] = REG_FIELD(0x31, 10, 10),
	[PKPWER_TMAX] = REG_FIELD(0x31, 8, 9),
	[EN_EXTILIM] = REG_FIELD(0x31, 7, 7),
	[EN_ICHG_IDCHG] = REG_FIELD(0x31, 6, 6),
	[Q2_OCP] = REG_FIELD(0x31, 5, 5),
	[ACX_OCP] = REG_FIELD(0x31, 4, 4),
	[EN_ACOC] = REG_FIELD(0x31, 3, 3),
	[ACOC_VTH] = REG_FIELD(0x31, 2, 2),
	[EN_BATOC] = REG_FIELD(0x31, 1, 1),
	[BATCOC_VTH] = REG_FIELD(0x31, 0, 0),
	/*REG32*/
	[EN_HIZ] = REG_FIELD(0x32, 15, 15),
	[RESET_REG] = REG_FIELD(0x32, 14, 14),
	[RESET_VINDPM] = REG_FIELD(0x32, 13, 13),
	[EN_OTG] = REG_FIELD(0x32, 12, 12),
	[EN_ICO_MODE] = REG_FIELD(0x32, 11, 11),
	[BATFETOFF_HIZ] = REG_FIELD(0x32, 1, 1),
	[PSYS_OTG_IDCHG] = REG_FIELD(0x32, 0, 0),
	/*REG33*/
	[ILIM2_VTH] = REG_FIELD(0x33, 11, 15),
	[ICRIT_DEG] = REG_FIELD(0x33, 9, 10),
	[VSYS_VTH] = REG_FIELD(0x33, 6, 7),
	[EN_PROCHOT_EXT] = REG_FIELD(0x33, 5, 5),
	[PROCHOT_WIDTH] = REG_FIELD(0x33, 3, 4),
	[PROCHOT_CLEAR] = REG_FIELD(0x33, 2, 2),
	[INOM_DEG] = REG_FIELD(0x33, 1, 1),
	/*REG34*/
	[IDCHG_VTH] = REG_FIELD(0x34, 10, 15),
	[IDCHG_DEG] = REG_FIELD(0x34, 8, 9),
	[PROCHOT_PROFILE_COMP] = REG_FIELD(0x34, 6, 6),
	[PROCHOT_PROFILE_ICRIT] = REG_FIELD(0x34, 5, 5),
	[PROCHOT_PROFILE_INOM] = REG_FIELD(0x34, 4, 4),
	[PROCHOT_PROFILE_IDCHG] = REG_FIELD(0x34, 3, 3),
	[PROCHOT_PROFILE_VSYS] = REG_FIELD(0x34, 2, 2),
	[PROCHOT_PROFILE_BATPRES] = REG_FIELD(0x34, 1, 1),
	[PROCHOT_PROFILE_ACOK] = REG_FIELD(0x34, 0, 0),
	/*REG35*/
	[ADC_CONV] = REG_FIELD(0x35, 15, 15),
	[ADC_START] = REG_FIELD(0x35, 14, 14),
	[ADC_FULLSCALE] = REG_FIELD(0x35, 13, 13),
	[EN_ADC_CMPIN] = REG_FIELD(0x35, 7, 7),
	[EN_ADC_VBUS] = REG_FIELD(0x35, 6, 6),
	[EN_ADC_PSYS] = REG_FIELD(0x35, 5, 5),
	[EN_ADC_IIN] = REG_FIELD(0x35, 4, 4),
	[EN_ADC_IDCHG] = REG_FIELD(0x35, 3, 3),
	[EN_ADC_ICHG] = REG_FIELD(0x35, 2, 2),
	[EN_ADC_VSYS] = REG_FIELD(0x35, 1, 1),
	[EN_ADC_VBAT] = REG_FIELD(0x35, 0, 0),
	/*REG3B*/
	[OTG_VOLTAGE] = REG_FIELD(0x3B, 6, 13),
	/*REG3C*/
	[OTG_CURRENT] = REG_FIELD(0x3C, 8, 14),
	/*REG3D*/
	[INPUT_VOLTAGE] = REG_FIELD(0x3D, 6, 13),
	/*REG3E*/
	[MIN_SYS_VOLTAGE] = REG_FIELD(0x3E, 8, 13),
	/*REG3F*/
	[INPUT_CURRENT] = REG_FIELD(0x3F, 8, 14),

	/*REGFE*/
	[MANUFACTURE_ID] = REG_FIELD(0xFE, 0, 7),
	/*REFFF*/
	[DEVICE_ID] = REG_FIELD(0xFF, 0, 7),
};

static const struct reg_field bq25703_reg_fields[] = {
	/*REG00*/
	[EN_LWPWR] = REG_FIELD(0x00, 15, 15),
	[WDTWR_ADJ] = REG_FIELD(0x00, 13, 14),
	[IDPM_AUTO_DISABLE] = REG_FIELD(0x00, 12, 12),
	[EN_OOA] = REG_FIELD(0x00, 10, 10),
	[PWM_FREQ] = REG_FIELD(0x00, 9, 9),
	[EN_LEARN] = REG_FIELD(0x00, 5, 5),
	[IADP_GAIN] = REG_FIELD(0x00, 4, 4),
	[IBAT_GAIN] = REG_FIELD(0x00, 3, 3),
	[EN_LDO] = REG_FIELD(0x00, 2, 2),
	[EN_IDPM] = REG_FIELD(0x00, 1, 1),
	[CHRG_INHIBIT] = REG_FIELD(0x00, 0, 0),
	/*REG0x02*/
	[CHARGE_CURRENT] = REG_FIELD(0x02, 6, 12),
	/*REG0x04*/
	[MAX_CHARGE_VOLTAGE] = REG_FIELD(0x04, 4, 14),
	/*REG20*/
	[AC_STAT] = REG_FIELD(0x20, 15, 15),
	[ICO_DONE] = REG_FIELD(0x20, 14, 14),
	[IN_VINDPM] = REG_FIELD(0x20, 12, 12),
	[IN_IINDPM] = REG_FIELD(0x20, 11, 11),
	[IN_FCHRG] = REG_FIELD(0x20, 10, 10),
	[IN_PCHRG] = REG_FIELD(0x20, 9, 9),
	[IN_OTG] = REG_FIELD(0x20, 8, 8),
	[F_ACOV] = REG_FIELD(0x20, 7, 7),
	[F_BATOC] = REG_FIELD(0x20, 6, 6),
	[F_ACOC] = REG_FIELD(0x20, 5, 5),
	[SYSOVP_STAT] = REG_FIELD(0x20, 4, 4),
	[F_LATCHOFF] = REG_FIELD(0x20, 2, 2),
	[F_OTG_OVP] = REG_FIELD(0x20, 1, 1),
	[F_OTG_OCP] = REG_FIELD(0x20, 0, 0),
	/*REG22*/
	[STAT_COMP] = REG_FIELD(0x22, 6, 6),
	[STAT_ICRIT] = REG_FIELD(0x22, 5, 5),
	[STAT_INOM] = REG_FIELD(0x22, 4, 4),
	[STAT_IDCHG] = REG_FIELD(0x22, 3, 3),
	[STAT_VSYS] = REG_FIELD(0x22, 2, 2),
	[STAT_BAT_REMOV] = REG_FIELD(0x22, 1, 1),
	[STAT_ADP_REMOV] = REG_FIELD(0x22, 0, 0),
	/*REG24*/
	[INPUT_CURRENT_DPM] = REG_FIELD(0x24, 8, 14),

	/*REG26H*/
	[OUTPUT_INPUT_VOL] = REG_FIELD(0x26, 8, 15),
	[OUTPUT_SYS_POWER] = REG_FIELD(0x26, 0, 7),
	/*REG28H*/
	[OUTPUT_DSG_CUR] = REG_FIELD(0x28, 8, 14),
	[OUTPUT_CHG_CUR] = REG_FIELD(0x28, 0, 6),
	/*REG2aH*/
	[OUTPUT_INPUT_CUR] = REG_FIELD(0x2a, 8, 15),
	[OUTPUT_CMPIN_VOL] = REG_FIELD(0x2a, 0, 7),
	/*REG2cH*/
	[OUTPUT_SYS_VOL] = REG_FIELD(0x2c, 8, 15),
	[OUTPUT_BAT_VOL] = REG_FIELD(0x2c, 0, 7),

	/*REG30*/
	[EN_IBAT] = REG_FIELD(0x30, 15, 15),
	[EN_PROCHOT_LPWR] = REG_FIELD(0x30, 13, 14),
	[EN_PSYS] = REG_FIELD(0x30, 12, 12),
	[RSNS_RAC] = REG_FIELD(0x30, 11, 11),
	[RSNS_RSR] = REG_FIELD(0x30, 10, 10),
	[PSYS_RATIO] = REG_FIELD(0x30, 9, 9),
	[CMP_REF] = REG_FIELD(0x30, 7, 7),
	[CMP_POL] = REG_FIELD(0x30, 6, 6),
	[CMP_DEG] = REG_FIELD(0x30, 4, 5),
	[FORCE_LATCHOFF] = REG_FIELD(0x30, 3, 3),
	[EN_SHIP_DCHG] = REG_FIELD(0x30, 1, 1),
	[AUTO_WAKEUP_EN] = REG_FIELD(0x30, 0, 0),
	/*REG32*/
	[PKPWR_TOVLD_REG] = REG_FIELD(0x32, 14, 15),
	[EN_PKPWR_IDPM] = REG_FIELD(0x32, 13, 13),
	[EN_PKPWR_VSYS] = REG_FIELD(0x32, 12, 12),
	[PKPWER_OVLD_STAT] = REG_FIELD(0x32, 11, 11),
	[PKPWR_RELAX_STAT] = REG_FIELD(0x32, 10, 10),
	[PKPWER_TMAX] = REG_FIELD(0x32, 8, 9),
	[EN_EXTILIM] = REG_FIELD(0x32, 7, 7),
	[EN_ICHG_IDCHG] = REG_FIELD(0x32, 6, 6),
	[Q2_OCP] = REG_FIELD(0x32, 5, 5),
	[ACX_OCP] = REG_FIELD(0x32, 4, 4),
	[EN_ACOC] = REG_FIELD(0x32, 3, 3),
	[ACOC_VTH] = REG_FIELD(0x32, 2, 2),
	[EN_BATOC] = REG_FIELD(0x32, 1, 1),
	[BATCOC_VTH] = REG_FIELD(0x32, 0, 0),
	/*REG34*/
	[EN_HIZ] = REG_FIELD(0x34, 15, 15),
	[RESET_REG] = REG_FIELD(0x34, 14, 14),
	[RESET_VINDPM] = REG_FIELD(0x34, 13, 13),
	[EN_OTG] = REG_FIELD(0x34, 12, 12),
	[EN_ICO_MODE] = REG_FIELD(0x34, 11, 11),
	[BATFETOFF_HIZ] = REG_FIELD(0x34, 1, 1),
	[PSYS_OTG_IDCHG] = REG_FIELD(0x34, 0, 0),
	/*REG36*/
	[ILIM2_VTH] = REG_FIELD(0x36, 11, 15),
	[ICRIT_DEG] = REG_FIELD(0x36, 9, 10),
	[VSYS_VTH] = REG_FIELD(0x36, 6, 7),
	[EN_PROCHOT_EXT] = REG_FIELD(0x36, 5, 5),
	[PROCHOT_WIDTH] = REG_FIELD(0x36, 3, 4),
	[PROCHOT_CLEAR] = REG_FIELD(0x36, 2, 2),
	[INOM_DEG] = REG_FIELD(0x36, 1, 1),
	/*REG38*/
	[IDCHG_VTH] = REG_FIELD(0x38, 10, 15),
	[IDCHG_DEG] = REG_FIELD(0x38, 8, 9),
	[PROCHOT_PROFILE_COMP] = REG_FIELD(0x38, 6, 6),
	[PROCHOT_PROFILE_ICRIT] = REG_FIELD(0x38, 5, 5),
	[PROCHOT_PROFILE_INOM] = REG_FIELD(0x38, 4, 4),
	[PROCHOT_PROFILE_IDCHG] = REG_FIELD(0x38, 3, 3),
	[PROCHOT_PROFILE_VSYS] = REG_FIELD(0x38, 2, 2),
	[PROCHOT_PROFILE_BATPRES] = REG_FIELD(0x38, 1, 1),
	[PROCHOT_PROFILE_ACOK] = REG_FIELD(0x38, 0, 0),
	/*REG3a*/
	[ADC_CONV] = REG_FIELD(0x3a, 15, 15),
	[ADC_START] = REG_FIELD(0x3a, 14, 14),
	[ADC_FULLSCALE] = REG_FIELD(0x3a, 13, 13),
	[EN_ADC_CMPIN] = REG_FIELD(0x3a, 7, 7),
	[EN_ADC_VBUS] = REG_FIELD(0x3a, 6, 6),
	[EN_ADC_PSYS] = REG_FIELD(0x3a, 5, 5),
	[EN_ADC_IIN] = REG_FIELD(0x3a, 4, 4),
	[EN_ADC_IDCHG] = REG_FIELD(0x3a, 3, 3),
	[EN_ADC_ICHG] = REG_FIELD(0x3a, 2, 2),
	[EN_ADC_VSYS] = REG_FIELD(0x3a, 1, 1),
	[EN_ADC_VBAT] = REG_FIELD(0x3a, 0, 0),

	/*REG06*/
	[OTG_VOLTAGE] = REG_FIELD(0x06, 6, 13),
	/*REG08*/
	[OTG_CURRENT] = REG_FIELD(0x08, 8, 14),
	/*REG0a*/
	[INPUT_VOLTAGE] = REG_FIELD(0x0a, 6, 13),
	/*REG0C*/
	[MIN_SYS_VOLTAGE] = REG_FIELD(0x0c, 8, 13),
	/*REG0e*/
	[INPUT_CURRENT] = REG_FIELD(0x0e, 8, 14),

	/*REG2E*/
	[MANUFACTURE_ID] = REG_FIELD(0x2E, 0, 7),
	/*REF2F*/
	[DEVICE_ID] = REG_FIELD(0x2F, 0, 7),
};

static struct bq25700_table bq25700_range_tables = {{
	[TBL_CHGCURSET]    = {       0,  8128000,  64000}, /* uA (setting) charge current */
	[TBL_CHGVOLSET]    = {       0, 16800000,  16000}, /* uV (setting) charge voltage */
	[TBL_INCURLIMIT]   = {   50000,  6400000,  50000}, /* uA (limit)   input current */
	[TBL_INVOL]        = { 3200000, 19520000,  64000}, /* uV (output)  input voltage */
	[TBL_SYSPWR]       = {       0,  3060000,  12000}, /* uV (output)  system power */
	[TBL_BATCHGCUR]    = {       0,  8128000,  64000}, /* uA (output)  battery charge current */
	[TBL_BATDISCUR]    = {       0, 32512000, 256000}, /* uA (output)  battery discharge current */
	[TBL_INCUR]        = {       0, 12750000,  50000}, /* uA (output)  input current */
	[TBL_CMPIN]        = {       0,  3060000,  12000}, /* uV (output)  CMPIN voltage */
	[TBL_SYSVOL]       = { 2880000, 19200000,  64000}, /* uV (output)  system voltage */
	[TBL_BATVOL]       = { 2880000, 19200000,  64000}, /* uV (output)  battery voltage */
	[TBL_OTGVOLSET]    = { 4480000, 20800000,  64000}, /* uV (setting) otg voltage */
	[TBL_OTGCURSET]    = {       0,  6350000,  50000}, /* uA (setting) otg output current */
	[TBL_INVOLSET]     = { 3200000, 19520000,  64000}, /* uV (setting) input voltage */
	[TBL_MINSYSVOLSET] = { 1024000, 12880000, 256000}, /* uV (setting) minimum system voltage */
}};

static struct bq25700_table bq25703_range_tables = {{
	[TBL_CHGCURSET]    = {       0,  8128000,  64000}, /* uA (setting) charge current */
	[TBL_CHGVOLSET]    = {       0, 16800000,  16000}, /* uV (setting) charge voltage */
	[TBL_INCURLIMIT]   = {   50000,  6400000,  50000}, /* uA (limit)   input current */
	[TBL_INVOL]        = { 3200000, 19520000,  64000}, /* uV (output)  input voltage */
	[TBL_SYSPWR]       = {       0,  3060000,  12000}, /* uV (output)  system power */
	[TBL_BATCHGCUR]    = {       0,  8128000,  64000}, /* uA (output)  battery charge current */
	[TBL_BATDISCUR]    = {       0, 32512000, 256000}, /* uA (output)  battery discharge current */
	[TBL_INCUR]        = {       0, 12750000,  50000}, /* uA (output)  input current */
	[TBL_CMPIN]        = {       0,  3060000,  12000}, /* uV (output)  CMPIN voltage */
	[TBL_SYSVOL]       = { 2880000, 19200000,  64000}, /* uV (output)  system voltage */
	[TBL_BATVOL]       = { 2880000, 19200000,  64000}, /* uV (output)  battery voltage */
	[TBL_OTGVOLSET]    = { 4480000, 20800000,  64000}, /* uV (setting) otg voltage */
	[TBL_OTGCURSET]    = {       0,  6350000,  50000}, /* uA (setting) otg output current */
	[TBL_INVOLSET]     = { 3200000, 19520000,  64000}, /* uV (setting) input voltage */
	[TBL_MINSYSVOLSET] = { 1024000, 16182000, 256000}, /* uV (setting) minimum system voltage */
}};

static struct bq25700_table sc8886_range_tables = {{
	[TBL_CHGCURSET]    = {       0,  8128000,  64000}, /* uA (setting) charge current */
	[TBL_CHGVOLSET]    = {       0, 16800000,  16000}, /* uV (setting) charge voltage */
	[TBL_INCURLIMIT]   = {   50000,  6400000,  50000}, /* uA (limit)   input current */
	[TBL_INVOL]        = { 3200000, 19520000,  64000}, /* uV (output)  input voltage */
	[TBL_SYSPWR]       = {       0,  3060000,  12000}, /* uV (output)  system power */
	[TBL_BATCHGCUR]    = {       0,  8128000,  64000}, /* uA (output)  battery charge current */
	[TBL_BATDISCUR]    = {       0, 32512000, 256000}, /* uA (output)  battery discharge current */
	[TBL_INCUR]        = {       0, 12750000,  50000}, /* uA (output)  input current */
	[TBL_CMPIN]        = {       0,  3060000,  12000}, /* uV (output)  CMPIN voltage */
	[TBL_SYSVOL]       = { 2880000, 19200000,  64000}, /* uV (output)  system voltage */
	[TBL_BATVOL]       = { 2880000, 19200000,  64000}, /* uV (output)  battery voltage */
	[TBL_OTGVOLSET]    = { 1280000, 20800000,  64000}, /* uV (setting) otg voltage */
	[TBL_OTGCURSET]    = {       0,  6350000,  50000}, /* uA (setting) otg output current */
	[TBL_INVOLSET]     = { 3200000, 19520000,  64000}, /* uV (setting) input voltage */
	[TBL_MINSYSVOLSET] = { 1024000, 16182000, 256000}, /* uV (setting) minimum system voltage */
}};

static const struct regmap_range bq25700_readonly_reg_ranges[] = {
	regmap_reg_range(0x20, 0x26),
	regmap_reg_range(0xFE, 0xFF),
};

static const struct regmap_access_table bq25700_writeable_regs = {
	.no_ranges = bq25700_readonly_reg_ranges,
	.n_no_ranges = ARRAY_SIZE(bq25700_readonly_reg_ranges),
};

static const struct regmap_range bq25700_volatile_reg_ranges[] = {
	regmap_reg_range(0x12, 0x12),
	regmap_reg_range(0x14, 0x15),
	regmap_reg_range(0x20, 0x26),
	regmap_reg_range(0x30, 0x35),
	regmap_reg_range(0x3B, 0x3F),
	regmap_reg_range(0xFE, 0xFF),
};

static const struct regmap_access_table bq25700_volatile_regs = {
	.yes_ranges = bq25700_volatile_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(bq25700_volatile_reg_ranges),
};

static const struct regmap_config bq25700_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,

	.max_register = 0xFF,
	.cache_type = REGCACHE_RBTREE,

	.wr_table = &bq25700_writeable_regs,
	.volatile_table = &bq25700_volatile_regs,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static const struct regmap_range bq25703_readonly_reg_ranges[] = {
	regmap_reg_range(0x20, 0x2F),
};

static const struct regmap_access_table bq25703_writeable_regs = {
	.no_ranges = bq25703_readonly_reg_ranges,
	.n_no_ranges = ARRAY_SIZE(bq25703_readonly_reg_ranges),
};

static const struct regmap_range bq25703_volatile_reg_ranges[] = {
	regmap_reg_range(0x00, 0x0F),
	regmap_reg_range(0x20, 0x3B),
};

static const struct regmap_access_table bq25703_volatile_regs = {
	.yes_ranges = bq25703_volatile_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(bq25703_volatile_reg_ranges),
};

static const struct regmap_config bq25703_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,

	.max_register = 0x3B,
	.cache_type = REGCACHE_RBTREE,

	.wr_table = &bq25703_writeable_regs,
	.volatile_table = &bq25703_volatile_regs,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int bq25700_field_read(struct bq25700_device *charger,
			      enum bq25700_fields field_id)
{
	int ret;
	int val;

	ret = regmap_field_read(charger->rmap_fields[field_id], &val);
	if (ret < 0)
		return ret;

	return val;
}

static int bq25700_field_write(struct bq25700_device *charger,
			       enum bq25700_fields field_id, unsigned int val)
{
	return regmap_field_write(charger->rmap_fields[field_id], val);
}

static int bq25700_get_chip_state(struct bq25700_device *charger,
				  struct bq25700_state *state)
{
	int i, ret;

	struct {
		enum bq25700_fields id;
		u8 *data;
	} state_fields[] = {
		{ AC_STAT,	&state->ac_stat },
		{ ICO_DONE,	&state->ico_done },
		{ IN_VINDPM,	&state->in_vindpm },
		{ IN_IINDPM,	&state->in_iindpm },
		{ IN_FCHRG,	&state->in_fchrg },
		{ IN_PCHRG,	&state->in_pchrg },
		{ IN_OTG,	&state->in_otg },
		{ F_ACOV,	&state->fault_acov },
		{ F_BATOC,	&state->fault_batoc },
		{ F_ACOC,	&state->fault_acoc },
		{ SYSOVP_STAT,	&state->sysovp_stat },
		{ F_LATCHOFF,	&state->fault_latchoff },
		{ F_OTG_OVP,	&state->fault_otg_ovp },
		{ F_OTG_OCP,	&state->fault_otg_ocp },
	};

	for (i = 0; i < ARRAY_SIZE(state_fields); i++) {
		ret = bq25700_field_read(charger, state_fields[i].id);
		if (ret < 0)
			return ret;
		dev_dbg(charger->dev, "chip state field 0x%x: %d\n",
			state_fields[i].id, ret);
		*state_fields[i].data = ret;
	}

	return 0;
}

static u32 bq25700_find_idx(struct bq25700_device *charger, u32 value,
			    enum bq25700_table_ids id)
{
	u32 idx;
	u32 rtbl_size;
	const struct bq25700_range *rtbl = &charger->info->table->range[id];

	rtbl_size = (rtbl->max - rtbl->min) / rtbl->step + 1;

	for (idx = 1;
	     idx < rtbl_size && (idx * rtbl->step + rtbl->min <= value); idx++)
		;

	return idx - 1;
}

static u32 bq25700_find_value(struct bq25700_device *charger,
			      enum bq25700_table_ids id, int regval)
{
	const struct bq25700_range *rtbl = &charger->info->table->range[id];

	return (regval * rtbl->step + rtbl->min);
}

static int bq25700_read_value(struct bq25700_device *charger,
			      enum bq25700_table_ids id,
			      enum bq25700_fields reg)
{
	int field;

	field = bq25700_field_read(charger, reg);
	if (field < 0)
		return field;

	return bq25700_find_value(charger, id, field);
}

#if defined(CONFIG_DEBUG_FS)
static void print_field_head(struct bq25700_device *charger, struct seq_file *m,
			     const char *f_name, enum bq25700_fields f_id)
{
	const struct reg_field *regf = &charger->info->reg_fields[f_id];

	seq_printf(m, "%-24s reg 0x%02x bit %2d-%-2d", f_name, regf->reg,
		   regf->lsb, regf->msb);
}

static void print_reg_raw(struct bq25700_device *charger, struct seq_file *m,
			  const char *f_name, enum bq25700_fields f_id)
{
	int reg;

	print_field_head(charger, m, f_name, f_id);
	reg = bq25700_field_read(charger, f_id);
	seq_printf(m, "%10d / 0x%x\n", reg, reg);
}

static void print_spec_val(struct seq_file *m, const char *fmt, const char *tag,
			   long val, const char *unit)
{
	int idx = 0;
	long left = val, right = 0;
	static const char *punits[] = { "u", "m", "", NULL };
	char buff[256] = {};

	if (val != 0) {
		while ((val >= 1000000) && punits[idx + 1])
			val /= 1000, idx++;
		if (val >= 1000 && punits[idx + 1]) {
			left = val / 1000;
			right = (val % 1000) * 10 / 1000;
			idx++;
		}
	} else
		idx = 2;
	if (right == 0)
		sprintf(buff, "%s%ld%s%s", tag, left, punits[idx], unit);
	else
		sprintf(buff, "%s%ld.%ld%s%s", tag, left, right, punits[idx],
			unit);
	seq_printf(m, fmt, buff);
}

static void print_reg_val(struct bq25700_device *charger, struct seq_file *m,
			  const char *f_name, enum bq25700_fields f_id,
			  const char *t_name, enum bq25700_table_ids t_id,
			  const char *unit)
{
	int reg, val;
	const struct bq25700_range *rtbl = &charger->info->table->range[t_id];

	print_field_head(charger, m, f_name, f_id);
	reg = bq25700_field_read(charger, f_id);
	val = bq25700_find_value(charger, t_id, reg);
	print_spec_val(m, "%6s", "", val, unit);
	seq_printf(m, " (table %-16s ", t_name);
	print_spec_val(m, "%-10s", "min ", rtbl->min, unit);
	print_spec_val(m, "%-10s", "max ", rtbl->max, unit);
	print_spec_val(m, "%-10s", "step ", rtbl->step, unit);
	seq_printf(m, " raw %10d / 0x%x)\n", reg, reg);
}

static int bq25700_charger_info_show(struct seq_file *m, void *v)
{
	struct bq25700_device *charger = m->private;

#define ATTR_APPEND_RAW(_field) \
	print_reg_raw(charger, m, #_field, _field)
#define ATTR_APPEND_VAL(_field, _table, _unit) \
	print_reg_val(charger, m, #_field, _field, #_table, _table, #_unit)

	ATTR_APPEND_RAW(EN_LWPWR);
	ATTR_APPEND_RAW(WDTWR_ADJ);
	ATTR_APPEND_RAW(IDPM_AUTO_DISABLE);
	ATTR_APPEND_RAW(EN_OOA);
	ATTR_APPEND_RAW(PWM_FREQ);
	ATTR_APPEND_RAW(EN_LEARN);
	ATTR_APPEND_RAW(IADP_GAIN);
	ATTR_APPEND_RAW(IBAT_GAIN);
	ATTR_APPEND_RAW(EN_LDO);
	ATTR_APPEND_RAW(EN_IDPM);
	ATTR_APPEND_RAW(CHRG_INHIBIT);
	ATTR_APPEND_VAL(CHARGE_CURRENT, TBL_CHGCURSET, A);
	ATTR_APPEND_VAL(MAX_CHARGE_VOLTAGE, TBL_CHGVOLSET, V);
	ATTR_APPEND_RAW(AC_STAT);
	ATTR_APPEND_RAW(ICO_DONE);
	ATTR_APPEND_RAW(IN_VINDPM);
	ATTR_APPEND_RAW(IN_IINDPM);
	ATTR_APPEND_RAW(IN_FCHRG);
	ATTR_APPEND_RAW(IN_PCHRG);
	ATTR_APPEND_RAW(IN_OTG);
	ATTR_APPEND_RAW(F_ACOV);
	ATTR_APPEND_RAW(F_BATOC);
	ATTR_APPEND_RAW(F_ACOC);
	ATTR_APPEND_RAW(SYSOVP_STAT);
	ATTR_APPEND_RAW(F_LATCHOFF);
	ATTR_APPEND_RAW(F_OTG_OVP);
	ATTR_APPEND_RAW(F_OTG_OCP);
	ATTR_APPEND_RAW(STAT_COMP);
	ATTR_APPEND_RAW(STAT_ICRIT);
	ATTR_APPEND_RAW(STAT_INOM);
	ATTR_APPEND_RAW(STAT_IDCHG);
	ATTR_APPEND_RAW(STAT_VSYS);
	ATTR_APPEND_RAW(STAT_BAT_REMOV);
	ATTR_APPEND_RAW(STAT_ADP_REMOV);
	ATTR_APPEND_VAL(INPUT_CURRENT_DPM, TBL_INCURLIMIT, A);
	ATTR_APPEND_VAL(OUTPUT_INPUT_VOL, TBL_INVOL, V);
	ATTR_APPEND_VAL(OUTPUT_SYS_POWER, TBL_SYSPWR, V);
	ATTR_APPEND_VAL(OUTPUT_DSG_CUR, TBL_BATDISCUR, A);
	ATTR_APPEND_VAL(OUTPUT_CHG_CUR, TBL_BATCHGCUR, A);
	ATTR_APPEND_VAL(OUTPUT_INPUT_CUR, TBL_INCUR, A);
	ATTR_APPEND_VAL(OUTPUT_CMPIN_VOL, TBL_CMPIN, V);
	ATTR_APPEND_VAL(OUTPUT_SYS_VOL, TBL_SYSVOL, V);
	ATTR_APPEND_VAL(OUTPUT_BAT_VOL, TBL_BATVOL, V);
	ATTR_APPEND_RAW(EN_IBAT);
	ATTR_APPEND_RAW(EN_PROCHOT_LPWR);
	ATTR_APPEND_RAW(EN_PSYS);
	ATTR_APPEND_RAW(RSNS_RAC);
	ATTR_APPEND_RAW(RSNS_RSR);
	ATTR_APPEND_RAW(PSYS_RATIO);
	ATTR_APPEND_RAW(CMP_REF);
	ATTR_APPEND_RAW(CMP_POL);
	ATTR_APPEND_RAW(CMP_DEG);
	ATTR_APPEND_RAW(FORCE_LATCHOFF);
	ATTR_APPEND_RAW(EN_SHIP_DCHG);
	ATTR_APPEND_RAW(AUTO_WAKEUP_EN);
	ATTR_APPEND_RAW(PKPWR_TOVLD_REG);
	ATTR_APPEND_RAW(EN_PKPWR_IDPM);
	ATTR_APPEND_RAW(EN_PKPWR_VSYS);
	ATTR_APPEND_RAW(PKPWER_OVLD_STAT);
	ATTR_APPEND_RAW(PKPWR_RELAX_STAT);
	ATTR_APPEND_RAW(PKPWER_TMAX);
	ATTR_APPEND_RAW(EN_EXTILIM);
	ATTR_APPEND_RAW(EN_ICHG_IDCHG);
	ATTR_APPEND_RAW(Q2_OCP);
	ATTR_APPEND_RAW(ACX_OCP);
	ATTR_APPEND_RAW(EN_ACOC);
	ATTR_APPEND_RAW(ACOC_VTH);
	ATTR_APPEND_RAW(EN_BATOC);
	ATTR_APPEND_RAW(BATCOC_VTH);
	ATTR_APPEND_RAW(EN_HIZ);
	ATTR_APPEND_RAW(RESET_REG);
	ATTR_APPEND_RAW(RESET_VINDPM);
	ATTR_APPEND_RAW(EN_OTG);
	ATTR_APPEND_RAW(EN_ICO_MODE);
	ATTR_APPEND_RAW(BATFETOFF_HIZ);
	ATTR_APPEND_RAW(PSYS_OTG_IDCHG);
	ATTR_APPEND_RAW(ILIM2_VTH);
	ATTR_APPEND_RAW(ICRIT_DEG);
	ATTR_APPEND_RAW(VSYS_VTH);
	ATTR_APPEND_RAW(EN_PROCHOT_EXT);
	ATTR_APPEND_RAW(PROCHOT_WIDTH);
	ATTR_APPEND_RAW(PROCHOT_CLEAR);
	ATTR_APPEND_RAW(INOM_DEG);
	ATTR_APPEND_RAW(IDCHG_VTH);
	ATTR_APPEND_RAW(IDCHG_DEG);
	ATTR_APPEND_RAW(PROCHOT_PROFILE_COMP);
	ATTR_APPEND_RAW(PROCHOT_PROFILE_ICRIT);
	ATTR_APPEND_RAW(PROCHOT_PROFILE_INOM);
	ATTR_APPEND_RAW(PROCHOT_PROFILE_IDCHG);
	ATTR_APPEND_RAW(PROCHOT_PROFILE_VSYS);
	ATTR_APPEND_RAW(PROCHOT_PROFILE_BATPRES);
	ATTR_APPEND_RAW(PROCHOT_PROFILE_ACOK);
	ATTR_APPEND_RAW(ADC_CONV);
	ATTR_APPEND_RAW(ADC_START);
	ATTR_APPEND_RAW(ADC_FULLSCALE);
	ATTR_APPEND_RAW(EN_ADC_CMPIN);
	ATTR_APPEND_RAW(EN_ADC_VBUS);
	ATTR_APPEND_RAW(EN_ADC_PSYS);
	ATTR_APPEND_RAW(EN_ADC_IIN);
	ATTR_APPEND_RAW(EN_ADC_IDCHG);
	ATTR_APPEND_RAW(EN_ADC_ICHG);
	ATTR_APPEND_RAW(EN_ADC_VSYS);
	ATTR_APPEND_RAW(EN_ADC_VBAT);
	ATTR_APPEND_VAL(OTG_VOLTAGE, TBL_OTGVOLSET, V);
	ATTR_APPEND_VAL(OTG_CURRENT, TBL_OTGCURSET, A);
	ATTR_APPEND_VAL(INPUT_VOLTAGE, TBL_INVOLSET, V);
	ATTR_APPEND_VAL(MIN_SYS_VOLTAGE, TBL_MINSYSVOLSET, V);
	ATTR_APPEND_VAL(INPUT_CURRENT, TBL_INCURLIMIT, A);
	ATTR_APPEND_RAW(MANUFACTURE_ID);
	ATTR_APPEND_RAW(DEVICE_ID);
	return 0;
}

DEFINE_SHOW_ATTRIBUTE(bq25700_charger_info);

static int bq25700_init_debugfs(struct bq25700_device *charger)
{
	charger->debugfs = debugfs_create_dir(dev_name(charger->dev), NULL);
	debugfs_create_file("charger_info", 0400, charger->debugfs, charger,
			    &bq25700_charger_info_fops);
	return 0;
}
#else
static int bq25700_init_debugfs(struct bq25700_device *charger)
{
	return 0;
}
#endif

static int bq25700_fw_probe(struct bq25700_device *charger)
{
	int ret, i;
	u32 property;
	const struct bq25700_range *range;
	struct bq25700_init_data *init = &charger->init_data;
	struct {
		char *name;
		bool voltage;
		bool optional;
		enum bq25700_table_ids tbl_id;
		u32 *conv_data; /* holds converted value from given property */
		u32 default_value;
	} props[] = {
		/* required properties */
		{ "ti,charge-current", false, false, TBL_CHGCURSET,
		  &init->ichg },
		{ "ti,max-charge-voltage", true, false, TBL_CHGVOLSET,
		  &init->max_chg_vol },
		{ "ti,input-current", false, true, TBL_INCURLIMIT,
		  &init->input_current, 2000000 },
		{ "ti,minimum-sys-voltage", true, false, TBL_MINSYSVOLSET,
		  &init->sys_min_voltage },
		{ "ti,otg-voltage", true, true, TBL_OTGVOLSET,
		  &init->otg_voltage, 5000000 },
		{ "ti,otg-current", false, true, TBL_OTGCURSET,
		  &init->otg_current, 2000000 },
	};

	/* initialize data for optional properties */
	for (i = 0; i < ARRAY_SIZE(props); i++) {
		ret = device_property_read_u32(charger->dev, props[i].name,
					       &property);
		if (ret < 0) {
			if (props[i].optional) {
				property = props[i].default_value;
			} else {
				dev_err(charger->dev, "missing property %s\n",
					props[i].name);
				return ret;
			}
		}
		range = &charger->info->table->range[props[i].tbl_id];
		if (property > range->max || property < range->min) {
			dev_err(charger->dev,
				"property %s out of range (%s %u > %u > %u)\n",
				props[i].name,
				props[i].voltage ? "voltage" : "current",
				property, range->min, range->max);
			return -ENODEV;
		}

		*props[i].conv_data =
			bq25700_find_idx(charger, property, props[i].tbl_id);
		dev_dbg(charger->dev, "%s, val: %d, tbl_id = %d\n",
			props[i].name, property, *props[i].conv_data);
	}

	return 0;
}

static int bq25700_hw_init(struct bq25700_device *charger)
{
	int ret;
	int i;

	const struct {
		enum bq25700_fields id;
		u32 value;
	} init_data[] = {
		{ CHARGE_CURRENT,	charger->init_data.ichg },
		{ MAX_CHARGE_VOLTAGE,	charger->init_data.max_chg_vol },
		{ INPUT_CURRENT,	charger->init_data.input_current },
		{ MIN_SYS_VOLTAGE,	charger->init_data.sys_min_voltage },
		{ OTG_VOLTAGE,		charger->init_data.otg_voltage },
		{ OTG_CURRENT,		charger->init_data.otg_current },
	};

	/* disable watchdog */
	ret = bq25700_field_write(charger, WDTWR_ADJ, 0);
	if (ret < 0)
		return ret;

	/* initialize currents/voltages and other parameters */
	for (i = 0; i < ARRAY_SIZE(init_data); i++) {
		ret = bq25700_field_write(charger, init_data[i].id,
					  init_data[i].value);
		if (ret < 0)
			return ret;
	}

	ret = bq25700_read_value(charger, TBL_CHGCURSET, CHARGE_CURRENT);
	dev_dbg(charger->dev, "charge current: %dmA\n", ret / 1000);
	ret = bq25700_read_value(charger, TBL_CHGVOLSET, MAX_CHARGE_VOLTAGE);
	dev_dbg(charger->dev, "max charge voltage: %dmV\n", ret / 1000);
	ret = bq25700_read_value(charger, TBL_INVOL, OUTPUT_INPUT_VOL);
	dev_dbg(charger->dev, "input voltage: %dmV\n", ret / 1000);
	ret = bq25700_read_value(charger, TBL_INCUR, OUTPUT_INPUT_CUR);
	dev_dbg(charger->dev, "input current: %dmA\n", ret / 1000);
	ret = bq25700_read_value(charger, TBL_MINSYSVOLSET, MIN_SYS_VOLTAGE);
	dev_dbg(charger->dev, "min sys voltage: %dmV\n", ret / 1000);

#define bq25700_do_field_write(charger, field, val)                           \
	ret = bq25700_field_write(charger, field, val);                       \
	if (ret < 0) {                                                        \
		dev_warn(charger->dev,                                        \
			 "bq25700_field_write " #field " failed: %d\n", ret); \
		return ret;                                                   \
	}

	/* Configure ADC for continuous conversions. This does not enable it. */
	bq25700_do_field_write(charger, EN_LWPWR, 0);
	bq25700_do_field_write(charger, ADC_CONV, 1);
	bq25700_do_field_write(charger, ADC_START, 1);
	bq25700_do_field_write(charger, ADC_FULLSCALE, 1);
	bq25700_do_field_write(charger, EN_ADC_CMPIN, 1);
	bq25700_do_field_write(charger, EN_ADC_VBUS, 1);
	bq25700_do_field_write(charger, EN_ADC_PSYS, 1);
	bq25700_do_field_write(charger, EN_ADC_IIN, 1);
	bq25700_do_field_write(charger, EN_ADC_IDCHG, 1);
	bq25700_do_field_write(charger, EN_ADC_ICHG, 1);
	bq25700_do_field_write(charger, EN_ADC_VSYS, 1);
	bq25700_do_field_write(charger, EN_ADC_VBAT, 1);

	ret = bq25700_get_chip_state(charger, &charger->state);
	if (ret)
		dev_warn(charger->dev, "read chip state failed: %d\n", ret);

	return 0;
}

static enum power_supply_property bq25700_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
};

static enum power_supply_property bq25700_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CAPACITY,
};

static int bq25700_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq25700_device *bq = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!bq->state.ac_stat)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (bq->state.in_fchrg || bq->state.in_pchrg)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq->state.ac_stat;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (!bq->state.fault_acoc && !bq->state.fault_acov &&
		    !bq->state.fault_batoc)
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		else if (bq->state.fault_batoc)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval =
			bq25700_read_value(bq, TBL_CHGCURSET, CHARGE_CURRENT);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = bq25700_read_value(bq, TBL_CHGVOLSET,
						 MAX_CHARGE_VOLTAGE);
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval =
			bq25700_read_value(bq, TBL_INCURLIMIT, INPUT_CURRENT);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		val->intval =
			bq25700_read_value(bq, TBL_INVOLSET, INPUT_VOLTAGE);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (!bq->state.ac_stat) {
			val->intval = 0;
			break;
		}
		val->intval =
			bq25700_read_value(bq, TBL_INVOL, OUTPUT_INPUT_VOL);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval =
			bq25700_read_value(bq, TBL_INCUR, OUTPUT_INPUT_CUR);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = bq->info->table->range[TBL_CHGCURSET].max;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = bq->info->table->range[TBL_CHGVOLSET].max;
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = bq->info->table->range[TBL_INVOLSET].max;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = bq->info->table->range[TBL_INVOL].min;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = bq->info->table->range[TBL_INVOL].max;
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = bq->info->table->range[TBL_INCUR].max;
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = bq->info->manufacturer;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bq->info->model;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (!bq->state.ac_stat)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		else if (bq->state.in_fchrg)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else if (bq->state.in_pchrg)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int bq25700_charger_set_property(struct power_supply *psy,
					enum power_supply_property prop,
					const union power_supply_propval *val)
{
	int idx;
	struct bq25700_device *bq = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		idx = bq25700_find_idx(bq, val->intval, TBL_INCURLIMIT);
		dev_dbg(bq->dev, "set input current limit: %duA (%d)\n",
			val->intval, idx);
		return bq25700_field_write(bq, INPUT_CURRENT, idx);

	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		idx = bq25700_find_idx(bq, val->intval, TBL_INVOLSET);
		dev_dbg(bq->dev, "set input voltage limit: %duV (%d)\n",
			val->intval, idx);
		return bq25700_field_write(bq, INPUT_VOLTAGE, idx);

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		idx = bq25700_find_idx(bq, val->intval, TBL_CHGCURSET);
		dev_dbg(bq->dev, "set charge current: %duA (%d)\n", val->intval,
			idx);
		return bq25700_field_write(bq, CHARGE_CURRENT, idx);

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		idx = bq25700_find_idx(bq, val->intval, TBL_CHGVOLSET);
		dev_dbg(bq->dev, "set max charge voltage: %duV (%d)\n",
			val->intval, idx);
		return bq25700_field_write(bq, MAX_CHARGE_VOLTAGE, idx);

	default:
		return -EINVAL;
	}
}

static int bq25700_charger_property_is_writeable(struct power_supply *psy,
						 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return true;
	default:
		return false;
	}
}

static int bq25700_read_battery_current(struct bq25700_device *bq)
{
	int chg, dis;

	chg = bq25700_read_value(bq, TBL_BATCHGCUR, OUTPUT_CHG_CUR);
	dis = bq25700_read_value(bq, TBL_BATDISCUR, OUTPUT_DSG_CUR);

	return chg > 0 ? chg : -dis;
}

static int bq25700_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret;
	struct bq25700_device *bq = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval =
			bq25700_read_value(bq, TBL_BATVOL, OUTPUT_BAT_VOL) >=
			bq->battery->voltage_min_design_uv;
		break;

	case POWER_SUPPLY_PROP_STATUS:
		ret = bq25700_read_battery_current(bq);
		if (ret <= -50000)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (ret >= 200000)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (!bq->state.fault_acoc && !bq->state.fault_acov &&
		    !bq->state.fault_batoc)
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		else if (bq->state.fault_batoc)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval =
			bq25700_read_value(bq, TBL_BATVOL, OUTPUT_BAT_VOL);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq25700_read_battery_current(bq);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = bq->info->table->range[TBL_BATVOL].min;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = bq->info->table->range[TBL_BATVOL].max;
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = bq->info->table->range[TBL_BATCHGCUR].max;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		ret = bq25700_read_value(bq, TBL_BATVOL, OUTPUT_BAT_VOL);
		val->intval =
			power_supply_batinfo_ocv2cap(bq->battery, ret, 20);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct power_supply_desc bq25700_charger_desc = {
	.name = "bq25700-charger",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = bq25700_charger_props,
	.num_properties = ARRAY_SIZE(bq25700_charger_props),
	.get_property = bq25700_charger_get_property,
	.set_property = bq25700_charger_set_property,
	.property_is_writeable = bq25700_charger_property_is_writeable,
};

static const struct power_supply_desc bq25700_battery_desc = {
	.name = "bq25700-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = bq25700_battery_props,
	.num_properties = ARRAY_SIZE(bq25700_battery_props),
	.get_property = bq25700_battery_get_property,
};

static int bq25700_supply_init(struct bq25700_device *bq)
{
	int ret;
	struct power_supply_config psy_cfg = {};

	psy_cfg.drv_data = bq;
	psy_cfg.fwnode = dev_fwnode(bq->dev);

	bq->supply_charger = devm_power_supply_register(
		bq->dev, &bq25700_charger_desc, &psy_cfg);
	if (PTR_ERR_OR_ZERO(bq->supply_charger)) {
		dev_err(bq->dev, "failed to register charger: %pE\n",
			bq->supply_charger);
		return PTR_ERR(bq->supply_charger);
	}

	if (of_property_present(bq->dev->of_node, "monitored-battery")) {
		bq->supply_battery = devm_power_supply_register(
			bq->dev, &bq25700_battery_desc, &psy_cfg);
		if (PTR_ERR_OR_ZERO(bq->supply_battery)) {
			dev_err(bq->dev, "failed to register battery: %pE\n",
				bq->supply_battery);
			return PTR_ERR(bq->supply_battery);
		}

		ret = power_supply_get_battery_info(bq->supply_battery,
						    &bq->battery);
		if (ret) {
			dev_err(bq->dev, "failed to get battery info: %d\n",
				ret);
			return ret;
		}
	}

	return 0;
}

static void bq25700_update(struct bq25700_device *charger)
{
	struct bq25700_state state = {};

	if (bq25700_get_chip_state(charger, &state) < 0)
		return;
	if (memcmp(&state, &charger->state, sizeof(state)) != 0) {
		memcpy(&charger->state, &state, sizeof(state));
		if (charger->supply_battery)
			power_supply_changed(charger->supply_battery);
		if (charger->supply_charger)
			power_supply_changed(charger->supply_charger);
	}
}

static irqreturn_t bq25700_irq_handler_thread(int irq, void *private)
{
	struct bq25700_device *charger = private;

	bq25700_update(charger);

	return IRQ_HANDLED;
}

static int bq25700_otg_vbus_enable(struct regulator_dev *dev)
{
	struct bq25700_device *charger = rdev_get_drvdata(dev);

	return bq25700_field_write(charger, EN_OTG, true);
}

static int bq25700_otg_vbus_disable(struct regulator_dev *dev)
{
	struct bq25700_device *charger = rdev_get_drvdata(dev);

	return bq25700_field_write(charger, EN_OTG, false);
}

static int bq25700_otg_vbus_is_enabled(struct regulator_dev *dev)
{
	struct bq25700_device *charger = rdev_get_drvdata(dev);

	return bq25700_field_read(charger, EN_OTG) ? 1 : 0;
}

static int bq25700_otg_vbus_get_voltage(struct regulator_dev *dev)
{
	struct bq25700_device *charger = rdev_get_drvdata(dev);

	return bq25700_read_value(charger, TBL_OTGVOLSET, OTG_VOLTAGE);
}

static int bq25700_otg_vbus_set_voltage(struct regulator_dev *dev, int min_uV,
					int max_uV, unsigned *selector)
{
	u32 idx;
	struct bq25700_device *charger = rdev_get_drvdata(dev);

	idx = bq25700_find_idx(charger, min_uV, TBL_OTGVOLSET);
	return bq25700_field_write(charger, OTG_VOLTAGE, idx);
}

static int bq25700_otg_vbus_get_current_limit(struct regulator_dev *dev)
{
	struct bq25700_device *charger = rdev_get_drvdata(dev);

	return bq25700_read_value(charger, TBL_OTGCURSET, OTG_CURRENT);
}

static int bq25700_otg_vbus_set_current_limit(struct regulator_dev *dev,
					      int min_uA, int max_uA)
{
	u32 idx;
	struct bq25700_device *charger = rdev_get_drvdata(dev);

	idx = bq25700_find_idx(charger, min_uA, TBL_OTGCURSET);

	return bq25700_field_write(charger, OTG_CURRENT, idx);
}

static const struct regulator_ops bq25700_otg_vbus_ops = {
	.enable = bq25700_otg_vbus_enable,
	.disable = bq25700_otg_vbus_disable,
	.is_enabled = bq25700_otg_vbus_is_enabled,
	.get_voltage = bq25700_otg_vbus_get_voltage,
	.set_voltage = bq25700_otg_vbus_set_voltage,
	.get_current_limit = bq25700_otg_vbus_get_current_limit,
	.set_current_limit = bq25700_otg_vbus_set_current_limit,
};

static const struct regulator_desc bq25700_otg_vbus_desc = {
	.name = "usb-otg-vbus",
	.of_match = "usb-otg-vbus",
	.regulators_node = of_match_ptr("regulators"),
	.owner = THIS_MODULE,
	.ops = &bq25700_otg_vbus_ops,
	.type = REGULATOR_VOLTAGE,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static int bq25700_register_otg_vbus_regulator(struct bq25700_device *charger)
{
	struct regulator_config config = {};

	config.dev = charger->dev;
	config.driver_data = charger;

	charger->otg_vbus_reg = devm_regulator_register(
		charger->dev, &bq25700_otg_vbus_desc, &config);
	if (IS_ERR(charger->otg_vbus_reg)) {
		dev_err(charger->dev,
			"failed to register otg vbus regulator: %pE\n",
			charger->otg_vbus_reg);
		return PTR_ERR(charger->otg_vbus_reg);
	}

	return 0;
}

static int bq25700_probe(struct i2c_client *client)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct bq25700_device *charger;
	int ret = 0;
	u32 i = 0;

	charger = devm_kzalloc(&client->dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -EINVAL;

	charger->dev = dev;

	charger->info = i2c_get_match_data(client);
	if (!charger->info)
		return dev_err_probe(dev, -ENODEV, "Failed to match device\n");

	if (charger->info->i2c_func != 0 &&
	    !i2c_check_functionality(adapter, charger->info->i2c_func))
		return dev_err_probe(dev, -EIO,
				     "Required I2C functionality not found\n");

	charger->regmap = devm_regmap_init_i2c(client, charger->info->reg_cfg);
	if (IS_ERR(charger->regmap))
		return dev_err_probe(dev, PTR_ERR(charger->regmap),
				     "Failed to initialize regmap\n");

	for (i = 0; i < charger->info->reg_fields_count; i++) {
		charger->rmap_fields[i] = devm_regmap_field_alloc(
			dev, charger->regmap, charger->info->reg_fields[i]);
		if (IS_ERR(charger->rmap_fields[i]))
			return dev_err_probe(dev,
					     PTR_ERR(charger->rmap_fields[i]),
					     "Cannot allocate regmap field\n");
	}

	i2c_set_clientdata(client, charger);

	charger->chip_id = bq25700_field_read(charger, DEVICE_ID);
	if (charger->chip_id < 0)
		return dev_err_probe(dev, charger->chip_id,
				     "Cannot read chip ID\n");

	ret = bq25700_fw_probe(charger);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Cannot read device properties\n");

	ret = bq25700_hw_init(charger);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Cannot initialize the chip\n");

	ret = bq25700_supply_init(charger);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Cannot initialize power supply\n");

	ret = bq25700_register_otg_vbus_regulator(charger);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Cannot initialize vbus regulator\n");

	if (client->irq) {
		ret = devm_request_threaded_irq(
			dev, client->irq, NULL, bq25700_irq_handler_thread,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT,
			"bq25700-irq", charger);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "Cannot initialize interrupt\n");
		enable_irq_wake(client->irq);
	}

	bq25700_init_debugfs(charger);

	dev_info(charger->dev, "found device %s %s\n",
		 charger->info->manufacturer, charger->info->model);

	return 0;
}

static void bq25700_shutdown(struct i2c_client *client)
{
	struct bq25700_device *charger = i2c_get_clientdata(client);

	if (!bq25700_field_read(charger, AC_STAT))
		bq25700_field_write(charger, EN_LWPWR, 1);
}

static void bq25700_remove(struct i2c_client *client)
{
	struct bq25700_device *charger = i2c_get_clientdata(client);

	debugfs_remove_recursive(charger->debugfs);
}

#ifdef CONFIG_PM_SLEEP
static int bq25700_pm_suspend(struct device *dev)
{
	struct bq25700_device *charger = dev_get_drvdata(dev);

	if (!bq25700_field_read(charger, AC_STAT))
		bq25700_field_write(charger, EN_LWPWR, 1);

	return 0;
}

static int bq25700_pm_resume(struct device *dev)
{
	struct bq25700_device *charger = dev_get_drvdata(dev);

	bq25700_field_write(charger, EN_LWPWR, 0);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(bq25700_pm_ops, bq25700_pm_suspend, bq25700_pm_resume);

static const struct bq25700_chip_info bq25700_info = {
	.manufacturer = "Texas Instruments",
	.model = "BQ25700",
	.table = &bq25700_range_tables,
	.reg_cfg = &bq25700_regmap_config,
	.reg_fields = bq25700_reg_fields,
	.reg_fields_count = ARRAY_SIZE(bq25700_reg_fields),
	.i2c_func = I2C_FUNC_SMBUS_WORD_DATA,
};

static const struct bq25700_chip_info bq25703_info = {
	.manufacturer = "Texas Instruments",
	.model = "BQ25703",
	.table = &bq25703_range_tables,
	.reg_cfg = &bq25703_regmap_config,
	.reg_fields = bq25703_reg_fields,
	.reg_fields_count = ARRAY_SIZE(bq25703_reg_fields),
};

static const struct bq25700_chip_info sc8886_info = {
	.manufacturer = "SouthChip",
	.model = "SC8886",
	.table = &sc8886_range_tables,
	.reg_cfg = &bq25703_regmap_config,
	.reg_fields = bq25703_reg_fields,
	.reg_fields_count = ARRAY_SIZE(bq25703_reg_fields),
};

static const struct bq25700_chip_info sc8885_info = {
	.manufacturer = "SouthChip",
	.model = "SC8885",
	.table = &bq25700_range_tables,
	.reg_cfg = &bq25700_regmap_config,
	.reg_fields = bq25700_reg_fields,
	.reg_fields_count = ARRAY_SIZE(bq25700_reg_fields),
	.i2c_func = I2C_FUNC_SMBUS_WORD_DATA,
};

static const struct i2c_device_id bq25700_i2c_ids[] = {
	{ "bq25700", (kernel_ulong_t)&bq25700_info },
	{ "bq25703", (kernel_ulong_t)&bq25703_info },
	{ "sc8886", (kernel_ulong_t)&sc8886_info },
	{ "sc8885", (kernel_ulong_t)&sc8885_info },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq25700_i2c_ids);

static const struct of_device_id bq25700_of_match[] = {
	{ .compatible = "ti,bq25700", &bq25700_info },
	{ .compatible = "ti,bq25703", &bq25703_info },
	{ .compatible = "southchip,sc8886", &sc8886_info },
	{ .compatible = "southchip,sc8885", &sc8885_info },
	{},
};
MODULE_DEVICE_TABLE(of, bq25700_of_match);

static struct i2c_driver bq25700_driver = {
	.probe		= bq25700_probe,
	.remove		= bq25700_remove,
	.shutdown	= bq25700_shutdown,
	.id_table	= bq25700_i2c_ids,
	.driver = {
		.name		= "bq25700-charger",
		.pm		= &bq25700_pm_ops,
		.of_match_table	= of_match_ptr(bq25700_of_match),
	},
};
module_i2c_driver(bq25700_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("shengfeixu <xsf@rock-chips.com>");
MODULE_AUTHOR("BigfootACA <bigfoot@classfun.cn>");
MODULE_DESCRIPTION("TI bq25700 Charger Driver");
