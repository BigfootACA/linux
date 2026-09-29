// SPDX-License-Identifier: GPL-2.0-only
/*
 * Xiaomi MCA charger control over PMIC GLINK
 *
 * Copyright (c) 2026 BigfootACA <bigfoot@classfun.cn>
 *
 * Boards using Xiaomi's "MCA" charging framework do not get the stock
 * Qualcomm battery manager service from their charger PD firmware. Instead the
 * firmware only proxies the charger block of the PMIC, and every policy
 * decision - including whether to charge at all - is expected to come from the
 * application processor. Without a driver the port negotiates USB Power
 * Delivery just fine and then nothing happens, because the buck charger is
 * never taken out of its disabled state.
 *
 * This driver speaks the property protocol that firmware expects: 276 byte
 * requests carrying a property id and a sequence number, answered by 280 byte
 * responses carrying a firmware return code.
 *
 * The charge curve below is the one the downstream device tree carries, so the
 * cell never sees a voltage or a current its vendor did not intend.
 *
 * The upper voltage limit is enforced here rather than handed to the charger.
 * Firmware's own termination voltage property reports 5200 mV, which is nowhere
 * near a safe stopping point for this pack, and writing the vendor's 4480 mV to
 * it is accepted but reads back as 5360 - so whatever that register means, it is
 * not millivolts of pack voltage, and trusting it to stop the charge would be a
 * guess with a battery on the other end. Taper on the measured pack voltage and
 * stop when it reaches the limit instead.
 */

#include <linux/auxiliary_bus.h>
#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/unaligned.h>
#include <linux/units.h>
#include <linux/workqueue.h>

#define MCA_OPCODE_READ			1
#define MCA_OPCODE_WRITE		2

#define MCA_PROP_BUCK_CHGR_STATUS	8193
#define MCA_PROP_BUCK_ENABLE_CHARGING	8195
#define MCA_PROP_BUCK_CHARGE_CURR	8197
#define MCA_PROP_BUCK_INPUT_CURR_LIMIT	8200
#define MCA_PROP_BUCK_ENABLE_AICL	8210
#define MCA_PROP_BUCK_RERUN_AICL	8211
#define MCA_PROP_BUCK_INPUT_SUSPEND	8217
#define MCA_PROP_USB_ONLINE		131073
#define MCA_PROP_USB_VOLTAGE_NOW	131074
#define MCA_PROP_USB_REAL_TYPE		131084
#define MCA_PROP_TYPEC_VERIFY_PROCESS	135170
#define MCA_PROP_TYPEC_PDO		135185
#define MCA_PROP_TYPEC_FIXED_VOL_REQ	135192
#define MCA_PROP_TYPEC_MAX_VOLT_SUPPORT	135197
#define MCA_PROP_BUCK_PACK_VBAT		8230
#define MCA_PROP_BUCK_PACK_TBAT		8234

/*
 * Source capabilities, as many fixed and programmable supplies as firmware
 * reports. Only fixed ones - the ones whose window has collapsed to a single
 * voltage - can be asked for with a plain voltage request.
 */
#define MCA_PDO_COUNT			7

struct mca_pdo {
	__le32 min_volt;
	__le32 max_volt;
	__le32 max_current;
};

/*
 * One leg of the charge curve: hold this current while the pack is below
 * max_volt_mv. Straight out of the downstream device tree's
 * mca_quick_charge_batt_para_gbl / normal_volt_para* tables.
 */
struct mca_charge_step {
	u32 max_volt_mv;
	u32 current_ma;
};

/*
 * Temperature band, from the same device tree's batt_para_lwn. Outside every
 * band the vendor stops charging altogether, and so do we.
 */
struct mca_charge_band {
	int temp_min;
	int temp_max;
	u32 float_volt_mv;
	const struct mca_charge_step *steps;
	unsigned int step_count;
};

static const struct mca_charge_step mca_steps_8_13[] = {
	{ 4493, 4320 }, { U32_MAX, 452 },
};

static const struct mca_charge_step mca_steps_13_18[] = {
	{ 4493, 7100 }, { U32_MAX, 452 },
};

static const struct mca_charge_step mca_steps_18_48[] = {
	{ 4150, 12400 }, { 4473, 7230 }, { U32_MAX, 452 },
};

static const struct mca_charge_step mca_steps_48_55[] = {
	{ 4053, 4320 }, { U32_MAX, 452 },
};

/*
 * The vendor's fast-charge tables push the pack to 4550 mV, which is only safe
 * with the cell modelling and the charge pump that come with them. These are
 * the plain ones, topping out at 4480 mV.
 */
static const struct mca_charge_band mca_charge_bands[] = {
	{  8, 13, 4500, mca_steps_8_13,  ARRAY_SIZE(mca_steps_8_13) },
	{ 13, 18, 4500, mca_steps_13_18, ARRAY_SIZE(mca_steps_13_18) },
	{ 18, 48, 4480, mca_steps_18_48, ARRAY_SIZE(mca_steps_18_48) },
	{ 48, 55, 4060, mca_steps_48_55, ARRAY_SIZE(mca_steps_48_55) },
};

/*
 * How much the input is allowed to draw. The hardware is rated far higher, but
 * nothing here manages the thermals the vendor stack does, so stay modest.
 */
static unsigned int power_limit_mw = 20000;
module_param(power_limit_mw, uint, 0644);
MODULE_PARM_DESC(power_limit_mw, "Input power budget in milliwatts");

/* Pack voltage the charge current budget is worked out against. */
#define MCA_BUDGET_REF_VOLT_MV		4000

#define MCA_MONITOR_INTERVAL_MS		10000

/* How far the pack must fall below the limit before charging resumes. */
#define MCA_RECHARGE_HYST_MV		80

/*
 * Firmware pushes these as notifications, meaning "re-read what you care
 * about". Everything else it may send is none of our business.
 */
#define MCA_NOTIF_CHARGER_TYPE_CHANGE	0
#define MCA_NOTIF_TYPEC_CHANGE		5

/* Values of MCA_PROP_BUCK_CHGR_STATUS worth naming. */
#define MCA_CHGR_STATUS_FULLON		3
#define MCA_CHGR_STATUS_DISABLED	7

#define MCA_DATA_LEN			256
#define MCA_REQUEST_TIMEOUT_MS		500

/*
 * A conservative input limit to come up with. AICL walks it back if the source
 * cannot hold it up, so this only has to be low enough not to brown out a
 * bus-powered hub before AICL gets a chance to react. Userspace can raise it
 * through input_current_limit once it knows what is attached.
 */
#define MCA_DEFAULT_INPUT_CURRENT_MA	500

struct mca_request {
	struct pmic_glink_hdr hdr;
	__le32 property_id;
	__le32 seq_num;
	u8 data[MCA_DATA_LEN];
};

struct mca_response {
	struct pmic_glink_hdr hdr;
	__le32 property_id;
	__le32 retcode;
	__le32 seq_num;
	u8 data[MCA_DATA_LEN];
};

/* Voltages in this domain are millivolts, currents milliamps. */
struct mca_notification {
	struct pmic_glink_hdr hdr;
	__le32 notification;
	u8 data[MCA_DATA_LEN];
};

struct xiaomi_mca {
	struct device *dev;
	struct pmic_glink_client *client;

	/* serializes requests: firmware allows a single outstanding one */
	struct mutex lock;
	struct completion ack;
	u32 seq;
	int error;
	u32 retcode;
	u8 data[MCA_DATA_LEN];

	struct work_struct setup_work;
	struct delayed_work monitor_work;
	struct power_supply *psy;
	u32 input_current_ma;
	u32 source_current_ma;
	u32 source_volt_mv;
	bool charging;
	bool link_up;

	/* last debugfs result, decoupled from the request lock */
	struct mutex result_lock;
	u8 result[MCA_DATA_LEN];
	size_t result_len;
	struct dentry *debugfs;
};

static int xiaomi_mca_xfer(struct xiaomi_mca *mca, u32 opcode, u32 property,
			   void *value, size_t len)
{
	struct mca_request req = {
		.hdr = {
			.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
			.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
			.opcode = cpu_to_le32(opcode),
		},
		.property_id = cpu_to_le32(property),
	};
	int ret;

	if (len > MCA_DATA_LEN)
		return -EINVAL;

	/*
	 * The power supply shows up before the charger PD service does, and
	 * userspace reads its properties as soon as it appears. Say "try again"
	 * rather than talking to a link that is not there yet.
	 */
	if (!mca->link_up)
		return -EAGAIN;

	guard(mutex)(&mca->lock);

	req.seq_num = cpu_to_le32(++mca->seq);
	if (opcode == MCA_OPCODE_WRITE && value)
		memcpy(req.data, value, len);

	mca->error = -ETIMEDOUT;
	reinit_completion(&mca->ack);

	/* The length is fixed: firmware knows the width of every property. */
	ret = pmic_glink_send(mca->client, &req, sizeof(req));
	if (ret) {
		dev_err(mca->dev, "failed to send property %u: %d\n", property, ret);
		return ret;
	}

	if (!wait_for_completion_timeout(&mca->ack,
					 msecs_to_jiffies(MCA_REQUEST_TIMEOUT_MS))) {
		dev_err(mca->dev, "timed out on property %u\n", property);
		return -ETIMEDOUT;
	}

	if (mca->error) {
		dev_err(mca->dev, "property %u failed, firmware code %u\n",
			property, mca->retcode);
		return mca->error;
	}

	if (opcode == MCA_OPCODE_READ && value)
		memcpy(value, mca->data, len);

	return 0;
}

static int xiaomi_mca_read_u32(struct xiaomi_mca *mca, u32 property, u32 *out)
{
	__le32 value;
	int ret;

	ret = xiaomi_mca_xfer(mca, MCA_OPCODE_READ, property, &value, sizeof(value));
	if (ret)
		return ret;

	*out = le32_to_cpu(value);
	return 0;
}

static int xiaomi_mca_write_u32(struct xiaomi_mca *mca, u32 property, u32 in)
{
	__le32 value = cpu_to_le32(in);

	return xiaomi_mca_xfer(mca, MCA_OPCODE_WRITE, property, &value, sizeof(value));
}

static int xiaomi_mca_write_u8(struct xiaomi_mca *mca, u32 property, u8 in)
{
	return xiaomi_mca_xfer(mca, MCA_OPCODE_WRITE, property, &in, sizeof(in));
}

/*
 * Ask the source for the highest fixed supply it offers that we can take, and
 * report back how much current it comes with. Running at 5 V is what makes the
 * input droop: the same power needs twice the current, and the drop across the
 * cable is enough for the input limiter to give up well below the charger's
 * rating. Returns 0 when the voltage was left alone.
 */
static int xiaomi_mca_negotiate_voltage(struct xiaomi_mca *mca, u32 *current_ma)
{
	struct mca_pdo pdo[MCA_PDO_COUNT];
	u32 best_volt = 0, best_current = 0;
	u32 limit, in_progress;
	int ret, i;

	ret = xiaomi_mca_read_u32(mca, MCA_PROP_TYPEC_MAX_VOLT_SUPPORT, &limit);
	if (ret || !limit)
		return 0;

	ret = xiaomi_mca_xfer(mca, MCA_OPCODE_READ, MCA_PROP_TYPEC_PDO,
			      pdo, sizeof(pdo));
	if (ret)
		return 0;

	for (i = 0; i < MCA_PDO_COUNT; i++) {
		u32 min_volt = le32_to_cpu(pdo[i].min_volt);
		u32 max_volt = le32_to_cpu(pdo[i].max_volt);

		if (!min_volt || min_volt != max_volt || max_volt > limit)
			continue;

		if (max_volt > best_volt) {
			best_volt = max_volt;
			best_current = le32_to_cpu(pdo[i].max_current);
		}
	}

	if (!best_volt)
		return 0;

	*current_ma = best_current;
	mca->source_current_ma = best_current;
	mca->source_volt_mv = best_volt;

	/*
	 * Firmware refuses a voltage change while an adapter authentication
	 * exchange is in flight, and answering an authentication we never
	 * started is not our business, so just leave the contract alone.
	 */
	ret = xiaomi_mca_read_u32(mca, MCA_PROP_TYPEC_VERIFY_PROCESS, &in_progress);
	if (ret || in_progress)
		return 0;

	ret = xiaomi_mca_write_u32(mca, MCA_PROP_TYPEC_FIXED_VOL_REQ, best_volt);
	if (ret)
		return 0;

	dev_info(mca->dev, "requested %u mV, source offers %u mA there\n",
		 best_volt, best_current);

	return best_volt;
}

static const struct power_supply_desc xiaomi_mca_psy_desc;

static const struct mca_charge_band *xiaomi_mca_band(int temp)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mca_charge_bands); i++)
		if (temp >= mca_charge_bands[i].temp_min &&
		    temp < mca_charge_bands[i].temp_max)
			return &mca_charge_bands[i];

	return NULL;
}

static u32 xiaomi_mca_step_current(const struct mca_charge_band *band, u32 volt_mv)
{
	unsigned int i;

	for (i = 0; i < band->step_count; i++)
		if (volt_mv < band->steps[i].max_volt_mv)
			return band->steps[i].current_ma;

	return band->steps[band->step_count - 1].current_ma;
}

/*
 * Follow the curve. Runs on a timer because both inputs move: the pack climbs
 * as it fills, which is what tapers the current, and its temperature decides
 * how much it may take at all.
 */
static void xiaomi_mca_monitor_work(struct work_struct *work)
{
	struct xiaomi_mca *mca = container_of(work, struct xiaomi_mca,
					      monitor_work.work);
	const struct mca_charge_band *band;
	u32 online, pack_mv, vbus_uv, charge_ma, input_ma;
	int temp;

	if (xiaomi_mca_read_u32(mca, MCA_PROP_USB_ONLINE, &online) || !online)
		goto again;

	if (xiaomi_mca_read_u32(mca, MCA_PROP_BUCK_PACK_VBAT, &pack_mv) ||
	    xiaomi_mca_read_u32(mca, MCA_PROP_BUCK_PACK_TBAT, (u32 *)&temp))
		goto again;

	band = xiaomi_mca_band(temp);
	if (!band) {
		dev_warn_ratelimited(mca->dev,
				     "pack at %d degrees is outside every charge band, stopping\n",
				     temp);
		xiaomi_mca_write_u8(mca, MCA_PROP_BUCK_ENABLE_CHARGING, 0);
		mca->charging = false;
		goto again;
	}

	/*
	 * The measured voltage includes the drop across the pack, so by the time
	 * the curve has tapered the current the reading is close to the real
	 * cell voltage. Stop there and only resume once it has settled back,
	 * which keeps this from chattering around the limit.
	 */
	if (pack_mv >= band->float_volt_mv) {
		if (mca->charging) {
			dev_info(mca->dev, "pack reached %u mV, charge complete\n",
				 pack_mv);
			xiaomi_mca_write_u8(mca, MCA_PROP_BUCK_ENABLE_CHARGING, 0);
			mca->charging = false;
		}
		goto again;
	}

	if (!mca->charging && pack_mv > band->float_volt_mv - MCA_RECHARGE_HYST_MV)
		goto again;

	/*
	 * Two independent ceilings: the curve says what the pack may take, the
	 * budget says what the input may draw. Apply both - the curve alone
	 * would ask for the vendor's fast-charge currents.
	 */
	charge_ma = min(xiaomi_mca_step_current(band, pack_mv),
			power_limit_mw / (MCA_BUDGET_REF_VOLT_MV / 1000));

	input_ma = mca->source_current_ma;
	if (!xiaomi_mca_read_u32(mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus_uv) &&
	    vbus_uv > MILLI)
		input_ma = min(input_ma, power_limit_mw * MILLI / (vbus_uv / MILLI));

	xiaomi_mca_write_u32(mca, MCA_PROP_BUCK_CHARGE_CURR, charge_ma);
	if (input_ma && input_ma != mca->input_current_ma) {
		if (!xiaomi_mca_write_u32(mca, MCA_PROP_BUCK_INPUT_CURR_LIMIT,
					  input_ma))
			mca->input_current_ma = input_ma;
	}
	xiaomi_mca_write_u8(mca, MCA_PROP_BUCK_ENABLE_CHARGING, 1);
	mca->charging = true;

	dev_dbg(mca->dev, "pack %u mV %d C: charge %u mA, input %u mA, limit %u mV\n",
		pack_mv, temp, charge_ma, input_ma, band->float_volt_mv);

again:
	schedule_delayed_work(&mca->monitor_work,
			      msecs_to_jiffies(MCA_MONITOR_INTERVAL_MS));
}

static void xiaomi_mca_setup_work(struct work_struct *work)
{
	struct xiaomi_mca *mca = container_of(work, struct xiaomi_mca, setup_work);
	u32 online, type, voltage, status;
	u32 input_current = mca->input_current_ma;
	int ret;

	ret = xiaomi_mca_read_u32(mca, MCA_PROP_USB_ONLINE, &online);
	if (ret)
		return;

	if (!online) {
		dev_dbg(mca->dev, "no input attached\n");
		goto changed;
	}

	if (!xiaomi_mca_read_u32(mca, MCA_PROP_USB_REAL_TYPE, &type) &&
	    !xiaomi_mca_read_u32(mca, MCA_PROP_USB_VOLTAGE_NOW, &voltage))
		dev_info(mca->dev, "input attached: type %u, %u uV\n", type, voltage);

	/*
	 * Order matters: leaving the input suspended makes every later setting
	 * a no-op, and enabling the charger before the input limit is known
	 * would let it pull whatever the previous limit happened to be.
	 */
	ret = xiaomi_mca_write_u8(mca, MCA_PROP_BUCK_INPUT_SUSPEND, 0);
	if (ret)
		return;

	if (xiaomi_mca_negotiate_voltage(mca, &input_current))
		mca->input_current_ma = input_current;

	ret = xiaomi_mca_write_u32(mca, MCA_PROP_BUCK_INPUT_CURR_LIMIT,
				   mca->input_current_ma);
	if (ret)
		return;

	/*
	 * Best effort: an old firmware without AICL still charges. Rerun it so
	 * the sweep reflects the supply we just asked for rather than the one
	 * that was attached a moment ago.
	 */
	xiaomi_mca_write_u8(mca, MCA_PROP_BUCK_ENABLE_AICL, 1);
	xiaomi_mca_write_u8(mca, MCA_PROP_BUCK_RERUN_AICL, 1);

	ret = xiaomi_mca_write_u8(mca, MCA_PROP_BUCK_ENABLE_CHARGING, 1);
	if (ret)
		return;

	mca->charging = true;

	if (!xiaomi_mca_read_u32(mca, MCA_PROP_BUCK_CHGR_STATUS, &status))
		dev_info(mca->dev, "charging enabled, charger status %u%s\n", status,
			 status == MCA_CHGR_STATUS_DISABLED ? " (still disabled)" : "");

	mod_delayed_work(system_wq, &mca->monitor_work, 0);

changed:
	/*
	 * Registering earlier would emit an uevent whose property reads all
	 * fail, because at probe time the charger PD service is not up yet.
	 */
	if (!mca->psy) {
		struct power_supply_config psy_cfg = { .drv_data = mca };

		mca->psy = devm_power_supply_register(mca->dev,
						     &xiaomi_mca_psy_desc,
						     &psy_cfg);
		if (IS_ERR(mca->psy)) {
			dev_err(mca->dev, "failed to register power supply: %pe\n",
				mca->psy);
			mca->psy = NULL;
		}
		return;
	}

	power_supply_changed(mca->psy);
}

static int xiaomi_mca_set_input_current(struct xiaomi_mca *mca, u32 microamp)
{
	u32 milliamp = microamp / MILLI;
	int ret;

	ret = xiaomi_mca_write_u32(mca, MCA_PROP_BUCK_INPUT_CURR_LIMIT, milliamp);
	if (ret)
		return ret;

	mca->input_current_ma = milliamp;
	return 0;
}

static int xiaomi_mca_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct xiaomi_mca *mca = power_supply_get_drvdata(psy);
	u32 value;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = xiaomi_mca_read_u32(mca, MCA_PROP_USB_ONLINE, &value);
		val->intval = !!value;
		return ret;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = xiaomi_mca_read_u32(mca, MCA_PROP_USB_VOLTAGE_NOW, &value);
		val->intval = value;
		return ret;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = xiaomi_mca_read_u32(mca, MCA_PROP_BUCK_INPUT_CURR_LIMIT, &value);
		val->intval = value * MILLI;
		return ret;
	case POWER_SUPPLY_PROP_STATUS:
		ret = xiaomi_mca_read_u32(mca, MCA_PROP_BUCK_CHGR_STATUS, &value);
		if (ret)
			return ret;

		if (value == MCA_CHGR_STATUS_DISABLED)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	default:
		return -EINVAL;
	}
}

static int xiaomi_mca_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct xiaomi_mca *mca = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (val->intval < 0)
			return -EINVAL;
		return xiaomi_mca_set_input_current(mca, val->intval);
	default:
		return -EINVAL;
	}
}

static int xiaomi_mca_property_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT;
}

static const enum power_supply_property xiaomi_mca_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static const struct power_supply_desc xiaomi_mca_psy_desc = {
	.name = "mca-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = xiaomi_mca_props,
	.num_properties = ARRAY_SIZE(xiaomi_mca_props),
	.get_property = xiaomi_mca_get_property,
	.set_property = xiaomi_mca_set_property,
	.property_is_writeable = xiaomi_mca_property_is_writeable,
};

/*
 * The firmware exposes far more properties than this driver has any business
 * programming on its own, and their widths are not self describing. Bringing a
 * new board up means poking at them by hand, so offer a raw window:
 *
 *   echo 'r 8193 4'	 > property	# read 4 bytes of property 8193
 *   echo 'r 135185 84'	 > property	# read a wider property, e.g. a PDO array
 *   echo 'w 8195 1 1'	 > property	# write value 1 as one byte
 *
 * Reads leave their result in the same file as hex bytes, low address first.
 */
static ssize_t xiaomi_mca_debugfs_write(struct file *file, const char __user *ubuf,
					size_t count, loff_t *ppos)
{
	struct xiaomi_mca *mca = file->private_data;
	u8 payload[MCA_DATA_LEN] = {};
	u32 property, value = 0;
	char buf[64], op;
	size_t len = 0;
	int ret, n;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	n = sscanf(buf, " %c %u %zu %u", &op, &property, &len, &value);
	if (n < 3 || len == 0 || len > MCA_DATA_LEN)
		return -EINVAL;

	switch (op) {
	case 'r':
		ret = xiaomi_mca_xfer(mca, MCA_OPCODE_READ, property, payload, len);
		break;
	case 'w':
		/* Only scalars are worth writing by hand. */
		if (n < 4 || len > sizeof(value))
			return -EINVAL;
		put_unaligned_le32(value, payload);
		ret = xiaomi_mca_xfer(mca, MCA_OPCODE_WRITE, property, payload, len);
		break;
	default:
		return -EINVAL;
	}

	if (ret)
		return ret;

	guard(mutex)(&mca->result_lock);
	memcpy(mca->result, payload, len);
	mca->result_len = len;

	return count;
}

static ssize_t xiaomi_mca_debugfs_read(struct file *file, char __user *ubuf,
				       size_t count, loff_t *ppos)
{
	struct xiaomi_mca *mca = file->private_data;
	char buf[MCA_DATA_LEN * 3 + MCA_DATA_LEN / 16 + 2];
	size_t len = 0;

	guard(mutex)(&mca->result_lock);

	/* hex_dump_to_buffer() only accepts a row of 16 or 32, so page it out. */
	for (size_t off = 0; off < mca->result_len; off += 16) {
		size_t row = min(mca->result_len - off, (size_t)16);

		len += hex_dump_to_buffer(mca->result + off, row, 16, 1,
					  buf + len, sizeof(buf) - len - 2, false);
		buf[len++] = '\n';
	}

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static const struct file_operations xiaomi_mca_debugfs_fops = {
	.open = simple_open,
	.read = xiaomi_mca_debugfs_read,
	.write = xiaomi_mca_debugfs_write,
	.llseek = default_llseek,
};

static void xiaomi_mca_callback(const void *data, size_t len, void *priv)
{
	const struct mca_notification *nty = data;
	const struct mca_response *resp = data;
	struct xiaomi_mca *mca = priv;
	u32 type;

	if (len < sizeof(struct pmic_glink_hdr))
		return;

	type = le32_to_cpu(resp->hdr.type);

	if (type == PMIC_GLINK_NOTIFY) {
		if (len < offsetof(struct mca_notification, data))
			return;

		switch (le32_to_cpu(nty->notification)) {
		case MCA_NOTIF_CHARGER_TYPE_CHANGE:
		case MCA_NOTIF_TYPEC_CHANGE:
			schedule_work(&mca->setup_work);
			break;
		}

		return;
	}

	/*
	 * Other clients share this owner, so a response is only ours if it has
	 * the right shape and answers the request we are actually waiting for.
	 */
	if (type != PMIC_GLINK_REQ_RESP || len != sizeof(*resp))
		return;

	if (le32_to_cpu(resp->seq_num) != mca->seq)
		return;

	mca->retcode = le32_to_cpu(resp->retcode);
	mca->error = mca->retcode ? -EIO : 0;

	if (!mca->error && le32_to_cpu(resp->hdr.opcode) == MCA_OPCODE_READ)
		memcpy(mca->data, resp->data, MCA_DATA_LEN);

	complete(&mca->ack);
}

static void xiaomi_mca_pdr_notify(void *priv, int state)
{
	struct xiaomi_mca *mca = priv;

	mca->link_up = state == SERVREG_SERVICE_STATE_UP;

	if (mca->link_up)
		schedule_work(&mca->setup_work);
	else
		cancel_delayed_work(&mca->monitor_work);
}

static int xiaomi_mca_probe(struct auxiliary_device *adev,
			    const struct auxiliary_device_id *id)
{
	struct xiaomi_mca *mca;
	int ret;

	mca = devm_kzalloc(&adev->dev, sizeof(*mca), GFP_KERNEL);
	if (!mca)
		return -ENOMEM;

	mca->dev = &adev->dev;
	mca->input_current_ma = MCA_DEFAULT_INPUT_CURRENT_MA;
	ret = devm_mutex_init(&adev->dev, &mca->lock);
	if (ret)
		return ret;
	ret = devm_mutex_init(&adev->dev, &mca->result_lock);
	if (ret)
		return ret;
	init_completion(&mca->ack);
	INIT_WORK(&mca->setup_work, xiaomi_mca_setup_work);
	INIT_DELAYED_WORK(&mca->monitor_work, xiaomi_mca_monitor_work);

	mca->client = devm_pmic_glink_client_alloc(&adev->dev,
						  PMIC_GLINK_OWNER_BATTMGR,
						  xiaomi_mca_callback,
						  xiaomi_mca_pdr_notify, mca);
	if (IS_ERR(mca->client))
		return PTR_ERR(mca->client);

	mca->debugfs = debugfs_create_dir(dev_name(&adev->dev), NULL);
	debugfs_create_file("property", 0600, mca->debugfs, mca,
			    &xiaomi_mca_debugfs_fops);

	auxiliary_set_drvdata(adev, mca);
	pmic_glink_client_register(mca->client);

	return 0;
}

static void xiaomi_mca_remove(struct auxiliary_device *adev)
{
	struct xiaomi_mca *mca = auxiliary_get_drvdata(adev);

	debugfs_remove_recursive(mca->debugfs);
	cancel_delayed_work_sync(&mca->monitor_work);
	cancel_work_sync(&mca->setup_work);
}

static const struct auxiliary_device_id xiaomi_mca_id_table[] = {
	{ .name = "pmic_glink.mca-charger", },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, xiaomi_mca_id_table);

static struct auxiliary_driver xiaomi_mca_driver = {
	.name = "xiaomi_mca_charger",
	.probe = xiaomi_mca_probe,
	.remove = xiaomi_mca_remove,
	.id_table = xiaomi_mca_id_table,
};
module_auxiliary_driver(xiaomi_mca_driver);

MODULE_DESCRIPTION("Xiaomi MCA charger over PMIC GLINK");
MODULE_LICENSE("GPL");
